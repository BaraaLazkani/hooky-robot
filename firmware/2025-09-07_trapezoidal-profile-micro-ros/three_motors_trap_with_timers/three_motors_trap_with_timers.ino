#include <string.h>
#include <math.h>

#define Y1_STEP_PIN PA0 // Pin [22] Arduino Mega.
#define Y2_STEP_PIN PA1 // Pin [23] Arduino Mega.
#define X_STEP_PIN PA2  // Pin [24] Arduino Mega.
#define Y1_DIR_PIN PC0  // Pin [37] Arduino Mega.
#define Y2_DIR_PIN PC1  // Pin [36] Arduino Mega.
#define X_DIR_PIN PC2   // Pin [35] Arduino Mega.

// micro-ROS includes
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

// Constants
const float r = 0.03;       // Roller radius [m]->[30mm]->[3cm].
const float max_speed = 0.40; // Maximum linear speed [m/s]
const float min_step_freq = 60.0f; // Minimum step frequency [Hz]
const float max_accel = 4.2f; // Maximum acceleration [m/s^2]

// Step distance conversion
const float oneStep2m_x = (2.0f * M_PI * r) / (200.0f * 8.0f);
const float oneStep2m_y = (2.0f * M_PI * r) / (200.0f * 8.0f);

// Conversion functions
inline float rad2step_freq_x(float w) { return w * ((200.0f * 8.0f) / (2.0f * M_PI)); }
inline float rad2step_freq_y(float w) { return w * ((200.0f * 8.0f) / (2.0f * M_PI)); }
inline float linear2step_freq_x(float v) { return rad2step_freq_x(v / r); }
inline float linear2step_freq_y(float v) { return rad2step_freq_y(v / r); }

// Motor control structure
struct Motor {
  volatile float target_pos_m;   // Target position [m]
  volatile float present_pos_m;    // Current position [m] (Updated every step!)
  volatile long step_count;      // Current step count
  volatile long step_target;     // Total steps for this move
  volatile bool moving;          // Movement flag
  volatile uint16_t min_ocr;     // Pre-calculated OCR for max speed
  volatile uint16_t max_ocr;     // Pre-calculated OCR for start speed
  volatile int16_t accel;    // OCR increment per 10ms tick (accel)
  volatile int16_t decel;    // OCR increment per 10ms tick (decel)
  volatile uint16_t accel_ticks; // Number of 10ms ticks for acceleration
  volatile uint16_t const_ticks; // Number of 10ms ticks for constant speed
  volatile uint16_t decel_ticks; // Number of 10ms ticks for deceleration
  volatile uint16_t move_tick;   // Current tick within the move profile
  int8_t direction;              // Movement direction (1 or -1)
  volatile float step_freq_max;  // Maximum step frequency for this move
};

Motor x_motor, y_motor;

// Global velocity variables
double vx = 0, vy = 0, dt_x = 5000.0, dt_y = 5000.0;

// micro-ROS objects
rcl_node_t node;
rcl_publisher_t motor_pos_pub;
rcl_subscription_t go_to_sub;
std_msgs__msg__Float32MultiArray motor_pos_msg;
std_msgs__msg__Float32MultiArray received_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// Helper functions
inline float Y_DISTANCE() { return fabs(y_motor.present_pos_m - y_motor.target_pos_m); }
inline float X_DISTANCE() { return fabs(x_motor.present_pos_m - x_motor.target_pos_m); }
inline long target_x_pos_steps() { return (long)(X_DISTANCE() / oneStep2m_x); }
inline long target_y_pos_steps() { return (long)(Y_DISTANCE() / oneStep2m_y); }

// NOW plan_move can use Motor* because Motor is already defined
void plan_move(struct Motor* m, float velocity, float distance, bool is_forward) {
  // Calculate step target
  float steps_per_m = (&x_motor == m) ? 1.0f/oneStep2m_x : 1.0f/oneStep2m_y;
  m->step_target = 2L * lroundf(distance * steps_per_m);
  m->direction = is_forward ? 1 : -1;

  // Calculate min_v
  float min_v = (&x_motor == m) ? min_step_freq * oneStep2m_x : min_step_freq * oneStep2m_y;

  // Calculate max step freq
  float v_max = velocity;
  float step_freq_max = (&x_motor == m) ? linear2step_freq_x(v_max) : linear2step_freq_y(v_max);
  m->step_freq_max = step_freq_max;

  // Calculate min OCR (max speed)
  double max_freq = step_freq_max;
  m->min_ocr = (max_freq > 0.1) ? (uint16_t)(1000000.0 / max_freq - 1.0) : 0;

  // Set fixed starting speed (~100 Hz)
  double min_freq = min_step_freq;
  m->max_ocr = (uint16_t)(1000000.0 / min_freq - 1.0);

  // Calculate phases
  float delta_v = v_max - min_v;
  if (delta_v <= 0.0f) {
    // Constant slow speed
    m->accel_ticks = 0;
    m->decel_ticks = 0;
    float total_time = (min_v > 0.001f) ? distance / min_v : 0;
    m->const_ticks = (uint16_t)(total_time * 1000.0f + 0.5f);
    m->step_freq_max = min_freq;
    m->min_ocr = m->max_ocr;
  } else {
    float d_ramp = (v_max * v_max - min_v * min_v) / max_accel;
    float t_accel = delta_v / max_accel;
    if (distance <= d_ramp) {
      // Triangle profile
      float v_peak = sqrt(min_v * min_v + max_accel * distance);
      m->step_freq_max = (&x_motor == m) ? linear2step_freq_x(v_peak) : linear2step_freq_y(v_peak);
      max_freq = m->step_freq_max;
      m->min_ocr = (max_freq > 0.1) ? (uint16_t)(1000000.0 / max_freq - 1.0) : 0;
      t_accel = (v_peak - min_v) / max_accel;
      m->accel_ticks = (uint16_t)(t_accel * 1000.0f + 0.5f);
      m->decel_ticks = m->accel_ticks;
      m->const_ticks = 0;
    } else {
      // Trapezoidal profile
      float d_const = distance - d_ramp;
      float t_const = d_const / v_max;
      m->accel_ticks = (uint16_t)(t_accel * 1000.0f + 0.5f);
      m->decel_ticks = m->accel_ticks;
      m->const_ticks = (uint16_t)(t_const * 1000.0f + 0.5f);
    }
  }

  // Not using linear OCR change anymore
  m->accel = 0;
  m->decel = 0;

  // Initialize motor control register with starting speed
  if (&x_motor == m) {
    OCR1A = m->max_ocr;
  } else {
    OCR3A = m->max_ocr;
  }
  m->moving = true;
}

void start_X_motor(bool state) {
  TIMSK1 = (state) ? (TIMSK1 | (1 << OCIE1A)) : (TIMSK1 & ~(1 << OCIE1A));
  x_motor.moving = state;
}

void start_Y_motor(bool state) {
  TIMSK3 = (state) ? (TIMSK3 | (1 << OCIE3A)) : (TIMSK3 & ~(1 << OCIE3A));
  y_motor.moving = state;
}

void setupTimer_X() {
  noInterrupts();
  start_X_motor(false);
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 65535; // Start with slow speed
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  interrupts();
}

void setupTimer_Y() {
  noInterrupts();
  start_Y_motor(false);
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  OCR3A = 65535; // Start with slow speed
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  interrupts();
}

void setupTimer_Y_TRAP() {
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  OCR4A = 1999; // (16,000,000 / (8 * 1000)) - 1
  TCCR4B |= (1 << WGM42);
  TCCR4B |= (1 << CS41);
  TIMSK4 &= ~(1 << OCIE4A); // Start disabled
  interrupts();
}

void setupTimer_X_TRAP() {
  noInterrupts();
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;
  OCR5A = 1999; // (16,000,000 / (8 * 1000)) - 1
  TCCR5B |= (1 << WGM52);
  TCCR5B |= (1 << CS51);
  TIMSK5 &= ~(1 << OCIE5A); // Start disabled
  interrupts();
}

// Subscription callback
void go_to_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  if (msg->data.size >= 2) {
    float target_x = msg->data.data[0] / 1000.0f;  // Convert mm to meters
    float target_y = msg->data.data[1] / 1000.0f;  // Convert mm to meters
    
    // Reset all counters and state
    x_motor.step_count = 0;
    x_motor.move_tick = 0;
    x_motor.moving = false;
    y_motor.step_count = 0;
    y_motor.move_tick = 0;
    y_motor.moving = false;

    // Store targets
    y_motor.target_pos_m = target_y;
    x_motor.target_pos_m = target_x;

    // Calculate deltas and distances
    float delta_x = x_motor.target_pos_m - x_motor.present_pos_m;
    float delta_y = y_motor.target_pos_m - y_motor.present_pos_m;
    float distance_x = fabs(delta_x);
    float distance_y = fabs(delta_y);

    // Calculate timing and velocities with signs
    float time_x = (distance_x > 0.001) ? distance_x / max_speed : 0;
    float time_y = (distance_y > 0.001) ? distance_y / max_speed : 0;
    
    if (distance_x >= distance_y) {
      vx = (time_x > 0) ? delta_x / time_x : 0;
      vy = (time_x > 0) ? delta_y / time_x : 0;
    } else {
      vx = (time_y > 0) ? delta_x / time_y : 0;
      vy = (time_y > 0) ? delta_y / time_y : 0;
    }

    // Plan moves
    plan_move(&x_motor, fabs(vx), distance_x, (delta_x > 0));
    plan_move(&y_motor, fabs(vy), distance_y, (delta_y > 0));

    // Set direction pins
    if (delta_x > 0) PORTC |= (1 << X_DIR_PIN);
    else PORTC &= ~(1 << X_DIR_PIN);
    
    if (delta_y > 0) PORTC |= (1 << Y1_DIR_PIN) | (1 << Y2_DIR_PIN);
    else PORTC &= ~((1 << Y1_DIR_PIN) | (1 << Y2_DIR_PIN));

    // Start planner timers
    TIMSK5 |= (1 << OCIE5A);
    TIMSK4 |= (1 << OCIE4A);

    // Start pulse timers
    if (x_motor.step_target > 0) start_X_motor(true);
    if (y_motor.step_target > 0) start_Y_motor(true);
  }
}

// ISRs
ISR(TIMER1_COMPA_vect) {
  PORTA ^= (1 << X_STEP_PIN);
  
  if (x_motor.moving) {
    x_motor.step_count++;
    x_motor.present_pos_m += x_motor.direction * oneStep2m_x;

    if (x_motor.step_count >= x_motor.step_target) {
      start_X_motor(false);
      x_motor.moving = false;
      vx = 0;
      x_motor.present_pos_m = x_motor.target_pos_m;
    }
  }
}

ISR(TIMER3_COMPA_vect) {
  PORTA ^= ((1 << Y1_STEP_PIN) | (1 << Y2_STEP_PIN));
  
  if (y_motor.moving) {
    y_motor.step_count++;
    y_motor.present_pos_m += y_motor.direction * oneStep2m_y;

    if (y_motor.step_count >= y_motor.step_target) {
      start_Y_motor(false);
      y_motor.moving = false;
      vy = 0;
      y_motor.present_pos_m = y_motor.target_pos_m;
    }
  }
}

ISR(TIMER4_COMPA_vect) {
  if (!y_motor.moving) return;
  
  y_motor.move_tick++;

  float freq;
  uint16_t ocr;

  if (y_motor.move_tick < y_motor.accel_ticks) {
    float progress = (float) y_motor.move_tick / y_motor.accel_ticks;
    freq = min_step_freq + (y_motor.step_freq_max - min_step_freq) * progress;
  }
  else if (y_motor.move_tick < (y_motor.accel_ticks + y_motor.const_ticks)) {
    freq = y_motor.step_freq_max;
  }
  else if (y_motor.move_tick < (y_motor.accel_ticks + y_motor.const_ticks + y_motor.decel_ticks)) {
    uint16_t decel_tick = y_motor.move_tick - (y_motor.accel_ticks + y_motor.const_ticks);
    float progress = (float) decel_tick / y_motor.decel_ticks;
    freq = y_motor.step_freq_max - (y_motor.step_freq_max - min_step_freq) * progress;
  }
  else {
    TIMSK4 &= ~(1 << OCIE4A);
    return;
  }

  ocr = (uint16_t)(1000000.0 / freq - 1.0);
  OCR3A = ocr;
}

ISR(TIMER5_COMPA_vect) {  
  if (!x_motor.moving) return;
  
  x_motor.move_tick++;

  float freq;
  uint16_t ocr;

  if (x_motor.move_tick < x_motor.accel_ticks) {
    float progress = (float) x_motor.move_tick / x_motor.accel_ticks;
    freq = min_step_freq + (x_motor.step_freq_max - min_step_freq) * progress;
  }
  else if (x_motor.move_tick < (x_motor.accel_ticks + x_motor.const_ticks)) {
    freq = x_motor.step_freq_max;
  }
  else if (x_motor.move_tick < (x_motor.accel_ticks + x_motor.const_ticks + x_motor.decel_ticks)) {
    uint16_t decel_tick = x_motor.move_tick - (x_motor.accel_ticks + x_motor.const_ticks);
    float progress = (float) decel_tick / x_motor.decel_ticks;
    freq = x_motor.step_freq_max - (x_motor.step_freq_max - min_step_freq) * progress;
  }
  else {
    TIMSK5 &= ~(1 << OCIE5A);
    return;
  }

  ocr = (uint16_t)(1000000.0 / freq - 1.0);
  OCR1A = ocr;
}

void setup() {
  // Initialize serial
  Serial.begin(115200);
  
  // Set pin modes
  DDRA |= (1 << Y1_STEP_PIN) | (1 << Y2_STEP_PIN) | (1 << X_STEP_PIN);
  DDRC |= (1 << Y1_DIR_PIN) | (1 << Y2_DIR_PIN) | (1 << X_DIR_PIN);
  PORTA = 0;
  PORTC = 0;

  // Initialize motor positions
  x_motor.present_pos_m = 0.355;
  y_motor.present_pos_m = 0.355;

  // Setup timers
  setupTimer_X();
  setupTimer_Y(); 
  setupTimer_Y_TRAP();
  setupTimer_X_TRAP();

  // micro-ROS setup
  set_microros_serial_transports(Serial);
  
  // Wait for agent connection
  Serial.println("Waiting for micro-ROS agent...");
  while (!rmw_uros_ping_agent(1000, 2)) {
    delay(1000);
    Serial.println("Waiting for micro-ROS agent...");
  }
  Serial.println("Micro-ROS agent connected!");
  
  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "arduino_node", "", &support);
  
  // Create publisher
  rclc_publisher_init_default(
    &motor_pos_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "motor_position");
  
  // Create subscriber
  rclc_subscription_init_default(
    &go_to_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "go_to");
  
  // Initialize executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &go_to_sub, &received_msg, &go_to_callback, ON_NEW_DATA);
  
  // Initialize motor position message
  motor_pos_msg.data.capacity = 2;
  motor_pos_msg.data.size = 2;
  motor_pos_msg.data.data = (float*)malloc(sizeof(float)*2);
  
  Serial.println("Ready - Waiting for ROS commands");
}

void loop() {
  // Publish current motor position
  motor_pos_msg.data.data[0] = x_motor.present_pos_m * 1000.0f;  // Convert meters to mm
  motor_pos_msg.data.data[1] = y_motor.present_pos_m * 1000.0f;  // Convert meters to mm
  rcl_publish(&motor_pos_pub, &motor_pos_msg, NULL);
  
  // Spin executor to process incoming messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  
  // Small delay to avoid flooding the ROS network
  delay(100);
}
