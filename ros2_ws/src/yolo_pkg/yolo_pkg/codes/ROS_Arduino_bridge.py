# Edited ROS_Arduino_bridge.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import serial
import time
import math

class ROS2ArduinoBridge(Node):
    def __init__(self):
        super().__init__('ros2_arduino_bridge')
        
        # Declare parameters for serial connection
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameters
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Initialize serial connection
        try:
            self.serial_connection = serial.Serial(port=port, baudrate=baud, timeout=1)
            time.sleep(2)  # Wait for connection to establish
            self.get_logger().info(f"Connected to Arduino Mega on {port} at {baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise
        
        # Store the current motor position from Arduino
        self.current_motor_x = 0.0
        self.current_motor_y = 0.0
        
        # Store the last sent position to Arduino
        self.last_sent_x = None
        self.last_sent_y = None
        
        # Store the latest puck data from present_points
        self.latest_puck = None
        
        # Flags to track if we've sent a position recently
        self.sent_stationary_position = False
        self.sent_predicted_position = False
        
        # Create subscribers
        self.present_points_sub = self.create_subscription(
            Float32MultiArray,
            'present_points',
            self.present_points_callback,
            10)
        
        self.motor_position_sub = self.create_subscription(
            Float32MultiArray,
            'motor_position',
            self.motor_position_callback,
            10)
        
        self.pred_points_sub = self.create_subscription(
            Float32MultiArray,
            'pred_points',
            self.pred_points_callback,
            10)
        
        # Create publisher for sending positions to Arduino
        self.go_to_pub = self.create_publisher(Float32MultiArray, 'go_to', 10)
        
        self.get_logger().info("ROS2-Arduino bridge initialized. Waiting for data...")

    def motor_position_callback(self, msg):
        """Callback for receiving current motor position from Arduino"""
        if len(msg.data) >= 2:
            self.current_motor_x = msg.data[0]
            self.current_motor_y = msg.data[1]
            self.get_logger().info(f"Received motor position: X={self.current_motor_x}, Y={self.current_motor_y}")
            
            # Check if motor has reached the last sent position (with 1mm tolerance)
            if self.last_sent_x is not None and abs(self.current_motor_x - self.last_sent_x) < 1.0 and abs(self.current_motor_y - self.last_sent_y) < 1.0:
                if self.latest_puck:
                    velocity_threshold = 5.0  # mm/s
                    if 0 <= self.latest_puck['y'] <= 700 and abs(self.latest_puck['vx']) < velocity_threshold and abs(self.latest_puck['vy']) < velocity_threshold:
                        # Send the present position
                        go_to_msg = Float32MultiArray()
                        go_to_msg.data = [self.latest_puck['x'], self.latest_puck['y']]
                        self.go_to_pub.publish(go_to_msg)
                        self.get_logger().info(f"Motor reached, puck stationary in range, sent present position: X={self.latest_puck['x']}, Y={self.latest_puck['y']}")
                        # Update last sent position
                        self.last_sent_x = self.latest_puck['x']
                        self.last_sent_y = self.latest_puck['y']
                    elif self.latest_puck['vy'] > 0:
                        # Send to home position x=400, y=0
                        go_to_msg = Float32MultiArray()
                        go_to_msg.data = [400.0, 0.0]
                        self.go_to_pub.publish(go_to_msg)
                        self.get_logger().info("Motor reached, vy positive, sent home position: X=400, Y=0")
                        # Update last sent position
                        self.last_sent_x = 400.0
                        self.last_sent_y = 0.0

    def present_points_callback(self, msg):
        """Callback for receiving present puck positions and velocities"""
        try:
            # Check if we have exactly 5 values (frame, x, y, vx, vy)
            if len(msg.data) != 5:
                self.get_logger().warn(f"Expected 5 values for present points, got {len(msg.data)}")
                return
                
            frame, x, y, vx, vy = msg.data
            
            # Always update the latest puck data
            self.latest_puck = {'x': x, 'y': y, 'vx': vx, 'vy': vy}
            
            # Check if velocities are close to zero and y position is within (0, 700)
            velocity_threshold = 5.0  # mm/s
            if abs(vx) < velocity_threshold and abs(vy) < velocity_threshold and 0 <= y <= 700:
                # Only send one message for this condition
                if not self.sent_stationary_position:
                    # Send this position to Arduino via go_to topic
                    go_to_msg = Float32MultiArray()
                    go_to_msg.data = [x, y]
                    self.go_to_pub.publish(go_to_msg)
                    self.sent_stationary_position = True
                    self.get_logger().info(f"Sent stationary position to Arduino: X={x}, Y={y}")
                    # Update last sent position
                    self.last_sent_x = x
                    self.last_sent_y = y
            else:
                # Reset flags if condition is no longer met
                self.sent_stationary_position = False
                self.sent_predicted_position = False
                
        except Exception as e:
            self.get_logger().error(f"Error processing present points: {e}")

    def pred_points_callback(self, msg):
        """Callback for receiving predicted puck positions"""
        try:
            # Check if we have a multiple of 3 values (frame, x, y for each prediction)
            if len(msg.data) % 3 != 0:
                self.get_logger().warn(f"Expected multiple of 3 values for pred points, got {len(msg.data)}")
                return
                
            # Process each predicted point
            for i in range(0, len(msg.data), 3):
                frame, x, y = msg.data[i:i+3]
                
                # Check if y position is within (0, 700)
                if 0 <= y <= 700:
                    # Calculate distance between motor and predicted position
                    distance = math.sqrt((x - self.current_motor_x)**2 + (y - self.current_motor_y)**2)
                    
                    # Calculate time for puck to get there (frame * 0.033)
                    puck_time = frame * 0.033
                    
                    # Calculate time for motor to get there (distance / 500 mm/s)
                    motor_time = distance / 500.0  # Assuming motor speed of 500 mm/s
                    
                    # Check if motor can reach position before puck
                    if motor_time <= puck_time:
                        # Only send one message for this condition
                        if not self.sent_predicted_position:
                            # Send this position to Arduino via go_to topic
                            go_to_msg = Float32MultiArray()
                            go_to_msg.data = [x, y]
                            self.go_to_pub.publish(go_to_msg)
                            self.sent_predicted_position = True
                            self.get_logger().info(f"Sent predicted position to Arduino: X={x}, Y={y}, Motor time: {motor_time:.2f}s, Puck time: {puck_time:.2f}s")
                            # Update last sent position
                            self.last_sent_x = x
                            self.last_sent_y = y
                            break  # Only send one position
                    
        except Exception as e:
            self.get_logger().error(f"Error processing predicted points: {e}")

    def destroy_node(self):
        # Clean up when node is destroyed
        if hasattr(self, 'serial_connection') and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info("Serial connection closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    ros2_arduino_bridge = ROS2ArduinoBridge()
    
    try:
        rclpy.spin(ros2_arduino_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        ros2_arduino_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
