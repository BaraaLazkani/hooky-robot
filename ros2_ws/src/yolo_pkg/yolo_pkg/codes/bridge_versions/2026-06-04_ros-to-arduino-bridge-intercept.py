#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
import threading


# ─────────────────────────────────────────────
# TUNE THIS: the Y line (cm) your robot defends.
# The bridge will find the predicted point where
# the puck crosses this line and send that X.
# Set it to your robot paddle's resting Y (cm).
# ─────────────────────────────────────────────
DEFENSE_Y_CM = 30.0


class ROS2ArduinoBridge(Node):

    def __init__(self):
        super().__init__('ros2_arduino_bridge')

        # -----------------------------
        # Serial Configuration
        # -----------------------------
        port = '/dev/ttyACM0'
        baud = 115200

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=1)
            time.sleep(2)
            self.get_logger().info(f'Connected to Arduino on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # -----------------------------
        # Rate limiting — 25 Hz
        # Fast enough for air hockey, safe for serial.
        # (Was 10 Hz before — that was the main lag source)
        # -----------------------------
        self.SEND_INTERVAL = 0.04        # seconds → 25 Hz
        self.last_send_time = 0.0

        # Duplicate filter
        self.last_sent_x = None
        self.last_sent_y = None
        self.POSITION_CHANGE_THRESHOLD = 0.3  # cm

        # -----------------------------
        # Prediction state
        # Stores the latest interception point from pred_points.
        # Falls back to current puck position if no prediction yet.
        # -----------------------------
        self.intercept_x_cm = None   # predicted X at defense line
        self.intercept_y_cm = DEFENSE_Y_CM
        self.prediction_age = 0.0    # seconds since last pred_points msg
        self.PREDICTION_TIMEOUT = 0.3  # fall back to present if pred is stale

        # -----------------------------
        # Background thread: drain Arduino serial output
        # Arduino prints position every 100ms — must read or buffer overflows
        # -----------------------------
        self._drain_running = True
        self._drain_thread = threading.Thread(target=self._drain_serial, daemon=True)
        self._drain_thread.start()

        # -----------------------------
        # Subscribers
        # -----------------------------
        # pred_points: predicted future trajectory → use for interception
        self.pred_sub = self.create_subscription(
            Float32MultiArray,
            'pred_points',
            self.pred_callback,
            10
        )

        # present_points: current puck position → fallback only
        self.present_sub = self.create_subscription(
            Float32MultiArray,
            'present_points',
            self.present_callback,
            10
        )

        self.get_logger().info('Bridge ready. Waiting for pred_points / present_points...')

    # -------------------------------------------------
    # DRAIN THREAD — keeps serial RX buffer empty
    # -------------------------------------------------
    def _drain_serial(self):
        while self._drain_running:
            try:
                if self.ser.is_open and self.ser.in_waiting > 0:
                    self.ser.readline().decode(errors='ignore')
            except Exception as e:
                self.get_logger().warn(f'Serial drain error: {e}')
            time.sleep(0.01)

    # -------------------------------------------------
    # PREDICTION CALLBACK
    # pred_points format: [frame, x_mm, y_mm,  frame, x_mm, y_mm, ...]
    # Strategy: find the predicted point where the puck
    # crosses DEFENSE_Y_CM — send that X to the robot.
    # If the puck never reaches DEFENSE_Y_CM in the
    # prediction window, use the closest point to it.
    # -------------------------------------------------
    def pred_callback(self, msg):
        data = msg.data
        if len(data) < 3:
            return

        # Parse flat list into (frame, x_cm, y_cm) triplets
        points = []
        for i in range(0, len(data) - 2, 3):
            frame = data[i]
            x_cm  = data[i + 1] / 10.0
            y_cm  = data[i + 2] / 10.0
            points.append((frame, x_cm, y_cm))

        if not points:
            return

        # Find the point closest to DEFENSE_Y_CM
        best = min(points, key=lambda p: abs(p[2] - DEFENSE_Y_CM))
        self.intercept_x_cm = best[1]
        self.intercept_y_cm = DEFENSE_Y_CM   # keep robot on the defense line
        self.prediction_age = time.time()

        self.get_logger().info(
            f'Intercept target: X={self.intercept_x_cm:.1f} cm  '
            f'(puck predicted near Y={best[2]:.1f} cm)'
        )

    # -------------------------------------------------
    # PRESENT CALLBACK
    # Current puck position — used as fallback when no
    # fresh prediction is available (puck is stationary
    # or just started moving)
    # -------------------------------------------------
    def present_callback(self, msg):
        if len(msg.data) != 5:
            return

        _, x_mm, y_mm, _, _ = msg.data

        # Use prediction if it's fresh enough
        prediction_is_fresh = (
            self.intercept_x_cm is not None and
            (time.time() - self.prediction_age) < self.PREDICTION_TIMEOUT
        )

        if prediction_is_fresh:
            x_cm = self.intercept_x_cm
            y_cm = self.intercept_y_cm
        else:
            # Fallback: chase current puck position
            x_cm = x_mm / 10.0
            y_cm = y_mm / 10.0

        self._send_to_arduino(x_cm, y_cm)

    # -------------------------------------------------
    # SEND TO ARDUINO — shared by both callbacks
    # -------------------------------------------------
    def _send_to_arduino(self, x_cm, y_cm):
        now = time.time()

        # Rate limit
        if now - self.last_send_time < self.SEND_INTERVAL:
            return

        # Duplicate filter
        if self.last_sent_x is not None and self.last_sent_y is not None:
            if (abs(x_cm - self.last_sent_x) < self.POSITION_CHANGE_THRESHOLD and
                    abs(y_cm - self.last_sent_y) < self.POSITION_CHANGE_THRESHOLD):
                return

        serial_string = f'{x_cm:.2f},{y_cm:.2f}\n'

        try:
            self.ser.write(serial_string.encode())
            self.last_send_time = now
            self.last_sent_x = x_cm
            self.last_sent_y = y_cm
            self.get_logger().info(f'TX -> {serial_string.strip()}')
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

    # -------------------------------------------------
    # Cleanup
    # -------------------------------------------------
    def destroy_node(self):
        self._drain_running = False
        self._drain_thread.join(timeout=1.0)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ROS2ArduinoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
