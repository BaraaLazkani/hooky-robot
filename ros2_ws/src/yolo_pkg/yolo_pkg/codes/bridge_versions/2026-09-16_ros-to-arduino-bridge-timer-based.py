#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
import threading


class ROS2ArduinoBridge(Node):

    def __init__(self):
        super().__init__('ros2_arduino_bridge')

        # -----------------------------
        # Serial Configuration
        # -----------------------------
        port = '/dev/ttyACM1'
        baud = 115200

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=1
            )
            time.sleep(2)
            self.get_logger().info(f'Connected to Arduino on {port}')

        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # -----------------------------
        # Rate limiting
        # We now send on a TIMER at a fixed rate, always, regardless of
        # whether the robot's own position has changed. The old code only
        # sent when present_callback fired AND the robot had moved more
        # than POSITION_CHANGE_THRESHOLD - that meant a still/slow-moving
        # robot would never forward fresh ball coordinates, so the Arduino
        # kept driving toward a stale/first-ever ball position.
        # -----------------------------
        self.SEND_INTERVAL = 0.3   # seconds -> ~3.3 Hz, adjust as needed

        # Latest known values (updated by subscriber callbacks as they arrive)
        self.robot_x_cm = None
        self.robot_y_cm = None
        self.pixel_x = -1.0
        self.pixel_y = -1.0
        self._data_lock = threading.Lock()

        # -----------------------------
        # Background thread: drain Arduino serial output
        # The Arduino prints position data every 100 ms.
        # If we never read it the OS RX buffer (~4096 bytes) fills
        # in ~18 seconds and breaks serial writes.
        # -----------------------------
        self._drain_running = True
        self._drain_thread = threading.Thread(
            target=self._drain_serial, daemon=True
        )
        self._drain_thread.start()

        # -----------------------------
        # Subscribers - these ONLY update the latest known values.
        # They no longer trigger the serial write directly.
        # -----------------------------
        self.present_sub = self.create_subscription(
            Float32MultiArray,
            'present_points',
            self.present_callback,
            10
        )
        self.pixel_sub = self.create_subscription(
            Float32MultiArray,
            'pixel_points',
            self.pixel_callback,
            10
        )

        # -----------------------------
        # Timer - this is what actually sends to the Arduino, on a fixed
        # schedule, using whatever the latest values are at that moment.
        # -----------------------------
        self.send_timer = self.create_timer(self.SEND_INTERVAL, self.send_to_arduino)

        self.get_logger().info('Waiting for present_points and pixel_points...')

    def pixel_callback(self, msg):
        if len(msg.data) < 2:
            return

        with self._data_lock:
            # NOTE: adjust these two divisors (9.8 / 10.23) to your actual
            # pixel-to-cm calibration for the ball's camera feed.
            self.pixel_x = msg.data[0] / 9.8
            self.pixel_y = msg.data[1] / 10.23

    def present_callback(self, msg):
        if len(msg.data) != 5:
            self.get_logger().warn(
                f'Expected 5 values, got {len(msg.data)}'
            )
            return

        frame, x, y, vx, vy = msg.data

        with self._data_lock:
            # Convert mm -> cm for Arduino
            self.robot_x_cm = x / 10.0
            self.robot_y_cm = y / 10.0

    # ---------------------------------
    # Background thread: continuously
    # read and discard Arduino output
    # ---------------------------------
    def _drain_serial(self):
        while self._drain_running:
            try:
                if self.ser.is_open and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    # Uncomment to see Arduino feedback:
                    # self.get_logger().info(f"Arduino: {line}")
            except Exception as e:
                self.get_logger().warn(f"Serial drain error: {e}")
            time.sleep(0.01)  # 100 Hz drain poll - keeps buffer empty

    # ---------------------------------
    # Timer callback: send the latest known robot + ball position
    # to the Arduino, every SEND_INTERVAL seconds, no matter what.
    # ---------------------------------
    def send_to_arduino(self):
        with self._data_lock:
            robot_x_cm = self.robot_x_cm
            robot_y_cm = self.robot_y_cm
            pixel_x = self.pixel_x
            pixel_y = self.pixel_y

        # Don't send until we've received at least one real reading of each
        if robot_x_cm is None or robot_y_cm is None:
            return

        # Order must match what the Arduino expects: robotX,robotY,ballX,ballY
        serial_string = f"{robot_x_cm:.2f},{robot_y_cm:.2f},{pixel_x:.0f},{pixel_y:.0f}\n"

        try:
            self.ser.write(serial_string.encode())
            self.get_logger().info(f"TX -> {serial_string.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ---------------------------------
    # Cleanup
    # ---------------------------------
    def destroy_node(self):
        self._drain_running = False
        self._drain_thread.join(timeout=1.0)

        if hasattr(self, 'ser'):
            if self.ser.is_open:
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
