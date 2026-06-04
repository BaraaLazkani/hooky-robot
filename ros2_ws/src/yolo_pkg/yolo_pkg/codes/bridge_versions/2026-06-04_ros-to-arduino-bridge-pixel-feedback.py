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
        # Send at most every SEND_INTERVAL seconds (10 Hz)
        # Prevents flooding the Arduino's serial buffer at 30 fps
        # -----------------------------
        self.SEND_INTERVAL = 0.3   # seconds → 10 Hz max
        self.last_send_time = 0.0

        # Duplicate filtering — don't resend the same position
        self.last_sent_x = None
        self.last_sent_y = None
        self.POSITION_CHANGE_THRESHOLD = 0.5  # cm — ignore tiny jitter
        # Latest pixel coordinates
        self.pixel_x = -1.0
        self.pixel_y = -1.0

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
        # Subscriber
        # -----------------------------
        self.present_sub = self.create_subscription(
            Float32MultiArray,
            'present_points',
            self.present_callback,
            10
        )
        self.pixel_sub = self.create_subscription(Float32MultiArray,'pixel_points',self.pixel_callback,10)

        self.get_logger().info('Waiting for present_points...')
    def pixel_callback(self, msg):

      if len(msg.data) < 2:
          return

      self.pixel_x = msg.data[0]
      self.pixel_y = msg.data[1]

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
            time.sleep(0.01)  # 100 Hz drain poll — keeps buffer empty

    # ---------------------------------
    # Receive puck position → send to Arduino
    # ---------------------------------
    def present_callback(self, msg):

        if len(msg.data) != 5:
            self.get_logger().warn(
                f'Expected 5 values, got {len(msg.data)}'
            )
            return

        frame, x, y, vx, vy = msg.data

        # Convert mm → cm for Arduino
        x_cm = x / 10.0
        y_cm = y / 10.0

        # --- Rate limit: skip if called too soon ---
        now = time.time()
        if now - self.last_send_time < self.SEND_INTERVAL:
            return

        # --- Duplicate filter: skip if position hasn't changed much ---
        if self.last_sent_x is not None and self.last_sent_y is not None:
            dx = abs(x_cm - self.last_sent_x)
            dy = abs(y_cm - self.last_sent_y)
            if dx < self.POSITION_CHANGE_THRESHOLD and dy < self.POSITION_CHANGE_THRESHOLD:
                return

        # --- Send to Arduino ---
        self.pixel_x=self.pixel_x/9.8;
        self.pixel_y=self.pixel_y/10.23;
        serial_string = f"{x_cm:.2f},{y_cm:.2f},"f"{self.pixel_y:.0f},"f"{self.pixel_x:.0f}\n"

        try:
            self.ser.write(serial_string.encode())
            self.last_send_time = now
            self.last_sent_x = x_cm
            self.last_sent_y = y_cm
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
