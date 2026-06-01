#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time


class ROS2ArduinoBridge(Node):

    def __init__(self):
        super().__init__('ros2_arduino_bridge')

        # -----------------------------
        # Serial Configuration
        # -----------------------------
        port = '/dev/ttyACM0'
        baud = 115200

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=1
            )

            time.sleep(2)

            self.get_logger().info(
                f'Connected to Arduino on {port}'
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to open serial port: {e}'
            )
            raise

        # -----------------------------
        # Subscriber
        # -----------------------------
        self.present_sub = self.create_subscription(
            Float32MultiArray,
            'present_points',
            self.present_callback,
            10
        )

        # prevent sending duplicates
        self.last_x = None
        self.last_y = None

        self.get_logger().info(
            'Waiting for present_points...'
        )

    # ---------------------------------
    # Receive puck position
    # ---------------------------------
    def present_callback(self, msg):

        if len(msg.data) != 5:
            self.get_logger().warn(
                f'Expected 5 values, got {len(msg.data)}'
            )
            return

        frame, x, y, vx, vy = msg.data


        serial_string = f"{x/10.0:.2f},{y/10.0:.2f}\n"

        try:
            self.ser.write(serial_string.encode())

            self.get_logger().info(
                f"TX -> {serial_string.strip()}"
            )

        except Exception as e:
            self.get_logger().error(
                f"Serial write failed: {e}"
            )

    # ---------------------------------
    # Cleanup
    # ---------------------------------
    def destroy_node(self):

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
