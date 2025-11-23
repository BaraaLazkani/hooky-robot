import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import sys

class ArduinoNode(Node):
    def __init__(self, serial_port, baud_rate):
        super().__init__('arduino_node')
        
        # Set up serial communication with Arduino
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial = serial.Serial(self.serial_port, self.baud_rate)

        # Set up subscriber for receiving sensor data from Arduino
        self.command = self.create_subscription(String, 'ros2ard', self.command_callback, 10)
        self.command
        
        self.ard_to_ros = self.create_publisher(String, 'ard2ros', 10)
        timer_period = 0.01
        
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def command_callback(self, msg):
        self.serial.write(msg.data.encode())

    def timer_callback(self):
        if self.serial.in_waiting > 0:
            data = self.serial.readline().decode().strip()
            self.ard_to_ros.publish(String(data=data))
    	
    	
    	
    

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 3:
        print("Usage: ros2 run arduino communication <serial_port> <baud_rate>")
        return
    serial_port = sys.argv[1]
    baud_rate = int(sys.argv[2])
    
    arduino_node = ArduinoNode(serial_port, baud_rate)
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

