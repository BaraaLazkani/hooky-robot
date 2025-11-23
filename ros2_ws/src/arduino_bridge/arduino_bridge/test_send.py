import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import sys

x = True
class MyNode(Node):
	def __init__(self):
		super().__init__('test_node')
		self.pub = self.create_publisher(String, 'ros2ard', 10)
		timer_period = 1
		self.timer = self.create_timer(timer_period, self.timer_callback)


	def timer_callback(self):
		global x
		if(x):
			data = "1"
		else:
			data = "0"
		self.pub.publish(String(data=data))
		x=not x
    	
    	
    	
    

def main(args=None):
	rclpy.init(args=args)
	node = MyNode()
	rclpy.spin(node)
	arduino_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

