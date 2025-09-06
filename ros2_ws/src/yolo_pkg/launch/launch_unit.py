from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='/home/jojo/htr/src/yolo_pkg/yolo_pkg/codes',
            executable='import_cv2_main_unit',
            name='trajectory_node',
            output='screen',
        ),
        Node(
            package='/home/jojo/htr/src/arduino_bridge/arduino_bridge',
            executable='ROS_Arduino_bridge',    
            name='bridge_node',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyACM0', 'baud_rate': 115200}]
        ),
    ])
