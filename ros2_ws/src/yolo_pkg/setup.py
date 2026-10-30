import os
from glob import glob
from setuptools import setup

package_name = 'yolo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 👇 this line makes colcon install your launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jojo',
    maintainer_email='jojo@example.com',
    description='YOLO puck detection, position mapping and trajectory prediction for the Hooky Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'import_cv2_main_unit = yolo_pkg.import_cv2_main_unit:main',
        'ROS_Arduino_bridge = yolo_pkg.ROS_Arduino_bridge:main',
        ],
    },

)
