from setuptools import setup

package_name = 'cone_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cone_detector_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='YOLO-based cone detector ROS2 node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cone_detector = cone_detector.cone_detector_node:main',
        ],
    },
)
