from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cone_detector_pkg',
            executable='cone_detector',
            name='cone_detector',
            output='screen',
            remappings=[
                ('/zed/left/image_rect_color', '/zed/left/image_rect_color'),
                ('/zed/depth/depth_registered', '/zed/depth/depth_registered'),
                ('/cones', '/cones'),
            ],
        ),
    ])
