from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('zed_wrapper') + '/launch/zed_camera.launch.py'
        ),
        launch_arguments={
            'camera_model': 'zed',
            'resolution': 'HD720',
            'depth_mode': '1'
        }.items()
    )

    cone_detector = Node(
        package='cone_detector',
        executable='cone_detector_node',
        name='cone_detector'
    )

    return LaunchDescription([
        zed_launch,
        cone_detector
    ])
