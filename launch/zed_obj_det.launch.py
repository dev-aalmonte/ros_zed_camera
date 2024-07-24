from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'ros_zed_point_cloud_test'

    return LaunchDescription([
        Node(
            package=package_name,
            executable='zed_od',
            output='screen'
        )
    ])