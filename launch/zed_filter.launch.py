import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
)

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    # Launch configuration variables
    start_zed_node = LaunchConfiguration('start_zed_node')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    # Rviz2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('zed_display_rviz2'),
        'rviz2',
        camera_model_val + '.rviz'
    )

    # Rviz2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name_val,
        executable='rviz2',
        name=camera_model_val +'_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
    )

    # ZED Wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_name': camera_name_val,
            'camera_model': camera_model_val
        }.items(),
        condition=IfCondition(start_zed_node)
    )

    return [
        rviz2_node,
        zed_wrapper_launch
    ]

def generate_launch_description():
    april_ros_share_dir = get_package_share_directory('apriltag_ros')
    tags_36h11_yaml_file = os.path.join(april_ros_share_dir, 'cfg', 'tags_36h11.yaml')
    package_name = 'ros_zed_point_cloud_test'

    return LaunchDescription([
        # Zed RVIZ Launch
        DeclareLaunchArgument(
            'start_zed_node',
            default_value='True',
            description='Set to `False` to start only Rviz2 if a ZED node is already running.'),
        DeclareLaunchArgument(
            'camera_name',
            default_value=TextSubstitution(text='zed'),
            description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
        DeclareLaunchArgument(
            'camera_model',
            description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
            choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual'],
            default_value='zed2i'
            ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Enable publication of the `odom -> camera_link` TF.',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'publish_map_tf',
            default_value='true',
            description='Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.',
            choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
        
        Node(
            package=package_name,
            executable='zed_filter',
            parameters=[
                {'MIN_X': -1},
                {'MAX_X': 0.75},
                {'MIN_Y': -0.2},
                {'MAX_Y': 0.2},
                {'MIN_Z': 0},
                {'MAX_Z': 1.25}
            ],
            output='screen'
        ),
        Node(
            package=package_name,
            executable='zed_segmentation',
            parameters=[
                {'MODEL_INDEX': 0},
                {'DISTANCE_THRESHOLD': 0.01},
                {'DISTANCE_TOLERANCE': 1},
                {'COLOR_TOLERANCE': 5},
                {'MIN_CLUSTER_SIZE': 35},
                {'MAX_CLUSTER_SIZE': 150}
            ],
            output='screen'
        ),
        Node(
            package=package_name,
            executable='zed_cluster',
            parameters=[
                {'MIN_CLUSTER_SIZE': 100},
                {'MAX_CLUSTER_SIZE': 5000}
            ],
            output='screen'
        ),
        Node(
            package='ros_obj_det',
            executable='yolo_obj_det',
            parameters=[
                {"package_share_path": FindPackageShare('ros_obj_det')},
                {'confidence_value': .5},
            ],
            output='screen'
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/zed/zed_node/left_raw/image_raw_color'),
                ('camera_info', '/zed/zed_node/left_raw/camera_info')
            ],
            parameters=[
                {'params_file': tags_36h11_yaml_file},
                {'tag': {'ids': [7], 'frames': ['Detection']}}
            ]
        ),
        Node(
            package='ros_obj_det',
            executable='apriltag_det',
            output='screen'
        )
    ])
