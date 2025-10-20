from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch mocap_vicon as a composable node"""
    return LaunchDescription([
        ComposableNodeContainer(
            name='vicon_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='mocap_vicon',
                    plugin='mocap::ViconDriverComponent',
                    name='vicon',
                    parameters=[
                        {'server_address': 'mocap.perch'},
                        {'frame_rate': 100},
                        {'max_accel': 10.0},
                        {'publish_tf': False},
                        {'publish_pts': False},
                        {'fixed_frame_id': 'mocap'},
                        # Set to [''] to take in ALL models from Vicon
                        {'model_list': ['']},
                    ],
                    remappings=[
                        # Uncomment and modify the remapping if needed
                        # ('vicon/model_name/odom', '/model_name/odom'),
                    ]
                )
            ],
            output='screen',
        )
    ])
