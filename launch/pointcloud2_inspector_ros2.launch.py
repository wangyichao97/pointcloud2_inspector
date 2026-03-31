from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('topic_name', default_value='/points_raw'),
        DeclareLaunchArgument('print_every_n_msg', default_value='1'),
        Node(
            package='lidar_tools',
            executable='pointcloud2_inspector_node',
            name='pointcloud2_inspector_node',
            output='screen',
            parameters=[{
                'topic_name': LaunchConfiguration('topic_name'),
                'print_every_n_msg': LaunchConfiguration('print_every_n_msg')
            }]
        )
    ])
