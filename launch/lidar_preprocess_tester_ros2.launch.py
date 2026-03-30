from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('topic_name', default_value='/points_raw'),
        DeclareLaunchArgument('lidar_type', default_value='6'),
        DeclareLaunchArgument('blind', default_value='1.0'),
        DeclareLaunchArgument('point_filter_num', default_value='1'),
        DeclareLaunchArgument('n_scans', default_value='6'),
        DeclareLaunchArgument('sort_by_time', default_value='false'),
        DeclareLaunchArgument('publish_full_cloud', default_value='true'),
        DeclareLaunchArgument('publish_surf_cloud', default_value='true'),
        Node(
            package='lidar_tools',
            executable='lidar_preprocess_tester_node',
            name='lidar_preprocess_tester_node',
            output='screen',
            parameters=[{
                'topic_name': LaunchConfiguration('topic_name'),
                'lidar_type': LaunchConfiguration('lidar_type'),
                'blind': LaunchConfiguration('blind'),
                'point_filter_num': LaunchConfiguration('point_filter_num'),
                'n_scans': LaunchConfiguration('n_scans'),
                'sort_by_time': LaunchConfiguration('sort_by_time'),
                'publish_full_cloud': LaunchConfiguration('publish_full_cloud'),
                'publish_surf_cloud': LaunchConfiguration('publish_surf_cloud')
            }]
        )
    ])
