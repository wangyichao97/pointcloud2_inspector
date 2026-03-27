from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/points_raw',
        description='PointCloud2 topic to inspect'
    )

    print_every_n_msg_arg = DeclareLaunchArgument(
        'print_every_n_msg',
        default_value='1',
        description='Print every Nth incoming message'
    )

    node = Node(
        package='pointcloud2_inspector',
        executable='pointcloud2_inspector_node',
        name='pointcloud2_inspector',
        output='screen',
        parameters=[{
            'topic_name': LaunchConfiguration('topic_name'),
            'print_every_n_msg': LaunchConfiguration('print_every_n_msg'),
        }]
    )

    return LaunchDescription([
        topic_name_arg,
        print_every_n_msg_arg,
        node,
    ])
