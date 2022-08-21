from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turn_on_robot',
            executable='base_node',
            remappings=[
                ('/cmd_vel','/turtle1/cmd_vel')
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map2odomTF',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        )
    ])
