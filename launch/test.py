import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='turn_on_robot').find('turn_on_robot')
    ld = launch.LaunchDescription()

    ld.add_action( 
        Node(
            package='turn_on_robot',
            executable='base_node'
        )
    )
    ld.add_action(
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
        )
    )
    return ld
