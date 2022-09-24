import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
import os

def generate_launch_description():
    # for turn_on_robot
    pkg_share = launch_ros.substitutions.FindPackageShare(package='turn_on_robot').find('turn_on_robot')
    # for ricky_bot_description
    pkg_share1 = launch_ros.substitutions.FindPackageShare(package='ricky_bot_description').find('ricky_bot_description')
    default_model_path = os.path.join(pkg_share1, 'src/ricky_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share1, 'rviz/urdf_config.rviz')
    # for sllidar_ros2
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for A1/A2 is 115200
    frame_id = LaunchConfiguration('frame_id', default='laser_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    
    # create empty launch description
    ld = launch.LaunchDescription()
    
    # launch arguments for ricky_bot_description
    ld.add_action(launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'))
    # launch arguments for sllidar_ros2
    ld.add_action(launch.actions.DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'))

    ld.add_action(launch.actions.DeclareLaunchArgument(
        'serial_baudrate',
        default_value=serial_baudrate,
        description='Specifying usb port baudrate to connected lidar'))
    
    ld.add_action(launch.actions.DeclareLaunchArgument(
        'frame_id',
        default_value=frame_id,
        description='Specifying frame_id of lidar'))

    ld.add_action(launch.actions.DeclareLaunchArgument(
        'inverted',
        default_value=inverted,
        description='Specifying whether or not to invert scan data'))

    ld.add_action(launch.actions.DeclareLaunchArgument(
        'angle_compensate',
        default_value=angle_compensate,
        description='Specifying whether or not to enable angle_compensate of scan data'))

    # nodes for ricky_bot_description
    ld.add_action(
        Node(package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'))
    ld.add_action(
        Node(package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]))
    # nodes for turn_on_robot
    ld.add_action( 
        Node(package='turn_on_robot',
            executable='base_node'))
    ld.add_action(
        Node(package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]))
    # node for sllidar_ros2
    ld.add_action(
        Node(package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate}],
            output='screen'),)
    return ld

