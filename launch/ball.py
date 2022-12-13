import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
import os

def generate_launch_description():
    ld = launch.LaunchDescription()

    ld.add_action(
        
        Node(
            package='gscam2',
            executable='gscam_main',
            name='left_cam',
            output='screen',
            parameters=
            [{
                'camera_info_url':'file:///home/ricky/ws/src/gscam2/cfg/left.yaml',
                'camera_name':'left',
                'gscam_config':'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1280,height=720, format=(string)NV12, framerate=15/1 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! videoconvert'
            }],
            remappings=[
                ('/image_raw', '/left/image_raw'),
                ('/camera_info', '/left/camera_info')
            ]
        )
    )
    
    ld.add_action(
        Node(
            package='gscam2',
            executable='gscam_main',
            name='right_cam',
            output='screen',
            parameters=
            [{
                'camera_info_url':'file:///home/ricky/ws/src/gscam2/cfg/right.yaml',
                'camera_name':'right',
                'gscam_config':'nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=1280,height=720, format=(string)NV12, framerate=15/1 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! videoconvert'
            }],
            remappings=[
                ('/image_raw', '/right/image_raw'),
                ('/camera_info', '/right/camera_info')
            ]
        )
    )
    
    ld.add_action(
        Node(
            package='ball_detect',
            executable='pick_ball_server',
            name='pick_ball_server',
            output='screen',
            remappings=[
                #('/image_raw', '/right/image_raw')
            ]
        )
    )
    ld.add_action(
        Node(
            package='ball_detect',
            executable='ball_detector',
            name='ball_detect',
            output='screen',
            remappings=[]
        )
    )
    return ld
