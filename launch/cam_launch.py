from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the Camera Driver
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'time_per_frame': [1, 30] # 30 FPS
            }]
        ),

        # 2. Start your Camera Processor
        Node(
            package='hardware_team_robot',
            executable='camera_node',
            name='camera_processor',
            output='screen',
            # You can add parameters here if you move HSV ranges to params later
        )
    ])