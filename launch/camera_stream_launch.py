from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arkvision_six_cameras',
            executable='camera_stream_node',
            name='camera_stream_node'
        )
    ])

