from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the list of camera IPs
    camera_ips = ['192.168.26.70', '192.168.26.71', '192.168.26.72', '192.168.26.73', '192.168.26.74', '192.168.26.75']

    # Create a LaunchDescription object
    ld = LaunchDescription()

    for ip in camera_ips:
        # Add a Node action for each camera IP
        camera_node = Node(
            package='arkvision_six_cameras',
            executable='camera_stream_node',  # Ensure this matches the name of your Python file without the .py extension
            name=f'camera_stream_node_{ip.split(".")[-1]}',  # Give each node a unique name based on the IP
            parameters=[{'camera_ip': ip}],
            output='screen'
        )
        ld.add_action(camera_node)

    return ld

