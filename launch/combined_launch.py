from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Include the blickfeld_qb2_ros2_driver launch file
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                                FindPackageShare("blickfeld_qb2_ros2_driver"),
                                'launch',
                                'blickfeld_qb2_ros2_driver.launch.py'
                        ])
                ])
        ),

        # Run the arkvision_six_cameras node
        Node(
            package='arkvision_six_cameras',
            executable='camera_stream_node',
            name='camera_stream_node'
        ),

        # Include the ros2_ouster driver launch file
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                                FindPackageShare('ros2_ouster'),
                                'launch',
                                'driver_launch.py'
                        ])
                ])
        )
    ])
