from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    blickfeld_container = ComposableNodeContainer(
        name="blickfeld_qb2_component",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="blickfeld_qb2_ros2_driver",
                plugin="blickfeld::ros_interop::Qb2Driver",
                name="blickfeld_qb2_driver",
                parameters=[
                    {
                        "fqdn": "192.168.26.26",
                        "frame_id": "lidar",
                        "point_cloud_topic": "/bf/points_raw",
                        "use_measurement_timestamp": False,
                        "publish_intensity": True,
                        "publish_point_id": True,
                    }
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        blickfeld_container,

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

