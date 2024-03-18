from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package where the launch files are located
    arkvision_six_cameras_pkg_prefix = FindPackageShare('arkvision_six_cameras').find('arkvision_six_cameras')
    blickfeld_qb2_ros2_driver_pkg_prefix = FindPackageShare('blickfeld_qb2_ros2_driver').find('blickfeld_qb2_ros2_driver')
    ouster_ros_pkg_prefix = FindPackageShare('ouster_ros').find('ouster_ros')

    # Define the path to the launch files
    arkvision_launch_file = PythonLaunchDescriptionSource([arkvision_six_cameras_pkg_prefix, '/launch/camera_stream_launch.py'])
    blickfeld_launch_file = PythonLaunchDescriptionSource([blickfeld_qb2_ros2_driver_pkg_prefix, '/launch/blickfeld_qb2_ros2_driver.launch.py'])
    ouster_launch_file = PythonLaunchDescriptionSource([ouster_ros_pkg_prefix, '/launch/driver.launch.py'])

    # Include the launch files
    return LaunchDescription([
        IncludeLaunchDescription(arkvision_launch_file),
        IncludeLaunchDescription(blickfeld_launch_file),
        IncludeLaunchDescription(ouster_launch_file)
    ])

