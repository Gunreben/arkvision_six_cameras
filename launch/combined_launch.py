from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package share directories
    ouster_share = FindPackageShare(package='ouster_ros').find('ouster_ros')
    blickfeld_share = FindPackageShare(package='blickfeld_qb2_ros2_driver').find('blickfeld_qb2_ros2_driver')

    # Define the launch file paths
    ouster_launch_path = ouster_share / 'launch' / 'driver.launch.py'
    blickfeld_launch_path = blickfeld_share / 'launch' / 'blickfeld_qb2_ros2_driver.launch.py'

    # Include the launch descriptions
    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ouster_launch_path),
        #launch_arguments={'parameter_overrides': 'your_parameters_here'}.items(),
    )

    blickfeld_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(blickfeld_launch_path),
        #launch_arguments={'parameter_overrides': 'your_parameters_here'}.items(),
    )

    # Return the LaunchDescription object
    return LaunchDescription([
        ouster_launch,
        blickfeld_launch,
        ouster_launch  # Assuming you want to launch the ouster driver twice for different configurations
    ])

