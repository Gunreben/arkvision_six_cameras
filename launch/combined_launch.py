from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration  # Corrected import for LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package where the launch files are located
    arkvision_six_cameras_pkg_prefix = FindPackageShare('arkvision_six_cameras').find('arkvision_six_cameras')
    blickfeld_qb2_ros2_driver_pkg_prefix = FindPackageShare('blickfeld_qb2_ros2_driver').find('blickfeld_qb2_ros2_driver')
    ouster_ros_pkg_prefix = FindPackageShare('ouster_ros').find('ouster_ros')
    zed_pkg_prefix = FindPackageShare('zed_wrapper').find('zed_wrapper')

    # Define the path to the launch files
    arkvision_launch_file = PythonLaunchDescriptionSource([arkvision_six_cameras_pkg_prefix, '/launch/camera_stream_launch.py'])
    blickfeld_launch_file = PythonLaunchDescriptionSource([blickfeld_qb2_ros2_driver_pkg_prefix, '/launch/blickfeld_qb2_ros2_driver.launch.py'])
    ouster_launch_file = PythonLaunchDescriptionSource([ouster_ros_pkg_prefix, '/launch/driver.launch.py'])
    zed_launch_file = PythonLaunchDescriptionSource([zed_pkg_prefix, '/launch/zed_camera.launch.py'])

    # Declare the camera_model launch argument
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='Model of the ZED camera.'
    )

    # Include the launch files with the ZED launch file having the camera_model argument passed
    return LaunchDescription([
        camera_model_arg,
        IncludeLaunchDescription(arkvision_launch_file),
        IncludeLaunchDescription(blickfeld_launch_file),
        IncludeLaunchDescription(ouster_launch_file),
        IncludeLaunchDescription(
            zed_launch_file,
            launch_arguments=[('camera_model', LaunchConfiguration('camera_model'))]  # Corrected argument passing
        )
    ])

