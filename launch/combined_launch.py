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
    novatel_pkg_prefix = FindPackageShare('novatel_oem7_driver').find('novatel_oem7_driver')
    
    # Define the path to the launch files
    arkvision_launch_file = PythonLaunchDescriptionSource([arkvision_six_cameras_pkg_prefix, '/launch/camera_stream_launch.py'])
    blickfeld_launch_file = PythonLaunchDescriptionSource([blickfeld_qb2_ros2_driver_pkg_prefix, '/launch/blickfeld_qb2_ros2_driver.launch.py'])
    ouster_launch_file = PythonLaunchDescriptionSource([ouster_ros_pkg_prefix, '/launch/driver.launch.py'])
    zed_launch_file = PythonLaunchDescriptionSource([zed_pkg_prefix, '/launch/zed_camera.launch.py'])
    novatel_launch_file = PythonLaunchDescriptionSource([novatel_pkg_prefix, '/launch/oem7_net.launch.py'])

    # Declare the camera_model launch argument
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='Model of the ZED camera.'
    )
    
    # Declare the novatel_ip launch argument
    novatel_ip_arg = DeclareLaunchArgument(
        'oem7_ip_addr',
        default_value='192.168.26.100',
        description='IP address of the novatel OEM7'
    )


    return LaunchDescription([
        camera_model_arg,
        novatel_ip_arg,
        IncludeLaunchDescription(arkvision_launch_file),
        IncludeLaunchDescription(blickfeld_launch_file),
        IncludeLaunchDescription(ouster_launch_file),
        IncludeLaunchDescription(
            zed_launch_file,
            launch_arguments=[('camera_model', LaunchConfiguration('camera_model'))]
        ),
        IncludeLaunchDescription(
            novatel_launch_file,
            launch_arguments=[('oem7_ip_addr', LaunchConfiguration('oem7_ip_addr'))]
        )
    ])

