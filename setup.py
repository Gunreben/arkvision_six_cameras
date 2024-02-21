from setuptools import find_packages, setup

package_name = 'arkvision_six_cameras'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lorenz Gunreben',
    maintainer_email='gunreben@wifa.uni-leipzig.de',
    description='ROS Publisher for 6 Arkvision Cameras',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
		'console_scripts': [
		'camera_stream_node = arkvision_six_cameras.camera_stream_node:main',
		'camera_stream_launch = arkvision_six_cameras.camera_stream_launch:main',
		],
    },
)
