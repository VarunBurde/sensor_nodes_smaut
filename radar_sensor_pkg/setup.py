from setuptools import find_packages, setup

package_name = 'radar_sensor_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/radar_sensor_launch.py']),
        ('share/' + package_name + '/config', ['config/radar_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Varun',
    maintainer_email='varun@example.com',
    description='Radar sensor package for ROS2 - Point cloud and telemetry data publishing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_pointcloud_publisher = radar_sensor_pkg.radar_pointcloud_publisher:main',
            'robot_telemetry_publisher = radar_sensor_pkg.robot_telemetry_publisher:main',
        ],
    },
)
