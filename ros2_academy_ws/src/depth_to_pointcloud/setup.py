from setuptools import setup, find_packages

package_name = 'depth_to_pointcloud'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/depth_to_pointcloud.launch.py', 'launch/complete_simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='malek',
    maintainer_email='malek.wahidi@inmind.ai',
    description='ROS2 package for converting depth images to point clouds',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_to_pointcloud_node = depth_to_pointcloud.depth_to_pointcloud_node:main',
        ],
    },
)
