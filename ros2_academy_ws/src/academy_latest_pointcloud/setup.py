from setuptools import setup, find_packages

package_name = 'academy_latest_pointcloud'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/latest_pointcloud.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cheayto',
    maintainer_email='cheaytox27@gmail.com',
    description='Service that returns the latest PointCloud2 built from depth + camera info.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'depth_to_pointcloud_service = academy_latest_pointcloud.depth_to_pointcloud_service:main',
        ],
    },
)