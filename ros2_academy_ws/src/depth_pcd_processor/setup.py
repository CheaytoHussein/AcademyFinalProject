from setuptools import find_packages, setup

package_name = 'depth_pcd_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'depth_pcd_processor/launch/pcd_processor.launch.py',
            'depth_pcd_processor/launch/complete_simulation.launch.py'
        ]),
    ],
    install_requires=['setuptools', 'open3d', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='malek',
    maintainer_email='malek.wahidi@inmind.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcd_processor_node = depth_pcd_processor.pcd_processor_node:main',
        ],
    },
)
