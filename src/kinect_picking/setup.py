from setuptools import find_packages, setup

package_name = 'kinect_picking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
   data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),

    ('share/' + package_name, ['package.xml']),

    # ✅ ADD THIS (launch files)
    ('share/' + package_name + '/launch',
        ['launch/kinect_picking.launch.py']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zikho',
    maintainer_email='zikho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'kinect_node = kinect_picking.kinect_node:main',
        'detection_node = kinect_picking.detection_node:main',
        'depth_extractor_node = kinect_picking.depth_extractor_node:main',
        'coord_transform_node = kinect_picking.coord_transform_node:main',
        'robot_commander_node = kinect_picking.robot_commander_node:main',
        'voice_command_node = kinect_picking.voice_command_node:main',
        'all_nodes = kinect_picking.all_nodes:main',
    ],
},
)