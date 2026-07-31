from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    kinect_node = Node(
    package='kinect_picking',
    executable='kinect_node',
)
    detection_node = Node(
    package='kinect_picking',
    executable='detection_node',
)
    depth_extractor_node = Node(
    package='kinect_picking',
    executable='depth_extractor_node',
)
    coord_transform_node = Node(
    package='kinect_picking',
    executable='coord_transform_node',
)
    voice_command_node = Node(
    package='kinect_picking',
    executable='voice_command_node',
)
    return LaunchDescription([
    kinect_node,
    detection_node,
    depth_extractor_node,
    coord_transform_node,
    voice_command_node
    ])
