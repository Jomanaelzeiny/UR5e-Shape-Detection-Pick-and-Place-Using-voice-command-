from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    use_fake = LaunchConfiguration('use_fake_hardware').perform(context)
    is_simulation = use_fake.lower() == 'true'

    # ── Kinect Nodes (start first) ───────────────────────────────────────
    kinect_node = Node(
        package='kinect_picking',
        executable='kinect_node',
        output='screen'
    )

    detection_node = Node(
        package='kinect_picking',
        executable='detection_node',
        output='screen'
    )

    depth_extractor_node = Node(
        package='kinect_picking',
        executable='depth_extractor_node',
        output='screen'
    )

    coord_transform_node = Node(
        package='kinect_picking',
        executable='coord_transform_node',
        output='screen'
    )

    voice_command_node = Node(
        package='kinect_picking',
        executable='voice_command_node',
        output='screen'
    )

    kinect_nodes = [
        kinect_node,
        detection_node,
        depth_extractor_node,
        coord_transform_node,
        voice_command_node,
    ]

    # ── UR Driver (after kinect is up) ───────────────────────────────────
    ur_driver = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py',
                    'ur_type:=ur5e',
                    'robot_ip:=192.168.1.102',
                    f'use_fake_hardware:={use_fake}'
                ],
                output='screen'
            )
        ]
    )

    # ── MoveIt2 + RViz ──────────────────────────────────────────────────
    ur_moveit = TimerAction(
        period=11.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'ur_moveit_config', 'ur_moveit.launch.py',
                    'ur_type:=ur5e',
                    'name:=ur5e',
                    'launch_rviz:=true'
                ],
                output='screen'
            )
        ]
    )

    # ── IK Commander (simulation) ────────────────────────────────────────
    ik_commander = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ur5e_grip_run_cpp', 'ik_commander',
                    '--ros-args', '--params-file',
                    '/home/jomana/ur_driver/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml'
                ],
                output='screen'
            )
        ]
    )

    # ── Gripper Adapter (real hardware only) ────────────────────────────
    gripper_adapter = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'robotiq_2f_urcap_adapter', 'robotiq_2f_adapter_node.py',
                    '--ros-args', '-p', 'robot_ip:=192.168.1.102'
                ],
                output='screen'
            )
        ]
    )

    # ── IK Commander (real hardware) ────────────────────────────────────
    ik_commander_real = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ur5e_grip_run_cpp', 'ik_commander',
                    '--ros-args', '--params-file',
                    '/home/jomana/ur_driver/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml'
                ],
                output='screen'
            )
        ]
    )

    if is_simulation:
        print("\n[INFO] Simulation mode  →  gripper adapter will NOT be launched\n")
        return kinect_nodes + [ur_driver, ur_moveit, ik_commander]
    else:
        print("\n[INFO] Real hardware mode  →  all nodes including gripper will be launched\n")
        return kinect_nodes + [ur_driver, ur_moveit, gripper_adapter, ik_commander_real]


def generate_launch_description():

    use_fake_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='true = simulation (no gripper), false = real hardware (gripper included)'
    )

    return LaunchDescription([
        use_fake_arg,
        OpaqueFunction(function=launch_setup)
    ])