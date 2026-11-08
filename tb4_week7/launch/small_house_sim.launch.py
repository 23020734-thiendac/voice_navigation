"""Launch the AWS RoboMaker Small House without a charging dock."""

import os
import shutil
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _default_paths():
    package_share = Path(get_package_share_directory('tb4_week7'))
    return package_share / 'simulation'


def _nvidia_driver_is_usable():
    nvidia_smi = shutil.which('nvidia-smi')
    if not nvidia_smi:
        return False
    try:
        subprocess.run(
            [nvidia_smi, '--query-gpu=name', '--format=csv,noheader'],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=2.0)
    except (OSError, subprocess.SubprocessError):
        return False
    return True


def generate_launch_description():
    world_root = _default_paths()
    bringup_share = get_package_share_directory('turtlebot4_ignition_bringup')
    turtlebot_description_share = get_package_share_directory('turtlebot4_description')
    create_common_share = get_package_share_directory('irobot_create_common_bringup')
    create_ignition_share = get_package_share_directory('irobot_create_ignition_bringup')
    gui_plugins_share = get_package_share_directory('turtlebot4_ignition_gui_plugins')
    create_plugins_share = get_package_share_directory('irobot_create_ignition_plugins')
    ros_gz_gazebo_share = get_package_share_directory('ros_gz_sim')

    if _nvidia_driver_is_usable():
        render_environment = [
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        ]
    else:
        render_environment = [
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
            SetEnvironmentVariable('MESA_LOADER_DRIVER_OVERRIDE', 'llvmpipe'),
        ]
    render_environment.append(SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'))

    world_file = LaunchConfiguration('world_file')
    models_path = LaunchConfiguration('models_path')
    world_path = LaunchConfiguration('world_path')
    world_name = LaunchConfiguration('bridge_world_name')
    model = LaunchConfiguration('model')

    resource_paths = [
        models_path, os.pathsep, world_path, os.pathsep, str(world_root),
        os.pathsep, os.path.join(bringup_share, 'worlds'), os.pathsep,
        os.path.join(create_ignition_share, 'worlds'), os.pathsep,
        str(Path(turtlebot_description_share).parent.resolve()), os.pathsep,
        str(Path(get_package_share_directory('irobot_create_description')).parent.resolve()),
    ]
    gui_plugin_paths = [
        os.path.join(gui_plugins_share, 'lib'), os.pathsep,
        os.path.join(create_plugins_share, 'lib'),
    ]

    ignition_launch = PathJoinSubstitution([
        ros_gz_gazebo_share, 'launch', 'gz_sim.launch.py'])
    gui_config = PathJoinSubstitution([
        bringup_share, 'gui', model, 'gui.config'])
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments={
            'gz_args': [world_file, ' -r -v 4 --gui-config ', gui_config],
        }.items())

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])

    turtlebot_xacro = PathJoinSubstitution([
        turtlebot_description_share, 'urdf', model, 'turtlebot4.urdf.xacro'])
    robot_bridge_launch = PathJoinSubstitution([
        bringup_share, 'launch', 'ros_ign_bridge.launch.py'])
    turtlebot_nodes_launch = PathJoinSubstitution([
        bringup_share, 'launch', 'turtlebot4_nodes.launch.py'])
    create_nodes_launch = PathJoinSubstitution([
        create_common_share, 'launch', 'create3_nodes.launch.py'])
    create_ignition_nodes_launch = PathJoinSubstitution([
        create_ignition_share, 'launch', 'create3_ignition_nodes.launch.py'])

    robot_name = 'turtlebot4'
    # The system turtlebot4_spawn launch also spawns standard_dock and starts
    # controllers before the world is ready. This package uses an explicit
    # world-aware spawn and intentionally does not create any dock model.
    robot_group = GroupAction([
        # Keep the ROS namespace empty. The Gazebo model is still named turtlebot4.
        # The standard Create 3 control launch expects controller_manager at root.
        # This keeps controller, odom, robot_description and TF in one tree.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': Command([
                    'xacro', ' ', turtlebot_xacro, ' ',
                    'gazebo:=ignition']),
            }],
            remappings=[
                ('joint_states', '/joint_states'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]),
        # The simulated Create 3 controller does not publish wheel-drop
        # joints. Publish their zero pose so robot_state_publisher can
        # complete the TF chain to left_wheel and right_wheel.
        Node(
            package='tb4_week7',
            executable='wheel_drop_state_publisher',
            name='wheel_drop_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[
                '-world', world_name,
                '-name', robot_name,
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
                '-topic', '/robot_description',
            ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_bridge_launch]),
            launch_arguments={
                'model': model,
                'robot_name': robot_name,
                'dock_name': 'standard_dock',
                'world': world_name,
                'use_sim_time': 'true',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot_nodes_launch]),
            launch_arguments={'model': model}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create_nodes_launch]),
            launch_arguments={'gazebo': 'ignition', 'namespace': LaunchConfiguration('namespace')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create_ignition_nodes_launch]),
            launch_arguments={
                'robot_name': robot_name,
                'dock_name': 'standard_dock',
            }.items()),
        Node(
            name='rplidar_stf', package='tf2_ros',
            executable='static_transform_publisher', output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'rplidar_link', 'turtlebot4/rplidar_link/rplidar'],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        Node(
            name='camera_stf', package='tf2_ros',
            executable='static_transform_publisher', output='screen',
            arguments=[
                '0', '0', '0', '1.5707', '-1.5707', '0',
                'oakd_rgb_camera_optical_frame',
                'turtlebot4/oakd_rgb_camera_frame/rgbd_camera'],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
    ])

    delayed_robot_spawn = TimerAction(
        period=LaunchConfiguration('spawn_delay_sec'),
        actions=[robot_group])


    return LaunchDescription(render_environment + [
        DeclareLaunchArgument('ros_domain_id', default_value='68'),
        DeclareLaunchArgument(
            'world_file', default_value=str(world_root / 'worlds' / 'small_house.world')),
        DeclareLaunchArgument(
            'models_path', default_value=str(world_root / 'models')),
        DeclareLaunchArgument(
            'world_path', default_value=str(world_root / 'worlds')),
        DeclareLaunchArgument(
            'bridge_world_name', default_value='warehouse'),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS namespace for the Create 3 controller; empty keeps odom and base_link at root.'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'spawn_delay_sec', default_value='12.0',
            description='Wait for Gazebo world before creating TurtleBot 4.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_paths),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_paths),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', gui_plugin_paths),
        ignition,
        clock_bridge,
        delayed_robot_spawn,
        LogInfo(msg='Dock sac khong duoc spawn trong Small House.'),
    ])
