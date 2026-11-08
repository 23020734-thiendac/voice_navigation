"""Small House semantic map and voice-controlled Nav2 navigation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ReplaceString


def generate_launch_description():
    week7_share = get_package_share_directory('tb4_week7')
    week7_share = get_package_share_directory('tb4_week7')
    navigation_share = get_package_share_directory('turtlebot4_navigation')
    map_file = Path(week7_share) / 'maps' / 'small_house_slam.yaml'
    zones_file = Path(week7_share) / 'config' / 'zones.yaml'
    sim_launch = PathJoinSubstitution([
        week7_share, 'launch', 'small_house_sim.launch.py'])
    localization_launch = PathJoinSubstitution([
        navigation_share, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution([
        navigation_share, 'launch', 'nav2.launch.py'])
    rviz_config = PathJoinSubstitution([
        week7_share, 'rviz', 'semantic_navigation.rviz'])
    nav2_params = ReplaceString(
        source_file=PathJoinSubstitution([
            navigation_share, 'config', 'nav2.yaml']),
        replacements={
            'max_vel_x: 0.26': 'max_vel_x: 0.35',
            'max_speed_xy: 0.26': 'max_speed_xy: 0.35',
            'max_vel_theta: 1.0': 'max_vel_theta: 1.20',
        })

    args = [
        DeclareLaunchArgument('ros_domain_id', default_value='68'),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('show_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('voice_backend', default_value='auto'),
        DeclareLaunchArgument('voice_model_path', default_value=''),
        DeclareLaunchArgument('voice_device', default_value='-1'),
        DeclareLaunchArgument('command_hold_sec', default_value='2.5'),
        # Small House loads many meshes and sensors. The robot must be created first
        # and its ros2_control plugin must publish /odom before localization
        # and Nav2 are started; otherwise Nav2 reports an unconnected TF tree.
        DeclareLaunchArgument('simulation_delay_sec', default_value='20.0'),
        DeclareLaunchArgument('localization_delay_sec', default_value='38.0'),
        DeclareLaunchArgument('nav2_delay_sec', default_value='48.0'),
        DeclareLaunchArgument('initial_pose_delay_sec', default_value='54.0'),
        DeclareLaunchArgument('semantic_delay_sec', default_value='54.0'),
        DeclareLaunchArgument('voice_delay_sec', default_value='56.0'),
        DeclareLaunchArgument('rviz_delay_sec', default_value='75.0'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'ros_domain_id': LaunchConfiguration('ros_domain_id'),
            'model': LaunchConfiguration('model'),
            'rviz': 'false',
            'spawn_delay_sec': LaunchConfiguration('simulation_delay_sec'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items())

    localization = TimerAction(
        period=LaunchConfiguration('localization_delay_sec'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map': str(map_file),
                'use_sim_time': 'true',
            }.items())])

    navigation = TimerAction(
        period=LaunchConfiguration('nav2_delay_sec'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
            }.items())])

    initial_pose = TimerAction(
        period=LaunchConfiguration('initial_pose_delay_sec'),
        actions=[Node(
            package='tb4_week7',
            executable='initial_pose_publisher',
            name='week7_initial_pose_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'x': ParameterValue(LaunchConfiguration('x'), value_type=float),
                'y': ParameterValue(LaunchConfiguration('y'), value_type=float),
                'yaw': ParameterValue(LaunchConfiguration('yaw'), value_type=float),
            }])])

    semantic = TimerAction(
        period=LaunchConfiguration('semantic_delay_sec'),
        actions=[Node(
            package='tb4_week7',
            executable='map_segmentation_node',
            name='semantic_map_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'map_yaml': str(map_file),
                'zones_yaml': str(zones_file),
            }])])

    voice = TimerAction(
        period=LaunchConfiguration('voice_delay_sec'),
        actions=[
            Node(
                package='tb4_week7',
                executable='voice_recognition_node',
                name='voice_recognition_node',
                additional_env={'PYTHONNOUSERSITE': ''},
                output='screen',
                parameters=[{
                    'backend': LaunchConfiguration('voice_backend'),
                    'model_path': LaunchConfiguration('voice_model_path'),
                    'device': ParameterValue(
                        LaunchConfiguration('voice_device'), value_type=int),
                    'command_hold_sec': ParameterValue(
                        LaunchConfiguration('command_hold_sec'), value_type=float),
                }]),
            Node(
                package='tb4_week7',
                executable='voice_navigation_node',
                name='voice_navigation_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'zones_yaml': str(zones_file),
                }]),
        ])

    rviz = TimerAction(
        period=LaunchConfiguration('rviz_delay_sec'),
        actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2_week7', output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('show_rviz')))])

    return LaunchDescription(args + [simulation, localization, navigation,
                                     initial_pose, semantic, voice, rviz])
