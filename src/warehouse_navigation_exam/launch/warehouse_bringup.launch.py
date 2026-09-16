#!/usr/bin/env python3

"""Start the warehouse simulation with mapping or localization and RViz.

Mapping is enabled by default and starts ``slam_toolbox`` while keeping AMCL and
the map server stopped. Set ``mapping:=false`` to use localization instead.
The navigation stack is optional; use ``navigation:=true`` when it is required.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('warehouse_navigation_exam')
    warehouse_share = get_package_share_directory('aws_robomaker_small_warehouse_world')
    gazebo_share = get_package_share_directory('gazebo_ros')
    nav2_share = get_package_share_directory('nav2_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    mapping = LaunchConfiguration('mapping')
    navigation = LaunchConfiguration('navigation')
    rviz = LaunchConfiguration('rviz')
    autostart = LaunchConfiguration('autostart')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    # Gazebo Classic needs its system material scripts (Gazebo/White,
    # Gazebo/shadow_caster, ...).  The MVS camera SDK also prepends its own
    # libusb to LD_LIBRARY_PATH in the user's shell, which can destabilize
    # gzclient.  Keep those changes local to this simulation launch.
    library_path = os.pathsep.join(
        entry for entry in os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep)
        if entry and not entry.startswith('/opt/MVS/lib/')
    )

    default_world = os.path.join(
        warehouse_share, 'worlds', 'no_roof_small_warehouse',
        'no_roof_small_warehouse.world')
    robot_sdf = os.path.join(
        package_share, 'models', 'turtlebot3_waffle_exam', 'model.sdf')
    robot_urdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'urdf',
        'turtlebot3_waffle.urdf')

    with open(robot_urdf, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world,
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=UnlessCondition(headless),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_warehouse_robot',
        output='screen',
        arguments=[
            '-entity', 'turtlebot3_waffle_exam',
            '-file', robot_sdf,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        condition=UnlessCondition(mapping),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
    )

    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')),
        condition=IfCondition(mapping),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(
                package_share, 'config', 'slam_mapping.yaml'),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        condition=IfCondition(navigation),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'rviz_launch.py')),
        condition=IfCondition(rviz),
        launch_arguments={
            'use_namespace': 'false',
            'rviz_config': rviz_config,
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value='/usr/share/gazebo-11',
        ),
        SetEnvironmentVariable(
            name='OGRE_RESOURCE_PATH',
            value='/usr/lib/x86_64-linux-gnu/OGRE-1.9.0',
        ),
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=library_path,
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo simulation clock.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Start Gazebo without its GUI.'),
        DeclareLaunchArgument(
            'mapping', default_value='false',
            description=(
                'Start slam_toolbox mapping and disable AMCL/map_server. '
                'Set false for localization/navigation mode.')),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Gazebo world file.'),
        DeclareLaunchArgument(
            'navigation', default_value='true',
            description='Start the Nav2 planner, controller and BT navigator.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with the Nav2 default configuration.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically activate lifecycle nodes.'),
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Initial robot x position in Gazebo world coordinates.'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Initial robot y position in Gazebo world coordinates.'),
        DeclareLaunchArgument(
            'z_pose', default_value='0.01',
            description='Initial robot z position in Gazebo world coordinates.'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(package_share, 'maps', 'second.yaml'),
            description='Map YAML file.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'amcl_localization.yaml'),
            description='AMCL/Nav2 parameter file.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz'),
            description='RViz configuration file.'),
        gazebo_launch,
        gazebo_client,
        spawn_robot,
        robot_state_publisher,
        localization,
        slam_mapping,
        navigation_launch,
        rviz_launch,
    ])
