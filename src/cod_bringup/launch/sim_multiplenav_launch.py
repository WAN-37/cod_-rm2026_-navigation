"""
Gazebo simulation launch for cod_bringup navigation stack.

Usage:
  # Launch Gazebo + navigation (SLAM mode, default):
  ros2 launch cod_bringup sim_multiplenav_launch.py

  # Navigation only (Gazebo already running):
  ros2 launch cod_bringup sim_multiplenav_launch.py launch_gazebo:=false

  # Localization mode with known map:
  ros2 launch cod_bringup sim_multiplenav_launch.py slam:=false map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    bring_up_dir = get_package_share_directory('cod_bringup')

    # ── Declare launch arguments ──────────────────────────────────────────
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='red_standard_robot1',
        description='Robot namespace (must match gz_world.yaml robot name)')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use Gazebo /clock')

    declare_slam = DeclareLaunchArgument(
        'slam', default_value='True',
        description='True = slam_toolbox online mapping; False = known map localization')

    declare_map = DeclareLaunchArgument(
        'map', default_value=os.path.join(bring_up_dir, 'maps', 'rmul2026.yaml'),
        description='Full path to map yaml (only used when slam:=false)')

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(bring_up_dir, 'params', 'sim_multiplenav2_params.yaml'),
        description='Nav2 parameter file for simulation')

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bring_up_dir, 'params', 'mapper_params_online_async.yaml'),
        description='slam_toolbox parameter file')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Start RViz')

    declare_launch_gazebo = DeclareLaunchArgument(
        'launch_gazebo', default_value='True',
        description='Also launch Gazebo simulator (set false if already running)')

    # ── Resolve substitutions ─────────────────────────────────────────────
    namespace       = LaunchConfiguration('namespace')
    use_sim_time    = LaunchConfiguration('use_sim_time')
    slam            = LaunchConfiguration('slam')
    map_yaml_file   = LaunchConfiguration('map')
    nav2_params     = LaunchConfiguration('nav2_params_file')
    slam_params     = LaunchConfiguration('slam_params_file')
    use_rviz        = LaunchConfiguration('use_rviz')
    launch_gazebo   = LaunchConfiguration('launch_gazebo')

    rviz_config = os.path.join(bring_up_dir, 'rviz', 'cod_nav.rviz')

    # ── Gazebo (optional) ─────────────────────────────────────────────────
    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rmu_gazebo_simulator'),
                'launch', 'bringup_sim.launch.py')),
        condition=IfCondition(launch_gazebo),
    )

    # ── Namespaced navigation group ───────────────────────────────────────
    nav_group = GroupAction(actions=[
        PushRosNamespace(namespace),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),

        # ─ odom → base_footprint TF from Gazebo ground-truth odometry ───────
        Node(
            package='fake_vel_transform',
            executable='odom_to_tf_node',
            name='odom_to_tf',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic':   'chassis_odometry_gt',
                'odom_frame':   'odom',
                'base_frame':   'base_footprint',
            }],
        ),

        # ─ LiDAR self-point filter ───────────────────────────────────────
        Node(
            package='cpp_lidar_filter',
            executable='lidar_filter_node',
            name='my_lidar_filter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic':  'livox/lidar',
                'output_topic': 'livox/lidar_filtered',
                'min_x': -0.8, 'max_x': 0.8,
                'min_y': -0.8, 'max_y': 0.8,
                'min_z': -0.8, 'max_z': 0.8,
                'negative': True,
                'leaf_size': 0.05,
            }],
        ),

        # ─ PointCloud → LaserScan (for slam_toolbox) ─────────────────────
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', 'livox/lidar'),
                ('scan',     'scan'),
            ],
            parameters=[{
                'use_sim_time':       use_sim_time,
                'target_frame':       'base_footprint',
                'transform_tolerance': 0.5,
                'min_height':  0.1,
                'max_height':  1.0,
                'angle_min':  -3.1416,
                'angle_max':   3.1416,
                'angle_increment': 0.0087,
                'scan_time':   0.3333,
                'range_min':   0.5,
                'range_max':  20.0,
                'use_inf':     True,
                'inf_epsilon': 1.0,
            }],
        ),

        # ═══════════════════ SLAM mode ════════════════════════════════════
        Node(
            condition=IfCondition(slam),
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[
                ('/map', 'map'),
                ('/map_metadata', 'map_metadata'),
            ],
            parameters=[
                slam_params,
                {'use_sim_time': use_sim_time,
                 'scan_topic': 'scan',
                 'base_frame': 'base_footprint',
                 'transform_publish_period': 0.05},  # must be >0 to publish map→odom TF
            ],
        ),

        # ═══════════════════ Localization mode ════════════════════════════
        # static map → odom (ground-truth odom ≈ world frame)
        Node(
            condition=IfCondition(PythonExpression(["not ", slam])),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bring_up_dir, 'launch', 'localization_launch.py')),
            condition=IfCondition(PythonExpression(["not ", slam])),
            launch_arguments={
                'namespace':       namespace,
                'map':             map_yaml_file,
                'use_sim_time':    use_sim_time,
                'autostart':       'true',
                'params_file':     nav2_params,
                'use_composition': 'False',
                'use_respawn':     'False',
            }.items(),
        ),

        # ═══════════════════ Nav2 navigation stack ════════════════════════
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bring_up_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'namespace':       namespace,
                'use_sim_time':    use_sim_time,
                'autostart':       'true',
                'params_file':     nav2_params,
                'use_composition': 'False',
                'use_respawn':     'False',
                'container_name':  'nav2_container',
            }.items(),
        ),

        # ─ RViz ──────────────────────────────────────────────────────────
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            remappings=[
                ('/tf',           'tf'),
                ('/tf_static',    'tf_static'),
                ('/map',          'map'),
                ('/goal_pose',    'goal_pose'),
                ('/clicked_point','clicked_point'),
                ('/initialpose',  'initialpose'),
            ],
        ),
    ])

    # ── Assemble ──────────────────────────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_slam)
    ld.add_action(declare_map)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_slam_params)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_launch_gazebo)
    ld.add_action(gazebo_bringup)
    ld.add_action(nav_group)
    return ld
