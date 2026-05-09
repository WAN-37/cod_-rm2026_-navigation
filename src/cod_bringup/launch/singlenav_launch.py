import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包的共享目录
    bring_up_dir = get_package_share_directory('cod_bringup')

    rviz_config_file = os.path.join(bring_up_dir, 'rviz', 'cod_nav.rviz')

    fast_lio_config_file = 'mid360.yaml'
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=os.path.join(bring_up_dir,'params','mapper_params_async.yaml')
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',default_value=os.path.join(bring_up_dir,'params','singlenav2_params.yaml')
    )
    declare_map_yaml_file = DeclareLaunchArgument(
        'map', default_value=os.path.join(bring_up_dir, 'maps', 'shafa.yaml'),
        description='Full path to map yaml file to load')
    declare_use_realsense = DeclareLaunchArgument(
        'use_realsense', default_value='false',
        description='Launch the Intel RealSense camera driver if true')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_realsense = LaunchConfiguration('use_realsense')

    # 定义节点和包含的launch文件
    load_nodes = GroupAction(
        actions=[
            Node(
                package='cpp_lidar_filter',
                executable='lidar_filter_node',
                name='my_lidar_filter',
                output='screen',
                parameters=[{
                    'input_topic': '/livox/lidar',
                    'output_topic': '/livox/lidar_filtered',
                    'min_x': -0.5, 'max_x': 0.5,
                    'min_y': -0.5, 'max_y': 0.5,
                    'min_z': -0.8, 'max_z': 0.8,
                    'negative': True,   # 挖掉车身
                    'leaf_size': 0.05   # 降采样
                }]
            ),
            Node(
                package="cod_bringup",
                executable="livox_tf_tuner.py",
                name="base_link_to_livox_frame",
                output="screen",
                parameters=[{
                    "x": -0.202446,
                    "y": -0.089251,
                    "z": 0.419590,
                    "roll": -1.071025,
                    "pitch": 0.0,
                    "yaw": 1.789491,
                    "frame_id": "base_link",
                    "child_frame_id": "livox_frame",
                }],
            ),
            Node(
                    package="small_point_lio",
                    executable="small_point_lio_node",
                    name="small_point_lio",
                    output="screen",
                    parameters=[
                        PathJoinSubstitution(
                            [
                                FindPackageShare("small_point_lio"),
                                "config",
                                "mid360.yaml",
                            ]
                        )
                    ],
            ),

            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.05",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
            ),
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="cod_serial_ul26",
                executable="cod_serial",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
                    )
                ),
                condition=IfCondition(use_realsense),
                launch_arguments={
                    'depth_module.depth_profile': '424x240x90',    # 最高帧率
                    #'enable_depth': 'true',
                    #'enable_color': 'false',                        # 不需要彩色，节省USB带宽
                    'pointcloud.enable': 'true',
                    'pointcloud.ordered_pc': 'false',               # 无序点云，减少处理开销
                    'pointcloud.allow_no_texture_points': 'true',   # 无彩色时必须开启
                    'spatial_filter.enable': 'true',                # 空间滤波降噪
                    'temporal_filter.enable': 'true',               # 时间滤波稳定深度
                    'decimation_filter.enable': 'false',            # 不再降分辨率，已经很低
                    'publish_tf': 'true',                           # 发布内部TF链
                    'depth_module.enable_auto_exposure': 'true',
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bring_up_dir,'launch','navigation_launch.py')),
                launch_arguments={
                                  'use_sim_time': "false",
                                  'autostart': "true",
                                  'params_file': nav2_params_file,
                                  'use_composition': 'False',
                                  'use_respawn': 'False',
                                  'container_name': 'nav2_container'}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bring_up_dir,'launch','localization_launch.py')),
                launch_arguments={
                                  'use_sim_time': "false",
                                  'autostart': "true",
                                  'map': map_yaml_file,
                                  'params_file': nav2_params_file,
                                  'use_composition': 'False',
                                  'use_respawn': 'False',
                                  'container_name': 'nav2_container'}.items()
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d',rviz_config_file],
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_nav2_params_file,
        declare_map_yaml_file,
        declare_use_realsense,
        load_nodes
    ])
