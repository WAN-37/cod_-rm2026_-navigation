# auto_save_map.launch.py
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    bringup_share_dir = Path(get_package_share_directory('cod_bringup')).resolve()
    install_dir = bringup_share_dir.parents[2]
    ws_dir = install_dir.parent
    maps_dir = ws_dir / 'src' / 'cod_bringup' / 'maps' / 'auto_save'

    def create_save_command(suffix: str) -> list:
        return [
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            f'source {ws_dir}/install/setup.bash && '
            f'mkdir -p {maps_dir} && '
            f'ros2 run nav2_map_server map_saver_cli -f {maps_dir}/auto_map_{suffix}'
        ]

    intervals = [ 30, 60, 90, 120, 150, 180, 210, 240, 270, 300]    #保存时间间隔
    for t in intervals:
        action = TimerAction(
            period=float(t),
            actions=[ExecuteProcess(cmd=create_save_command("$(date +%H%M%S)"), output='screen')]
        )
        ld.add_action(action)

    return ld