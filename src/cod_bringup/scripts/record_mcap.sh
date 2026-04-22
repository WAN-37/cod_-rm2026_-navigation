#!/usr/bin/env bash
set -euo pipefail

# ============================================================
#  MCAP 录包脚本
#  用法: ./record_mcap.sh [bag_name]
#  停止: Ctrl+C
#
#  不想录的话题 → 在下面列表里行首加 # 注释掉即可
# ============================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd -- "${PACKAGE_ROOT}/../.." && pwd)"

# --- 环境检测 ---
if ! command -v ros2 >/dev/null 2>&1; then
  if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
    . "${WORKSPACE_ROOT}/install/setup.bash"
  fi
fi
if ! command -v ros2 >/dev/null 2>&1; then
  if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
    . "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  fi
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "未找到 ros2，请先 source 你的 ROS2 环境。" >&2
  exit 1
fi

# --- 输出路径 ---
BAG_NAME="${1:-bag_$(date +%Y%m%d_%H%M%S)}"
BAG_ROOT="${BAG_ROOT:-${WORKSPACE_ROOT}/bags}"
mkdir -p "$BAG_ROOT"
BAG_DIR="${BAG_ROOT}/${BAG_NAME}"

# ============================================================
#  录制话题列表 —— 不想录的话题直接在行首加 # 注释掉
# ============================================================
TOPICS=(
  # ---------- TF / 时钟 ----------
  /tf
  /tf_static
  # /clock

  # ---------- 雷达原始数据 ----------
  /livox/lidar
  /livox/lidar_filtered
  /livox/imu

  # ---------- 里程计 / 定位 ----------
  /odom
  /Odometry
  /lidar_odom_raw
  /cloud_registered
  /pose

  # ---------- SLAM ----------
  /scan
  /map
  /map_metadata
  /slam_toolbox/feedback
  /slam_toolbox/graph_visualization
  /slam_toolbox/scan_visualization
  /slam_toolbox/update

  # ---------- 导航规划 ----------
  /cmd_vel
  /cmd_vel_nav
  /cmd_vel_teleop
  /aft_cmd_vel
  /goal_pose
  /initialpose
  /plan
  /plan_smoothed
  /unsmoothed_plan
  /local_plan
  /transformed_global_plan
  /trajectories
  /lookahead_point
  /speed_limit
  /preempt_teleop
  /clicked_point

  # ---------- Costmap ----------
  /global_costmap/costmap
  /global_costmap/costmap_updates
  /global_costmap/footprint
  /global_costmap/published_footprint
  /local_costmap/costmap
  /local_costmap/costmap_updates
  /local_costmap/footprint
  /local_costmap/published_footprint
  /local_costmap/voxel_grid

  # ---------- 调试 / 可视化 ----------
  /crop_box_marker
  /back_up_free_space_markers
  /behavior_tree_log
  /diagnostics
  /mcu_data

  # ---------- 以下默认不录（太大或太杂），需要时去掉注释 ----------
  # /global_costmap/costmap_raw
  # /local_costmap/costmap_raw
  # /parameter_events
  # /rosout
  # /bond
  # /behavior_server/transition_event
  # /bt_navigator/transition_event
  # /controller_server/transition_event
  # /planner_server/transition_event
  # /smoother_server/transition_event
  # /velocity_smoother/transition_event
  # /waypoint_follower/transition_event
  # /global_costmap/global_costmap/transition_event
  # /local_costmap/local_costmap/transition_event
)

# ============================================================
#  打印录制信息
# ============================================================
echo "========================================"
echo "  MCAP 录包"
echo "========================================"
echo "输出目录: ${BAG_DIR}"
echo "话题数量: ${#TOPICS[@]}"
echo "----------------------------------------"
for t in "${TOPICS[@]}"; do
  echo "  ${t}"
done
echo "----------------------------------------"
echo "按 Ctrl+C 停止录制"
echo "========================================"

# ============================================================
#  开始录制
# ============================================================
set +e
ros2 bag record -s mcap -o "$BAG_DIR" "${TOPICS[@]}"
STATUS=$?
set -e

# --- 录制结束，打印结果 ---
shopt -s nullglob
MCAP_FILES=("${BAG_DIR}"/*.mcap)
shopt -u nullglob

if [ -d "$BAG_DIR" ]; then
  echo "录制目录: ${BAG_DIR}"
fi
if [ "${#MCAP_FILES[@]}" -gt 0 ]; then
  echo "MCAP 文件: ${MCAP_FILES[0]}"
  echo "Foxglove 里直接 Open local file 打开这个 .mcap 文件即可回放。"
fi

if [ "$STATUS" -eq 130 ]; then
  exit 0
fi
exit "$STATUS"
