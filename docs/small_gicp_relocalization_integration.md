# 将 `small_gicp_relocalization` 接入 `cod_-rm2026_-navigation` 的方案

## 目标

当前 `cod_-rm2026_-navigation` 的实车导航链路没有真正的全局重定位：

- `small_point_lio` 发布局部里程计 `odom -> base_link` 和 `/Odometry`。
- Nav2 使用 `map` 作为全局规划坐标系，使用 `odom` 作为局部控制坐标系。
- `singlenav_launch.py` 里目前靠静态 `map -> odom` 补齐 TF 链；`multiplenav_launch.py` 则主要跑 `slam_toolbox`。

参考 `pb2025_sentry_nav_new/small_gicp_relocalization` 后，推荐接入一个 3D 点云重定位模块，让它持续估计并发布动态 `map -> odom`，从而把 LIO 的局部 `odom` 系对齐到先验 `map` 系。

最终目标 TF 链为：

```text
map --动态重定位--> odom --small_point_lio--> base_link --fake_vel_transform--> base_link_fake
                                            \--static--> livox_frame
```

Nav2 继续使用现有参数：

- 全局坐标系：`map`
- 局部坐标系：`odom`
- 机器人底盘坐标系：`base_link_fake`

## 参考工程的重定位链路

参考包路径：

```text
/root/ros_ws/pb2025_sentry_nav_new/small_gicp_relocalization
```

核心文件：

```text
small_gicp_relocalization/src/small_gicp_relocalization.cpp
small_gicp_relocalization/include/small_gicp_relocalization/small_gicp_relocalization.hpp
pb2025_nav_bringup/launch/localization_launch.py
pb2025_nav_bringup/config/reality/nav2_params.yaml
```

`small_gicp_relocalization` 做的事情：

- 读取先验 3D PCD 点云地图：`prior_pcd_file`
- 订阅当前局部注册点云：`registered_scan`
- 订阅 RViz/Nav2 初始位姿：`initialpose`
- 每 500 ms 用 small_gicp 做一次当前点云与先验地图的 GICP 配准
- 以 20 Hz 发布 `map -> odom`

输入输出关系：

```text
prior_pcd_file + registered_scan + initialpose(optional)
                 |
                 v
small_gicp_relocalization
                 |
                 v
map -> odom TF
```

关键参数参考：

```yaml
small_gicp_relocalization:
  ros__parameters:
    use_sim_time: false
    num_threads: 4
    num_neighbors: 20
    global_leaf_size: 0.15
    registered_leaf_size: 0.05
    max_dist_sq: 3.0
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    robot_base_frame: "base_link"
    lidar_frame: "livox_frame"
```

## `cod_-rm2026_-navigation` 当前可复用的接口

### 1. LIO 输出

当前工程已有：

```text
/root/ros_ws/cod_-rm2026_-navigation/src/small_point_lio
```

`small_point_lio_node.cpp` 发布：

- `/Odometry`
  - 类型：`nav_msgs/msg/Odometry`
  - frame：`odom`
  - child frame：`base_link`
- `/lidar_odom_raw`
  - 类型：`nav_msgs/msg/Odometry`
  - frame：`lidar_odom`
  - child frame：`livox_frame`
- `/cloud_registered`
  - 类型：`sensor_msgs/msg/PointCloud2`
  - frame：`odom`

其中 `/cloud_registered` 已经非常接近参考工程里的 `registered_scan`，可以直接 remap：

```text
registered_scan <- /cloud_registered
```

不一定需要照搬 `pb2025_sentry_nav_new/loam_interface`，因为 `cod` 里的 `small_point_lio` 已经主动发布了 `odom` frame 下的 `/cloud_registered`。

### 2. TF 外参

当前 `cod_bringup` 里通过 `livox_tf_tuner.py` 发布静态：

```text
base_link -> livox_frame
```

实车参数目前在 `multiplenav_launch.py` / `singlenav_launch.py` 中为：

```text
x=-0.202446, y=-0.089251, z=0.419590
roll=-1.071025, pitch=0.0, yaw=1.789491
```

重定位节点启动前必须确保这条静态 TF 已经可查，否则它加载 PCD 时会一直等待或报 TF lookup failed。

### 3. Nav2 地图服务

当前已有：

```text
src/cod_bringup/launch/localization_launch.py
src/cod_bringup/params/singlenav2_params.yaml
src/cod_bringup/params/multiplenav2_params.yaml
```

`localization_launch.py` 现在只启动 `map_server` 和 `lifecycle_manager_localization`，没有启动真正的定位节点。接入 small_gicp 后，建议把 `small_gicp_relocalization` 放在这个 launch 里。

## 推荐接入方案

### 总体思路

推荐分成 4 步：

1. 把 `small_gicp_relocalization` 包引入 `cod_-rm2026_-navigation/src/`
2. 准备与当前 LIO `odom` 坐标系一致的 3D PCD 先验地图
3. 在 `cod_bringup` 参数文件中加入 `small_gicp_relocalization` 参数
4. 修改 launch，让非 SLAM 导航模式启动 `map_server + small_gicp_relocalization + Nav2`，并取消静态 `map -> odom`

## Step 1：引入 `small_gicp_relocalization` 包

建议直接复制参考包：

```text
from: /root/ros_ws/pb2025_sentry_nav_new/small_gicp_relocalization
to:   /root/ros_ws/cod_-rm2026_-navigation/src/small_gicp_relocalization
```

当前机器上已存在 small_gicp 系统依赖：

```text
/usr/local/include/small_gicp
/usr/local/lib/libsmall_gicp.so
/usr/local/lib/cmake/small_gicp
```

如果换机器编译，需要先确认 small_gicp 已安装，否则会出现类似找不到：

```text
small_gicp/pcl/pcl_registration.hpp
small_gicp/util/downsampling_omp.hpp
```

还建议在 `src/cod_bringup/package.xml` 增加运行依赖：

```xml
<exec_depend>small_gicp_relocalization</exec_depend>
```

这样 bringup 包和重定位包的依赖关系更明确。

## Step 2：准备 3D PCD 先验地图

`small_gicp_relocalization` 使用的是 3D PCD 地图，不是 Nav2 的 2D 栅格地图。

当前 `small_point_lio` 支持保存 PCD：

- 参数文件：`src/small_point_lio/config/mid360.yaml`
- 参数：`save_pcd`
- 服务：通常是 `/map_save`；如果给节点加了 namespace，则是 `/<namespace>/map_save`
- 默认输出：`small_point_lio` 包源码目录下的 `pcd/scan.pcd`

建图建议流程：

1. 关闭重定位，只运行 LIO / SLAM 建图流程。
2. 将 `mid360.yaml` 里的 `save_pcd` 临时改为 `true`，或在 launch 中覆盖该参数。
3. 手动遥控机器人覆盖比赛场地。
4. 调用保存服务：

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

5. 将生成的 `scan.pcd` 复制到 `cod_bringup` 下，例如：

```text
src/cod_bringup/pcd/rmul2026.pcd
```

注意：

- 推荐第一版直接使用 `small_point_lio` 的 `map_save` 生成 PCD。
- `small_point_lio` 保存的是内部 `pointcloud_odom_frame`，而发布 `/cloud_registered` 时又套了一次 `base_link <- livox_frame` 外参；参考工程的 `small_gicp_relocalization` 在加载 PCD 时也会查询 `base_frame <- lidar_frame` 并对 PCD 做同样变换，所以这两者是配套的。
- 如果你不是用 `small_point_lio` 的 `map_save`，而是直接把 `/cloud_registered` 录包后转成 PCD，那么这个 PCD 已经是发布后的点云坐标；此时照搬参考节点会在加载时再套一次外参，可能导致地图和实时点云错位。遇到这种做法，需要修改重定位节点的 PCD 加载变换逻辑，或明确让加载变换为 identity。
- 不要混用其他外参、其他机器人、其他初始朝向下生成的 PCD。
- 如果重新调整 `base_link -> livox_frame` 外参，最好重新生成 PCD 或至少重新验证重定位效果。

## Step 3：在 Nav2 参数文件中加入重定位参数

建议先在实车使用的参数文件加入，例如：

```text
src/cod_bringup/params/singlenav2_params.yaml
src/cod_bringup/params/multiplenav2_params.yaml
```

追加：

```yaml
small_gicp_relocalization:
  ros__parameters:
    use_sim_time: false
    num_threads: 4
    num_neighbors: 20
    global_leaf_size: 0.15
    registered_leaf_size: 0.05
    max_dist_sq: 3.0
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    robot_base_frame: "base_link"
    lidar_frame: "livox_frame"
```

参数解释：

- `map_frame`
  - 全局地图坐标系，和 Nav2 global frame 保持一致。
- `odom_frame`
  - LIO 局部里程计坐标系，当前是 `odom`。
- `base_frame`
  - 用于加载 PCD 时做点云坐标修正，当前应使用 `base_link`；它会和 `lidar_frame` 一起决定加载 PCD 时套用的 `base_link <- livox_frame` 变换。
- `robot_base_frame`
  - 处理 `/initialpose` 时使用的机器人本体坐标系，当前建议用 `base_link`，不要用 Nav2 的 `base_link_fake`。
- `lidar_frame`
  - 当前 Livox 坐标系，使用 `livox_frame`。
- `global_leaf_size`
  - 先验地图降采样体素大小。
- `registered_leaf_size`
  - 当前扫描降采样体素大小。
- `max_dist_sq`
  - GICP 对应点最大平方距离；如果场地结构稀疏或初始误差大，可以适当增大。

## Step 4：修改 `localization_launch.py`

目标是在 `cod_bringup/launch/localization_launch.py` 增加：

- `prior_pcd_file` launch argument
- `small_gicp_relocalization` 节点
- `registered_scan` remap 到 `/cloud_registered`

推荐节点形式：

```python
Node(
    package='small_gicp_relocalization',
    executable='small_gicp_relocalization_node',
    name='small_gicp_relocalization',
    output='screen',
    respawn=use_respawn,
    respawn_delay=2.0,
    parameters=[configured_params, {'prior_pcd_file': prior_pcd_file}],
    remappings=remappings + [('registered_scan', '/cloud_registered')],
    arguments=['--ros-args', '--log-level', log_level],
)
```

`lifecycle_nodes` 仍然只需要包含：

```python
lifecycle_nodes = ['map_server']
```

原因是 `small_gicp_relocalization` 不是 lifecycle node，不要放进 `nav2_lifecycle_manager` 的 `node_names`。

如果保留 composition 模式，也需要在 `LoadComposableNodes` 中加入：

```python
ComposableNode(
    package='small_gicp_relocalization',
    plugin='small_gicp_relocalization::SmallGicpRelocalizationNode',
    name='small_gicp_relocalization',
    parameters=[configured_params, {'prior_pcd_file': prior_pcd_file}],
    remappings=[('registered_scan', '/cloud_registered')],
)
```

但当前 `cod` 实车 launch 基本传 `use_composition:=False`，第一阶段可以只保证普通 Node 路径可用。

## Step 5：修改顶层 launch

### `singlenav_launch.py`

当前 `singlenav_launch.py` 中有静态：

```text
map -> odom
```

接入重定位后必须避免重复发布同一条 TF。否则会出现 TF 冲突，Nav2 / RViz 看到的机器人全局位姿会抖动或跳变。

推荐改法：

1. 增加 `prior_pcd_file` launch argument，默认指向：

```text
src/cod_bringup/pcd/rmul2026.pcd
```

2. 增加 `use_relocalization` launch argument，默认 `true`。
3. 只有 `use_relocalization:=false` 时才发布静态 `map -> odom`。
4. include `localization_launch.py` 时传入：

```python
'prior_pcd_file': prior_pcd_file
```

推荐模式：

```text
use_relocalization=true  -> small_gicp 发布 map->odom
use_relocalization=false -> static_transform_publisher 发布 map->odom
```

### `multiplenav_launch.py`

当前 `multiplenav_launch.py` 启动了 `slam_toolbox`。如果想切到先验地图重定位模式，需要二选一：

- `slam_toolbox` 发布 `map -> odom`
- `small_gicp_relocalization` 发布 `map -> odom`

不能同时发布。

推荐新增一个模式开关：

```text
localization_mode:=slam      # 现状：slam_toolbox 建图/定位
localization_mode:=relocalization  # 使用 PCD 重定位
localization_mode:=static    # 调试兜底，静态 map->odom
```

第一阶段更简单的做法是先只改 `singlenav_launch.py`，验证 small_gicp 重定位闭环稳定后，再把 `multiplenav_launch.py` 改成模式化。

## Step 6：启动顺序建议

推荐实车启动顺序：

1. `livox_ros_driver2` 发布 `/livox/lidar`、`/livox/imu`
2. `cpp_lidar_filter` 发布 `/livox/lidar_filtered`
3. `livox_tf_tuner.py` 发布 `base_link -> livox_frame`
4. `small_point_lio` 发布：
   - `odom -> base_link`
   - `/Odometry`
   - `/cloud_registered`
5. `map_server` 加载 2D 栅格地图
6. `small_gicp_relocalization` 加载 PCD 并发布 `map -> odom`
7. `fake_vel_transform` 发布 `base_link -> base_link_fake`
8. Nav2 controller/planner/bt 等节点启动

如果 small_gicp 启动太早，可能出现：

```text
TF lookup failed ... Retrying...
```

这通常说明 `base_link -> livox_frame` 或 LIO 输出还没起来。短时间重试是正常的；如果一直失败，应检查 TF 树。

## Step 7：验证命令

启动后检查节点：

```bash
ros2 node list | grep small_gicp
```

检查输入点云：

```bash
ros2 topic hz /cloud_registered
ros2 topic echo /cloud_registered --once
```

检查重定位 TF：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map base_link_fake
```

检查是否有重复发布 `map -> odom`：

```bash
ros2 run tf2_tools view_frames
```

或观察 `/tf` 是否同时存在两个 `map -> odom` 来源。

检查 RViz：

- Fixed Frame 设为 `map`
- 显示 `/map`
- 显示 `/cloud_registered`
- 显示 TF
- 通过 `2D Pose Estimate` 发布 `/initialpose`

正确现象：

- `/cloud_registered` 点云能稳定落在 `odom` 下。
- `small_gicp_relocalization` 日志不再持续报没有点云或 TF 失败。
- `map -> odom` 随 GICP 结果小幅更新。
- 机器人在 RViz 的 `map` 下位置与真实场地一致。

## 初始位姿使用方式

`small_gicp_relocalization` 支持 `/initialpose`。

建议流程：

1. 机器人放在地图中已知位置。
2. RViz Fixed Frame 设为 `map`。
3. 点击 `2D Pose Estimate`，给一个大致位姿。
4. 等待 1 到 3 秒，让 GICP 累积点云并收敛。
5. 再发送 Nav2 goal。

注意：

- GICP 不是魔法全局搜索，初始误差太大时可能收敛到错误位置。
- 初始位姿方向比位置更敏感，建议尽量给准 yaw。
- 场地几何高度重复或长走廊结构中，可能出现局部最优匹配。

## 2D map 与 3D PCD 的关系

接入后会同时有两份地图：

- 2D 栅格地图 `.yaml + .pgm/.png`
  - 给 Nav2 `map_server`、global costmap、planner 使用。
- 3D PCD 地图 `.pcd`
  - 给 `small_gicp_relocalization` 做点云配准使用。

两份地图必须在同一个 `map` 坐标定义下。

如果 2D map 和 PCD map 原点不一致，会出现：

- 重定位看起来成功，但机器人在 2D 栅格地图上整体偏移。
- Nav2 规划路径与真实场地不重合。
- global costmap 的 static layer 与实时点云障碍不对齐。

建议用同一次建图流程生成 3D PCD 和 2D map，或者明确记录两者之间的平移/旋转关系。

## 和 AMCL 的区别

这套方案不是 AMCL。

AMCL 输入通常是：

```text
2D LaserScan + 2D occupancy grid + odom
```

而这里是：

```text
3D registered PointCloud2 + 3D PCD map + LIO odom
```

两者最终都服务于同一个目标：维护 `map -> odom`。因此无论使用 AMCL、slam_toolbox 还是 small_gicp，都必须保证同一时间只有一个模块发布 `map -> odom`。

## 推荐的最小改动清单

第一阶段建议只做以下最小闭环：

1. 复制 `small_gicp_relocalization` 到 `src/`。
2. 新建目录：

```text
src/cod_bringup/pcd
```

3. 放入先验地图：

```text
src/cod_bringup/pcd/rmul2026.pcd
```

4. 修改 `cod_bringup/CMakeLists.txt`，把 `pcd` 安装到 share：

```cmake
install(
  DIRECTORY launch params rviz maps wps behavior_trees pcd
  DESTINATION share/${PROJECT_NAME}
)
```

5. `singlenav2_params.yaml` 加 `small_gicp_relocalization` 参数块。
6. `localization_launch.py` 加 `prior_pcd_file` 参数和 `small_gicp_relocalization` 节点。
7. `singlenav_launch.py`：
   - 传入 `prior_pcd_file`
   - 重定位开启时禁用静态 `map -> odom`
8. 编译并 source：

```bash
colcon build --symlink-install --packages-select small_gicp_relocalization cod_bringup
source install/setup.bash
```

9. 启动验证：

```bash
ros2 launch cod_bringup singlenav_launch.py use_relocalization:=true
```

## 风险点和排查

### 1. `map -> odom` 重复发布

症状：

- RViz 中机器人跳动。
- TF warning 提示重复 publisher。
- Nav2 global pose 抖动。

处理：

- 关闭 `singlenav_launch.py` 中静态 `map_to_odom`。
- 关闭 `slam_toolbox` 的 TF 发布。
- 确保只剩 small_gicp 发布 `map -> odom`。

### 2. 没有 `registered_scan`

症状：

```text
No accumulated points to process.
```

处理：

- 确认 `/cloud_registered` 有频率。
- 确认 launch remap：`registered_scan -> /cloud_registered`。
- 确认 `small_point_lio` 正常初始化。

### 3. TF lookup failed

常见缺失：

```text
base_link -> livox_frame
odom -> base_link
```

处理：

```bash
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo odom base_link
```

如果 `base_link -> livox_frame` 缺失，先检查 `livox_tf_tuner.py` 是否启动。

### 4. PCD 坐标系不一致

症状：

- GICP 可以收敛，但定位整体偏一大截。
- 机器人在 2D map 上不对。

处理：

- 用当前同一套外参和同一套 LIO 重新录制 PCD。
- 第一版优先使用 `small_point_lio` 的 `map_save` 保存逻辑，不要直接拿 `/cloud_registered` 转 PCD。
- 如果必须使用 `/cloud_registered` 转出来的 PCD，需要同步修改 `small_gicp_relocalization::loadGlobalMap()` 中加载 PCD 后的外参变换逻辑，避免二次套 `base_link <- livox_frame`。

### 5. 初始位姿太差导致误匹配

症状：

- `map -> odom` 突然跳到错误位置。
- 机器人在相似区域之间错配。

处理：

- RViz 重新发布更准确的 `/initialpose`。
- 降低车速，原地慢转一圈增加点云约束。
- 适当调大 `max_dist_sq`，但不要过大，否则会引入错误对应点。

## 建议结论

`cod_-rm2026_-navigation` 最适合直接复用 `pb2025_sentry_nav_new/small_gicp_relocalization`，因为当前工程已经具备它最需要的输入：

```text
/cloud_registered in odom frame
odom -> base_link
base_link -> livox_frame
```

真正需要补的是：

- 引入 `small_gicp_relocalization` 包
- 提供 PCD 先验地图
- 修改 `localization_launch.py` 启动该节点
- 禁止静态或 SLAM 节点重复发布 `map -> odom`

建议先在 `singlenav_launch.py` 上完成最小闭环验证，再决定是否把 `multiplenav_launch.py` 改造成 `slam / relocalization / static` 三模式。
