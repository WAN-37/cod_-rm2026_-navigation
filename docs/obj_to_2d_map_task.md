# OBJ 转 2D 先验地图任务文档

## 一、目标

把官方提供的 `/root/ros_ws/rmuc2026_v1_2_0.obj` 区域赛 3D 模型，按一个固定高度阈值切片投影到地面平面，离线生成一张 Nav2 可直接加载的 2D 先验栅格地图，覆盖**自家半场**，给 `cod_bringup` 的 `singlenav_launch.py` 使用。

最终用法预期为：

```bash
ros2 launch cod_bringup singlenav_launch.py \
    map:=/root/ros_ws/cod_-rm2026_-navigation/src/cod_bringup/maps/rmuc2026_home_half.yaml
```

## 二、当前上下文回顾

### 1. 现有方案性质

`ros2 launch cod_bringup singlenav_launch.py` 实际是 **「先验 2D 地图 + LIO 里程计」** 方案：

```text
map --static_transform_publisher--> odom --small_point_lio--> base_link --fake_vel_transform--> base_link_fake
                                                          \--static--> livox_frame
```

- 没有 `slam_toolbox`
- 没有 AMCL
- 没有 small_gicp 重定位
- `nav2_map_server` 负责加载 `map` 参数指向的 `*.yaml`
- `map -> odom` 由静态 TF 固定为单位变换
- 真正的位姿来自 `small_point_lio` 的 `odom -> base_link`

也就是说：**这张先验地图只用于全局 costmap 的静态层和路径规划，定位漂移由 LIO 决定**。这正是为什么需要一张准确的先验地图。

### 2. 参考地图 `shafa`

`@/root/ros_ws/cod_-rm2026_-navigation/src/cod_bringup/maps/shafa.yaml` 内容：

```yaml
image: shafa.png
mode: trinary
resolution: 0.05
origin: [-6.68, -8.05, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

`@/root/ros_ws/cod_-rm2026_-navigation/src/cod_bringup/maps/shafa.png` 是典型 Nav2 三值调色板 PNG：

- **尺寸：** `246 x 609 px` （对应 `12.3 m x 30.45 m`）
- **分辨率：** `0.05 m/px`
- **调色板：**
  - `(205,205,205)` 灰色 = unknown
  - `(254,254,254)` 白色 = free
  - `(0,0,0)` 黑色 = occupied

OBJ 生成的地图将复用同一套格式、同一组阈值、同一个调色板，以便和 `singlenav2_params.yaml` 既有 costmap 行为对齐。

### 3. OBJ 文件

`@/root/ros_ws/rmuc2026_v1_2_0.obj` 头几行示例：

```text
v  10094.649975 2204.968345 -1638.343628
```

数值量级初步判断单位为 **毫米**，需要在脚本里通过 bounding box 自动确认。

## 三、计划要做的事情

分成 5 个步骤，前两步生成 preview 给你看，后面再做正式裁剪和导出。

### 步骤 1：写一个离线转换脚本

新增：

```text
src/cod_bringup/scripts/obj_to_occupancy_map.py
```

并通过 `src/cod_bringup/CMakeLists.txt` 安装，类似你已有的 `livox_tf_tuner.py`、`fit_livox_extrinsic_translation.py`。

脚本职责：

- 解析 `*.obj` 顶点 `v` 和面片 `f`
- 自动统计 bounding box，输出 `xyz` 范围、推测的单位（mm 或 m）和高度轴
- 支持参数：
  - **`--input`**：OBJ 路径
  - **`--unit-scale`**：默认 `0.001`（mm 转 m）
  - **`--up-axis`**：`x/y/z`，默认自动判断
  - **`--ground-z`**：地面高度，默认自动估计为最低点
  - **`--obstacle-min-height`**：障碍最低高度，相对 `ground_z`，默认 `0.10 m`
  - **`--obstacle-max-height`**：障碍最高高度，相对 `ground_z`，默认 `2.0 m`（避开屋顶/天花板）
  - **`--resolution`**：默认 `0.05 m/px`
  - **`--roi`**：`x_min,x_max,y_min,y_max`（米，map 坐标系），不给就导出全场
  - **`--map-origin`**：`x,y` map 坐标系下图片左下角位置，不给就根据 ROI 自动算
  - **`--rotate-deg`**：可选，把 OBJ 平面绕地面法向旋转，对齐机器人初始朝向
  - **`--translate`**：可选，平移 OBJ 平面，对齐机器人初始位置
  - **`--out-prefix`**：输出文件名前缀
  - **`--outside-roi`**：`black` / `unknown`，ROI 之外像素的填充
  - **`--inflate-pixels`**：可选，离线膨胀像素数，默认 0（膨胀交给 Nav2）
- 投影逻辑：
  - 对每个三角面，把任一顶点位于 `[obstacle_min_height, obstacle_max_height]` 高度区间内的面，整体投影到地面平面
  - 用扫描线把投影三角形栅格化到障碍图层
- 输出：
  - **`<prefix>.png`**：和 `shafa.png` 同款 3 色调色板 PNG
  - **`<prefix>.yaml`**：和 `shafa.yaml` 同结构

### 步骤 2：生成全场 preview

先不裁剪、不平移、不旋转，直接以 OBJ 原始坐标导出一张：

```text
src/cod_bringup/maps/rmuc2026_preview.png
src/cod_bringup/maps/rmuc2026_preview.yaml
```

用途：

- 肉眼确认 OBJ 单位和高度轴判断对不对
- 确认场地长宽是否符合 RMUC 实际尺寸
- 确认哪一侧是自家半场
- 确认是否需要镜像、旋转
- 决定 ROI 范围和 `origin` 怎么设

我会把 preview 用 RViz `map_server` 加载步骤一起写在脚本注释里。

### 步骤 3：选择自家半场 ROI 和坐标对齐方式

根据 preview 与你的确认（见下方「需要你确认的信息」），决定：

- 自家半场 `x/y` 范围
- OBJ 坐标 → ROS `map` 坐标的旋转 / 平移 / 翻转
- 地图 `origin`
- ROI 外部填黑还是填灰

### 步骤 4：生成正式自家半场地图

输出：

```text
src/cod_bringup/maps/rmuc2026_home_half.png
src/cod_bringup/maps/rmuc2026_home_half.yaml
```

格式参照 `shafa.yaml`：

```yaml
image: rmuc2026_home_half.png
mode: trinary
resolution: 0.05
origin: [origin_x, origin_y, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### 步骤 5：和 Nav2/RViz 验证

只验证，不改 launch 默认值：

- 用 `map:=...rmuc2026_home_half.yaml` 启动 `singlenav_launch.py`
- 在 RViz 里检查障碍位置、机器人初始位姿、waypoint 是否落在白色区
- 如果对得上，再单独修改 `singlenav_launch.py` 的 `declare_map_yaml_file` 默认值（这一步不在本任务里做，等你确认后再做）

## 四、产出物清单

- **新增脚本：** `src/cod_bringup/scripts/obj_to_occupancy_map.py`
- **CMakeLists 改动：** `src/cod_bringup/CMakeLists.txt` 安装脚本
- **预览地图：** `src/cod_bringup/maps/rmuc2026_preview.png` + `.yaml`
- **正式地图：** `src/cod_bringup/maps/rmuc2026_home_half.png` + `.yaml`
- **本任务文档：** `docs/obj_to_2d_map_task.md`（即本文件）

## 五、需要你确认的信息

为了避免预览迭代次数过多，下面这几项最好先告诉我，能给一个就给一个，剩下的我用默认值跑 preview 后再让你选。

### 必须确认（影响最终地图是否能用）

1. **自家半场是哪一边？** 红方 / 蓝方？或者按场地坐标说一下大致范围（比如「OBJ 中 `y > 0` 的那半边」）。
2. **机器人初始位姿在 ROS `map` 坐标系下是什么？**
   - 例如「开机时 base_link 在 `(0, 0, 0)`，朝向 +x」
   - 或者「开机时 base_link 在自家补给区中心，朝向敌方半场」
   - 这一项决定 OBJ 坐标怎么平移/旋转到 `map`，以及 `origin` 怎么写。
3. **是否需要把 OBJ 坐标整体旋转到 ROS 习惯朝向？** 例如希望「`map.x` 指向敌方半场」。

### 建议确认（有默认值，你不说我就按默认来）

4. **障碍最低高度阈值：** 默认 `0.10 m`（高于地面 10cm 的结构算障碍）。需要更激进可以设 `0.05 m`。
5. **障碍最高高度阈值：** 默认 `2.0 m`（避免吊顶、灯架被算成障碍）。
6. **地图分辨率：** 默认 `0.05 m/px`，和 `shafa` 一致。
7. **自家半场之外的区域：** 默认 `black`（黑色障碍，强制 Nav2 不规划过去）。可改 `unknown`，但配合现在 `allow_unknown: true` 不太稳。
8. **是否在导图时做形态学膨胀：** 默认 `不膨胀`，膨胀交给 Nav2 的 `inflation_layer`（你现在 `singlenav2_params.yaml` 里 `inflation_radius: 0.75/0.55` 已经配过了）。
9. **OBJ 单位：** 默认按 `0.001` 把毫米转米；如果脚本统计出来 bounding box 不像场地尺寸，会自动告警让我们手动改。
10. **高度轴：** 默认让脚本自动判断；如果你已经知道是 `z`，可以直接告诉我。

### 完全可选

11. **是否需要把这张地图作为 `singlenav_launch.py` 的新默认 `map` 参数？** 这一步等你 RViz 验证后再决定，本任务不动 launch。
12. **是否需要同时输出一份**「敌方半场也可见、但被标黑」**的整场地图**，方便以后改打全场时直接换 ROI。

## 六、你只需要先回答两件事

如果懒得逐条回，最少给我这两个就能开工：

- **自家半场大概是 OBJ 里哪一块**（红方/蓝方，或一句话方向描述）
- **机器人开机位置在 `map` 坐标系下放在哪里、朝向哪边**

其余我先用默认值跑 preview，然后我们看图再调。
