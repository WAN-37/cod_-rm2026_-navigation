# COD_RMUL2026_Navigation
## 项目简介

- **运行环境**
  - Ubuntu 22.04
  - ROS2 Humble
  - Livox MID-360
- 基于Nav2框架开发的导航功能包
- 控制器选用MPPI
- Slam_Toolbox同时导航建图
- 单点导航/多点导航
- 定位与地图处理
- 全局/局部路径规划
- 导航控制与状态切换

## 仓库结构
```bash
.
├── cod_bringup                     #Navigation2导航启动文件、机器人运动参数、地图存储、csv多点文件
├── cpp_lidar_filter                #剪裁去除机器人自身点云
├── fake_vel_transform              #TF转换
├── goal_approach_controller        #Nav2控制器wrapper：在接近目标时限制线速度，防止高速冲过目标点
├── pb_nav2_plugins                 #Navigation2插件库，控制机器人执行后退行为
├── pb_omni_pid_pursuit_controller  #PID控制器
├── pointcloud_to_laserscan         #点云转换，pointcloud->laserscan
├── ros2_simple_serial              #串口通信
├── small_point_lio                 #point_lio提供里程计，odom->base_link
└── README.md
```

## 使用说明
### 前置工作
- 安装[Livox SDK2] https://github.com/Livox-SDK/Livox-SDK2 
- 雷达启动包 https://github.com/Livox-SDK/livox_ros_driver2.git 

- 安装 `rosdep`  
   参考官方文档或使用如下命令进行安装：

   ```shell
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   ```
- 安装依赖
  ```shell
  mkdir ~/COD26
  git clone https://gitee.com/codnavgation/cod_-rm2026_-navigation.git
  cd cod_-rm2026_-navigation 
  rosdep install --from-paths src --ignore-src -r -y
  ```
- 构建方式
  ```shell
  colcon build --symlink
  source install/setup.bash
- 运行方式（启动导航前先启动雷达）
  - 多点导航
  ```shell
  ros2 launch cod_bringup multiplenav_launch.py
  ```
  - 单点导航
  ```shell
  ros2 launch cod_bringup singlenav_launch.py
  ```



