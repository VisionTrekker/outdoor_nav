# outdoor_nav

outdoor 导航项目，基于 ROS 2 构建，融合激光雷达感知、SLAM 与 Nav2 导航框架。
该项目运行在 NVIDIA Orin NX 开发板上。

项目需求：
无人机获取目标人物的位置后，将其GPS坐标发送给地面端小车。小车根据自身GPS坐标和目标GPS坐标，进行导航避障。
暂时为减少开发GPS和IMU融合算法的工作量，现使用PX4融合定位来获取小车状态。

采用的架构 —— 全局无图路点导航 + 局部动态避障：
小车接收到无人机发送的目标 GPS 后，做局部坐标投影变换，计算其在局部坐标系的位置，然后导航避障
1. 确立局部坐标“原点”
   - 小车上电并在室外获得稳定 GPS 信号（3D fix）后，Pixhawk 的 EKF2 算法会初始化。它会把小车当前所在位置的 GPS 坐标锁定为 全局坐标原点`gp_origin`（经纬度）和 局部坐标系原点 (0, 0, 0)
   - 局部坐标系：
     - 原点：GPS 信号（3D fix）后的位置（订阅 /mavros/global_position/gp_origin）
     - 方向：固定的 ENU （东北天坐标系 X-东，Y-北，Z-天）
2. 接收无人机目标点：ROS2 节点通过电台接收无人机发来目标点GPS坐标；
3. 坐标投影转换 (核心步骤，把基于 WGS84 的经纬度直接投影到以原点为中心的平面坐标)：使用PX4-Autopolit中的球面局部切平面投影（PX4-Autopilot/src/lib/geo/geo.cpp）
4. 执行导航与避障（系统里所有数据都统一到了米制单位）：
   - 小车当前状态（位姿 + 速度）： 来自 /mavros/local_position/odom
   - 目标点的局部位置

## 项目结构

```
outdoor_nav/
├── src/driver/                    # 硬件驱动
│   ├── livox_ros_driver2/         # Livox 激光雷达驱动 (submodule)
│   └── ros2can/                   # CAN 总线通信节点
├── src/navigation2/               # 导航相关包
│   ├── nav2_common/               # Nav2 公共库
│   ├── nav2_controller/           # 控制器
│   ├── nav2_costmap_2d/           # 代价地图
│   ├── nav2_msgs/                 # 自定义消息
│   ├── nav2_regulated_pure_pursuit_controller/  # 纯追踪控制器
│   ├── nav2_util/                 # 工具函数
│   └── spatio_temporal_voxel_layer/             # 3D 体素层 (submodule)
├── src/slam/                      # SLAM
│   └── FAST_LIO/                  # 快速激光 SLAM (submodule)
└── src/robot_launch/              # 启动文件与配置
```

## 环境要求

- **操作系统**：Ubuntu 22.04
- **ROS 发行版**：ROS 2 Humble
- **编译工具**：`colcon`、`CMake >= 3.16`

### 安装 ROS 2 Humble

参考官方文档：[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

一键安装命令： `source <(wget -qO- http://fishros.com/install)`

安装后记得 source 环境：

```bash
source /opt/ros/humble/setup.bash
```

## 克隆仓库

本项目包含 **Git Submodule**，请使用 `--recurse-submodules` 参数克隆：

```bash
git clone --recurse-submodules git@github.com:VisionTrekker/outdoor_nav.git
cd outdoor_nav
```

如果已经克隆但忘记拉取子模块，可执行：

```bash
git submodule update --init --recursive
```

## 安装依赖

在项目根目录下，使用 `rosdep` 安装所有依赖：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> 若某些依赖无法通过 `rosdep` 自动安装（如 Livox-SDK2、OpenVDB 等），请根据对应子模块的 README 手动安装。

## 编译

项目提供了自动化编译脚本：

```bash
./build.sh
```

> 等价于：`colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF`

编译完成后，source 本地工作空间：

```bash
source install/setup.bash
```

建议将 source 命令添加到 `~/.bashrc` 中：

```bash
echo "source [path_to_outdoor_nav]/install/setup.bash" >> ~/.bashrc
```

## 实机测试流程

按以下顺序依次启动（每步建议新开终端）：

### 1. 重启小车

### 2. 启动小车运控

```bash
bash launch_car.sh
```

> 等价于 `ros2 launch ros2can car_driver.launch.py`

### 3. 启动 PX4 融合定位

```bash
bash launch_loc.sh
```

> 等价于 `ros2 launch mavros px4.launch fcu_url:="/dev/ttyACM0:230400"`

### 4. 启动导航

```bash
bash launch.sh
```

> 等价于 `ros2 launch robot_launch gp_goal_nx_nav2.launch.py`

### 5. 设置局部坐标原点并下发目标点

- **设置局部坐标原点**（需与 `src/robot_launch/config/gp_goal_nx_nav2.yaml` 中的参考坐标一致）

  ```bash
  ros2 topic pub --once /mavros/global_position/set_gp_origin geographic_msgs/msg/GeoPointStamped \
    "{header: {frame_id: 'map'}, position: {latitude: 30.8135718, longitude: 120.8338169, altitude: 12.318126322268082}}"
  ```

  对应的配置参数：

  ```yaml
  nav2_input_bridge:
    ros__parameters:
      use_sim_time: false
      local_odom_topic: "/mavros/local_position/odom"
      reference_latitude: 30.8135718
      reference_longitude: 120.8338169
      reference_altitude: 12.318126322268082
      goal_input_topic: "/gp_goal"
      goal_output_topic: "/goal_pose"
      goal_output_frame: "map"
      require_nav_sat_fix: true
      goal_yaw_from_bearing: false
  ```

- **验证原点设置成功**

  ```bash
  ros2 topic echo /mavros/global_position/gp_origin
  ```

- **下发 GPS 目标点**

  ```bash
  ros2 topic pub --once /gp_goal sensor_msgs/msg/NavSatFix \
    "{header: {frame_id: 'gps'}, latitude: 30.8135718, longitude: 120.8338169, altitude: 12.318126322268082}"
  ```

## 模块调试（开发场景）

实机流程已经封装了最常用的启动脚本，如需单独调试某个模块，可直接使用：

### 建图（FAST_LIO + Livox）

```bash
ros2 launch robot_launch nav2.launch.py
```

### 单独启动导航（不带 GPS 目标输入）

```bash
ros2 launch robot_launch nx_nav2.launch.py
```

> 更多 launch 文件及参数请参考 `src/robot_launch/launch/` 目录。

## 常见问题

1. **Submodule 目录为空**
   - 确保克隆时使用了 `--recurse-submodules`，或后续执行 `git submodule update --init --recursive`。

2. **编译时找不到 Livox-SDK2**
   - 请参考 `src/driver/livox_ros_driver2/README.md` 手动编译安装 Livox-SDK2。

3. **spatio_temporal_voxel_layer 编译失败**
   - 该包依赖 OpenVDB，若 `rosdep` 未自动安装，请手动安装 `libopenvdb-dev`：
     ```bash
     sudo apt install libopenvdb-dev
     ```
