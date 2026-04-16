# outdoor_nav

outdoor 导航项目，基于 ROS 2 构建，融合激光雷达感知、SLAM 与 Nav2 导航框架，运行在 NVIDIA Orin NX 开发板上。

## 系统架构与运行逻辑

**目标**：无人机获取目标人物的 GPS 坐标后发送给地面端小车，小车根据自身 GPS 位置和目标 GPS 位置进行全局无图路点导航 + 局部动态避障。

为减少 GPS/IMU 融合算法的开发量，小车状态（位姿 + 速度）直接采用 **PX4 EKF2 融合定位**（`/mavros/local_position/odom`）。

### 坐标系统

1. **局部坐标原点**：小车上电并在室外获得稳定 GPS 信号（3D fix）后，Pixhawk EKF2 将当前位置锁定为全局原点 `gp_origin`（经纬度），对应局部坐标系原点 `(0, 0, 0)`
   - 方向：固定 ENU（X-东，Y-北，Z-天）
2. **坐标投影**：`nav2_input_bridge` 使用 **PX4 球面局部切平面投影**（`PX4-Autopilot/src/lib/geo/geo.cpp`），把 WGS84 经纬度投影到以原点为中心的米制平面坐标

### 目标输入

`nav2_input_bridge` 同时接收两类 GPS 目标点：

- **`/gp_goal`**：手动测试/地面站注入（`sensor_msgs/NavSatFix`）
- **`/uav/target_gps`**：无人机通过电台发来的目标坐标

节点将目标坐标转换为局部 ENU 后发布到 `/goal_pose`，Nav2 控制器执行路径跟踪与避障。

### YOLO 停止机制

当同机运行的 YOLO 检测到目标人物时，发布 **`/target_detected`**（`std_msgs/Bool`，`data=true`）。

`nav2_input_bridge` 收到后：
1. **立即发布当前小车位置**作为新的 `goal_pose`，让 Nav2 就地停止
2. **置位 `target_detected` 标志**，此后**忽略所有新的 GPS 目标点**（无论来自 `/gp_goal` 还是 `/uav/target_gps`）

> 如需重新启动任务，必须重启导航 launch。

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

### 1. 重启小车

> 每次实机测试前建议先重启小车。因为小车与遥控器建立通信后，可能会导致 NVIDIA Orin NX 板的控制断连，重启可恢复稳定的控制链路。

### 2. 一键启动全车系统

编译完成后，在项目根目录下直接运行：

```bash
bash launch_all.sh
```

> 该脚本会自动 source ROS 2 环境和本地 install，然后启动 `outdoor_all.launch.py`。

该 launch 会依次启动：
1. 小车运控（`ros2can car_driver.launch.py`）
2. PX4 融合定位（`mavros px4.launch`）
3. 导航栈（`gp_goal_nx_nav2.launch.py`，含 Livox 驱动、Nav2 控制器、局部避障等）
4. **自动设置局部坐标原点**：启动 10 秒后自动发布 `set_gp_origin`，并订阅反馈验证；若失败会自动重试（最多 3 次）

启动后观察终端日志，当看到 `gp_origin verified successfully` 时，系统已就绪。

### 下发目标点

系统就绪后，在**另一个终端**下发 GPS 目标点：

```bash
# 手动测试
ros2 topic pub --once /gp_goal sensor_msgs/msg/NavSatFix \
  "{header: {frame_id: 'gps'}, latitude: 30.8135718, longitude: 120.8338169, altitude: 12.318126322268082}"

# 或模拟无人机链路
ros2 topic pub --once /uav/target_gps sensor_msgs/msg/NavSatFix \
  "{header: {frame_id: 'gps'}, latitude: 30.8135718, longitude: 120.8338169, altitude: 12.318126322268082}"
```

### 手动测试 YOLO 停止机制

在小车行驶过程中，可通过以下命令模拟 YOLO 检测到目标，触发立即停车并忽略后续新目标点：

```bash
ros2 topic pub --once /target_detected std_msgs/msg/Bool "{data: true}"
```

> 触发后如需重新接受目标点，必须重启导航 launch。

若需单独观察原点反馈：

```bash
ros2 topic echo /mavros/global_position/gp_origin
```

原点设置参数位于 `src/robot_launch/config/gp_goal_nx_nav2.yaml`：

```yaml
nav2_input_bridge:
  ros__parameters:
    reference_latitude: 30.8135718
    reference_longitude: 120.8338169
    reference_altitude: 12.318126322268082
    goal_input_topic: "/gp_goal"
    uav_goal_input_topic: "/uav/target_gps"
    target_detected_topic: "/target_detected"
    goal_output_topic: "/goal_pose"
    goal_output_frame: "map"
```

### 备选：分步手动启动

若某模块需要单独调试，仍可分别启动：

```bash
# 终端 1：小车运控
bash launch_car.sh

# 终端 2：PX4 融合定位
bash launch_loc.sh

# 终端 3：导航
bash launch.sh
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

4. **YOLO 触发后小车会停在哪里？会不会掉头回来？**
   - 从 YOLO 检测到目标 → 发布 stop goal，端到端延迟约 **150-250 ms**（含图像推理、ROS2 传输、Nav2 控制周期、CAN 响应）。
   - 按当前线速度(`FollowPath/desired_linear_vel`) **0.5 m/s** 估算，惯性滑行约 **0.1-0.15 m**。
   - Nav2 的 `general_goal_checker/xy_goal_tolerance` 为 **0.25 m**，因此滑行距离通常落在容忍范围内，**不会触发掉头返回**，仅会原地减速停车。
   - 若未来提高车速或地面极滑导致滑行超过 0.25 m，可能出现倒车修正，届时可引入“提前制动距离”或“检测框相对位姿”优化。
