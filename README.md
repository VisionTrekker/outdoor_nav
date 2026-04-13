# outdoor_nav

 outdoor 导航项目，基于 ROS 2 构建，融合激光雷达感知、SLAM 与 Nav2 导航框架。

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
- **ROS 发行版**：ROS 2 Humble Hawksbill
- **编译工具**：`colcon`、`CMake >= 3.16`

### 安装 ROS 2 Humble

参考官方文档：[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

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

该脚本等价于：

```bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

编译完成后，source 本地工作空间：

```bash
source install/setup.bash
```

建议将 source 命令添加到 `~/.bashrc` 中：

```bash
echo "source /media/lenovo/disk/KT5/outdoor_nav/install/setup.bash" >> ~/.bashrc
```

## 运行

### 1. 启动建图（FAST_LIO + Livox）

```bash
ros2 launch robot_launch nav2.launch.py
```

或根据实际场景选择对应的 launch 文件：

```bash
ros2 launch robot_launch gp_goal_nx_nav2.launch.py
```

### 2. 启动 CAN 驱动

```bash
ros2 launch ros2can car_driver.launch.py
```

### 3. 单独启动导航

```bash
ros2 launch robot_launch nx_nav2.launch.py
```

> 具体 launch 文件及参数含义请参考 `src/robot_launch/launch/` 目录下的源码与配置。

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

## 贡献

欢迎提交 Issue 和 Pull Request。

## 许可证

各子模块遵循其原始仓库的许可证，其余代码按项目实际情况声明。
