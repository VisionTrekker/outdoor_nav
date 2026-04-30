# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Build all packages
./build.sh

# Build with tests disabled (default)
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

## Launch Scripts

```bash
# Full system (运控 + PX4 + 导航 + 原点设置)
bash launch_all.sh

# Navigation only (no GPS origin)
bash launch.sh

# Navigation with GPS goal input
bash launch_all.sh use_auto_origin:=false

# 分步启动
bash launch_car.sh   # CAN 运控
bash launch_loc.sh   # PX4 EKF2
bash launch.sh       # 导航栈
```

## Architecture

### 坐标系统
- **ENU 坐标系**：原点为 `gp_origin`（小车上电后锁定），X-东，Y-北，Z-天
- **GPS 投影**：`nav2_input_bridge::llaToEnu()` 使用球面切平面投影（与 PX4 MapProjection 一致）
- **椭球投影备用**：代码中以 `#if 0` 注释保留 WGS84 椭球算法，5km+ 场景可将 `#if 0` 改为 `#if 1` 启用

### 核心自定义包

#### nav2_input_bridge
- 订阅 `/gp_goal` 和 `/uav/target_gps`（`sensor_msgs/NavSatFix`）
- 将 GPS 坐标转换为 ENU 局部坐标，发布到 `/goal_pose`
- 接收 `/target_detected`（YOLO 检测信号），通过 `stop_planner` + 取消 FollowPath action 实现立即停车

#### nav2_straight_planner
- LifecycleNode，实现直线路径规划器
- 订阅 `/goal_pose`，发布 FollowPath action goal 到 `controller_server`
- 订阅 `stop_planner` topic，收到后取消当前 FollowPath，记录停止位置
- 拒绝与停止位置距离 < 0.1m 的新目标（防止重复触发）

### YOLO 停止机制（方案B）
```
YOLO 检测 → /target_detected → nav2_input_bridge
                                    ↓
                          发布 stop_planner=true
                                    ↓
                          取消 FollowPath action
                                    ↓
                              straight_planner
                                    ↓
                          记录停止位置，拒绝近处目标
```

### PX4 定位融合
- 小车使用 Pixhawk EKF2 融合定位，通过 `/mavros/local_position/odom` 提供位姿
- ENU 原点通过 `set_gp_origin` 服务设置（自动或手动）

## Submodules

```bash
# 克隆时拉取所有子模块
git clone --recurse-submodules git@github.com:VisionTrekker/outdoor_nav.git

# 后续更新子模块
git submodule update --init --recursive
```

- `src/slam/FAST_LIO/` — 激光 SLAM
- `src/driver/livox_ros_driver2/` — Livox 激光雷达驱动
- `src/navigation2/spatio_temporal_voxel_layer/` — 3D 体素层

## 关键文件

| 文件 | 作用 |
|------|------|
| `src/robot_launch/launch/outdoor_all.launch.py` | 全系统启动入口 |
| `src/robot_launch/config/gp_goal_nx_nav2.yaml` | nav2_input_bridge 参数配置 |
| `src/navigation2/nav2_input_bridge/src/input_bridge_node.cpp` | GPS→ENU 转换、YOLO 停止逻辑 |
| `src/navigation2/nav2_straight_planner/src/straight_planner.cpp` | 直线规划器、停止机制 |
