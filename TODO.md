当前项目是室外定位导航项目，完整介绍在@README.md。你需要阅读相关文档和整个所有代码。理清项目架构和逻辑。检查当前项目是否存在Bug。
需求：

我们现在是直接使用PX4板子里的融合定位算法，即结合其自身的IMU+双天线GPS全局定位坐标和航向角来获取小车融合后的全局坐标、局部位姿、坐标原点这些数据，然后提供给导航避障。
如果后续想到摒弃PX4板子这个硬件，自己从其开源代码PX4-Autopilot中摘取关键部分的代码，结合从Mid360的IMU+双天线GPS全局定位坐标和航向角，来获取上述数据。
你需要评估这个改动方案的难度和工作量。
目的：摒弃PX4硬件的主要原因是成本问题和更灵活的融合算法
当前系统路径：BG-620双天线GPS → 计算真北航向 → 通过AGRICA → PX4 EKF融合 → MAVROS输出
之后BG-620通过USB口直接与Orin NX连接：BG-620双天线GPS --NMEA/GPRMC/AGRICA--> Orin NX --> robot\_localization融合 --> /odom (等效当前 /mavros/local\_position/odom)
双天线航向角是接收机计算后，通过AGRICA消息发送给PX4，PX4解算出航向后，将其作为输入参与计算并输出/mavros/local\_position/odom。

NMEA数据输出：$GNGGA,055234.00,4004.73879510,N,11614.19821957,E,1,28,0.7,61.8089,M,-8.4923,M,,*50
Log 头，UTC 时间，纬度，纬度方向，经度，经度方向，GPS 状态（1=单点、3=GPS PPS 模式、4=RTK INT、5=RTK Float），卫星数，水平精度因子，海拔高度（参考 MSL 大地水准面），海拔高度单位（M = m），大地水准面相对地球椭球面的高度，相对高度单位（M = m），差分数据龄期，差分基站 ID，校验和
NMEA数据输出：$GNRMC,061402.00,A,4004.73846648,N,11614.19829285,E,0.003,12.5,301221,6.9,W,A,V*78
Log 头，UTC 时间，状态（A=数据可用，V=接收机警告），纬度，纬度方向，经度，经度方向，地面速率（单位为节），地面航向（单位为度，从北向起顺时针计算），日期:（ddmmyy），磁偏角（单位度），磁偏角方向，模式（A=自主模式、F=RTK Float、P=高精度模式、R=RTK Int），定位状态（S=安全、C=注意、U=危险、V=定位状态不可用），校验和
Unicore数据输出：#AGRICA,97,GPS,FINE,2190,363942000,0,0,18,12;GNSS,232,21,12,30,5,5,24,1,0,5,15,1,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.005,-0.003,0.001,0.004,0.042,0.050,0.044,40.07898274722,116.23663152683,60.0036,-2160488.6213,4383615.6655,4084732.9679,1.8493,1.8902,4.4654,0.0000,0.0000,0.0000,0.00000000000,0.00000000000,0.0000,-0.00000000000,0.00000000000,0.0000,363942000,0.000,15.213205,-8.492279,0.000000,0.000000,5,0,0,0\*0b2e294a
消息头，GNSS，指令长度（从GNSS到CRC 校验，整包数据长度232字节），UTC 时间-年，UTC 时间-月，UTC 时间-日，UTC 时间-时，UTC 时间-分，UTC 时间-秒，流动站定位状态（1：单点定位解、4：固定解、5：浮动解），主从天线 Heading 解状态（0：无效解、4：固定解、5：浮动解），参与解算 GPS 卫星数，参与解算北斗卫星数，参与解算 GLONASS 卫星数，基站到流动站基线向量（北方向分量），基站到流动站基线向量（东方向分量），基站到流动站基线向量（天顶向分量），基站到流动站基线向量（北方向分量标准差），基站到流动站基线向量（东方向分量标准差），基站到流动站基线向量（天顶向分量标准差），航向角，俯仰角，横滚角，速度大小，北方向速度，东方向速度，天顶方向速度，北方向速度标准差，东方向速度标准差，天顶方向速度标准差，流动站纬度（-90~~90 度），流动站经度（-180~~180），流动站高程，...


# rsync -avz --progress /media/lenovo/disk/KT5/outdoor\_nav/src/navigation2/nav2\_input\_bridge/ nhy\@10.88.103.186:\~/ww/outdoor\_nav2/src/navigation2/nav2\_input\_bridge

当前仓库是一个outdoor 导航项目，基于 ROS 2 构建，融合激光雷达感知与 Nav2 导航框架，运行在 NVIDIA Orin NX 开发板上。
我创建了go2w（当前也正处于go2w分支下），用于将这套算法部署在宇树的go2w上。
现在有一些关于该项目总体说明和当前部署到go2w的开发计划文档，你需要阅读README.md，docs下的所有文档，以及所有代码来全面理解这个项目。
当前项目地址：/media/lenovo/disk/KT5/outdoor\_nav
仿真和宇树ros2等所在路径：/media/lenovo/disk/KT5/sim\_ws
官方SDK开发文档：docs/Go2-W SDK开发指南.md
现在已经制定了实机部署计划：docs/Go2W实机部署计划.md。
目前已经完成了模块A的开发、测试和代码提交，现在正在进行模块D的开发。

注意：每完成一个具有里程碑的代码开发和测试验证，就要提交一个commit.

你先理解整个项目，之后我们再进行下一步

```bash
# Python 转换工具
from scipy.spatial.transform import Rotation
r = Rotation.from_euler('xyz', [roll, pitch, yaw])  # 单位：弧度
qx, qy, qz, qw = r.as_quat()
```


## 外参标定
- M9N GPS在PX4中心的后方7cm，上方10.8cm
- PX4中心在狗躯干中心的后方4cm，上方9cm
- mid360在狗躯干中心的前方18cm,上方13cm, pitch向下倾斜18度
- 狗躯干中心在正常站立状态下距地面高度40cm








---

# 已完成 (Completed)

✅ 2026-06-09: FAST-LIVO2 ↔ mavros frame align (input_bridge integration), EKF2_EV_CTRL=3, 12 unit tests (3 math + 9 state machine)
   - 详见 `docs/superpowers/specs/2026-06-04-fastlvio2-mavros-frame-align-design.md` 和 `docs/superpowers/plans/2026-06-04-fastlvio2-mavros-frame-align.md`
   - 实现：`AlignStateMachine` 5 状态机 (INIT/WAITING_DATA/READY_TO_LATCH/LATCHED/RELATCHING/FATAL), 8 condition latch
   - 部署 tag: `align-v1-20260609`
   - **未完成 (deferred)**: T10 现场 bag replay 测试（需要 RTK FIXED GPS + FAST-LIVO2 现场）
   - **安全 (deferred to production hardening)**:
     - `relatch_srv_` Trigger 服务无认证 (HIGH)；需要 SROS2 + HMAC challenge
     - 参控 topic 名称可被 `ros2 param set` 重绑 (HIGH)；需要 allowlist 验证
