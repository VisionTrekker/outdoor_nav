# Go2-W SDK开发指南

## [软件服务接口](https://support.unitree.com/home/zh/Go2-W_developer/Software_Interface_Services)

### 1. DDS 通信接口

更新时间：2024-09-09 20:45:21

**unitree_sdk2** 是在 DDS 上做了一层封装，支持 DDS 组件的 Qos 配置，对应用开发提供简单的封装接口，并实现了基于 DDS Topic 的 RPC 机制，适用于在 Go2-W 内部进程间以及 Go2-W 外部与内部的进程间通过发布/订阅和请求/响应两种方式完成不同场景下的数据通信。

说明：**unitree_sdk2** 安装配置教程，本文不再赘述，请查阅 [《快速开始》](https://support.unitree.com/home/zh/Go2-W_developer/Quick_Start)篇

#### 接口说明

##### unitree::robot::ChannelFactory

`unitree::robot::ChannelFactory` 在unitree::robot 下，提供了一个ChannelFactory 的单例，用于创建基于 DDS Topic 的 Channel。ChannelFactory 在使用前必须调用初始化接口，以对底层的 DDS 配置进行初始化，调用方法如

```
unitree::robot::ChannelFactory::Instance()->Init(0)
```

各接口描述如下：

|  **函数名**  | **Instance**                                                 |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | static unitree::robot::ChannelFactory* unitree::robot::ChannelFactory::Instance() |
| **功能概述** | 获取单例静态指针。                                           |
|   **参数**   | **无**                                                       |
|  **返回值**  | unitree::robot::ChannelFactory 单例静态指针。                |
|   **备注**   |                                                              |

|  **函数名**  | **Init**                                                     |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | void Init(int32_t domainId, const std::string& networkInterface = "", bool enableSharedMemory = false) |
| **功能概述** | 指定 Domain Id, 网卡名、是否使用共享内存三个初始化参数，对ChannelFactory 初始化。 |
|   **参数**   | **domainId**：默认构造DdsParticipant的domain id； **networkInterface**：指定网卡名，默认为空； **enableSharedMemory**：是否使用共享内存，默认为 false。 |
|  **返回值**  | 无                                                           |
|   **备注**   | 如果 **networkInterface** 为空，会生成自动选择网卡配置； 在 Go2-W 外部开发应用程序时，enableSharedMemory 需设置为 false。 |

|  **函数名**  | **Init**                                                     |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | void Init(const std::string& ddsParameterFileName = "")      |
| **功能概述** | 指定配置文件初始化 DdsParticipant。                          |
|   **参数**   | **ddsParameterFileName** ：指定一个 json 格式的配置文件路径，当ddsParameterFileName为空时，默认为当前程序目录下的名为 dds_parameter.json 的配置文件。 |
|  **返回值**  | 无                                                           |
|   **备注**   | 如果文件不存在, 会抛出配置文件不存在的异常; 如果指定的网不可用，会抛出DDS异常。 |

|  **函数名**  | **Init**                              |
| :----------: | ------------------------------------- |
| **函数原型** | void Init(const JsonMap& param)       |
| **功能概述** | 指定JsonMap配置初始化DdsParticipant。 |
|   **参数**   | **param** ：JsonMap格式配置。         |
|  **返回值**  | 无                                    |
|   **备注**   | 如果指定的网不可用, 会抛出DDS异常。   |

|  **函数名**  | CreateSendChannel                                            |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | template ChannelPtr CreateSendChannel(const std::string& name) |
| **功能概述** | 指定 name，创建一个指定消息类型为 MSG 的用来发送数据的 Channel。 |
|   **参数**   | **name**：Channel 名。                                       |
|  **返回值**  | template ChannelPtr                                          |
|   **备注**   |                                                              |

|  **函数名**  | CreateRecvChannel                                            |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | template ChannelPtr CreateRecvChannel(const std::string& name, std::function<void(const void*)> callback, int32_t queuelen = 0) |
| **功能概述** | 指定 name，创建一个指定消息类型为 MSG 的用来接收数据的 Channel。 |
|   **参数**   | **name**：Channel 名； **callback**：消息到达时的回调函数； **queuelen**：消息缓存队列的长度；如果长度为 0，不启用消息缓存内列。 |
|  **返回值**  | template ChannelPtr                                          |
|   **备注**   | 如果消息在回调函数中处理耗时较长，建议启用消息缓存队列，避免 DDS 回调线程阻塞。 |

|  **函数名**  | **Release**                                           |
| :----------: | ----------------------------------------------------- |
| **函数原型** | void Release()                                        |
| **功能概述** | 释放 ChannelFactory 静态资源。                        |
|   **参数**   | 无                                                    |
|  **返回值**  | 无                                                    |
|   **备注**   | unitree::robot::ChannelFactory::Instance()->Release() |

##### unitree::robot::ChannelPublisher

`unitree::robot::ChannelPublisher` 类实现了指定类型的消息发布功能。

|           类名            | 构造与析构                                                   |
| :-----------------------: | ------------------------------------------------------------ |
| template ChannelPublisher | explicit ChannelPublisher(const std::string& channelName); ~ChannelPublisher() |

注意：

unitree::robot::ChannelPublisher 构造时使用了unitree::robot::ChannelFactory::Instance()->CreateSendChannel(mChannelName)，所以在使用unitree::robot::ChannelPublisher 之前需要先初始化unitree::robot::ChannelFactory 单例。

|  **函数名**  | **InitChannel**                    |
| :----------: | ---------------------------------- |
| **函数原型** | void InitChannel()                 |
| **功能概述** | 初始化 Channel，准备用于发送消息。 |
|   **参数**   | 无                                 |
|  **返回值**  | 无                                 |
|   **备注**   |                                    |

|  **函数名**  | **CloseChannel**    |
| :----------: | ------------------- |
| **函数原型** | void CloseChannel() |
| **功能概述** | 关闭 Channel。      |
|   **参数**   | 无                  |
|  **返回值**  | 无                  |
|   **备注**   |                     |

|  **函数名**  | **Write**                          |
| :----------: | ---------------------------------- |
| **函数原型** | bool Write(const MSG& msg)         |
| **功能概述** | 发送指定类型的消息。               |
|   **参数**   | msg：发送 类型为 MSG 的消息。      |
|  **返回值**  | true：发送成功； false：发送失败。 |
|   **备注**   |                                    |

##### unitree::robot::ChannelSubscriber

`unitree::robot::ChannelSubscriber` 实现了指定类型的消息订阅功能。

|          **类名**          | 构造与析构                                                   |
| :------------------------: | ------------------------------------------------------------ |
| template ChannelSubscriber | explicit ChannelSubscriber(const std::string& channelName); ~ChannelSubscriber() |

注意：

unitree::robot::ChannelSubscriber 构造时使用了unitree::robot::ChannelFactory::Instance()->CreateSendChannel(mChannelName)，所以在使用unitree::robot::ChannelSubscriber 之前需要先初始化unitree::robot::ChannelFactory 单例。

|  **函数名**  | **InitChannel**                                              |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | void InitChannel(const std::function<void(const void*)>& callback, int64_t queuelen = 0) |
| **功能概述** | 初始化 Channel，准备用于接收或处理消息。                     |
|   **参数**   | **callback**：消息到达时的回调函数； **queuelen**：消息缓存队列的长度；如果长度为 0，不启用消息缓存内列。 |
|  **返回值**  | 无                                                           |
|   **备注**   | 如果消息在回调函数中处理耗时较长，建议启用消息缓存队列，避免 DDS 回调线程阻塞。 |

|  **函数名**  | **CloseChannel**    |
| :----------: | ------------------- |
| **函数原型** | void CloseChannel() |
| **功能概述** | 关闭 Channel。      |
|   **参数**   | 无                  |
|  **返回值**  | 无                  |
|   **备注**   |                     |

|  **函数名**  | **GetLastDataAvailableTime**                                 |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | int64_t GetLastDataAvailableTime()                           |
| **功能概述** | 获取最后一次收到消息的时间。                                 |
|   **参数**   | 无                                                           |
|  **返回值**  | Channel 未初始化时返回-1；否则返回从系统启动开始从 0 单调递增的时间戳，精确到微秒。 |
|   **备注**   |                                                              |

#### Service Client 接口

Go2-W 内部的多个服务组件通过 RPC Client 的方式对外提供部分功能接口，用户能方便的通过 Client 的接口实现对

Go2-W 的控制或数据获取。

###### 部分通用错误码列表

| 错误号 |       错误描述       |             备注              |
| :----: | :------------------: | :---------------------------: |
|  3001  |       未知错误       |       客户端/服务端返回       |
|  3102  |     请求发送错误     |          客户端返回           |
|  3103  |      API 未注册      |          客户端返回           |
|  3104  |       请求超时       |          客户端返回           |
|  3105  | 请求与响应数据不匹配 |          客户端返回           |
|  3106  |     响应数据无效     |          客户端返回           |
|  3107  |       租约无效       |          客户端返回           |
|  3201  |     响应发送错误     | 发生在服务端,不会返回到客户端 |
|  3202  |    服务端内部错误    |          服务端返回           |
|  3203  |  API 在服务端未实现  |          服务端返回           |
|  3204  |     API 参数错误     |          服务端返回           |
|  3205  |      请求被拒绝      |          服务端返回           |
|  3206  |       租约无效       |          服务端返回           |
|  3207  |      租约已存在      |          服务端返回           |

###### 部分客户端通用接口描述

|  **函数名**  | **Init**                                  |
| :----------: | ----------------------------------------- |
| **函数原型** | void Init()                               |
| **功能概述** | 客户端初始化，完成客户端 API 注册等逻辑。 |
|   **参数**   | 无                                        |
|  **返回值**  | 无                                        |
|   **备注**   |                                           |

|  **函数名**  | **SetTimeout**                      |
| :----------: | ----------------------------------- |
| **函数原型** | void SetTimeout(float timeout)      |
| **功能概述** | 设置 RPC 请求超时时间。             |
|   **参数**   | timeout：单位秒。                   |
|  **返回值**  | 无                                  |
|   **备注**   | 如未设置超时，默认超时时间为 1 秒。 |

|  **函数名**  | **WaitLeaseApplied**               |
| :----------: | ---------------------------------- |
| **函数原型** | void WaitLeaseApplied()            |
| **功能概述** | 申请租约，函数会阻塞至申请到租约。 |
|   **参数**   | 无                                 |
|  **返回值**  | 无                                 |
|   **备注**   | 只在启用了租约时有效。             |

|  **函数名**  | **GetApiVersion**                  |
| :----------: | ---------------------------------- |
| **函数原型** | const std::string& GetApiVersion() |
| **功能概述** | 获取客户端 API 版本。              |
|   **参数**   | 无                                 |
|  **返回值**  | 返回客户端 API 版本。              |
|   **备注**   |                                    |

|  **函数名**  | **GetApiVersion**                        |
| :----------: | ---------------------------------------- |
| **函数原型** | const std::string& GetServerApiVersion() |
| **功能概述** | 获取服务端 API 版本。                    |
|   **参数**   | 无                                       |
|  **返回值**  | 返回服务端 API 版本。                    |
|   **备注**   |                                          |



### 3. 底层服务接口

更新时间：2025-09-02 15:24:14

底层通信主要是获取电机，电池，遥控器，IMU数据并发布 **rt/lowstate**，订阅控制命令 **rt/lowcmd** 并控制电机、电池。

#### 接口说明

##### 底层控制指令

用户可通过订阅 DDS 话题 “**rt/lowcmd**” 来发送电机、电池、自动充电、电机电源开关的控制指令，数据格式如 **LowCmd_.idl** 所示。

- **电机控制命令**：机身共 16 个电机，其中腿部 12 个关节电机顺序与 Go2 机器人一致，其余4个为轮子电机， 电机的指令可参考 **MotorCmd_.idl** 。
- **电池控制命令**：关闭机身电池，详情参考 **BmsCmd_.idl** 。
- **自动充电命令**：控制自动充电的命令，**&0xFE**->自动充电打开 , **|0x01**->自动充电关闭。
- **电机电源控制命令**：控制电机电源的命令，**&0xFD**->16个电机的电源开关打开 , **|0x02**->16个电机的电源开关关闭。

```C
// generated from rosidl_generator_dds_idl/resource/idl.idl.em
    // with input from unitree_go:msg/LowCmd.idl
    // generated code does not contain a copyright notice
    #include "BmsCmd_.idl"
    #include "MotorCmd_.idl"
    
    #ifndef __unitree_go__msg__low_cmd__idl__
    #define __unitree_go__msg__low_cmd__idl__
    
    
    module unitree_go {
    
    module msg {
    
    module dds_ {
    
    
    struct LowCmd_ {
    octet head[2];             //帧头，数据校验用（0xFE,0xEF）。
    
    octet level_flag;          //保留，目前不用。
    octet frame_reserve;       //保留，目前不用。
    unsigned long sn[2];       //保留，目前不用。
    unsigned long version[2];  //保留，目前不用。
    unsigned short bandwidth;  //保留，目前不用。
      
    // FR_0 -> 0 , FR_1 -> 1  , FR_2 -> 2   电机控制顺序，目前只用12电机，后面保留。
    // FL_0 -> 3 , FL_1 -> 4  , FL_2 -> 5
    // RR_0 -> 6 , RR_1 -> 7  , RR_2 -> 8
    // RL_0 -> 9 , RL_1 -> 10 , RL_2 -> 11
    // FR_Wheel -> 12 , FL_Wheel -> 13 , RR_Wheel -> 14, RL_Wheel -> 15
    unitree_go::msg::dds_::MotorCmd_ motor_cmd[20];   //电机控制命令数据
    unitree_go::msg::dds_::BmsCmd_ bms_cmd;           //电池控制命令数据
      
    octet wireless_remote[40];  //保留，目前不用。
    octet led[12];              //已经改为内部控制，目前不用。
    octet fan[2];               //已经改为内部控制，目前不用。
    
    // &0xFE          自动充电打开              ,  |0x01       自动充电关闭
    // &0xFD          12个电机的电源开关打开    ,  |0x02       12个电机的电源开关关闭
    octet gpio;
      
    unsigned long reserve;  //保留位
    unsigned long crc;      //数据CRC校验用,为32crc校验用。
    
    };
    
    
    };  // module dds_
    
    };  // module msg
    
    };  // module unitree_go
    
    
    #endif  // __unitree_go__msg__low_cmd__idl__
```

###### **MotorCmd_.idl**

电机控制命令的实时信息，用于运动控制。

```C
#ifndef __unitree_go__msg__motor_cmd__idl__
#define __unitree_go__msg__motor_cmd__idl__


module unitree_go {

module msg {

module dds_ {


struct MotorCmd_ {
octet mode;  //电机控制模式（Foc模式（工作模式）-> 0x01 ，stop模式（待机模式）-> 0x00。）
float q;     //关节目标位置
float dq;    //关节目标速度
float tau;   //关节目标力矩
float kp;    //关节刚度系数
float kd;    //关节阻尼系数
unsigned long reserve[3];   //保留位


};


};  // module dds_

};  // module msg

};  // module unitree_go


#endif  // __unitree_go__msg__motor_cmd__idl__   
```

###### **BmsCmd_.idl**

关闭机身电池指令。

```C
// generated from rosidl_generator_dds_idl/resource/idl.idl.em
// with input from unitree_go:msg/BmsCmd.idl
// generated code does not contain a copyright notice

#ifndef __unitree_go__msg__bms_cmd__idl__
#define __unitree_go__msg__bms_cmd__idl__


module unitree_go {

module msg {

module dds_ {


struct BmsCmd_ {
octet off;         //关闭电池：（命令：0xA5）
octet reserve[3];  //保留


};


};  // module dds_

};  // module msg

};  // module unitree_go


#endif  // __unitree_go__msg__bms_cmd__idl__
```

##### 底层数据接收

用户可通过发布 DDS 话题 “**rt/lowState**” 来获取电机、电池、IMU、遥控器数据，数据格式如 **LowState_.idl** 所示。

- **电机状态信息**：共16 个电机，获取顺序可以按照如下进行，详情参考**MotorState_.idl** 。
- **IMU状态信息**：包含了三轴的加速度和角速度信息，四元数信息，欧拉角信息，温度信息，详情参考 **IMUState_.idl**。
- **电池状态信息**：包含了电池版本、状态信息、电池电量信息、充放电、循环次数、温度、单节电池电压。详情参考 **BmsState_.idl**。
- **foot_force**[4]：代表每条腿的足端力信息。顺序（0-FR，1-FL，2-RR, 3-RL）。
- **bit_flag**：各个组件的状态信息，用于实时检测各个组件的状态，（1.检测电机、小 MCU、遥控器、电池、运控命令是否超时。2.检测板载电流是否有错误标志，触发硬件电流保护就会产生错误标志。3.检测是否处于自动充电状态）。
- **fan_frequency**[4]：风扇转速和堵转检测（0-左后转速 , 1-右后转速，2-前转速，单位转/分钟）（堵转检测：3-&0x01：左后堵转 , &0x02：右后堵转，&0x04：前堵转）。
- **保护机制**：**adc_reel**->卷线器电流，**temperature_ntc1**->主板中心温度值，**temperature_ntc2**->自动充电温度，**power_v**->电池电压，**power_a**->电机电流。

```C
// generated from rosidl_generator_dds_idl/resource/idl.idl.em
// with input from unitree_go:msg/LowState.idl
// generated code does not contain a copyright notice
#include "BmsState_.idl"
#include "IMUState_.idl"
#include "MotorState_.idl"

#ifndef __unitree_go__msg__low_state__idl__
#define __unitree_go__msg__low_state__idl__


module unitree_go {

module msg {

module dds_ {


struct LowState_ {
octet head[2];               //帧头，数据校验用（0xFE,0xEF）。

octet level_flag;            //沿用的，但是目前不用。
octet frame_reserve;         //沿用的，但是目前不用。
unsigned long sn[2];         //已经改为文件存储形式，目前没用。
unsigned long version[2];    //沿用的，但是目前不用。
unsigned short bandwidth;    //沿用的，但是目前不用。。
  
unitree_go::msg::dds_::IMUState_ imu_state;           //IMU数据信息。

// FR_0 -> 0 , FR_1 -> 1  , FR_2 -> 2   电机顺序，目前只用16电机，后面保留。
// FL_0 -> 3 , FL_1 -> 4  , FL_2 -> 5
// RR_0 -> 6 , RR_1 -> 7  , RR_2 -> 8
// RL_0 -> 9 , RL_1 -> 10 , RL_2 -> 11
// FR_Wheel -> 12 , FL_Wheel -> 13 , RR_Wheel -> 14, RL_Wheel -> 15
unitree_go::msg::dds_::MotorState_ motor_state[20];   //电机总数据。
unitree_go::msg::dds_::BmsState_ bms_state;           //电池总数据。

short foot_force[4];        //足端力（范围0-4095），可按照实际数值显示。（数组：0-FR，1-FL，2-RR, 3-RL）
short foot_force_est[4];    //沿用的，但是目前不用。

unsigned long tick;         //1ms计时用，按照1ms递增。
octet wireless_remote[40];  //遥控器原始数据。

//&0x80 -  电机               超时标志          1-超时   0-正常
//&0x40 -  小Mcu              超时标志          1-超时   0-正常
//&0x20 -  遥控器             超时标志          1-超时   0-正常
//&0x10 -  电池               超时标志          1-超时   0-正常

//&0x04 -  自动充电           自动充电状态标志  1-不充电           0-充电
//&0x02 -  板载电流错误标志   错误标志          1-板载电流异常     0-正常
//&0x01 -  运控命令超时       超时标志          1-超时             0-正常
octet bit_flag;             //各个组件状态显示
  
float adc_reel;             //卷线器电流（范围：0 - 3A）。
octet temperature_ntc1;    	//主板中心温度值（范围：-20 - 100℃）。
octet temperature_ntc2;    	//自动充电温度（范围：-20 - 100℃）。
float power_v;      				//此电压值为主板电压 -> 电池电压 。
float power_a;      				//此电流值为主板电流值 -> 电机电流。
  
unsigned short fan_frequency[4];    //风扇转速（目前可按照实际数值显示0-10000）。（0-左后转速 , 1-右后转速，2-前转速，单位转/分钟）（堵转检测：3-&0x01：左后堵转 , &0x02：右后堵转，&0x04：前堵转）

unsigned long reserve;   //保留位。
unsigned long crc;       //数据CRC校验用。

};


};  // module dds_

};  // module msg

};  // module unitree_go


#endif  // __unitree_go__msg__low_state__idl__
```

###### IMUState_.idl

1. **quaternion**[4]：提供机身姿态信息之实时的四元数信息。（0-w, 1-x, 2-y, 3-z）。
2. **rpy**[3]：提供机身机身姿态信息之实时的欧拉角信息。（0-roll，1-pitch，2-yaw ）。
3. **gyroscope**[3]：提供机身机身姿态信息之实时的三轴角速度信息。（0-x, 1-y, 2-z）。
4. **accelerometer**[3]: 提供机身机身姿态信息之实时的三轴加速度信息。（0-x, 1-y, 2-z）。

```C
// generated from rosidl_generator_dds_idl/resource/idl.idl.em
// with input from unitree_go:msg/IMUState.idl
// generated code does not contain a copyright notice

#ifndef __unitree_go__msg__imu_state__idl__
#define __unitree_go__msg__imu_state__idl__


module unitree_go {

module msg {

module dds_ {


struct IMUState_ {
float quaternion[4];    //四元数数据

float gyroscope[3];     //角速度信息（0 -> x ,0 -> y ,0 -> z）

float accelerometer[3]; //加速度信息（0 -> x ,0 -> y ,0 -> z）

float rpy[3];           //欧拉角信息：默认为弧度值（可按照实际情况改为角度值），可按照实际数值显示（弧度值范围：-7 - +7，显示3位小数）。（数组：0-roll（翻滚角），1-pitch（俯仰角），2-yaw（偏航角））。

octet temperature;      //IMU 温度信息（摄氏度）。

};


};  // module dds_

};  // module msg

};  // module unitree_go


#endif  // __unitree_go__msg__imu_state__idl__
```

###### **MotorState_.idl**

电机反馈的实时信息，用于运动控制。

```C
// generated from rosidl_generator_dds_idl/resource/idl.idl.em
// with input from unitree_go:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef __unitree_go__msg__motor_state__idl__
#define __unitree_go__msg__motor_state__idl__


module unitree_go {

module msg {

module dds_ {


struct MotorState_ {
octet mode;     //电机控制模式（Foc模式（工作模式）-> 0x01 ，stop模式（待机模式）-> 0x00。）
float q;        //关节反馈位置信息：默认为弧度值（可按照实际情况改为角度值），可按照实际数值显示（弧度值范围：-7 - +7，显示3位小数）。
float dq;       //关节反馈速度
float ddq;      //关节反馈加速度
float tau_est;  //关节反馈力矩
  
float q_raw;    //沿用的，但是目前不用。
float dq_raw;   //沿用的，但是目前不用。
float ddq_raw;  //沿用的，但是目前不用。
octet temperature;          //电机温度信息：类型：int8_t ，可按照实际数值显示（范围：-100 - 150）。
unsigned long lost;         //电机丢包信息：可按照实际数值显示（范围：0-9999999999）。
unsigned long reserve[2];   //当前电机通信频率+电机错误标志位：（数组：0-电机错误标志位（范围：0-255，可按照实际数值显示），1-当前电机通信频率（范围：0-800，可按照实际数值显示））


};


};  // module dds_

};  // module msg

};  // module unitree_go


#endif  // __unitree_go__msg__motor_state__idl__
```

###### **BmsState_.idl**

- **version_high** 和 **version_low**： 组成电池版本信息。
- **status**：显示未开启电池、唤醒事件、电池预充电中、电池正常充电中、电池正常放电中、电池自放电中、电池存在警告、等待按键复位警告中、复位中这些状态。
- **soc**：电池电量信息（1% - 100%）。
- **current**：充放电信息：
  - 正：代表充电，
  - 负：代表放电
- **cycle**：充电循环次数。
- **bq_ntc**[2]：电池内部两个NTC的温度
  - 0- BAT1
  - 1- BAT2
- **mcu_ntc**[2]：电池NTC数组
  - 0 - RES
  - 1 - MOS
- **cell_vol**[15]：单节电压。

```C
// generated from rosidl_generator_dds_idl/resource/idl.idl.em
// with input from unitree_go:msg/BmsState.idl
// generated code does not contain a copyright notice

#ifndef __unitree_go__msg__bms_state__idl__
#define __unitree_go__msg__bms_state__idl__


module unitree_go {

module msg {

module dds_ {


struct BmsState_ {
octet version_high;    //电池版本
octet version_low;     //电池版本

// 0：SAFE,（未开启电池）
// 1：WAKE_UP,（唤醒事件）

// 6：PRECHG, （电池预冲电中）
// 7：CHG, （电池正常充电中）
// 8：DCHG, （电池正常放电中）
// 9：SELF_DCHG, （电池自放电中）

// 11：ALARM, （电池存在警告）
// 12：RESET_ALARM, （等待按键复位警告中）
// 13：AUTO_RECOVERY （复位中）
octet status;          //电池状态信息。
  
octet soc;             //电池电量信息：（类型：uint8_t）(范围1% - 100%)
long current;          //充放电信息：（正：代表充电，负代表放电）可按照实际数值显示
unsigned short cycle;  //充电循环次数
octet bq_ntc[2];       //电池内部两个NTC的温度（int8_t）（范围：-100 - 150）。  0- BAT1; 1- BAT2

octet mcu_ntc[2];      //电池NTC数组：0 - RES，1 - MOS （int8_t）（范围：-100 - 150）。

unsigned short cell_vol[15];      //电池内部15节电池的电压。


};


};  // module dds_

};  // module msg

};  // module unitree_go


#endif  // __unitree_go__msg__bms_state__idl__
```

### 4. 高层运动服务接口

更新时间：2026-04-08 15:03:29

#### 高层控制接口

##### 高层控制接口的调用方式

Go2w 的高层接口与Go2 兼容，可通过调用 Go2 的 sport_client，来给 Go2w 发送速度控制等运动指令。

```C++
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unistd.h>

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  //argv[1]由终端传入，为机器人连接的网卡名称
  
  //创建sport client对象
  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f);//超时时间
  sport_client.Init();


  sport_client.StandUp(); //站立
  sleep(3);//延迟3s
  sport_client.StandDown(); //趴下
  sleep(3);

  return 0;
}
```

##### 高层运动控制接口介绍

|  **函数名**  | **Damp**                                                     |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | int32_t Damp()                                               |
| **功能概述** | 进入阻尼状态。                                               |
|   **参数**   | 无                                                           |
|  **返回值**  | 调用成功返回  0，否则返回相关错误码。                        |
|   **备注**   | 所有电机关节停止运动并进入阻尼状态。该模式具有最高的优先级，用于突发情况下的急停。 |

|  **函数名**  | **BalanceStand**                                             |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | int32_t BalanceStand()                                       |
| **功能概述** | 解除锁定。                                                   |
|   **参数**   | 无                                                           |
|  **返回值**  | 调用成功返回  0，否则返回相关错误码。                        |
|   **备注**   | 解除关节电机锁定，从正常站立模式切换到平衡站立模式。这一模式下机身姿态和高度会始终保持平衡，不受地形的影响。 |

|  **函数名**  | **StopMove**                                           |
| :----------: | ------------------------------------------------------ |
| **函数原型** | int32_t StopMove()                                     |
| **功能概述** | 停下当前动作，将绝大多数指令恢复成默认值。             |
|   **参数**   | 无                                                     |
|  **返回值**  | 调用成功返回  0，否则返回相关错误码。                  |
|   **备注**   | 停下当前的运动，并将机器人内部的运动参数恢复到默认值。 |

|  **函数名**  | **StandUp**                                                  |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | int32_t StandUp()                                            |
| **功能概述** | 关节锁定，站高。                                             |
|   **参数**   | 无                                                           |
|  **返回值**  | 调用成功返回  0，否则返回相关错误码。                        |
|   **备注**   | 机器狗正常站高，电机关节保持锁定。相比于平衡站立模式，该模式下机器狗的姿态不会始终保持平衡。默认的站立高度为 0.44m 。如果机器人处在较陡的斜坡上，该接口不会响应。 `注意：锁定步态容易导致过热，请谨慎使用` |

|  **函数名**  | **StandDown**                                                |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | int32_t StandDown()                                          |
| **功能概述** | 关节锁定，站低。                                             |
|   **参数**   | 无                                                           |
|  **返回值**  | 调用成功返回  0，否则返回相关错误码。                        |
|   **备注**   | 机器狗趴下，趴下后电机会进入待机状态。为了安全，机器人需处在站立锁定或阻尼状态下，该接口才会响应 |

|  **函数名**  | **RecoveryStand**                                            |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | int32_t RecoveryStand()                                      |
| **功能概述** | 恢复站立。                                                   |
|   **参数**   | 无                                                           |
|  **返回值**  | 调用成功返回  0，否则返回相关错误码。                        |
|   **备注**   | 从翻倒或趴下状态恢复至平衡站立状态。不论是否翻倒，都会恢复至站立。机器人需要处于阻尼或趴下状态该接口才会响应 |

|  **函数名**  | **Move**                                                     |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | int32_t Move(float vx, float vy, float vyaw)                 |
| **功能概述** | 移动。                                                       |
|   **参数**   | **单位 (m/s)** 默认模式：    **vx:** 默认档位 [-1.5~1.5], 高速档位 [-2.5, 2.5]     **vy:** [-0.6~0.6]     **vyaw:** [-1.0~1.0] 爬楼梯模式：     **vx:** [-0.7~0.7]    **vy:** [-0.5~0.5]     **vyaw:** [-1.0~1.0] 爬高模式：     **vx:** [-0.5~0.5]    **vy:** [-0.4~0.4]    **vyaw:** [-0.6~0.6] |
|  **返回值**  | 调用成功返回  0，否则返回相关错误码。                        |
|   **备注**   | 控制移动速度。设定的速度为机体坐标系表示下的速度。           |

|  **函数名**  | **SwitchGait**                                               |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | int32_t SwitchGait(int d)                                    |
| **功能概述** | 切换步态。                                                   |
|   **参数**   | **d：** 步态枚举值，取值  0~2，0 为默认模式， 1 为爬楼梯模式 (地形行走模式)，2 为 爬高模式 |
|  **返回值**  | 调用成功返回  0，否则返回相关错误码。                        |
|   **备注**   | **此接口暂不提供!**  默认模式：速度较高，具备不高于5cm的地形通过能力，但不支持较复杂的地形。 爬楼梯模式(地形行走)：具备楼梯等复杂地形的通过能力，`设备若有负载，建议使用此步态爬楼梯，默认步态爬楼梯和负载性能较差`。 攀爬模式：能够翻越高度不高于70 cm，宽度不低于15 cm 的规则高台或墙面 |

|  **函数名**  | **SpeedLevel**                                               |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | int32_t SpeedLevel(int level)                                |
| **功能概述** | 设置速度档位。                                               |
|   **参数**   | **level**：速度档位枚举值，取值  -1 为慢速，0 为正常，1 为快速。 |
|  **返回值**  | 调用成功返回  0，否则返回相关错误码。                        |
|   **备注**   | 速度档位仅在默认模式下生效                                   |

|  **函数名**  | **GetState**                                                 |
| :----------: | ------------------------------------------------------------ |
| **函数原型** | int32_t GetState(const std::vector[std::string](std::string)& _vector, std::map<std::string, std::string>& _map); |
| **功能概述** | 获取机器人当前状态，包括： speedLevel：速度档位 gait 步态类型 |
|   **参数**   | **_vector**：机器人状态名称的vector引用；**_map**： 状态名和对应的值的map引用。 |
|  **返回值**  | 调用成功返回 0，否则返回相关错误码。                         |
|   **备注**   | state_map["speedLevel"] 速度档位：   0: 默认速度   1: 高速 state_map["gait"] 步态类型：   0: 默认模式   1: 爬楼梯模式   2: 爬高模式   3: 倒立模式  -1: 站立锁定状态  -2: 趴下状态  -3: 阻尼状态 具体使用方法可参考Go2w的高层控制例程 |

### 6. ROS2服务接口

更新时间：2024-09-02 15:33:31

#### 说明

**unitree_sdk2** 基于 cyclonedds 实现了一个易用的机器人数据通信机制，应用开发者可以利用这一接口实现机器人的数据通讯和指令控制(**支持Go2、Go2-W、B2和H1**)。 https://github.com/unitreerobotics/unitree_sdk2
ROS2 也使用 DDS 作为通讯工具，因此 Go2、Go2-W、B2 和 H1 机器人的底层可以兼容 ROS2，使用 ROS2 自带的 msg 直接进行通讯和控制，而无需通过sdk接口转发。

#### 环境配置

##### 系统要求

测试过的系统和ROS2版本

| 系统         | ROS2 版本 |
| ------------ | --------- |
| Ubuntu 20.04 | foxy      |
| Ubuntu 22.04 | humble    |

下文以 ROS2 foxy 为例，如需使用其他版本的 ROS2，在相应的地方替换 foxy 为当前的 ROS2 版本名称即可。

note

ROS2 不同发行版的 API 可能存在差异，例如 rosbag 的调用方法等，仓库中的例程在 ROS2 foxy 下开发，如果使用的是其他 ROS2 发行版，请参考官方文档进行调整。

ROS2 foxy 的安装可参考: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

`ctrl+alt+T` 打开终端，克隆仓库：https://github.com/unitreerobotics/unitree_ros2

```bash
git clone https://github.com/unitreerobotics/unitree_ros2
```

其中

- **cyclonedds_ws** 文件夹为编译和安装 机器人 ROS2 msg 的工作空间，在子文件夹 `cyclonedds_ws/unitree/unitree_go`和`cyclonedds_ws/unitree/unitree_api`中定义了机器人状态获取和控制相关的 ROS2 msg。

##### 安装 Unitree ROS2 功能包

###### 1. 安装依赖

```bash
sudo apt install ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-rosidl-generator-dds-idl
```

note

为了方便接口的使用，推荐同时安装 [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)

###### 2. 编译 cyclone-dds

由于 Go2-W 使用的是 **cyclonedds 0.10.2** 版本，因此需要先更改 ROS2 的 DDS 实现。详见：https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html

编译 cyclonedds 前请确保在启动终端时**没有** source ros2 相关的环境变量，否则会导致 cyclonedds 编译报错。如果安装 ROS2 时在~/.bashrc中添加了 " `source /opt/ros/foxy/setup.bash` "，需要修改 ~/.bashrc 文件将其删除：

```bash
sudo apt install gedit
sudo gedit ~/.bashrc
```

在弹出的窗口中，注释掉 ROS2 相关的环境变量，例如：

```bash
# source /opt/ros/foxy/setup.bash 
```

在终端中执行以下操作编译 cyclone-dds

```bash
cd ~/unitree_ros2/cyclonedds_ws/src
#克隆cyclonedds仓库
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds #编译cyclonedds
```

###### 3. 编译 unitree_go 和 unitree_api 功能包

编译好 cyclone-dds 后就需要 Ros2 相关的依赖来完成 Go2-W 功能包的编译，因此编译前需要先 source ROS2 的环境变量。

```bash
source /opt/ros/foxy/setup.bash #source ROS2 环境变量
colcon build #编译工作空间下的所有功能包
```

##### 连接到 Go2-W

###### 1. 配置网络

使用网线连接 Go2-W 和计算机，使用 **ifconfig** 查看网络信息，确认机器人连接到的以太网网卡。（例如如图中的enp3s0，以实际为准）
![image](https://doc-cdn.unitree.com/static/2023/8/9/809e422d82334e5d82160cafc694c96a_1450x830.png)

接着打开网络设置，找到机器人所连接的网卡，进入 IPv4 ，将 IPv4 方式改为手动，地址设置为192.168.123.99，子网掩码设置为255.255.255.0，完成后点击应用，等待网络重新连接。
![image](https://doc-cdn.unitree.com/static/2023/8/9/466aea1654c9421595f38243dc5cea6b_1946x1620.png)

打开 setup.sh 文件

```bash
sudo gedit ~/unitree_ros2/setup.sh
```

bash 的内容如下：

```bash
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/foxy/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp3s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

其中 "enp3s0" 为 Go2-W 所连接的网卡名称，根据实际情况修改为对应的网卡名称。在终端中执行：

```bash
source ~/unitree_ros2/setup.sh
```

即可完成 Go2-W 开发环境的设置。如果不希望每次打开新终端都执行一次 bash 脚本，也可将setup.sh 中的内容写入到 ~/.bashrc中，但是当系统有多个 Ros 环境共存需要注意。

**补充说明**：如果电脑没有连接到机器人，但仍希望能使用 unitree ros2 实现仿真等功能， 可以使用本地回环 "lo" 作为网卡:

```bash
source ~/unitree_ros2/setup_local.sh # 使用 "lo" 作为网卡
```

或:

```bash
source ~/unitree_ros2/setup_default.sh # 不指定网卡
```

###### 2. 连接测试

完成上述配置后，建议重启一下电脑再进行测试。
确保机器人连接正确，打开终端输入 `ros2 topic list`，可以看见如下话题：
![image](https://doc-cdn.unitree.com/static/2023/8/9/8ba3181e1f1446d784e8e40b21968db3_610x640.png)

打开终端输入 `ros2 topic echo /sportmodestate` 后，可以看见该话题的数据如下图所示，说明机器人与电脑已经正常通讯：
![image](https://doc-cdn.unitree.com/static/2023/8/9/5c3e505ff5724c14a4002d27caf41e4d_462x951.png)

###### 3. 编译和测试例程

`ctrl+alt+T` 打开终端，在终端中执行如下命令，编译测试例程：

```bash
source ~/unitree_ros2/setup.sh
cd ~/unitree_ros2/example
colcon build
```

编译完成后在终端中运行:

```bash
./install/unitree_ros2_example/bin/read_motion_state
```

可以看到终端中输出的机器人状态信息：

```bash
[INFO] [1697525196.266174885] [motion_state_suber]: Position -- x: 0.567083; y: 0.213920; z: 0.052338; body height: 0.320000
[INFO] [1697525196.266230044] [motion_state_suber]: Velocity -- vx: -0.008966; vy: -0.001431; vz: -0.019455; yaw: -0.002131
[INFO] [1697525196.266282725] [motion_state_suber]: Foot position and velcity relative to body -- num: 0; x: 0.204149; y: -0.145194; z: -0.067804, vx: 0.002683; vy: 0.003745; vz: -0.010052
[INFO] [1697525196.266339057] [motion_state_suber]: Foot position and velcity relative to body -- num: 1; x: 0.204200; y: 0.145049; z: -0.068205, vx: -0.001954; vy: -0.003442; vz: -0.004828
[INFO] [1697525196.266392028] [motion_state_suber]: Foot position and velcity relative to body -- num: 2; x: -0.183385; y: -0.159294; z: -0.039468, vx: -0.000739; vy: -0.002028; vz: -0.004532
[INFO] [1697525196.266442766] [motion_state_suber]: Foot position and velcity relative to body -- num: 3; x: -0.182412; y: 0.159754; z: -0.039045, vx: -0.002803; vy: -0.001381; vz: -0.004794
[INFO] [1697525196.316189064] [motion_state_suber]: Gait state -- gait type: 1; raise height: 0.090000
```

#### 例程和使用

Go2-W 机器人底层采用与 ROS2 兼容的 DDS 通信方式，当安装和配置好 Unitree Go2-W ROS2 环境后，可以通过订阅 ROS2 的 topic 实现机器人状态的获取和指令控制。

##### 状态获取

###### 1. 高层状态获取

高层状态为机器人的速度、位置、足端位置等与运动相关的状态。高层状态的获取可通过订阅"`lf/sportmodestate`"或"`sportmodestate`" topic 实现，其中"`lf`"表示低频率。高层状态的 msg 定义如下：

```C++
TimeSpec stamp //时间戳
uint32 error_code //错误代码
IMUState imu_state //IMU状态
uint8 mode //运动模式
/*
运动模式
0. idle, default stand
1. balanceStand
2. pose
3. locomotion
4. reserve
5. lieDown
6. jointLock
7. damping
8. recoveryStand
9. reserve
10. sit
11. frontFlip
12. frontJump
13. frontPounc
*/
float32 progress //是否动作执行状态：0. dance false; 1. dance true
uint8 gait_type //步态类型
/*
步态类型
0.idle  
1.trot  
2.run  
3.climb stair  
4.forwardDownStair   
9.adjust
*/
float32 foot_raise_height //抬腿高度
float32[3] position //当前位置
float32 body_height //机体高度
float32[3] velocity //线速度
float32 yaw_speed //偏行速度
float32[4] range_obstacle //障碍物范围 
int16[4] foot_force //足端力数值
float32[12] foot_position_body //足端相对于机体的位置
float32[12] foot_speed_body //足端相对于机体的速度
```

高层状态信息的具体解释可参考：https://support.unitree.com/home/zh/developer/sports_services

读取高层状态的完整例程位于 `/example/src/read_motion_state.cpp`
编译完例程后，在终端中运行`./install/unitree_ros2_example/bin/read_motion_state`，可查看运行结果。

###### 2. 低层状态获取

低层状态为机器人的关节电机、电源信息等底层状态。通过订阅"`lf/lowstate`"或"`lowstate`" topic，可实现低层状态的获取。低层状态的 msg 定义如下：

```C++
uint8[2] head
uint8 level_flag
uint8 frame_reserve
uint32[2] sn
uint32[2] version
uint16 bandwidth
IMUState imu_state //IMU状态
MotorState[20] motor_state //电机状态
BmsState bms_state
int16[4] foot_force //足端力数值
int16[4] foot_force_est //估计的足端力
uint32 tick
uint8[40] wireless_remote
uint8 bit_flag
float32 adc_reel
int8 temperature_ntc1
int8 temperature_ntc2
float32 power_v //电池电压
float32 power_a //电池电流
uint16[4] fan_frequency 
uint32 reserve
uint32 crc
```

其中 MotorState 为关节电机的状态信息，其定义如下：

```C++
uint8 mode //运动模式
float32 q //当前角度
float32 dq //当前角速度
float32 ddq //当前角加速度
float32 tau_est //估计的外力
float32 q_raw //当前角度原始数值
float32 dq_raw //当前角速度原始数值
float32 ddq_raw //当前角加速度原始数值
int8 temperature //温度
uint32 lost
uint32[2] reserve
```

低层状态信息的具体解释可参考: https://support.unitree.com/home/zh/developer/Basic_services
读取低层状态的完整例程序位于：`example/src/read_low_state.cpp`
在终端中运行`./install/unitree_ros2_example/bin/read_low_state`，可查看低层状态获取例程的运行结果。

###### 3. 遥控器状态获取

通过订阅"`/wirelesscontroller`" topic可获取遥控器的摇杆数值和按键键值。遥控器状态的msg定义如下

```C++
float32 lx //左边摇杆x
float32 ly //左边摇杆y
float32 rx //右边摇杆x
float32 ry //右边摇杆y
uint16 keys //键值
```

遥控器状态和遥控器键值的相关定义可参考：https://support.unitree.com/home/zh/developer/Get_remote_control_status

读取遥控器状态的完整例程序见：`example/src/read_wireless_controller.cpp`
在终端中运行`./install/unitree_ros2_example/bin/read_low_state`，可查看遥控器状态获取例程的运行结果。

##### 机器人控制

###### 1. 运动控制

Go2-W 机器人的运动指令是通过请求响应的方式实现的，通过订阅"`/api/sport/request`"，并发送运动 unitree_api::msg::Request 消息可以实现高层的运动控制。其中不同运动接口的 Request 消息可调用 SportClient(位于`/example/src/common/ros2_sport_client.cpp`)类来获取，例如实现 Go2-W 的姿态控制：

```C++
//创建一个ros2 pubilsher
rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

SportClient sport_req;//实例化一个sportclient
unitree_api::msg::Request req; //创建一个运动请求msg
sport_req.Euler(req,roll,pitch,yaw); //获取欧拉角运动请求消息，并赋值给req

req_puber->publish(req); //发布数据
```

关于SportClient运动控制接口的具体解释可参考：https://support.unitree.com/home/zh/developer/sports_services

高层运动控制的完整例程位于：`example/src/sport_mode_ctrl.cpp`
在终端中运行`./install/unitree_ros2_example/bin/sport_mode_ctrl`，等待 1s 后，机器人会沿着 X 方向来回走动。

###### 2. 电机控制

通过订阅"`/lowcmd`" topic，并发送`unitree_go::msg::LowCmd`可以实现对电机的力矩、位置、和速度控制。底层控制指令的 msg 定义如下:

```C++
uint8[2] head
uint8 level_flag
uint8 frame_reserve
uint32[2] sn
uint32[2] version
uint16 bandwidth
MotorCmd[20] motor_cmd //电机指令
BmsCmd bms_cmd
uint8[40] wireless_remote
uint8[12] led
uint8[2] fan
uint8 gpio
uint32 reserve
uint32 crc
```

其中 motor_cmd 为电机指令:

```C++
uint8 mode;  //电机控制模式（Foc模式（工作模式）-> 0x01 ，stop模式（待机模式）-> 0x00
float q;     //关节目标位置
float dq;    //关节目标速度
float tau;   //关节目标力矩
float kp;    //关节刚度系数
float kd;    //关节阻尼系数
unsigned long reserve[3];   //保留位
```

低层指令的具体解释可参考：https://support.unitree.com/home/zh/developer/Basic_services

电机控制的完整例程见 `example/src/low_level_ctrl.cpp`，编译后在终端执行`./install/unitree_ros2_example/bin/low_level_ctrl`，左后腿的机身电机和小腿电机会转动到对应关节角度。

##### Rviz 可视化

由于 Go2-W 机器人底层兼容了 ROS2 的 topic 机制，因此可以使用 rviz 工具来可视化 Go2-W 机器人的状态信息。下面以查看机器人的点云数据为例：
首先列出所有 topic：

```bash
ros2 topic list
```

![image](https://z1.ax1x.com/2023/10/20/piFtteJ.png)

可以找到雷达点云的 topic：

```bash
utlidar/cloud
```

接着查看点云的 frame_id：

```
ros2 topic echo --no-arr /utlidar/cloud
```

可以看到点云数据的frame_id为utlidar_lidar
![image](https://z1.ax1x.com/2023/10/20/piFtdF1.png)

最后打开 rviz2：

```
ros2 run rviz2 rviz2
```

在 rviz2 添加 Go2 点云 topic: utlidar/cloud。修改 world_frame 为 utlidar_lidar 即可看到雷达输出的点云。

![image](https://z1.ax1x.com/2023/10/20/piFtsyD.png)
![image](https://z1.ax1x.com/2023/10/20/piFtyOe.png)



### 7. Python 服务接口

更新时间：2024-09-03 12:28:23

#### unitree_sdk2_python

unitree_sdk2 python 接口

#### 安装

##### 依赖

- python>=3.8
- cyclonedds==0.10.2
- numpy
- opencv-python

##### 安装 unitree_sdk2_python

在终端中执行：

```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```

##### FAQ

######## 1. `pip3 install -e .` 遇到报错

```bash
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```

该错误提示找不到 cyclonedds 路径。首先编译安装cyclonedds：

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_DDSPERF=OFF
cmake --build . --target install
```

进入 unitree_sdk2_python 目录，设置 `CYCLONEDDS_HOME` 为刚刚编译好的 cyclonedds 所在路径，再安装 unitree_sdk2_python

```bash
cd ~/unitree_sdk2_python
export CYCLONEDDS_HOME="/{path to your cyclonedds}/cyclonedds/install"
pip3 install -e .
```

详细见：
https://pypi.org/project/cyclonedds/#installing-with-pre-built-binaries

#### 使用

python sdk2 接口与 unitree_skd2的接口保持一致，通过请求响应或订阅发布topic实现机器人的状态获取和控制。相应的例程位于`/example`目录下。在运行例程前，需要根据文档 [《快速开始》](https://support.unitree.com/home/zh/developer/Quick_start) 配置好机器人的网络连接。

##### DDS通讯

在终端中执行：

```bash
python3 ./example/helloworld/publisher.py
```

打开新的终端，执行：

```bash
python3 ./example/helloworld/subscriber.py
```

可以看到终端输出的数据信息。`publisher.py` 和 `subscriber.py` 传输的数据定义在 `user_data.py` 中，用户可以根据需要自行定义需要传输的数据结构。

##### 高层状态和控制

高层接口的数据结构和控制方式与unitree_sdk2一致。具体可见：https://support.unitree.com/home/zh/developer/sports_services

###### 高层状态

终端中执行：

```bash
python3 ./example/high_level/read_highstate.py enp2s0
```

其中 `enp2s0` 为机器人所连接的网卡名称，请根据实际情况修改。

###### 高层控制

终端中执行：

```bash
python3 ./example/high_level/sportmode_test.py enp2s0
```

其中 `enp2s0` 为机器人所连接的网卡名称，请根据实际情况修改。
该例程提供了几种测试方法，可根据测试需要选择:

```python
test.StandUpDown() # 站立趴下
# test.VelocityMove() # 速度控制
# test.BalanceAttitude() # 姿态控制
# test.TrajectoryFollow() # 轨迹跟踪
# test.SpecialMotions() # 特殊动作
```

##### 底层状态和控制

底层接口的数据结构和控制方式与unitree_sdk2一致。具体可见：https://support.unitree.com/home/zh/developer/Basic_services

###### 底层状态

终端中执行：

```bash
python3 ./example/low_level/lowlevel_control.py enp2s0
```

其中 `enp2s0` 为机器人所连接的网卡名称，请根据实际情况修改。程序会输出右前腿hip关节的状态、IMU和电池电压信息。

###### 底层电机控制

首先使用 app 关闭高层运动服务(sport_mode)，否则会导致指令冲突。
终端中执行：

```bash
python3 ./example/low_level/lowlevel_control.py enp2s0
```

其中 `enp2s0` 为机器人所连接的网卡名称，请根据实际情况修改。左后腿 hip 关节会保持在0角度 (安全起见，这里设置 kp=10, kd=1)，左后腿 calf 关节将持续输出 1Nm 的转矩。

##### 遥控器状态获取

终端中执行：

```bash
python3 ./example/wireless_controller/wireless_controller.py enp2s0
```

其中 `enp2s0` 为机器人所连接的网卡名称，请根据实际情况修改。
终端将输出每一个按键的状态。对于遥控器按键的定义和数据结构可见： https://support.unitree.com/home/zh/developer/Get_remote_control_status

##### 前置摄像头

使用opencv获取前置摄像头(确保在有图形界面的系统下运行, 按 ESC 退出程序):

```bash
python3 ./example/front_camera/camera_opencv.py enp2s0
```

其中 `enp2s0` 为机器人所连接的网卡名称，请根据实际情况修改。

##### 灯光音量控制

```bash
python3 ./example/vui_client/vui_client_example.py enp2s0
```

其中 `enp2s0` 为机器人所连接的网卡名称，请根据实际情况修改。机器人将循环调节音量和灯光亮度。该接口详细见 https://support.unitree.com/home/zh/developer/VuiClient

## 例程参考

### 1. DDS 通信例程

更新时间：2024-09-02 16:03:16

例程路径： `unitree_sdk2/example/helloworld/publisher.cpp`

```C++
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>                                  
#include "HelloWorldData.hpp"

#define TOPIC "TopicHelloWorld"

using namespace unitree::robot;
using namespace unitree::common;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
      std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
      exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    //argv[1]由终端传入，为机器人连接的网卡名称

    /*
     * New ChannelPublisherPtr
     */
    ChannelPublisherPtr<HelloWorldData::Msg> publisher = ChannelPublisherPtr<HelloWorldData::Msg>(new ChannelPublisher<HelloWorldData::Msg>(TOPIC));

    /*
     * Init channel
     */
    publisher->InitChannel();

    while (true)
    {
        /*
         * Send message
         */
        HelloWorldData::Msg msg(unitree::common::GetCurrentTimeMillisecond(), "HelloWorld.");
        publisher->Write(msg);
        sleep(1);
    }   

    return 0;
}
```

例程路径： `unitree_sdk2/example/helloworld/subscriber.cpp`

```C++
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "HelloWorldData.hpp"

#define TOPIC "TopicHelloWorld"

using namespace unitree::robot;
using namespace unitree::common;

void Handler(const void* msg)
{
    const HelloWorldData::Msg* pm = (const HelloWorldData::Msg*)msg;

    std::cout << "userID:" << pm->userID() << ", message:" << pm->message() << std::endl;
}

int main()
{
    /*
     * Init with defalue domain 0.
     */
    ChannelFactory::Instance()->Init(0);
  
    /*
     * New ChannelSubscriberPtr
     */
    ChannelSubscriberPtr<HelloWorldData::Msg> subscriber = ChannelSubscriberPtr<HelloWorldData::Msg>(new ChannelSubscriber<HelloWorldData::Msg>(TOPIC));

    /*
     * Init channel
     */
    subscriber->InitChannel(std::bind(Handler, std::placeholders::_1), 1);

    sleep(5);
  
    /*
     * Close channel
     */
    subscriber->CloseChannel();

    std::cout << "reseted. sleep 3" << std::endl;

    sleep(3);

    /*
     * Init channel use last input parameter.
     */
    subscriber->InitChannel();

    /*
     * Loop wait message.
     */
    while (true)
    {
        sleep(10);
    }

    return 0;
}
```

例程路径： `unitree_sdk2/example/client/sport_client_example.cpp`

```C++
#include <unitree/robot/go2/sport/sport_client.hpp>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
      std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
      exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    //argv[1]由终端传入，为机器人连接的网卡名称
  
    unitree::robot::go2::SportClient sc;

    /*
     * Set request timeout 1.0s
     */
    sc.SetTimeout(1.0f);
    sc.Init();

    //Test Api

    while (true)
    {
        int32_t ret = sc.Move(0.5, 0.0, 0.0);

        std::cout << "Call sc.Move ret:" << ret << std::endl;

        usleep(1000);
    }

    return 0;
}
```

### 2. 高层运动控制例程

更新时间：2024-09-25 17:17:18

本文介绍如何使用 Unitree SDK2 来调用 Go2w 的高层控制接口。

#### 高层控制例程运行测试

高层的例程文件位于 `unitree_sdk2/example/go2w/go2w_sport_client.cpp`。编译运行：

```bash
./go2w_sport_test [机器人网卡名]
```

会提示输入list来获取测试选项

```bash
Input "list " to list all test option ...
```

输入 list 并回车后会提示所有的测试选项

```bash
list
damp, id: 0
stand_up, id: 1
stand_down, id: 2
move, id: 3
stop_move, id: 4
speed_level, id: 5
switch_gait, id: 6
get_state, id: 7
recovery, id: 8
balance, id: 9
```

根据提示输入对应的字符串或id并回车即可实现对应功能的测试，例如输入 damp 或 0 回车，机器人就会进入阻尼状态。需要注意的是，每一次输入只会进行一次请求。

#### 高层控制例程解析

该例程序首先对测试用的接口进行枚举：

```C++
// 测试选项
struct TestOption
{
    std::string name; // 名称
    int id;           // id
};

const vector<TestOption> option_list =
    {{"damp", 0},        // 阻尼
     {"stand_up", 1},    // 站立锁定
     {"stand_down", 2},  // 趴下
     {"move", 3},        // 速度控制
     {"stop_move", 4},   // 停止运动
     {"speed_level", 5}, // 速度档位
     {"switch_gait", 6}, // 步态切换
     {"get_state", 7},   // 获取状态
     {"recovery", 8},    // 恢复站立
     {"balance", 9}};    // 平衡站立
```

接着实现了一个捕获终端输入的类，根据终端输入的内容确定待测试的功能

```C++
class UserInterface
{
public:
    UserInterface(){};
    ~UserInterface(){};

    void terminalHandle()
    {
        std::string input;
        std::getline(std::cin, input);

        // 如果输入的是 list 则输出所有的测试选项名称和id
        if (input.compare("list") == 0)
        {
            for (TestOption option : option_list)
            {
                std::cout << option.name << ", id: " << option.id << std::endl;
            }
        }

        // 如果输入的名名称或id在枚举的测试选项中，则记录测试选项名称和id
        for (TestOption option : option_list)
        {
            if (input.compare(option.name) == 0 || ConvertToInt(input) == option.id)
            {
                test_option_->id = option.id;
                test_option_->name = option.name;
                std::cout << "Test: " << test_option_->name << ", test_id: " << test_option_->id << std::endl;
            }
        }
    };

    // 待测试的功能选项
    TestOption *test_option_;
};
```

在主函数中

```C++
int main(int argc, char **argv)
{   
    // 初始化 dds
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);


    // 待测试的选项
    TestOption test_option;
    test_option.id = 1;

    // 初始化sportclient
    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(20.0f);
    sport_client.Init();

    // 初始化用户终端
    UserInterface user_interface;
    // 将user_interface.test_option_ 指向 test_option，用于传递终端输入的结果
    user_interface.test_option_ = &test_option; 

    std::cout << "Input \"list \" to list all test option ..." << std::endl;
    long res_count = 0;
    while (1)
    {
        auto time_start_trick = std::chrono::high_resolution_clock::now();
        static const constexpr auto dt = std::chrono::microseconds(20000); // 50Hz
        
        // 等待终端输入，并解析
        user_interface.terminalHandle();

        int res = 1;
        if (test_option.id == 0)
        {
            res = sport_client.Damp();
        }
        else if (test_option.id == 1)
        {
            res = sport_client.StandUp();
        }
        else if (test_option.id == 2)
        {
            res = sport_client.StandDown();
        }
        else if (test_option.id == 3)
        {   
            // 此处为调用一次 Move 指令，机器人会移动1s后停下
            // 如果循环调用 Move 则可以持续移动，1s 中内无Move请求，机器人会自动停下。
            res = sport_client.Move(0.5, 0, 0);
        }
        else if (test_option.id == 4)
        {
            res = sport_client.StopMove();
        }
        else if (test_option.id == 5)
        {
            res = sport_client.SpeedLevel(1);
        }
        else if (test_option.id == 6)
        {
            res = sport_client.SwitchGait(1);
        }
        else if (test_option.id == 7)
        {
            std::map<std::string, std::string> state_map;
            std::vector<std::string> state_name = {"speedLevel", "gait "};
            res = sport_client.GetState(state_name, state_map);
            std::cout << "Speed level: " << state_map["speedLevel"] << ", Gait: " << state_map["gait"] << std::endl;
        }
        else if (test_option.id == 8)
        {
            res = sport_client.RecoveryStand();
        }
        else if (test_option.id == 9)
        {
            res = sport_client.BalanceStand();
        }

        // 输出请求结果
        if (res < 0)
        {
            res_count += 1;
            std::cout << "Request error for: " << option_list[test_option.id].name << ", code: " << res << ", count: " << res_count << std::endl;
        }
        else
        {
            res_count = 0;
            std::cout << "Request successed: " << option_list[test_option.id].name << ", code: " << res << std::endl;
        }
        std::this_thread::sleep_until(time_start_trick + dt);
    }
    return 0;
}
```

### 3. 底层运动控制例程

更新时间：2024-09-25 17:17:20

本文介绍如何使用 **unitree_sdk2** 调用底层接口开发 Go2-W 机器人，相关的代码文件为 `unitree_sdk2/example/go2w` 路径下的 `go2w_stand_example.cpp`。在本例程中会示范如何调用底层接口控制 Go2-W 机器人站立，并往复运动一定距离后趴下。如果有任何不理解的地方，欢迎联系[宇树技术支持部门](https://www.unitree.com/cn/connect) 请求帮助！

#### 环境依赖

##### 系统环境

推荐在 Ubuntu 20.04 系统下进行开发, 暂不支持在 Mac、Windows 系统下进行开发, 且不支持在 Go2-W 的内置电脑上开发。

##### 网络环境

需将用户电脑中与 Go2-W 机器人通信的网卡设置在 123 网段下，且该网卡的ip地址建议设置为 `192.168.123.222` (“222”可以改成其他)。不允许将该网卡的ip地址设置为 `192.168.123.161`，此为 Go2-W 机器人内置电脑的 ip 地址。

#### 安装与编译

> **以下叙述假设工作目录（working directory）为用户主目录**。

##### 安装 unitree_sdk2

下载[ unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) 压缩包并解压至用户主目录。打开一个终端，并依次执行下列命令以安装 unitree_sdk2:

```C++
cd ~/unitree_sdk2/
mkdir build
cd build
cmake ..
sudo make install
```

或者安装到指定目录下:

```C++
cd ~/unitree_sdk2/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

注意

在上述命令中，CMAKE_INSTALL_PREFIX 后面用于指定将 unitree_sdk2 的安装至 /opt/unitree_robotics 目录下，如果您想将 unitree_sdk2 安装在其他目录，修改此路径即可。

##### 例程编译

打开一个终端，并依次执行下列命令以编译例程：

```C++
cd ~/unitree_sdk2
mkdir build
cd build
cmake ..
make
```

运行上文中的 make 命令后，若进度进行到100%且没有报错，则意味着编译成功。

![img](https://doc-cdn.unitree.com/static/2023/9/12/2292a3ec23e24723beb64bf61e2aa2ec_390x292.png)

若成功执行 make 命令, 生成的例程会在 build/bin 目录下。下图中显示为绿色字符的即为编译成功的例程对应的二进制文件。

![img](https://doc-cdn.unitree.com/static/2023/9/12/f9a3aeb045bf4b3a90e79c66fedc4bd5_596x127.png)

注意

由于 unitree_sdk2 会持续不断地更新，上图中的可执行文件可能会与最新的有所不同。

##### 配置网络环境

运行例程时控制命令将会通过局域网从用户电脑发送至 Go2-W 机器人内置电脑，故在这之前需要通过一些必要的配置步骤将这两台电脑组成一个局域网。

配置步骤：

1. 用网线的一端连接 Go2-W 机器人，另一端连接用户电脑，并开启电脑的 USB Ethernet 后进行配置。 Go2-W 机载电脑的 IP 地地址为 192.168.123.161，故需将电脑 USB Ethernet 地址设置为与机器狗同一网段，如在 Address 中输入 192.168.123.222 (“222”可以改成其他)。

![img](https://doc-cdn.unitree.com/static/2023/8/31/0e3f45a8cf65464d85485e0731ee3b42_897x156.png) ![img](https://doc-cdn.unitree.com/static/2023/9/6/0f51cb9b12f94f0cb75070d05118c00a_980x816.jpg)

为了测试用户电脑与Go2机器人内置电脑是否正常连接，可在终端中输入ping 192.168.123.161 进行检测，出现下图类似内容即为连接成功。

![img](https://doc-cdn.unitree.com/static/2023/8/31/393207d38e3a49cda738a418036ae8f0_1088x438.png)

1. 查看 123 网段对应的网卡名字
   通过ifconfig命令查看123网段的网卡名字，如下图所示:

![img](https://doc-cdn.unitree.com/static/2023/9/7/96fea4461bb64dfcaaa9e430e130b403_715x504.png)

如上图所示，ip 为 192.168.123.222 对应的网卡名字为 enxf8e43b808e06。用户需要记住此名字，在运行例程时其将会作为必要参数。

#### 例程运行

编译成功后 unitree_sdk2/build/bin 文件夹中会有有 low_level、high_level、wireless 等例程对应的二进制文件。 这些例程总体上可分为高层控制例程和底层控制例程，其中详情，请查看《[例程参考](https://support.unitree.com/home/zh/developer/Basic_motion_control)》篇。下文以运行 stand_example_go2w 例程为例，该例子为底层控制例程，该例程会控制 Go2-W 机器人站立，并往复运动一段距离，最后趴下。

##### 关闭运控服务

在运行该例程前需要先关闭 Go2-W 的主运控服务(wheeled_sport)。 可在 App-设置-服务状态 里点击对应的服务关闭。也可以通过调用服务状态开关接口关闭运控服务，此方式例程在本文讲解的 [stand_example_go2 ](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2w/stand_example_go2w.cpp)例程中也有体现，在本例程中展示了如何通过调用 [ServiceSwitch ](https://support.unitree.com/home/zh/developer/RobotStateClient)接口关闭运控服务。

注意

之所以需要先关闭主运控服务(wheeled_sport)，这是因为底层控制例程也相当于一个运控服务，它们均会发送控制指令给 Go2-W ，如果多个运控同时存在，则 Go2-W 机器人会同时接收两个或多个控制指令而产生混乱，造成机器狗失控。故在运行底层控制例程序前，需要确保对应的服务处于关闭状态。

![img](https://doc-cdn.unitree.com/static/2024/9/8/e2720f7462c74db4acb5a45d051b683e_815x399.png)

##### 运行例程

打开一个终端，并依次执行下列命令以运行例程。注：第二行命令中的"enxf8e43b808e06"字符串需要用户自行替换为用户电脑中对应于123网段的网卡名字。

```C++
cd ~/unitree_sdk2/build/bin/
sudo ./stand_example_go2w enxf8e43b808e06
```

注意

该例程会使 Go2-W 机器人站立，并往复运动一定距离，为了保护机器人，在运行此例程前应使机器人趴于地面，并保证四周环境无遮挡。

##### 运行结果预览

若终端界面出现与下图相似画面，则表示运行成功

![img](https://doc-cdn.unitree.com/static/2024/9/8/bf4b29c89b064a0c96351063d355b193_732x414.gif)

下图为运行 stand_example_go2w 例程的预期实物效果：

![img](https://doc-cdn.unitree.com/static/2024/9/9/272abbf6167b4835906b4f6d5ea8f5e2_1280x720.gif)

#### 程序解析

本小节将会针对本文例程序对应的 [stand_example_go2w.cpp ](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2w/stand_example_go2w.cpp)源码文件中的程序进行讲解。
首先关注 main 函数头部位置

```c++
Custom custom;
custom.InitRobotStateClient();
while(custom.queryServiceStatus("wheeled_sport"))
{
  std::cout<<"Try to deactivate the service: "<<"wheeled_sport"<<std::endl;
  custom.activateService("wheeled_sport",0);
  sleep(1);
}
custom.Init();
```

首先创建 Custom 类的一个对象 custom。custom 是一个自定义类，在此类写了本例程程序的所有逻辑。接下来是调用 custom 对象的 InitRobotStateClient() 函数。接下来会调用 queryServiceStatus 函数查询 go2w 的运控服务 **wheeled_sport** ，如果此运控服务存在，则会调用 activateService 函数停止运控服务，只有运控服务停止后方可继续。在 cpp 代码文件中也可以找到 InitRobotStateClient 函数的定义：

```c++
void Custom::InitRobotStateClient()
{
    rsc.SetTimeout(10.0f); 
    rsc.Init();
}
```

rsc 是 RobotStateClient 类的对象。调用此对象的 ServiceSwitch 方法可以关闭或开启某个运控服务，但在此之前需要进行一些初始化设置。通过调用 SetTimeout(10.0f) 设置服务调用超时时间为 10 s。同时通过 Init() 函数进行初始化。接下来在初始化 rsc 后，通过调用 queryServiceStatus() 函数查询对应服务的状态：

```c++
int Custom::queryServiceStatus(const std::string& serviceName)
{
    std::vector<ServiceState> serviceStateList;
    int ret,serviceStatus;
    ret = rsc.ServiceList(serviceStateList);
    size_t i, count=serviceStateList.size();
    for (i=0; i<count; i++)
    {
        const ServiceState& serviceState = serviceStateList[i];
        if(serviceState.name == serviceName)
        {
            if(serviceState.status == 0)
            {
                std::cout << "name: " << serviceState.name <<" is activate"<<std::endl;
                serviceStatus = 1;
            }
            else
            {
                std::cout << "name:" << serviceState.name <<" is deactivate"<<std::endl;
                serviceStatus = 0;
            } 
        }    
    }
    return serviceStatus;
    
}
```

在该函数中通过参数 serviceName 送入用户需要查询的服务状态，本例程中需要查询的是 "wheeled_sport" 服务，如果该服务此时在运行状态，则会返回 1，如果此服务此时在关闭状态，则会返回0. 如果查询到服务在运行，则调用 activateService() 函数进行关闭。此函数定义为：

```c++
void Custom::activateService(const std::string& serviceName,int activate)
{
    float ret = rsc.ServiceSwitch(serviceName, activate);  
}
```

第一个参数为想要设置的服务的名字，第二个参数为想要设置的状态（0 表示关闭，1 表示启动）。
接下来调用 Init 函数进行正式初始化。

```c++
void Custom::Init()
{
    InitLowCmd();

    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &Custom::LowCmdWrite, this);
}
```

在此函数中首先调用 `InitLowCmd()` 函数初始化 `unitree_go::msg::dds_::LowCmd_` 类型结构体 `low_cmd`。同时初始化 `lowcmd_publisher` 和 `lowstate_subscriber` 类。初始化后即可调用 `lowcmd_publisher` 的 `Write()` 函数将控制命令发送至 Go2-W 机器人，实现对机器人的控制。而在此之前需要获取 Go2-W 机器人的状态信息，在 `lowstate_subscriber` 对象中绑定了 Custom 类中的 `LowStateMessageHandler` 函数作为回调函数。即每当 Go2-W 向外发送一次机器人状态信息就会触发一次 `LowStateMessageHandler` 函数的调用。用户在对获取的机器人状态信息同时加入一些自定义用户逻辑后才可形成控制命令发送给机器人，在例成中通过 `CreateRecurrentThreadEx()` 函数创建一个多线程来进行相关的处理。在本例程中绑定 Custom类的 `LowCmdWrite` 方法作为
线程函数。参数中的 2000 表示运行此线程函数的时间间隔，2000 表示以 2 ms（即 500 hz）时间间隔运行 `LowCmdWrite` 函数。
对于 `LowCmdWrite` 函数

```c++
void Custom::LowCmdWrite()
{
    if(_percent_4<1)
    {
        std::cout<<"Read sensor data example: "<<std::endl;
        std::cout<<"Joint 0 pos: "<<low_state.motor_state()[0].q()<<std::endl;
        std::cout<<"Imu accelerometer : "<<"x: "<<low_state.imu_state().accelerometer()[0]<<" y: "<<low_state.imu_state().accelerometer()[1]<<" z: "<<low_state.imu_state().accelerometer()[2]<<std::endl;
        std::cout<<"Foot force "<<low_state.foot_force()[0]<<std::endl;
        std::cout<<std::endl;
    }
    if((_percent_4 == 1) && ( done == false))
    {
        std::cout<<"The example is done! "<<std::endl;
        std::cout<<std::endl;
        done = true;
    }

    motiontime++;
    if(motiontime>=500)
    {
        if(firstRun)
        {
            for(int i = 0; i < 12; i++)
            {
                _startPos[i] = low_state.motor_state()[i].q();
            }
            firstRun = false;
        }

        _percent_1 += (float)1 / _duration_1;
        _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
        if (_percent_1 < 1)
        {
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_1) * _startPos[j] + _percent_1 * _targetPos_1[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        
        }
        if ((_percent_1 == 1)&&(_percent_2 < 1))
        {
            _percent_2 += (float)1 / _duration_2;
            _percent_2 = _percent_2 > 1 ? 1 : _percent_2;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_2) * _targetPos_1[j] + _percent_2 * _targetPos_2[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }

        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3<1))
        {
            _percent_3 += (float)1 / _duration_3;
            _percent_3 = _percent_3 > 1 ? 1 : _percent_3;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() =  _targetPos_2[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
            if(_percent_3<0.4)
            {
                for (int j = 12; j < 16; j++)
                {
                    low_cmd.motor_cmd()[j].q() =  0;
                    low_cmd.motor_cmd()[j].kp() = 0;
                    low_cmd.motor_cmd()[j].dq() = 3;

                    low_cmd.motor_cmd()[j].kd() = Kd;
                    low_cmd.motor_cmd()[j].tau() = 0;
                }
                            
            }
            else if((_percent_3>=0.4)&&(_percent_3<0.8))
            {
                for (int j = 12; j < 16; j++)
                {
                    low_cmd.motor_cmd()[j].q() =  0;
                    low_cmd.motor_cmd()[j].kp() = 0;
                    low_cmd.motor_cmd()[j].dq() = -3;

                    low_cmd.motor_cmd()[j].kd() = Kd;
                    low_cmd.motor_cmd()[j].tau() = 0;
                }
                            
            }
            else if(_percent_3>=0.8)
            {
                for (int j = 12; j < 16; j++)
                {
                    low_cmd.motor_cmd()[j].q() =  0;
                    low_cmd.motor_cmd()[j].kp() = 0;
                    low_cmd.motor_cmd()[j].dq() = 0;

                    low_cmd.motor_cmd()[j].kd() = Kd;
                    low_cmd.motor_cmd()[j].tau() = 0;
                } 
            }
            
        }
        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3==1)&&((_percent_4<=1)))
        {
            _percent_4 += (float)1 / _duration_4;
            _percent_4 = _percent_4 > 1 ? 1 : _percent_4;
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_4) * _targetPos_2[j] + _percent_4 * _targetPos_3[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }
        low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
        lowcmd_publisher->Write(low_cmd);
    }
   
}
```

在此函数主要写了实现 Go2-W 机器人站立，并往复运动一定距离，最后趴下的逻辑。此函数有些长，让我们关注主要的内容。
此函数运行时 motiontime 会自增，当自增到 500 时才会执行主要的逻辑。这是因为在 `custom.Init()` 函数中在通过 `lowstate_subscriber->InitChannel()` 函数绑定的 `Custom::LowStateMessageHandler` 回调函数后，此回调函数会立刻启动。接下来通过 `CreateRecurrentThreadEx()` 函数绑定线程函数 `Custom::LowCmdWrite` 后，此线程也会立刻启动。上述过程时间非常短，而在 `LowCmdWrite` 中首先需要能获取机器人当前的关节角度信息后方可进行正确的角度插值。故 motiontime 自增到 500 的作用是延时一定时间，即当能更新机器人的状态信息后才可进行接下来的逻辑处理。

| 变量       | 说明                  |
| ---------- | --------------------- |
| _percent_1 | 控制 Go2-W 关节收拢   |
| _percent_2 | 控制 Go2-W 站立       |
| _percent_3 | 控制 Go2-W 前进与后退 |
| _percent_4 | 控制 Go2-W 趴下       |

在 Go2-W 机器人站立，并往复运动一定距离，最后趴下，这样一个过程中，在程序上主要通过四个变量进行控制相应过程。即 _percent_1 、_percent_2 、_percent_3、_percent_4，其作用见于上述表格。在这些变量为 0 时，表示对应的动作尚未开始，在 0 和1 之间时，表示动作在进行之中。当变量等于 1 时，表示动作执行结束。由于此部分代码逻辑较长，在本文不在赘述，在此节选一部分内容进行讲解。

```c++
if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3<1))
        {
            _percent_3 += (float)1 / _duration_3;
            _percent_3 = _percent_3 > 1 ? 1 : _percent_3;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() =  _targetPos_2[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
            if(_percent_3<0.4)
            {
                for (int j = 12; j < 16; j++)
                {
                    low_cmd.motor_cmd()[j].q() =  0;
                    low_cmd.motor_cmd()[j].kp() = 0;
                    low_cmd.motor_cmd()[j].dq() = 3;
                    low_cmd.motor_cmd()[j].kd() = Kd;
                    low_cmd.motor_cmd()[j].tau() = 0;
                }
                            
            }
            else if((_percent_3>=0.4)&&(_percent_3<0.8))
            {
                for (int j = 12; j < 16; j++)
                {
                    low_cmd.motor_cmd()[j].q() =  0;
                    low_cmd.motor_cmd()[j].kp() = 0;
                    low_cmd.motor_cmd()[j].dq() = -3;
                    low_cmd.motor_cmd()[j].kd() = Kd;
                    low_cmd.motor_cmd()[j].tau() = 0;
                }
                            
            }
            else if(_percent_3>=0.8)
            {
                for (int j = 12; j < 16; j++)
                {
                    low_cmd.motor_cmd()[j].q() =  0;
                    low_cmd.motor_cmd()[j].kp() = 0;
                    low_cmd.motor_cmd()[j].dq() = 0;
                    low_cmd.motor_cmd()[j].kd() = Kd;
                    low_cmd.motor_cmd()[j].tau() = 0;
                } 
            }
            
        }
```

对于头部的 if 语句：

```c++
if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3<1))
```

此 if 语句表示在机器人执行完 Go2-W 关节收拢（_percent_1）、Go2-W 站立_percent_2） 后才可进行接下来的这部分逻辑。

```c++
_percent_3 += (float)1 / _duration_3;
_percent_3 = _percent_3 > 1 ? 1 : _percent_3;
```

随着程序的进行，_percent_3 会自增来控制 Go2-W 前进与后退 这个动作的进度。 可以看到 _duration_3 变量控制着此动作的持续时间。用户可自行调整 _duration_3 变量的值来控制此动作的时间长短。

```c++
for (int j = 0; j < 12; j++)
{
    low_cmd.motor_cmd()[j].q() =  _targetPos_2[j];
    low_cmd.motor_cmd()[j].dq() = 0;
    low_cmd.motor_cmd()[j].kp() = Kp;
    low_cmd.motor_cmd()[j].kd() = Kd;
    low_cmd.motor_cmd()[j].tau() = 0;
}
```

此语句表示控制 Go2-W 的前12个关节保持特定的角度。因为在 Go2-W 轮子转动时，我们希望机器人的躯干能保持不动，进而维持机器人的稳定。

```c++
if(_percent_3<0.4)
{
    for (int j = 12; j < 16; j++)
    {
        low_cmd.motor_cmd()[j].q() =  0;
        low_cmd.motor_cmd()[j].kp() = 0;
        low_cmd.motor_cmd()[j].dq() = 3;
        low_cmd.motor_cmd()[j].kd() = Kd;
        low_cmd.motor_cmd()[j].tau() = 0;
    }           
}
```

此段语句表示设置 Go2-W 机器人的四个轮子以 3 rad/s 的期望速度进行旋转。在摩擦力的作用下，机器人即可向前运动。这个动作的持续时间在 _percent_3<0.4 的情况下进行，用户也可通过调整此 if 语句,来调整机器人向前运动的持续时间。剩下的 if 语句功能与上述类似，本文不在此赘述。最后在经过一系列处理后通过调用 lowcmd_publisher 对象的 Write 方法向 Go2-w 机器人发送控制命令，故此完成对 Go2-W 机器人的控制。

```c++
low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
lowcmd_publisher->Write(low_cmd);
```
