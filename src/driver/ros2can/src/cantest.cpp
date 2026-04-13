
/********************************************************************************
                                    测试编译说明
将CAN1,CAN2数据线并联,测试数据收发。(实际使用的时候可以只开启配置一个CAN通道)
将  libcanbus.tar 拷贝到目录  /usr/local/lib 下面解压
配置环境变量:                export
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib 编译方法                     gcc
cantest.cpp -lcanbus -lusb-1.0 -o can2test 静态编译方法                 gcc
cantest.cpp -lrt -Wl,-Bstatic -lcanbus -lusb-1.0 -Wl,-Bdynamic -Wl,-ludev -o
can2test 运行                       ./can2test
如果提示ludev错误，请安装libudev-dev：   sudo apt-get install libudev-dev
如果编译过程中找不到cc1plus 请安装              sudo apt-get install --reinstall
build-essential
*********************************************************************************/

#include "canbus.h"
#include <string.h>
#include <unistd.h>
int dev = 0;
int cpot0 = 0;
int cpot1 = 1;
Can_Msg txmsg[100];
Can_Msg rxmsg[100];

int can1_send_msgs(void) // CAN1发送数据
{
  int ret;
  memset(&txmsg[0], 0, sizeof(txmsg[0]));
  txmsg[0].ID = 0x111;
  txmsg[0].Data[0] = 0x00;
  txmsg[0].Data[1] = 0x00;
  txmsg[0].Data[2] = 0x00;
  txmsg[0].Data[3] = 0xc8;
  txmsg[0].Data[4] = 0x00;
  txmsg[0].Data[5] = 0x00;
  txmsg[0].Data[6] = 0x00;
  txmsg[0].Data[7] = 0x00;
  txmsg[0].DataLen = 8;
  ret = CAN_Transmit(dev, cpot1, txmsg, 1, 100);
  printf("CAN_Transmit items: %d\r\n", ret);
  return ret;
}
int can1_send_msgs1(void) // CAN1发送数据
{
  int ret;
  memset(&txmsg[0], 0, sizeof(txmsg[0]));
  txmsg[0].ID = 0x421;
  txmsg[0].Data[0] = 0x01;
  txmsg[0].Data[1] = 0x00;
  txmsg[0].Data[2] = 0x00;
  txmsg[0].Data[3] = 0x00;
  txmsg[0].Data[4] = 0x00;
  txmsg[0].Data[5] = 0x00;
  txmsg[0].Data[6] = 0x00;
  txmsg[0].Data[7] = 0x00;
  txmsg[0].DataLen = 8;
  ret = CAN_Transmit(dev, cpot1, txmsg, 1, 100);
  printf("CAN_Transmit items: %d\r\n", ret);
  return ret;
}
int can2_receive_msgs(void) // CAN2接收数据
{
  int ret, i;
  memset(&rxmsg[0], 0, sizeof(rxmsg[0]));
  ret = CAN_Receive(dev, cpot1, rxmsg, 100, 100);
  printf("CAN_Receive items: %d\r\n", ret);

  printf("Receive msg: id = %x ,data: ", rxmsg[0].ID);

  for (i = 0; i < rxmsg[0].DataLen; i++) {
    printf("%02x ", (unsigned char)rxmsg[0].Data[i]);
  }
  printf("\r\n");
  return ret;
}

int main(int argc, char *argv[]) {
  int devs, ret;
  Can_Config cancfg;
  devs = CAN_ScanDevice(); //扫描CAN设备
  printf("CAN_ScanDevice = %d\r\n", devs);
  if (devs <= 0)
    return -1;
  ret = CAN_OpenDevice(dev, cpot0); //打开通道0(CAN1)
  printf("CAN_OpenDevice0 = %d\r\n", ret);
  if (ret != 0)
    return -1;
  printf("CAN_OpenDevice0 succeed!!\r\n");
  ret = CAN_OpenDevice(dev, cpot1); //打开通道1(CAN2)
  printf("CAN_OpenDevice1 = %d\r\n", ret);
  if (ret != 0)
    return -1;
  printf("CAN_OpenDevice1 succeed!!\r\n");
  cancfg.model = 0;
  cancfg.configs = 0;
  cancfg.baudrate = 500000; //设置波特率500k(500*1000)
  cancfg.configs |= 0x0001; //接通内部匹配电阻
  cancfg.configs |= 0x0002; //开启离线唤醒模式
  cancfg.configs |= 0x0004; //开启自动重传

  ret = CAN_Init(dev, 0, &cancfg); //初始化CAN1
  printf("CAN_Init0 = %d\r\n", ret);
  if (ret != 0)
    return -1;
  printf("CAN_Init0 succeed!!\r\n");
  CAN_SetFilter(dev, cpot0, 0, 0, 0, 0, 1); //设置接收所有数据

  ret = CAN_Init(dev, cpot1, &cancfg); //初始化CAN2
  printf("CAN_Init1 = %d\r\n", ret);
  if (ret != 0)
    return -1;
  printf("CAN_Init1 succeed!!\r\n");
  CAN_SetFilter(dev, cpot1, 0, 0, 0, 0, 1); //设置接收所有数据
  if (ret != 0)
    return -1;
  can1_send_msgs1(); // CAN1发送数据
  sleep(3);

  while (true) {
    can1_send_msgs(); // CAN1发送数据
    sleep(0.1);
  }

  can2_receive_msgs(); // CAN2接收数据

  CAN_CloseDevice(dev, cpot0); //关闭CAN1
  CAN_CloseDevice(dev, cpot1); //关闭CAN2
  return 0;
}
