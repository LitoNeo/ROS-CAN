#pragma once
#ifndef USB2CAN_CORE_H
#define USB2CAN_CORE_H

#include <can_msgs/battery.h>
#include <can_msgs/ecu.h>
#include <can_msgs/vehicle_status.h>
#include <iostream>
#include <ros/ros.h>

#include "Msg.h"
#include "unistd.h"
#include <cstdlib>
#include <ctime>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace USB2CAN
{
struct Param
{
  uint8_t mrun_num;
  std::string name;
  int debug_can_port;
  int main_can_port;
  ros::Publisher battery_status_publisher;
  ros::Publisher vehicle_status_publisher;
};

// static DWORD CAN_device = VCI_USBCAN2; // 如果变量要写到头文件.h中,则最好使用static,以避免multi
//                                        // definitation问题
static DWORD CAN_id = 0; // 默认采用第一个CAN设备, (目前也只有一个CAN设备,即USBCAN2)
// static DWORD CAN_port = 0;

static VCI_BOARD_INFO pInfo; // 获取/存储设备信息
static int count = 0;        // 数据列表中,泳衣存储列表序号

static VCI_INIT_CONFIG config;

static pthread_t threadid;
static Param *param;

// void ExitHandler(int sig);

class CAN_app
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle p_nh;

  ros::Subscriber sub_ecu;
  ros::Publisher pub_vehicle_status;
  ros::Publisher pub_battery_status;

  bool param_debug;
  int debug_can_port;
  int main_can_port;

  bool param_show_sending_msg;

  double pre_steer;

  void initROS();

  void ecu_cb(const can_msgs::ecu::ConstPtr &msg);

public:
  CAN_app();
  ~CAN_app();

  void run();

  // void ExitHandler(int sig);
};

} // namespace USB2CAN
#endif