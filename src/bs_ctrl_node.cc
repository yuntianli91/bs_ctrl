// @file        This file is in compliance with GNU GPL V3 license.
// @brief   
// @author      Yuntian Li (yuntianli@pmlabs.com.cn)
// @version     1.0
// @date        Last modified at 2025-06-05
// @copyright   Copyright (c) {2024 - } Purple Mountain Lab UAV TEAM.

#include "base_station_ctrl/bs_ctrl.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "bs_ctrl_node");
  ros::NodeHandle nh("~");

  bs_ctrl::Bs_Ctrl bs_ctrl(nh);
  // start ctrl thread
  bs_ctrl.startCtrlThread();
  // start all ros sub, pub, srv, etc.
  ros::spin();
  // stop ctrl thread
  bs_ctrl.stopCtrlThread();
  
  return 0;
}