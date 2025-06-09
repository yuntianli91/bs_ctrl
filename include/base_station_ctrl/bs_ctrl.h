// @file        This file is in compliance with GNU GPL V3 license.
// @brief       ROS Class for base station uav control
// @author      Yuntian Li (yuntianli@pmlabs.com.cn)
// @version     1.0
// @date        Last modified at 2025-06-05
// @copyright   Copyright (c) {2024 - } Purple Mountain Lab UAV TEAM.

#include <thread>
#include <mutex>
#include <queue>
#include <spdlog/spdlog.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "base_station_ctrl/geo_utils.h"
#include "base_station_ctrl/GetDeltaAngle.h"
#include "base_station_ctrl/SetLocalOrigin.h"

namespace bs_ctrl{

class Bs_Ctrl{
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // no need for C++ 17
  Bs_Ctrl() = delete;
  explicit Bs_Ctrl(ros::NodeHandle &nh);
  ~Bs_Ctrl() {}

  void startCtrlThread();
  void stopCtrlThread();
 
 private:
  bool init_local_origin_{false};
  std::mutex mtx_local_pose_;
  std::mutex mtx_global_pose_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_local_pose_;  // subsriber of local pose
  ros::Subscriber sub_global_pose_;  // subsriber of global pose
  ros::Publisher pub_att_cmd_;      // publisher of att cmds 

  unsigned int queue_size_{50};   // default queue_size (50 Hz for 1 min)
  std::queue<sensor_msgs::NavSatFix> all_global_poses_;
  std::queue<geometry_msgs::PoseStamped> all_local_poses_;  
  // Eigen::Vector3d t_;     // translations of uav, store in x, y, z
  // Eigen::Quaterniond q_;  // quaternions of uav, store in w, x, y, z

  std::unique_ptr<geo_utils::AlignBeam> ptr_align_;
  std::thread ctrl_thread_;

  Eigen::Quaterniond des_q_;  // desired quaternion
  Eigen::Vector3d des_t_;     // desired position

  // all thread functions
  void CtrlThreadProcess();
  // all subscriber callback functions
  void LocalPoseCb(const geometry_msgs::PoseStampedConstPtr &msg);
  void GlobalPoseCb(const sensor_msgs::NavSatFixConstPtr &msg);
  
  // all server callback functions
  bool SetLocalOriginCb(base_station_ctrl::SetLocalOrigin::Request &req,
                        base_station_ctrl::SetLocalOrigin::Response &res);
  bool GetDeltaAngleCb(base_station_ctrl::GetDeltaAngle::Request &req,
                       base_station_ctrl::GetDeltaAngle::Response &res);
  
  // @brief   get latest curresponding msg
  // @tparam  T : template param
  // @param   msg_queue : 
  // @param   curr_time : 
  // @return  T : 
  template<typename T>
  bool getLatestMsg(std::queue<T> &msg_queue, double curr_time, T &latest_msg){
    // 
    if (curr_time < msg_queue.front().header.stamp.toSec()){
      return false;
    }
    
    while(!msg_queue.empty()){
      if (msg_queue.front().header.stamp.toSec() > curr_time){
        break;
      }

      latest_msg = msg_queue.front();
      msg_queue.pop();
    }

    return true;
  }
};

} // namespace bs_ctrl