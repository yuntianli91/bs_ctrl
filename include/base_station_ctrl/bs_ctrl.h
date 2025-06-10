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
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_srvs/SetBool.h>
#include "base_station_ctrl/geo_utils.h"
#include "base_station_ctrl/AlignBeamYaw.h"
#include "base_station_ctrl/SetLocalOrigin.h"

namespace bs_ctrl{
typedef struct UavState{
  bool is_connected_ = false;    
  bool is_armed_ = false;
  std::string mode_ = "STABILIZED";
  
  unsigned short landed_state_ = 1; // 0-undefined, 1-on ground, 
                                    // 2-in air, 3-takeoff 4-landing      

  // TODO : add rc related
  unsigned int rc_chs[8];       // rc channel values
  // local & global pose
  Eigen::Quaterniond curr_q_;   // current q in local
  Eigen::Vector3d curr_t_;      // current t in local
  Eigen::Vector3d curr_lla_;    // current lla in WGS-84
}sUavState;

class Bs_Ctrl{
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // no need for C++ 17
  Bs_Ctrl() = delete;
  explicit Bs_Ctrl(ros::NodeHandle &nh);
  ~Bs_Ctrl() {}

  void startCtrlThread();
  void stopCtrlThread();

  // @brief   set frequency of ctrl thread, default is 20Hz
  // @param   freq : 
  void setCtrlRate(float freq){
    ptr_ctrl_rate_->reset();
    ptr_ctrl_rate_ = std::make_unique<ros::Rate>(freq);
  }
 
 private:
  // ============== ros
  ros::NodeHandle nh_;
  ros::Subscriber sub_local_pose_;    // subsriber of local pose
  ros::Subscriber sub_global_pose_;   // subsriber of global pose
  ros::Subscriber sub_rc_;            // subscriber of rc input
  ros::Subscriber sub_state_;         // subscriber of mav state
  ros::Subscriber sub_ext_state_;     // subscriber of extra state

  ros::Publisher pub_att_cmd_;      // publisher of att cmds 
  ros::Publisher pub_local_pose_;   // publisher of local pose

  ros::ServiceServer server_align_;
  ros::ServiceServer server_takeoff_;
  ros::ServiceServer server_set_origin_;

  ros::ServiceClient client_arm_;       // client for arming
  ros::ServiceClient client_set_mode_;  // client for set flight mode

  std::unique_ptr<ros::Rate> ptr_ctrl_rate_;

  // ==============  all message queues and mutex
  std::mutex mtx_uav_state_;
  sUavState curr_uav_state_;
  // ============== others
  bool init_local_origin_{false};

  std::unique_ptr<geo_utils::AlignBeam> ptr_align_;
  std::thread ctrl_thread_;

  Eigen::Quaterniond des_q_;  // desired quaternion
  Eigen::Vector3d des_t_;     // desired position

  mavros_msgs::SetMode offb_set_mode_;
  mavros_msgs::CommandBool arm_cmd_;

  // all thread functions
  void CtrlThreadProcess();

  // all subscriber callback functions
  void initAllSub();
  void StateCb(const mavros_msgs::StateConstPtr &msg);
  void ExtStateCb(const mavros_msgs::ExtendedStateConstPtr &msg);
  void RcCb(const mavros_msgs::RCInConstPtr &msg);
  void LocalPoseCb(const geometry_msgs::PoseStampedConstPtr &msg);
  void GlobalPoseCb(const sensor_msgs::NavSatFixConstPtr &msg);
  
  // all server callback functions
  void initAllSrv();
  bool SetLocalOriginCb(base_station_ctrl::SetLocalOrigin::Request &req,
                        base_station_ctrl::SetLocalOrigin::Response &res);
  bool AlignBeamYawCb(base_station_ctrl::AlignBeamYaw::Request &req,
                      base_station_ctrl::AlignBeamYaw::Response &res);
  bool TakeoffLandCb(std_srvs::SetBool::Request &req, 
                     std_srvs::SetBool::Response &res);
  
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