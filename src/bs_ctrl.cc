// @file        This file is in compliance with GNU GPL V3 license.
// @brief   
// @author      Yuntian Li (yuntianli@pmlabs.com.cn)
// @version     1.0
// @date        Last modified at 2025-06-05
// @copyright   Copyright (c) {2024 - } Purple Mountain Lab UAV TEAM.

#include "base_station_ctrl/bs_ctrl.h"

namespace bs_ctrl{
Bs_Ctrl::Bs_Ctrl(ros::NodeHandle &nh) : nh_(nh)
{
  ptr_align_ = std::make_unique<geo_utils::AlignBeam>(0., 0., 0.);

  sub_local_pose_ = nh_.subscribe("/mavros/local_position/pose", 10, 
                                  &Bs_Ctrl::LocalPoseCb, this);
  sub_global_pose_ = nh_.subscribe("/mavros/global_position", 10, 
                                   &Bs_Ctrl::GlobalPoseCb, this);
}

void Bs_Ctrl::startCtrlThread(){
  ctrl_thread_ = std::thread(&Bs_Ctrl::CtrlThreadProcess, this);
}

void Bs_Ctrl::stopCtrlThread(){
  if (ctrl_thread_.joinable()){
    ctrl_thread_.join();
  }
}

void Bs_Ctrl::LocalPoseCb(const geometry_msgs::PoseStampedConstPtr &msg){
  std::lock_guard<std::mutex> guard(mtx_local_pose_);  
  if (all_local_poses_.size() >= queue_size_){
    all_local_poses_.pop();
  }
  all_local_poses_.push(*msg);
}

void Bs_Ctrl::GlobalPoseCb(const sensor_msgs::NavSatFixConstPtr &msg){
  std::lock_guard<std::mutex> guard(mtx_global_pose_);
  if (all_global_poses_.size() >= queue_size_){
    all_global_poses_.pop();
  }
  all_global_poses_.push(*msg);
}

bool Bs_Ctrl::SetLocalOriginCb(base_station_ctrl::SetLocalOrigin::Request &req,
                               base_station_ctrl::SetLocalOrigin::Response &res)
{
  ptr_align_->setOriginLLA(req.lat, req.lon, req.alt);
  init_local_origin_ = true;
  res.success = true;
  res.message = "";
  
  return true;
}

bool Bs_Ctrl::GetDeltaAngleCb(base_station_ctrl::GetDeltaAngle::Request &req,
                              base_station_ctrl::GetDeltaAngle::Response &res)
{
  // origin of local cartesian must be set first.
  if (init_local_origin_){
    res.success = false;
    res.message = "Origin of Local Cartesian has not been set,\
                   Call service SetLocalOrigin first !";    
    return true;
  }

  // create delay lock objects
  std::unique_lock<std::mutex> lock_local(mtx_local_pose_, std::defer_lock);
  std::unique_lock<std::mutex> lock_global(mtx_global_pose_, std::defer_lock);

  double curr_time = ros::Time::now().toSec();
  geometry_msgs::PoseStamped curr_local_pose;
  sensor_msgs::NavSatFix curr_global_pose;
  
  // get global LLA and local attitude
  std::lock(lock_local, lock_global);
  if (!getLatestMsg(all_local_poses_, curr_time, curr_local_pose) || 
      !getLatestMsg(all_global_poses_, curr_time, curr_global_pose))
  {
    res.success = false;
    res.message = "UAV State not ready, please wait a moment and try again.";
    return true;
  }
  lock_local.unlock();
  lock_global.unlock();

  // calculate align vector
  Eigen::Quaterniond curr_q(curr_local_pose.pose.orientation.w,
                            curr_local_pose.pose.orientation.x,
                            curr_local_pose.pose.orientation.y,
                            curr_local_pose.pose.orientation.z);
  geo_utils::sDeltaAngles d_angles = ptr_align_->getAlignVec(
    req.lat, req.lon, req.alt,
    curr_global_pose.latitude, curr_global_pose.longitude, 
    curr_global_pose.altitude, curr_q);
  
  // update desired q and t;
  des_q_ = curr_q * Eigen::Quaterniond(
    Eigen::AngleAxisd(d_angles.d_yaw, Eigen::Vector3d::UnitZ()));
  res.d_pitch = d_angles.d_pitch;
  res.success = true;
  res.message = "";
  return true;
}

void Bs_Ctrl::CtrlThreadProcess(){

}

} //  namespace bs_ctrl
