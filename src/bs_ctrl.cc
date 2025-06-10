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
  ptr_ctrl_rate_ = std::make_unique<ros::Rate>(20.0);
  ptr_align_ = std::make_unique<geo_utils::AlignBeam>(0., 0., 0.);
  des_t_ = Eigen::Vector3d(0., 0., 2.);
  des_q_ = Eigen::Quaterniond(1.0, 0., 0., 0.);

  // init mode and arm cmd
  offb_set_mode_.request.custom_mode = "STABILIZED";
  arm_cmd_.request.value = false;
  
  // init all subs and srvs
  initAllSub();
  initAllSrv();
  // init all pubs
  pub_local_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(
    "/mavros/setpoint_position/local", 10);
}

void Bs_Ctrl::initAllSub(){
  sub_local_pose_ = nh_.subscribe("/mavros/local_position/pose", 10, 
                                  &Bs_Ctrl::LocalPoseCb, this);
  sub_global_pose_ = nh_.subscribe("/mavros/global_position/global", 10, 
                                   &Bs_Ctrl::GlobalPoseCb, this);
  sub_state_ = nh_.subscribe("/mavros/state", 10, &Bs_Ctrl::StateCb, this);
  sub_ext_state_ = nh_.subscribe("/mavros/extended_state", 10, 
                                &Bs_Ctrl::ExtStateCb, this);
  sub_rc_ = nh_.subscribe("/mavros/rc/in", 10, &Bs_Ctrl::RcCb, this);
}

void Bs_Ctrl::initAllSrv(){
  server_align_ = nh_.advertiseService("align_beam", 
                                       &Bs_Ctrl::AlignBeamYawCb, this);
  server_takeoff_ = nh_.advertiseService("takeoff",
                                         &Bs_Ctrl::TakeoffLandCb, this);
  server_set_origin_ = nh_.advertiseService("set_origin", 
                                            &Bs_Ctrl::SetLocalOriginCb, this);

  client_arm_ = 
    nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  client_set_mode_ = 
    nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

void Bs_Ctrl::startCtrlThread(){
  ctrl_thread_ = std::thread(&Bs_Ctrl::CtrlThreadProcess, this);
}

void Bs_Ctrl::stopCtrlThread(){
  if (ctrl_thread_.joinable()){
    ctrl_thread_.join();
  }
}

void Bs_Ctrl::StateCb(const mavros_msgs::StateConstPtr &msg){
  std::lock_guard<std::mutex> guard(mtx_uav_state_);
  curr_uav_state_.is_armed_ = msg->armed;
  curr_uav_state_.is_connected_ = msg->connected;
  curr_uav_state_.mode_ = msg->mode;
}

void Bs_Ctrl::ExtStateCb(const mavros_msgs::ExtendedStateConstPtr &msg){
  std::lock_guard<std::mutex> guard(mtx_uav_state_);
  curr_uav_state_.landed_state_ = msg->landed_state;
}

void Bs_Ctrl::RcCb(const mavros_msgs::RCInConstPtr &msg){
  // TODO : add rc control logic

}

void Bs_Ctrl::LocalPoseCb(const geometry_msgs::PoseStampedConstPtr &msg){
  std::lock_guard<std::mutex> guard(mtx_uav_state_);  
  curr_uav_state_.curr_q_ = Eigen::Quaterniond(msg->pose.orientation.w,
                                               msg->pose.orientation.x,
                                               msg->pose.orientation.y,
                                               msg->pose.orientation.z);
  curr_uav_state_.curr_t_ = Eigen::Vector3d(msg->pose.position.x,
                                            msg->pose.position.y,
                                            msg->pose.position.z);
}

void Bs_Ctrl::GlobalPoseCb(const sensor_msgs::NavSatFixConstPtr &msg){
  std::lock_guard<std::mutex> guard(mtx_uav_state_);
  // check fix status
  if (msg->status.status < 0){
    return;
  }

  curr_uav_state_.curr_lla_ = Eigen::Vector3d(msg->latitude, 
                                              msg->longitude, 
                                              msg->altitude);
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

bool Bs_Ctrl::AlignBeamYawCb(base_station_ctrl::AlignBeamYaw::Request &req,
                              base_station_ctrl::AlignBeamYaw::Response &res)
{
  // origin of local cartesian must be set first.
  if (!init_local_origin_){
    res.success = false;
    res.message = "Origin of Local Cartesian has not been set, Call service SetLocalOrigin first !";    
    return true;
  }

  // TODO: ros单线程spin直接不需要加锁，而CtrlThread只进行了读操作，这里可以不加锁
  std::lock_guard<std::mutex> guard(mtx_uav_state_);
  // calculate align vector
  geo_utils::sDeltaAngles d_angles = ptr_align_->getAlignVec(
    req.lat, req.lon, req.alt,
    curr_uav_state_.curr_lla_.x(), 
    curr_uav_state_.curr_lla_.y(), 
    curr_uav_state_.curr_lla_.z(), 
    curr_uav_state_.curr_q_);
  
  res.d_pitch = d_angles.d_pitch / M_PI * 180.0;
  res.d_yaw = d_angles.d_yaw / M_PI * 180.0;
  res.success = true;
  res.message = "";
  
  // update desired q and t;
  des_q_ = curr_uav_state_.curr_q_ * Eigen::Quaterniond(
    Eigen::AngleAxisd(d_angles.d_yaw, Eigen::Vector3d::UnitZ()));

  return true;
}

bool Bs_Ctrl::TakeoffLandCb(std_srvs::SetBool::Request &req,
                            std_srvs::SetBool::Response &res)
{
  std::lock_guard<std::mutex> guard(mtx_uav_state_);
  if (req.data){
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    arm_cmd_.request.value = true;
    des_q_ = curr_uav_state_.curr_q_;
  }
  else{
    offb_set_mode_.request.custom_mode = "AUTO.LAND";
  }
  return true;
}

void Bs_Ctrl::CtrlThreadProcess(){
  // create delay lock objects
  std::unique_lock<std::mutex> lock_state(mtx_uav_state_, std::defer_lock);
  ros::Time last_request = ros::Time::now();

  while(ros::ok()){
    // check fcu is connected or not
    lock_state.lock();
    if (!curr_uav_state_.is_connected_){
      lock_state.unlock();
      continue;
    }  

    geometry_msgs::PoseStamped des_pose;
    des_pose.pose.position.x = des_t_.x();
    des_pose.pose.position.y = des_t_.y();
    des_pose.pose.position.z = des_t_.z();
    des_pose.pose.orientation.w = des_q_.w();
    des_pose.pose.orientation.x = des_q_.x();
    des_pose.pose.orientation.y = des_q_.y();
    des_pose.pose.orientation.z = des_q_.z();

    bool curr_armed = curr_uav_state_.is_armed_;
    std::string curr_mode = curr_uav_state_.mode_;

    lock_state.unlock();

    if (offb_set_mode_.request.custom_mode != curr_mode &&
        ((ros::Time::now() - last_request) > ros::Duration(2.0)))
    {
      if(client_set_mode_.call(offb_set_mode_) &&
                offb_set_mode_.response.mode_sent)
      {
        spdlog::info("Change flight mode to {}.", 
                     offb_set_mode_.request.custom_mode);
      }
      last_request = ros::Time::now();
    }
    else{
      if (curr_mode == "OFFBOARD" &&
          !curr_armed && 
          ((ros::Time::now() - last_request) > ros::Duration(2.0)))
      {
        if(client_arm_.call(arm_cmd_) && arm_cmd_.response.success)
        {
          spdlog::info("Tring to arm vehicle...");
        }
        last_request = ros::Time::now();
      }
    }

    // pub desired pose
    pub_local_pose_.publish(des_pose);

    ptr_ctrl_rate_->sleep();
  }

}

} //  namespace bs_ctrl
