// @file        This file is in compliance with GNU GPL V3 license.
// @brief       ROS Class for base station uav control
// @author      Yuntian Li (yuntianli@pmlabs.com.cn)
// @version     1.0
// @date        Last modified at 2025-06-05
// @copyright   Copyright (c) {2024 - } Purple Mountain Lab UAV TEAM.

#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>
#include <spdlog/spdlog.h>

namespace geo_utils{

constexpr double kDEG2RAD = M_PI / 180.;
constexpr double kRAD2DEG = 180. / M_PI;

typedef struct DeltaAngles{
  double d_yaw, d_pitch, d_roll;
}sDeltaAngles;

class AlignBeam{
 public:
  // @brief   Create AlignBeam object with explicit constructor
  // @param   lat : latitude of local cartesian (ENU)
  // @param   lon : longitude of local cartesian (ENU)
  // @param   alt : altitude of local cartesian (ENU)
  explicit AlignBeam(double lat, double lon, double alt);

  ~AlignBeam() {} 
  
  // @brief   set origin of local cartesian
  // @param   lat : latitude of origin in degree
  // @param   lon : longitude of origin in degree
  // @param   alt : altitude of origin in degree
  void setOriginLLA(double lat, double lon, double alt){
    lat_orig_ = lat;
    lon_orig_ = lon;
    alt_orig_ = alt;
    geo_cart_.Reset(lat_orig_, lon_orig_);
  }
  
  // @brief   get align vector from uav to dest
  // @param   lat_d : latitude of destination in degree
  // @param   lon_d : longitude of destination in degree
  // @param   alt_d : altitude of destination in degree
  // @param   lat_u : latitude of base station uav in degree
  // @param   lon_u : longitude of base station uav in degree
  // @param   alt_u : altitude of base station uav in degree
  // @param   q : quaternion of base station uav
  // @return  sDeltaAngles : align vector
  sDeltaAngles getAlignVec(double lat_d, double lon_d, double alt_d,
                           double lat_u, double lon_u, double alt_u,
                           const Eigen::Quaterniond &q);

 private:
  bool init_orig_{false};   // coordinate of Origin initialization flag 
  double lat_orig_{0.};   // latitude of local cartesian (ENU)
  double lon_orig_{0.};   // longitude of local cartesian (ENU)
  double alt_orig_{0.};   // altitude of local cartesian (ENU)

  GeographicLib::LocalCartesian geo_cart_;  // local cartesian for LLA % ENU

  Eigen::Matrix3d enu_C_ned_;   // C from ned to enu
  Eigen::Matrix4d enu_T_b_;           //  T from b to enu
};

} // namespace geo_utils