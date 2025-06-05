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
  AlignBeam() = delete;

  // @brief   Create AlignBeam object with explicit constructor
  // @param   lat : latitude of local cartesian
  // @param   lon : longitude of local cartesian
  // @param   alt : altitude of local cartesian
  explicit AlignBeam(double lat, double lon, double alt);

  ~AlignBeam() {} 
  
  // @brief   
  // @param   lat : 
  // @param   lon : 
  // @param   alt : 
  void setOriginLLA(double lat, double lon, double alt)
    : lat_orig_(lat), lon_orig_(lon), alt_orig_(alt) {}

 private:
  bool init_orig_{false};   // coordinate of Origin initialization flag
  
  // Origin LLA coordinate of Local Cartesian
  double lat_orig_, lon_orig_, alt_orig_;    

  GeographicLib::LocalCartesian geo_cart_;  // local cartesian for LLA % ENU

  const Eigen::Matrix3d enu_C_ned_; // 
}

} // namespace geo_utils