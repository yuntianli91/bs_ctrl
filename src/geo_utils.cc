// @file        This file is in compliance with GNU GPL V3 license.
// @brief   
// @author      Yuntian Li (yuntianli@pmlabs.com.cn)
// @version     1.0
// @date        Last modified at 2025-06-05
// @copyright   Copyright (c) {2024 - } Purple Mountain Lab UAV TEAM.

#include "base_station_ctrl/geo_utils.h"

namespace geo_utils{
AlignBeam::AlignBeam(double lat, double lon, double alt)
  : lat_orig_(lat), lon_orig_(lon), alt_orig_(alt)
{
  enu_C_ned_ << 0., 1., 0., 1., 0., 0., 0., 0., -1.;
  enu_T_b_ = Eigen::Matrix4d::Identity();

  geo_cart_ = GeographicLib::LocalCartesian(GeographicLib::Geocentric::WGS84());
  geo_cart_.Reset(lat, lon, alt);
}

sDeltaAngles AlignBeam::getAlignVec(double lat_d, double lon_d, double alt_d,
                                    double lat_u, double lon_u, double alt_u,
                                    const Eigen::Quaterniond &q)
{
  double x_d, y_d, z_d;   // coordinate of dest in ENU
  double x_u, y_u, z_u;   // coordinate of base station uav in ENU

  geo_cart_.Forward(lat_d, lon_d, alt_d, x_d, y_d, z_d);
  geo_cart_.Forward(lat_u, lon_u, alt_u, x_u, y_u, z_u);

  enu_T_b_.block<3, 3>(0, 0) = enu_C_ned_ * q.toRotationMatrix();
  enu_T_b_.block<3, 1>(0, 3) = Eigen::Vector3d(x_u, y_u, z_u);
  
  // get align vector and convert to body frame
  Eigen::Vector4d d_vec_enu(x_d - x_u, y_d - y_u, z_d - z_u, 1.0);
  Eigen::Vector4d d_vec_b = enu_T_b_.transpose() * d_vec_enu;

  // check minimal threshold
  for (ssize_t i = 0; i < 3; i++){
    d_vec_b[i] = (abs(d_vec_b[i]) < 1.0e-9 ? 0. : d_vec_b[i]);
  }

  sDeltaAngles d_angles;
  d_angles.d_yaw = atan2(d_vec_b.y(), d_vec_b.x());
  d_angles.d_pitch = asin(d_vec_b.z() / d_vec_b.norm());
  d_angles.d_roll = 0.;

  return d_angles;
}
} // namespace geo_utils