/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2024 Jacob Moore
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "standalone_sim/include/standalone_sensors.hpp"

namespace rosflight_sim
{

StandaloneSensors::StandaloneSensors()
  : SensorInterface()
{
  declare_parameters();

  initialize_sensors();
}

void StandaloneSensors::declare_parameters()
{
  // TODO: These params need to be updated with empirically derived values, using the latest
  //   hardware (i.e. not the cheap boards with the cheap sensors)

  // Get Sensor Parameters
  this->declare_parameter("gyro_stdev", 0.0226);
  this->declare_parameter("gyro_bias_range", 0.25);
  this->declare_parameter("gyro_bias_walk_stdev", 0.00001);

  this->declare_parameter("acc_stdev", 0.2);
  this->declare_parameter("acc_bias_range", 0.6);
  this->declare_parameter("acc_bias_walk_stdev", 0.00001);

  this->declare_parameter("mag_stdev", 3000/1e9); // from nano tesla to tesla
  this->declare_parameter("k_mag", 7.0);
  this->declare_parameter("inclination", 1.139436457);
  this->declare_parameter("declination", 0.1857972802);
  this->declare_parameter("total_intensity", 50716.3 / 1e9);  // nanoTesla converted to tesla.

  this->declare_parameter("baro_stdev", 4.0);
  this->declare_parameter("baro_bias_range", 500);
  this->declare_parameter("baro_bias_walk_stdev", 0.1);

  this->declare_parameter("airspeed_stdev", 1.15);
  this->declare_parameter("airspeed_bias_range", 0.15);
  this->declare_parameter("airspeed_bias_walk_stdev", 0.001);

  this->declare_parameter("sonar_stdev", 0.03);
  this->declare_parameter("sonar_min_range", 0.25);
  this->declare_parameter("sonar_max_range", 8.0);

  // Get the desired altitude at the ground (for baro and LLA)
  this->declare_parameter("origin_altitude", 1387.0);
  this->declare_parameter("origin_latitude", 40.2463724);
  this->declare_parameter("origin_longitude", -111.6474138);

  this->declare_parameter("horizontal_gnss_stdev", 0.21);
  this->declare_parameter("vertical_gnss_stdev", 0.4);
  this->declare_parameter("gnss_velocity_stdev", 0.01);
  this->declare_parameter("k_gnss", 1.0/1100);

  this->declare_parameter("gravity", 9.81);
  this->declare_parameter("mass", 2.25);
}

void initialize_sensors()
{
  // Configure noise
  normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);

  // Initialize biases
  double gyro_bias_range = this->get_parameter("gyro_bias_range");
  double acc_bias_range = this->get_parameter("acc_bias_range");
  for (int i=0; i<3; ++i) {
    gyro_bias_[i] = gyro_bias_range * uniform_distribution_(bias_generator_);
    acc_bias_[i] = acc_bias_range * uniform_distribution_(bias_generator_);
  }

  baro_bias_ = this->get_parameter("baro_bias_range") * uniform_distribution_(bias_generator_);
  airspeed_bias_ = this->get_parameter("airspeed_bias_range") * uniform_distribution_(bias_generator_);

  // Initialize magnetometer
  double inclination = this->get_parameter("inclination");
  double declination = this->get_parameter("declination");
  double total_intensity = this->get_parameter("total_intensity");
  inertial_magnetic_field_ << cos(inclination) * cos(declination)
                           , cos(inclination) * sin(declination)
                           , sin(inclination);                     // In NED coordinates
  inertial_magnetic_field_.normalize();
  inertial_magnetic_field_ *= total_intensity;
}

// TODO: Move this to the dynamics object (for reading off truth from Gazebo)
// void StandaloneSensors::sensors_init()
// {
//   using SC = gazebo::common::SphericalCoordinates;
//   using Ang = ignition::math::Angle;
//   sph_coord_.SetSurfaceType(SC::SurfaceType::EARTH_WGS84);
//   sph_coord_.SetLatitudeReference(Ang(deg2Rad(origin_latitude_)));
//   sph_coord_.SetLongitudeReference(Ang(deg2Rad(origin_longitude_)));
//   sph_coord_.SetElevationReference(origin_altitude_);
//   // Force x-axis to be north-aligned. I promise, I will change everything to ENU in the next commit
//   sph_coord_.SetHeadingOffset(Ang(M_PI / 2.0));
// }

sensor_msgs::msg::Imu StandaloneSensors::imu_update(const rosflight_msgs::msg::SimState & state, const geometry_msgs::msg::TwistStamped & forces)
{
  sensor_msgs::msg::Imu out_msg;

  // Get the current rotation from inertial to the body frame
  geometry_msgs::msg::TransformStamped q_inertial_to_body;
  q_inertial_to_body.transform.rotation = state.pose.rotation;

  Eigen::Vector3d velocity(state.velocity);
  Eigen::Vector3d acceleration(state.acceleration);

  Eigen::Vector3d y_acc;
  // this is James's egregious hack to overcome wild imu while sitting on the ground
  // TODO: Check the directions of these rotations. It seemed like the previous implementation did the reverse of this.
  if (velocity.norm() < 0.05) {
    tf2::doTransform(gravity_, y_acc, q_inertial_to_body);
  } else if (abs(state.pose.translation.z) < 0.3) {
    tf2::doTransform(state.acceleration - gravity_, y_acc, q_inertial_to_body);
  } else {
    double mass = this->get_parameter("mass").as_double();
    y_acc << forces.twist.linear.x / mass,
           , forces.twist.linear.y / mass,
           , forces.twist.linear.z / mass;
  }

  // TODO: figure out how to determine if the aircraft's motors are spinning
  double acc_stdev = this->get_parameter("acc_stdev").as_double(); 
  double acc_bias_walk_stdev = this->get_parameter("acc_bias_walk_stdev").as_double(); 
  for (int i=0; i<3; ++i) {
    // Apply normal noise (only if armed, because most of the noise comes from motors
    if (motors_spinning) {
      y_acc[i] += acc_stdev * normal_distribution_(noise_generator_);
    }

    // Perform bias Walk for biases
    acc_bias_[i] += acc_bias_walk_stdev * normal_distribution_(noise_generator_);
    y_acc[i] += acc_bias_[i];
  }

  // Package acceleration into the output message
  out_msg.linear_acceleration.x = y_acc[0];
  out_msg.linear_acceleration.y = y_acc[1];
  out_msg.linear_acceleration.z = y_acc[2];

  Eigen::Vector3d y_gyro(state.angular_velocity);

  double gyro_stdev = this->get_parameter("gyro_stdev").as_double(); 
  double gyro_bias_walk_stdev = this->get_parameter("gyro_bias_walk_stdev").as_double(); 
  for (int i=0; i<3; ++i) {
    // Normal Noise from motors
    if (motors_spinning) {
      y_gyro[i] += gyro_stdev * normal_distribution_(noise_generator_);
    }

    // bias Walk for bias
    gyro_bias_[i] += gyro_bias_walk_stdev * normal_distribution_(noise_generator_);
    y_gyro[i] += gyro_bias_[i];
  }
  
  // Package angular velocity into output message
  out_msg.angular_velocity.x = y_gyro[0];
  out_msg.angular_velocity.y = y_gyro[1];
  out_msg.angular_velocity.z = y_gyro[2];

  // TODO: time manager
  out_msg.header.stamp = clock_micros();

  return out_msg;
}

sensor_msgs::msg::Temperature StandaloneSensors::imu_temperature_update(const rosflight_msgs::msg::State & state)
{
  sensor_msgs::msg::Temperature temp;
  temp.temperature = 27.0;   // In deg C (as in message spec)
  return temp;
}

sensor_msgs::msg::MagneticField StandaloneSensors::mag_update(const rosflight_msgs::msg::SimState & state)
{
  geometry_msgs::msg::TransformStamped q_inertial_to_body;
  q_inertial_to_body.transform.rotation = state.pose.rotation;

  Eigen::Vector3d y_mag;
  tf2::doTransform(inertial_magnetic_field_, y_mag, q_inertial_to_body);
  // Apply walk
  y_mag += mag_gauss_markov_eta_;
  
  // Increment the Gauss-Markov noise
  Eigen::Vector3d noise;
  noise << mag_stdev_ * normal_distribution_(noise_generator_)
         , mag_stdev_ * normal_distribution_(noise_generator_)
         , mag_stdev_ * normal_distribution_(noise_generator_);

  float T_s = 1.0/mag_update_frequency_;
  
  double k_mag = this->get_parameter("k_mag").as_double(); 
  mag_gauss_markov_eta_ = std::exp(-k_mag*T_s) * mag_gauss_markov_eta_ + T_s*noise;

  // Package data into message
  sensor_msgs::msg::MagneticField out_msg;
  out_msg.magnetic_field.x = y_mag[0];
  out_msg.magnetic_field.y = y_mag[1];
  out_msg.magnetic_field.z = y_mag[2];

  return out_msg;
}

rosflight_msgs::msg::Barometer StandaloneSensors::baro_update(const rosflight_msgs::msg::SimState & state)
{
  // Invert measurement model for pressure and temperature
  double alt = -state.pose.translation.z + origin_altitude_;

  // Convert to the true pressure reading
  double y_baro = 101325.0f
    * (float) pow((1 - 2.25694e-5 * alt), 5.2553); // TODO: Add these parameters to the parameters.

  rho_ = 1.225 * pow(y_baro / 101325.0, 0.809736894596450);

  // Add noise
  double baro_stdev = this->get_parameter("baro_stdev").as_double(); 
  y_baro += baro_stdev * normal_distribution_(noise_generator_);

  // Perform bias walk
  double baro_bias_walk_stdev = this->get_parameter("baro_bias_walk_stdev").as_double(); 
  baro_bias_ += baro_bias_walk_stdev * normal_distribution_(noise_generator_);

  // Add bias walk
  y_baro += baro_bias_;

  // Package the return message
  rosflight_msg::msg::Barometer out_msg;
  out_msg.pressure = (float) y_baro;
  out_msg.temperature = 27.0f + 273.15f;

  return out_msg;
}

// TODO: Fix the GNSS messages first
rosflight_msgs::msg::GNSS StandaloneSensors::gnss_update(const rosflight_msgs::msg::SimState & state)
{
  // using Vec3 = ignition::math::Vector3d;
  // using Coord = gazebo::common::SphericalCoordinates::CoordinateType;

  // double T_s = 1.0/gnss_update_rate_;

  // GazeboPose local_pose = GZ_COMPAT_GET_WORLD_POSE(link_);
  // Vec3 local_pos = GZ_COMPAT_GET_POS(local_pose) + gnss_gauss_markov_eta_;

  // Vec3 pos_noise(horizontal_gnss_stdev_ * normal_distribution_(noise_generator_),
  //                horizontal_gnss_stdev_ * normal_distribution_(noise_generator_),
  //                vertical_gnss_stdev_ * normal_distribution_(noise_generator_));
  // gnss_gauss_markov_eta_ = std::exp(-k_gnss_*T_s) * gnss_gauss_markov_eta_ + T_s*pos_noise;


  // Vec3 local_vel = GZ_COMPAT_GET_WORLD_LINEAR_VEL(link_);
  // Vec3 vel_noise(gnss_velocity_stdev_ * normal_distribution_(noise_generator_),
  //                gnss_velocity_stdev_ * normal_distribution_(noise_generator_),
  //                gnss_velocity_stdev_ * normal_distribution_(noise_generator_));
  // local_vel += vel_noise;

  // Vec3 ecef_pos = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::ECEF);
  // Vec3 ecef_vel = sph_coord_.VelocityTransform(local_vel, Coord::LOCAL, Coord::ECEF);
  // Vec3 lla = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::SPHERICAL);
  // 
  // gnss_.lat = (int) std::round(rad2Deg(lla.X()) * 1e7);
  // gnss_.lon = (int) std::round(rad2Deg(lla.Y()) * 1e7);
  // gnss_.height = (int) std::round(lla.Z() * 1e3);

  // // For now, we have defined the Gazebo Local Frame as NWU.  This should be fixed in a future
  // // commit
  // gnss_.vel_n = (int) std::round(local_vel.X() * 1e3);
  // gnss_.vel_e = (int) std::round(-local_vel.Y() * 1e3);
  // gnss_.vel_d = (int) std::round(-local_vel.Z() * 1e3);

  // gnss_.fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_3D_FIX;
  // gnss_.time_of_week = GZ_COMPAT_GET_SIM_TIME(world_).Double() * 1000;
  // gnss_.time = GZ_COMPAT_GET_SIM_TIME(world_).Double();
  // // TODO: time manager
  // gnss_.nanos =
  //   (uint64_t) std::round((GZ_COMPAT_GET_SIM_TIME(world_).Double() - (double) gnss_.time) * 1e9);

  // gnss_.h_acc = (int) std::round(horizontal_gnss_stdev_ * 1000.0);
  // gnss_.v_acc = (int) std::round(vertical_gnss_stdev_ * 1000.0);

  // gnss_.ecef.x = (int) std::round(ecef_pos.X() * 100);
  // gnss_.ecef.y = (int) std::round(ecef_pos.Y() * 100);
  // gnss_.ecef.z = (int) std::round(ecef_pos.Z() * 100);
  // gnss_.ecef.p_acc = (int) std::round(gnss_.h_acc / 10.0);
  // gnss_.ecef.vx = (int) std::round(ecef_vel.X() * 100);
  // gnss_.ecef.vy = (int) std::round(ecef_vel.Y() * 100);
  // gnss_.ecef.vz = (int) std::round(ecef_vel.Z() * 100);
  // gnss_.ecef.s_acc = (int) std::round(gnss_velocity_stdev_ * 100);

  // // TODO: time manager
  // gnss_.rosflight_timestamp = clock_micros();

  // using Vec3 = ignition::math::Vector3d;
  // using Vec3 = ignition::math::Vector3d;
  // using Coord = gazebo::common::SphericalCoordinates::CoordinateType;

  // // TODO: Do a better job of simulating the wander of GNSS
  // 
  // auto now = std::chrono::system_clock::now();
  // auto now_c = std::chrono::system_clock::to_time_t(now);
  // auto now_tm = std::localtime(&now_c);
  // 
  // gnss_full_.year = now_tm->tm_year + 1900;
  // gnss_full_.month = now_tm->tm_mon + 1;
  // gnss_full_.day = now_tm->tm_mday;
  // gnss_full_.hour = now_tm->tm_hour;
  // gnss_full_.min = now_tm->tm_min;
  // gnss_full_.sec = now_tm->tm_sec;
  // gnss_full_.valid = 1;

  // gnss_full_.lat = (int) std::round(rad2Deg(lla.X()) * 1e7);
  // gnss_full_.lon = (int) std::round(rad2Deg(lla.Y()) * 1e7);
  // gnss_full_.height = (int) std::round(lla.Z() * 1e3);
  // gnss_full_.height_msl = gnss_full_.height; // TODO

  // // For now, we have defined the Gazebo Local Frame as NWU.  This should be
  // // fixed in a future commit
  // gnss_full_.vel_n = (int) std::round(local_vel.X() * 1e3);
  // gnss_full_.vel_e = (int) std::round(-local_vel.Y() * 1e3);
  // gnss_full_.vel_d = (int) std::round(-local_vel.Z() * 1e3);

  // gnss_full_.fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_3D_FIX;
  // gnss_full_.time_of_week = GZ_COMPAT_GET_SIM_TIME(world_).Double() * 1000;
  // gnss_full_.num_sat = 15;
  // // TODO
  // gnss_full_.t_acc = 0;
  // gnss_full_.nano = 0;

  // gnss_full_.h_acc = (int) std::round(horizontal_gnss_stdev_ * 1000.0);
  // gnss_full_.v_acc = (int) std::round(vertical_gnss_stdev_ * 1000.0);

  // // Again, TODO switch to using ENU convention per REP
  // double vn = local_vel.X();
  // double ve = -local_vel.Y();
  // double ground_speed = std::sqrt(vn * vn + ve * ve);
  // gnss_full_.g_speed = (int) std::round(ground_speed * 1000);

  // double head_mot = std::atan2(ve, vn);
  // gnss_full_.head_mot = (int) std::round(rad2Deg(head_mot) * 1e5);
  // gnss_full_.p_dop = 0.0; // TODO
  // gnss_full_.rosflight_timestamp = clock_micros();

  rosflight_msgs::msg::GNSS out_msg;
  return out_msg;
}

sensor_msgs::msg::Range StandaloneSensors::sonar_update(const rosflight_msgs::msg::SimState & state)
{
  double alt = -state.pose.translation.z;
  double sonar_min_range = this->get_parameter("sonar_min_range").as_double();
  double sonar_max_range = this->get_parameter("sonar_max_range").as_double();
  double sonar_stdev = this->get_parameter("sonar_stdev_").as_double();

  sensor_msgs::msg::Range out_msg; 
  if (alt < sonar_min_range) {
    out_msg.range = (float) sonar_min_range;
  } else if (alt > sonar_max_range) {
    out_msg.range = (float) sonar_max_range;
  } else {
    out_msg.range = (float) (alt + sonar_stdev * normal_distribution_(noise_generator_));
  }

  return out_msg;
}

rosflight_msgs::msg::Airspeed StandaloneSensors::diff_pressure_update(const rosflight_msgs::msg::SimState & state)
{
  // Calculate Airspeed
  Eigen::Vector3d velocity(state.air_velocity);
  double Va = velocity.norm();

  // Invert Airpseed to get sensor measurement
  double y_as = rho_ * Va * Va / 2.0; // Page 130 in the UAV Book

  // Add noise
  double airspeed_stdev = this->get_parameter("airspeed_stdev").as_double(); 
  double airspeed_bias_walk_stdev = this->get_parameter("airspeed_bias_walk_stdev").as_double(); 

  y_as += airspeed_stdev * normal_distribution_(noise_generator_);

  // Perform bias walk
  airspeed_bias_ += airspeed_bias_walk_stdev * normal_distribution_(noise_generator_);
  y_as += airspeed_bias_;

  // Package the return message 
  rosflight_msgs::msg::Airspeed out_msg;
  out_msg.differential_pressure = (float) y_as;
  out_msg.temperature = 27.0 + 273.15;
}

void StandaloneSensors::battery_update(const rosflight_msgs::msg::SimState & state)
{
  battery_voltage_ = 15 * battery_voltage_multiplier_;
  battery_current_ = 1 * battery_current_multiplier_;
}


} // rosflight_sim
