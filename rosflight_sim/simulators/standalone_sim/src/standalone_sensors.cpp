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

#include "standalone_sensors.hpp"
#include <cstdint>
#include <random>

namespace rosflight_sim
{

StandaloneSensors::StandaloneSensors()
  : SensorInterface()
  // , bias_generator_(std::chrono::system_clock::now().time_since_epoch().count()) // Uncomment if you would like to
                                                                                    // have bias biases for the sensors
                                                                                    // on each flight. Delete next line.
  , bias_generator_(0)
  , noise_generator_(std::chrono::system_clock::now().time_since_epoch().count())
{
  declare_parameters();

  initialize_sensors();
}

void StandaloneSensors::declare_parameters()
{
  // TODO: These params need to be updated with empirically derived values, using the latest
  //   hardware (i.e. not the cheap boards with the cheap sensors)

  // declare sensor parameters
  this->declare_parameter("gyro_stdev", 0.00226);
  this->declare_parameter("gyro_bias_range", 0.25);
  this->declare_parameter("gyro_bias_walk_stdev", 0.033);
  this->declare_parameter("gyro_bias_model_tau", 400.0);
  this->declare_parameter("gyro_bias_model_x0", 0.003);
  this->declare_parameter("gyro_bias_model_y0", -0.002);
  this->declare_parameter("gyro_bias_model_z0", 0.001);
  this->declare_parameter("k_gyro", 20.0);

  this->declare_parameter("acc_stdev", 0.2);
  this->declare_parameter("acc_bias_range", 0.6);
  this->declare_parameter("acc_bias_walk_stdev", 0.00001);

  this->declare_parameter("mag_stdev", 3000.0/1e9); // from nano tesla to tesla
  this->declare_parameter("k_mag", 7.0);
  this->declare_parameter("inclination", 1.139436457);
  this->declare_parameter("declination", 0.1857972802);
  this->declare_parameter("total_intensity", 50716.3 / 1e9);  // nanoTesla converted to tesla.

  this->declare_parameter("baro_stdev", 4.0);
  this->declare_parameter("baro_bias_range", 500.0);
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
  this->declare_parameter("mass", 4.5);
}

void StandaloneSensors::initialize_sensors()
{
  // Configure noise
  normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);

  // Initialize biases
  double gyro_bias_range = this->get_parameter("gyro_bias_range").as_double();
  double acc_bias_range = this->get_parameter("acc_bias_range").as_double();
  for (int i=0; i<3; ++i) {
    gyro_bias_[i] = gyro_bias_range * uniform_distribution_(bias_generator_);
    acc_bias_[i] = acc_bias_range * uniform_distribution_(bias_generator_);
  }

  gyro_bias_instability_ << this->get_parameter("gyro_bias_model_x0").as_double()
                          , this->get_parameter("gyro_bias_model_y0").as_double()
                          , this->get_parameter("gyro_bias_model_z0").as_double();

  baro_bias_ = this->get_parameter("baro_bias_range").as_double() * uniform_distribution_(bias_generator_);
  airspeed_bias_ = this->get_parameter("airspeed_bias_range").as_double() * uniform_distribution_(bias_generator_);

  // Initialize magnetometer
  double inclination = this->get_parameter("inclination").as_double();
  double declination = this->get_parameter("declination").as_double();
  double total_intensity = this->get_parameter("total_intensity").as_double();
  inertial_magnetic_field_ << cos(inclination) * cos(declination)
                            , -cos(inclination) * sin(declination)
                            , sin(inclination);                     // In NED coordinates
  inertial_magnetic_field_.normalize();
  inertial_magnetic_field_ *= total_intensity;
}

sensor_msgs::msg::Imu StandaloneSensors::imu_update(const rosflight_msgs::msg::SimState & state, const geometry_msgs::msg::WrenchStamped & forces)
{
  sensor_msgs::msg::Imu out_msg;

  // Get the current rotation from body to inertial
  Eigen::Quaterniond q_body_to_inertial(state.pose.orientation.w,
                                        state.pose.orientation.x,
                                        state.pose.orientation.y,
                                        state.pose.orientation.z);

  Eigen::Vector3d gravity(0.0, 0.0, this->get_parameter("gravity").as_double());
  Eigen::Vector3d gravity_body = q_body_to_inertial.inverse() * gravity;

  // Accelerometer measurement in the body frame
  Eigen::Vector3d y_acc;

  // this is James's egregious hack to overcome wild imu while sitting on the ground
  Eigen::Vector3d velocity_body(state.twist.linear.x, state.twist.linear.y, state.twist.linear.z);
  if (velocity_body.norm() < 0.05) {
    y_acc = -gravity_body;
  } else {
    Eigen::Vector3d acceleration_body(state.acceleration.linear.x,
                                      state.acceleration.linear.y,
                                      state.acceleration.linear.z);
    y_acc = acceleration_body - gravity_body;
  }

  // Determine if the aircraft is armed
  rosflight_msgs::msg::Status current_status = get_current_status();
  bool motors_spinning = current_status.armed;

  double acc_stdev = this->get_parameter("acc_stdev").as_double(); 
  double acc_bias_walk_stdev = this->get_parameter("acc_bias_walk_stdev").as_double(); 
  for (int i=0; i<3; ++i) {
    // Apply normal noise (only if armed, because most of the noise comes from motors
    if (motors_spinning) {
      y_acc[i] += acc_stdev * normal_distribution_(noise_generator_);
    }

    // Perform bias Walk for biases
    // TODO: Do we need to scale this by dt? Look at what people do. The faster you run the imu, the faster the bias will change.
    acc_bias_[i] += acc_bias_walk_stdev * normal_distribution_(noise_generator_);
    y_acc[i] += acc_bias_[i];
  }

  // Package acceleration into the output message
  out_msg.linear_acceleration.x = y_acc[0];
  out_msg.linear_acceleration.y = y_acc[1];
  out_msg.linear_acceleration.z = y_acc[2];

  Eigen::Vector3d y_gyro;
  y_gyro << state.twist.angular.x, state.twist.angular.y, state.twist.angular.z;

  double gyro_stdev = this->get_parameter("gyro_stdev").as_double(); 
  double gyro_bias_walk_stdev = this->get_parameter("gyro_bias_walk_stdev").as_double(); 
  for (int i=0; i<3; ++i) {
    // Normal Noise from motors
    if (motors_spinning) {
      y_gyro[i] += gyro_stdev * normal_distribution_(noise_generator_);
    }
  }

  // Add constant gyro bias
  y_gyro += gyro_bias_;

  // Add bias walk
  y_gyro += gyro_bias_gauss_markov_eta_;

  Eigen::Vector3d noise;
  noise << gyro_bias_walk_stdev * normal_distribution_(noise_generator_)
        , gyro_bias_walk_stdev * normal_distribution_(noise_generator_)
        , gyro_bias_walk_stdev * normal_distribution_(noise_generator_);
  double T_s = 1.0 / get_imu_update_frequency();
  double k_gyro = this->get_parameter("k_gyro").as_double();
  gyro_bias_gauss_markov_eta_ = std::exp(-k_gyro*T_s) * gyro_bias_gauss_markov_eta_ + T_s*noise;

  // Add bias instability
  y_gyro += bias_model();

  // Package angular velocity into output message
  out_msg.angular_velocity.x = y_gyro[0];
  out_msg.angular_velocity.y = y_gyro[1];
  out_msg.angular_velocity.z = y_gyro[2];

  out_msg.header.stamp = this->get_clock()->now();
  return out_msg;
}

Eigen::Vector3d StandaloneSensors::bias_model() 
{
  double T = 1/get_imu_update_frequency();
  double alpha = T/(T+this->get_parameter("gyro_bias_model_tau").as_double()); 
  gyro_bias_instability_ *= (1 - alpha);
  return gyro_bias_instability_;
}

sensor_msgs::msg::Temperature StandaloneSensors::imu_temperature_update(const rosflight_msgs::msg::SimState & state)
{
  sensor_msgs::msg::Temperature temp;
  temp.temperature = 27.0;   // In deg C (as in message spec)
  return temp;
}

sensor_msgs::msg::MagneticField StandaloneSensors::mag_update(const rosflight_msgs::msg::SimState & state)
{
  // Get the current rotation from body to inertial
  Eigen::Quaterniond q_body_to_inertial(state.pose.orientation.w,
                                        state.pose.orientation.x,
                                        state.pose.orientation.y,
                                        state.pose.orientation.z);

  Eigen::Vector3d y_mag = q_body_to_inertial.inverse() * inertial_magnetic_field_;

  // Apply walk
  y_mag += mag_gauss_markov_eta_;
  
  // Increment the Gauss-Markov noise
  double mag_stdev = this->get_parameter("mag_stdev").as_double();
  Eigen::Vector3d noise;
  noise << mag_stdev * normal_distribution_(noise_generator_)
         , mag_stdev * normal_distribution_(noise_generator_)
         , mag_stdev * normal_distribution_(noise_generator_);

  float T_s = 1.0/get_mag_update_frequency();
  
  double k_mag = this->get_parameter("k_mag").as_double(); 
  mag_gauss_markov_eta_ = std::exp(-k_mag*T_s) * mag_gauss_markov_eta_ + T_s*noise;

  // Package data into message
  sensor_msgs::msg::MagneticField out_msg;
  out_msg.magnetic_field.x = y_mag[0];
  out_msg.magnetic_field.y = y_mag[1];
  out_msg.magnetic_field.z = y_mag[2];

  out_msg.header.stamp = this->get_clock()->now();
  return out_msg;
}

rosflight_msgs::msg::Barometer StandaloneSensors::baro_update(const rosflight_msgs::msg::SimState & state)
{
  // Invert measurement model for pressure and temperature
  double alt = -state.pose.position.z + this->get_parameter("origin_altitude").as_double();

  // Convert to the true pressure reading
  double y_baro = 101325.0f
    * (float) pow((1 - 2.25694e-5 * alt), 5.2553);

  rho_ = 1.225 * pow(y_baro / 101325.0, 0.809736894596450);

  // Add noise
  double baro_stdev = this->get_parameter("baro_stdev").as_double(); 
  y_baro += baro_stdev * normal_distribution_(noise_generator_);

  // Perform bias walk
  double baro_bias_walk_stdev = this->get_parameter("baro_bias_walk_stdev").as_double(); 
  // TODO: Scale this by dt -- otherwise bias will change faster with a faster baro rate
  baro_bias_ += baro_bias_walk_stdev * normal_distribution_(noise_generator_);

  // Add bias walk
  y_baro += baro_bias_;

  // Package the return message
  rosflight_msgs::msg::Barometer out_msg;
  out_msg.pressure = (float) y_baro;
  out_msg.temperature = 27.0f + 273.15f;

  out_msg.header.stamp = this->get_clock()->now();
  return out_msg;
}

rosflight_msgs::msg::GNSS StandaloneSensors::gnss_update(const rosflight_msgs::msg::SimState & state)
{
  rosflight_msgs::msg::GNSS out_msg;

  // Compute GNSS location
  Eigen::Vector3d body_pose;
  body_pose << state.pose.position.x, state.pose.position.y, state.pose.position.z; // inertial NED (m)

  // Add random walk, then update random walk
  body_pose += gnss_gauss_markov_eta_;

  double T_s = 1.0/get_gnss_update_frequency();
  double h_std = this->get_parameter("horizontal_gnss_stdev").as_double();
  double v_std = this->get_parameter("vertical_gnss_stdev").as_double();
  double k_gnss = this->get_parameter("k_gnss").as_double();
  Eigen::Vector3d pos_noise(h_std * normal_distribution_(noise_generator_),
                            h_std * normal_distribution_(noise_generator_),
                            v_std * normal_distribution_(noise_generator_));
  gnss_gauss_markov_eta_ = std::exp(-k_gnss*T_s) * gnss_gauss_markov_eta_ + T_s*pos_noise;

  // Compute LLA (assuming spherical earth model)
  double init_lat = this->get_parameter("origin_latitude").as_double();
  double init_lon = this->get_parameter("origin_longitude").as_double();
  double init_alt = this->get_parameter("origin_altitude").as_double();
  double lat = 180.0 / (EARTH_RADIUS * M_PI) * body_pose(0) + init_lat;
  double lon = 180.0 / (EARTH_RADIUS * M_PI) 
    / cos(init_lat * M_PI / 180.0) * body_pose(1) + init_lon;
  double alt = -body_pose(2) + init_alt;

  out_msg.lat = lat;
  out_msg.lon = lon;
  out_msg.alt = alt;

  // Compute GNSS velocity
  Eigen::Vector3d body_vel;
  body_vel << state.twist.linear.x, state.twist.linear.y, state.twist.linear.z;  // NED
  double vel_std = this->get_parameter("gnss_velocity_stdev").as_double();
  Eigen::Vector3d vel_noise(vel_std * normal_distribution_(noise_generator_),
                            vel_std * normal_distribution_(noise_generator_),
                            vel_std * normal_distribution_(noise_generator_));
  body_vel += vel_noise;

  // Body to inertial rotation
  Eigen::Quaterniond q_body_to_inertial(state.pose.orientation.w,
                                        state.pose.orientation.x,
                                        state.pose.orientation.y,
                                        state.pose.orientation.z);

  Eigen::Vector3d global_vel = q_body_to_inertial * body_vel;
  out_msg.vel_n = global_vel(0);
  out_msg.vel_e = global_vel(1);
  out_msg.vel_d = global_vel(2);

  // Fill in the rest of the message
  out_msg.fix_type = rosflight_msgs::msg::GNSS::GNSS_FIX_TYPE_3D_FIX;

  out_msg.num_sat = 15;
  out_msg.horizontal_accuracy = h_std;
  out_msg.vertical_accuracy = v_std;
  out_msg.speed_accuracy = vel_std;

  // GNSS time
  int64_t now = static_cast<int64_t>(this->get_clock()->now().nanoseconds()); // nanoseconds() returns time since unix epoch, not just fractional time
  out_msg.gnss_unix_seconds = now / 1'000'000'000;
  out_msg.gnss_unix_nanos = static_cast<int32_t>(now % 1'000'000'000);

  // Estimated ROS time of the last packet
  out_msg.header.stamp = rclcpp::Time(now);

  return out_msg;
}

sensor_msgs::msg::Range StandaloneSensors::sonar_update(const rosflight_msgs::msg::SimState & state)
{
  double alt = -state.pose.position.z;
  double sonar_min_range = this->get_parameter("sonar_min_range").as_double();
  double sonar_max_range = this->get_parameter("sonar_max_range").as_double();
  double sonar_stdev = this->get_parameter("sonar_stdev").as_double();

  sensor_msgs::msg::Range out_msg; 
  if (alt < sonar_min_range) {
    out_msg.range = (float) sonar_min_range;
  } else if (alt > sonar_max_range) {
    out_msg.range = (float) sonar_max_range;
  } else {
    out_msg.range = (float) (alt + sonar_stdev * normal_distribution_(noise_generator_));
  }

  out_msg.header.stamp = this->get_clock()->now();
  return out_msg;
}

rosflight_msgs::msg::Airspeed StandaloneSensors::diff_pressure_update(const rosflight_msgs::msg::SimState & state, const geometry_msgs::msg::Vector3Stamped & wind)
{
  // Calculate Airspeed
  // Rotate wind velocities into the body frame
  Eigen::Quaterniond q_body_to_inertial(state.pose.orientation.w,
                                        state.pose.orientation.x,
                                        state.pose.orientation.y,
                                        state.pose.orientation.z);

  Eigen::Vector3d body_vel(state.twist.linear.x, state.twist.linear.y, state.twist.linear.z);
  Eigen::Vector3d inertial_wind_velocity(wind.vector.x, wind.vector.y, wind.vector.z);
  Eigen::Vector3d body_air_velocity = body_vel - q_body_to_inertial.inverse() * inertial_wind_velocity;

  // Airspeed sensor only measures x component of the airspeed in the body frame
  double u = body_air_velocity(0);

  // Invert Airpseed to get sensor measurement
  double y_as = rho_ * u * u / 2.0; // Page 130 in the UAV Book

  // Add noise
  double airspeed_stdev = this->get_parameter("airspeed_stdev").as_double(); 
  double airspeed_bias_walk_stdev = this->get_parameter("airspeed_bias_walk_stdev").as_double(); 

  y_as += airspeed_stdev * normal_distribution_(noise_generator_);

  // Perform bias walk
  // TODO: Scale this by dt
  airspeed_bias_ += airspeed_bias_walk_stdev * normal_distribution_(noise_generator_);
  y_as += airspeed_bias_;

  // Package the return message 
  rosflight_msgs::msg::Airspeed out_msg;
  out_msg.differential_pressure = (float) y_as;
  out_msg.temperature = 27.0 + 273.15;

  out_msg.header.stamp = this->get_clock()->now();
  return out_msg;
}

rosflight_msgs::msg::BatteryStatus StandaloneSensors::battery_update(const rosflight_msgs::msg::SimState & state)
{
  rosflight_msgs::msg::BatteryStatus out_msg;

  // Send without battery and current multipliers. These will get added in the sil board
  out_msg.voltage = 15;
  out_msg.current = 1;

  out_msg.header.stamp = this->get_clock()->now();
  return out_msg;
}

} // rosflight_sim

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosflight_sim::StandaloneSensors>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
