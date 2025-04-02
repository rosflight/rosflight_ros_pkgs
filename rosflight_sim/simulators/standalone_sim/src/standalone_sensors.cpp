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
  this->declare_parameter("gyro_stdev", 0.0226);
  this->declare_parameter("gyro_bias_range", 0.25);
  this->declare_parameter("gyro_bias_walk_stdev", 0.00001);

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
  this->declare_parameter("mass", 2.25);

  this->declare_parameter("initial_latitude", 40.24647);
  this->declare_parameter("initial_longitude", -111.64857);
  this->declare_parameter("initial_altitude", 1387.0);
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

  baro_bias_ = this->get_parameter("baro_bias_range").as_double() * uniform_distribution_(bias_generator_);
  airspeed_bias_ = this->get_parameter("airspeed_bias_range").as_double() * uniform_distribution_(bias_generator_);

  // Initialize magnetometer
  double inclination = this->get_parameter("inclination").as_double();
  double declination = this->get_parameter("declination").as_double();
  double total_intensity = this->get_parameter("total_intensity").as_double();
  inertial_magnetic_field_ << cos(inclination) * cos(declination)
                           , cos(inclination) * sin(declination)
                           , sin(inclination);                     // In NED coordinates
  inertial_magnetic_field_.normalize();
  inertial_magnetic_field_ *= total_intensity;
}

sensor_msgs::msg::Imu StandaloneSensors::imu_update(const rosflight_msgs::msg::SimState & state, const geometry_msgs::msg::WrenchStamped & forces)
{
  sensor_msgs::msg::Imu out_msg;

  // Get the current rotation from inertial to the body frame
  geometry_msgs::msg::TransformStamped q_inertial_to_body;
  q_inertial_to_body.transform.rotation = state.pose.orientation;

  // Form body to inertial rotation
  geometry_msgs::msg::TransformStamped q_body_to_inertial = q_inertial_to_body;
  q_body_to_inertial.transform.rotation.x *= -1;
  q_body_to_inertial.transform.rotation.y *= -1;
  q_body_to_inertial.transform.rotation.z *= -1;

  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  gravity[2] = this->get_parameter("gravity").as_double();

  Eigen::Vector3d velocity, acceleration; // Acceleration and velocity expressed in the body frame
  velocity << state.twist.linear.x, state.twist.linear.y, state.twist.linear.z;
  acceleration << state.acceleration.linear.x, state.acceleration.linear.y, state.acceleration.linear.z;

  // Rotate acceleration into inertial frame and subtract gravity
  tf2::doTransform(acceleration, acceleration, q_body_to_inertial);
  Eigen::Vector3d acceleration_minus_gravity = acceleration - gravity;

  Eigen::Vector3d y_acc;

  // this is James's egregious hack to overcome wild imu while sitting on the ground
  if (velocity.norm() < 0.05) {
    Eigen::Vector3d minus_gravity = -gravity;
    tf2::doTransform(minus_gravity, y_acc, q_inertial_to_body);
  // TODO: Do we need this branch? Seems equivalent to the third branch
  // } else if (abs(state.pose.position.z) < 0.3) {
  //   tf2::doTransform(acceleration_minus_gravity, y_acc, q_inertial_to_body);
  } else {
    // Rotate the acceleration_minus_gravity into the body frame
    tf2::doTransform(acceleration_minus_gravity, y_acc, q_inertial_to_body);
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

    // bias Walk for bias
    gyro_bias_[i] += gyro_bias_walk_stdev * normal_distribution_(noise_generator_);
    y_gyro[i] += gyro_bias_[i];
  }
  
  // Package angular velocity into output message
  out_msg.angular_velocity.x = y_gyro[0];
  out_msg.angular_velocity.y = y_gyro[1];
  out_msg.angular_velocity.z = y_gyro[2];

  out_msg.header.stamp = this->now();
  return out_msg;
}

sensor_msgs::msg::Temperature StandaloneSensors::imu_temperature_update(const rosflight_msgs::msg::SimState & state)
{
  sensor_msgs::msg::Temperature temp;
  temp.temperature = 27.0;   // In deg C (as in message spec)
  return temp;
}

sensor_msgs::msg::MagneticField StandaloneSensors::mag_update(const rosflight_msgs::msg::SimState & state)
{
  geometry_msgs::msg::TransformStamped q_inertial_to_body;
  q_inertial_to_body.transform.rotation = state.pose.orientation;

  Eigen::Vector3d y_mag;
  tf2::doTransform(inertial_magnetic_field_, y_mag, q_inertial_to_body);
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

  out_msg.header.stamp = this->now();
  return out_msg;
}

rosflight_msgs::msg::Barometer StandaloneSensors::baro_update(const rosflight_msgs::msg::SimState & state)
{
  // Invert measurement model for pressure and temperature
  double alt = -state.pose.position.z + this->get_parameter("origin_altitude").as_double();

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
  rosflight_msgs::msg::Barometer out_msg;
  out_msg.pressure = (float) y_baro;
  out_msg.temperature = 27.0f + 273.15f;

  out_msg.header.stamp = this->now();
  return out_msg;
}

rosflight_msgs::msg::GNSS StandaloneSensors::gnss_update(const rosflight_msgs::msg::SimState & state)
{
  rosflight_msgs::msg::GNSS out_msg;

  // Compute GNSS location 
  Eigen::Vector3d local_pose;
  local_pose << state.pose.position.x, state.pose.position.y, state.pose.position.z; // inertial NED (m)

  // Add random walk, then update random walk
  local_pose += gnss_gauss_markov_eta_;

  double T_s = 1.0/get_gnss_update_frequency();
  double h_std = this->get_parameter("horizontal_gnss_stdev").as_double();
  double v_std = this->get_parameter("vertical_gnss_stdev").as_double();
  double k_gnss = this->get_parameter("k_gnss").as_double();
  Eigen::Vector3d pos_noise(h_std * normal_distribution_(noise_generator_),
                            h_std * normal_distribution_(noise_generator_),
                            v_std * normal_distribution_(noise_generator_));
  gnss_gauss_markov_eta_ = std::exp(-k_gnss*T_s) * gnss_gauss_markov_eta_ + T_s*pos_noise;

  // Compute LLA (assuming spherical earth model)
  double init_lat = this->get_parameter("initial_latitude").as_double();
  double init_lon = this->get_parameter("initial_longitude").as_double();
  double init_alt = this->get_parameter("initial_altitude").as_double();
  double lat = 180.0 / (EARTH_RADIUS * M_PI) * local_pose(0) + init_lat;
  double lon = 180.0 / (EARTH_RADIUS * M_PI) 
    / cos(init_lat * M_PI / 180.0) * local_pose(1) + init_lon;
  double alt = -local_pose(2) + init_alt;

  out_msg.lat = lat;
  out_msg.lon = lon;
  out_msg.alt = alt;

  // Compute GNSS velocity
  Eigen::Vector3d local_vel;
  local_vel << state.twist.linear.x, state.twist.linear.y, state.twist.linear.z;  // NED
  double vel_std = this->get_parameter("gnss_velocity_stdev").as_double();
  Eigen::Vector3d vel_noise(vel_std * normal_distribution_(noise_generator_),
                            vel_std * normal_distribution_(noise_generator_),
                            vel_std * normal_distribution_(noise_generator_));
  local_vel += vel_noise;

  Eigen::Quaterniond quat(state.pose.orientation.w,
                          state.pose.orientation.x,
                          state.pose.orientation.y,
                          state.pose.orientation.z);

  Eigen::Vector3d global_vel = quat.toRotationMatrix().transpose()*local_vel;
  out_msg.vel_n = global_vel(0);
  out_msg.vel_e = global_vel(1);
  out_msg.vel_d = global_vel(2);

  // Fill in the rest of the message
  out_msg.fix_type = rosflight_msgs::msg::GNSS::GNSS_FIX_TYPE_3D_FIX;

  // Add GNSS time
  auto curr_nanos = this->get_clock()->now().nanoseconds();
  auto now = std::chrono::system_clock::time_point(std::chrono::nanoseconds(1)*curr_nanos);
  auto now_c = std::chrono::system_clock::to_time_t(now);
  auto now_tm = std::localtime(&now_c);

  out_msg.year = now_tm->tm_year + 1900;
  out_msg.month = now_tm->tm_mon + 1;
  out_msg.day = now_tm->tm_mday;
  out_msg.hour = now_tm->tm_hour;
  out_msg.min = now_tm->tm_min;
  out_msg.sec = now_tm->tm_sec;

  out_msg.num_sat = 15;
  out_msg.horizontal_accuracy = h_std;
  out_msg.vertical_accuracy = v_std;
  out_msg.speed_accuracy = vel_std;
  out_msg.header.stamp = this->now();

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

  out_msg.header.stamp = this->now();
  return out_msg;
}

rosflight_msgs::msg::Airspeed StandaloneSensors::diff_pressure_update(const rosflight_msgs::msg::SimState & state, const geometry_msgs::msg::Vector3Stamped & wind)
{
  // Calculate Airspeed
  Eigen::Vector3d ground_velocity, inertial_wind_velocity, air_velocity;
  ground_velocity << state.twist.linear.x, state.twist.linear.y, state.twist.linear.z;
  inertial_wind_velocity << wind.vector.x, wind.vector.y, wind.vector.z;
  air_velocity = ground_velocity - inertial_wind_velocity;
  double Va = air_velocity.norm();

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

  out_msg.header.stamp = this->now();
  return out_msg;
}

rosflight_msgs::msg::BatteryStatus StandaloneSensors::battery_update(const rosflight_msgs::msg::SimState & state)
{
  rosflight_msgs::msg::BatteryStatus out_msg;

  // Send without battery and current multipliers. These will get added in the sil board
  out_msg.voltage = 15;
  out_msg.current = 1;

  out_msg.header.stamp = this->now();
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
