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

namespace rosflight_sim
{

GazeboSensors::GazeboSensors()
{
  // Initialize the sensors
}

void GazeboSensors::simulator_setup()
{

}

void SILBoard::gazebo_setup(gazebo::physics::LinkPtr link, gazebo::physics::WorldPtr world,
                            gazebo::physics::ModelPtr model, rclcpp::Node::SharedPtr node,
                            std::string mav_type)
{
  link_ = link;
  world_ = world;
  model_ = model;
  node_ = node;
  mav_type_ = std::move(mav_type);

  auto bind_host = node_->get_parameter_or<std::string>("gazebo_host", "localhost");
  int bind_port = node_->get_parameter_or<int>("gazebo_port", 14525);
  auto remote_host = node_->get_parameter_or<std::string>("ROS_host", "localhost");
  int remote_port = node_->get_parameter_or<int>("ROS_port", 14520);

  set_ports(bind_host, bind_port, remote_host, remote_port);
  gzmsg << "ROSflight SIL Conneced to " << remote_host << ":" << remote_port << " from "
        << bind_host << ":" << bind_port << "\n";

  // TODO: These params need to be updated with empirically derived values, using the latest
  //   hardware (i.e. not the cheap boards with the cheap sensors)

  // Get communication delay parameters, in nanoseconds
  serial_delay_ns_ = node_->get_parameter_or<long>("serial_delay_ns", 0.006 * 1e9);

  // Get Sensor Parameters
  gyro_stdev_ = node_->get_parameter_or<double>("gyro_stdev", 0.0226);
  gyro_bias_range_ = node_->get_parameter_or<double>("gyro_bias_range", 0.25);
  gyro_bias_walk_stdev_ = node_->get_parameter_or<double>("gyro_bias_walk_stdev", 0.00001);

  acc_stdev_ = node_->get_parameter_or<double>("acc_stdev", 0.2);
  acc_bias_range_ = node_->get_parameter_or<double>("acc_bias_range", 0.6);
  acc_bias_walk_stdev_ = node_->get_parameter_or<double>("acc_bias_walk_stdev", 0.00001);

  mag_stdev_ = node_->get_parameter_or<double>("mag_stdev", 3000/1e9); // from nano tesla to tesla
  k_mag_ = node_->get_parameter_or<double>("k_mag", 7.0);

  baro_stdev_ = node_->get_parameter_or<double>("baro_stdev", 4.0);
  baro_bias_range_ = node_->get_parameter_or<double>("baro_bias_range", 500);
  baro_bias_walk_stdev_ = node_->get_parameter_or<double>("baro_bias_walk_stdev", 0.1);

  airspeed_stdev_ = node_->get_parameter_or<double>("airspeed_stdev", 1.15);
  airspeed_bias_range_ = node_->get_parameter_or<double>("airspeed_bias_range", 0.15);
  airspeed_bias_walk_stdev_ = node_->get_parameter_or<double>("airspeed_bias_walk_stdev", 0.001);

  sonar_stdev_ = node_->get_parameter_or<double>("sonar_stdev", 0.03);
  sonar_min_range_ = node_->get_parameter_or<double>("sonar_min_range", 0.25);
  sonar_max_range_ = node_->get_parameter_or<double>("sonar_max_range", 8.0);

  imu_update_rate_ = node_->get_parameter_or<double>("imu_update_rate", 1000.0);
  imu_update_period_us_ = (uint64_t) (1e6 / imu_update_rate_);

  mag_update_rate_ = node_->get_parameter_or<double>("mag_update_rate", 50.0);
  mag_update_period_us_ = (uint64_t) (1e6 / mag_update_rate_);

  GZ_COMPAT_SET_X(mag_gauss_markov_eta_, 0.0);
  GZ_COMPAT_SET_Y(mag_gauss_markov_eta_, 0.0);
  GZ_COMPAT_SET_Z(mag_gauss_markov_eta_, 0.0);

  gnss_update_rate_ = node_->get_parameter_or<double>("gnss_update_rate", 10.0);
  gnss_update_period_us_ = (uint64_t) (1e6 / gnss_update_rate_);

  baro_update_rate_ = node_->get_parameter_or<double>("baro_update_rate", 50.0);
  baro_update_period_us_ = (uint64_t) (1e6 / baro_update_rate_);

  diff_pressure_update_rate_ = node_->get_parameter_or<double>("diff_pressure_update_rate", 50.0);
  diff_pressure_update_period_us_ = (uint64_t) (1e6 / diff_pressure_update_rate_);

  sonar_update_rate_ = node_->get_parameter_or<double>("sonar_update_rate", 50.0);
  sonar_update_period_us_ = (uint64_t) (1e6 / sonar_update_rate_);

  rc_update_rate_ = node_->get_parameter_or<double>("rc_update_rate", 50.0);
  rc_update_period_us_ = (uint64_t) (1e6 / rc_update_rate_);

  battery_update_rate_ = node_->get_parameter_or<double>("battery_update_rate", 5.0);
  battery_update_period_us_ = (uint64_t) (1e6 / battery_update_rate_);

  mass_ = node_->get_parameter_or<double>("mass", 2.28);
  rho_ = node_->get_parameter_or<double>("rho", 1.225);

  // Get the desired altitude at the ground (for baro and LLA)

  origin_altitude_ = node_->get_parameter_or<double>("origin_altitude", 1387.0);
  origin_latitude_ = node_->get_parameter_or<double>("origin_latitude", 40.2463724);
  origin_longitude_ = node_->get_parameter_or<double>("origin_longitude", -111.6474138);

  horizontal_gnss_stdev_ = node_->get_parameter_or<double>("horizontal_gnss_stdev", 0.21);
  vertical_gnss_stdev_ = node_->get_parameter_or<double>("vertical_gnss_stdev", 0.4);
  gnss_velocity_stdev_ = node_->get_parameter_or<double>("gnss_velocity_stdev", 0.01);
  k_gnss_ = node_->get_parameter_or<double>("k_gnss", 1.0/1100);
  
  GZ_COMPAT_SET_X(gnss_gauss_markov_eta_, 0.0);
  GZ_COMPAT_SET_Y(gnss_gauss_markov_eta_, 0.0);
  GZ_COMPAT_SET_Z(gnss_gauss_markov_eta_, 0.0);

  // Configure Noise
  normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);

  gravity_ = GZ_COMPAT_GET_GRAVITY(world_);

  // Initialize the Sensor Biases
  GZ_COMPAT_SET_X(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_X(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Y(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Z(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));
  baro_bias_ = baro_bias_range_ * uniform_distribution_(bias_generator_);
  airspeed_bias_ = airspeed_bias_range_ * uniform_distribution_(bias_generator_);

  prev_vel_1_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  prev_vel_2_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  prev_vel_3_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  last_time_ = GZ_COMPAT_GET_SIM_TIME(world_);
  next_imu_update_time_us_ = 0;
  next_mag_update_time_us_ = 0;
}


void GazeboSensors::sensors_init()
{
  // Initialize the Biases
  GZ_COMPAT_SET_X(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_, gyro_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_X(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Y(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));
  GZ_COMPAT_SET_Z(acc_bias_, acc_bias_range_ * uniform_distribution_(bias_generator_));

  // Gazebo coordinates is NWU and Earth's magnetic field is defined in NED, hence the negative signs
  auto inclination_ = node_->get_parameter_or<double>("inclination", 1.139436457);
  auto declination_ = node_->get_parameter_or<double>("declination", 0.1857972802);
  double total_intensity = node_->get_parameter_or<double>("total_intensity", 50716.3 / 1e9); // nanoTesla converted to tesla.
  
  GZ_COMPAT_SET_Z(inertial_magnetic_field_, sin(-inclination_));
  GZ_COMPAT_SET_X(inertial_magnetic_field_, cos(-inclination_) * cos(-declination_));
  GZ_COMPAT_SET_Y(inertial_magnetic_field_, cos(-inclination_) * sin(-declination_));
  inertial_magnetic_field_ = inertial_magnetic_field_.Normalized();
  inertial_magnetic_field_ *= total_intensity;

  using SC = gazebo::common::SphericalCoordinates;
  using Ang = ignition::math::Angle;
  sph_coord_.SetSurfaceType(SC::SurfaceType::EARTH_WGS84);
  sph_coord_.SetLatitudeReference(Ang(deg2Rad(origin_latitude_)));
  sph_coord_.SetLongitudeReference(Ang(deg2Rad(origin_longitude_)));
  sph_coord_.SetElevationReference(origin_altitude_);
  // Force x-axis to be north-aligned. I promise, I will change everything to ENU in the next commit
  sph_coord_.SetHeadingOffset(Ang(M_PI / 2.0));
}

void GazeboSensors::imu_update(rosflight_msgs::msg::State state, bool motors_spinning)
{
  GazeboQuaternion q_I_NWU = GZ_COMPAT_GET_ROT(GZ_COMPAT_GET_WORLD_POSE(link_));
  GazeboVector current_vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector y_acc;
  GazeboPose local_pose = GZ_COMPAT_GET_WORLD_POSE(link_);

  // this is James's egregious hack to overcome wild imu while sitting on the ground
  if (GZ_COMPAT_GET_LENGTH(current_vel) < 0.05) {
    y_acc = q_I_NWU.RotateVectorReverse(-gravity_);
  } else if (local_pose.Z() < 0.3) {
    y_acc = q_I_NWU.RotateVectorReverse(GZ_COMPAT_GET_WORLD_LINEAR_ACCEL(link_) - gravity_);
  } else {
    y_acc.Set(f_x / mass_, -f_y / mass_, -f_z / mass_);
  }

  // Apply normal noise (only if armed, because most of the noise comes from motors
  // TODO: figure out how to determine if the aircraft's motors are spinning
  if (motors_spinning) {
    GZ_COMPAT_SET_X(y_acc,
                    GZ_COMPAT_GET_X(y_acc) + acc_stdev_ * normal_distribution_(noise_generator_));
    GZ_COMPAT_SET_Y(y_acc,
                    GZ_COMPAT_GET_Y(y_acc) + acc_stdev_ * normal_distribution_(noise_generator_));
    GZ_COMPAT_SET_Z(y_acc,
                    GZ_COMPAT_GET_Z(y_acc) + acc_stdev_ * normal_distribution_(noise_generator_));
  }

  // Perform bias Walk for biases
  GZ_COMPAT_SET_X(acc_bias_,
                    acc_bias_[0]
                    + acc_bias_walk_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Y(acc_bias_,
                    acc_bias_[1]
                    + acc_bias_walk_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Z(acc_bias_,
                    acc_bias_[2]
                    + acc_bias_walk_stdev_ * normal_distribution_(noise_generator_));

  // Add constant Bias to measurement
  GZ_COMPAT_SET_X(y_acc, GZ_COMPAT_GET_X(y_acc) + GZ_COMPAT_GET_X(acc_bias_));
  GZ_COMPAT_SET_Y(y_acc, GZ_COMPAT_GET_Y(y_acc) + GZ_COMPAT_GET_Y(acc_bias_));
  GZ_COMPAT_SET_Z(y_acc, GZ_COMPAT_GET_Z(y_acc) + GZ_COMPAT_GET_Z(acc_bias_));

  // Convert to NED for output
  accel_[0] = GZ_COMPAT_GET_X(y_acc);
  accel_[1] = (float) -GZ_COMPAT_GET_Y(y_acc);
  accel_[2] = (float) -GZ_COMPAT_GET_Z(y_acc);

  GazeboVector y_gyro = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

  // Normal Noise from motors
  if (motors_spinning) {
    GZ_COMPAT_SET_X(y_gyro,
                    GZ_COMPAT_GET_X(y_gyro) + gyro_stdev_ * normal_distribution_(noise_generator_));
    GZ_COMPAT_SET_Y(y_gyro,
                    GZ_COMPAT_GET_Y(y_gyro) + gyro_stdev_ * normal_distribution_(noise_generator_));
    GZ_COMPAT_SET_Z(y_gyro,
                    GZ_COMPAT_GET_Z(y_gyro) + gyro_stdev_ * normal_distribution_(noise_generator_));
  }

  // bias Walk for bias
  GZ_COMPAT_SET_X(gyro_bias_,
                    gyro_bias_[0]
                    + gyro_bias_walk_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_,
                    gyro_bias_[1]
                    + gyro_bias_walk_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_,
                    gyro_bias_[2]
                    + gyro_bias_walk_stdev_ * normal_distribution_(noise_generator_));

  // Apply Constant Bias
  GZ_COMPAT_SET_X(y_gyro, GZ_COMPAT_GET_X(y_gyro) + GZ_COMPAT_GET_X(gyro_bias_));
  GZ_COMPAT_SET_Y(y_gyro, GZ_COMPAT_GET_Y(y_gyro) + GZ_COMPAT_GET_Y(gyro_bias_));
  GZ_COMPAT_SET_Z(y_gyro, GZ_COMPAT_GET_Z(y_gyro) + GZ_COMPAT_GET_Z(gyro_bias_));

  // Convert to NED for output
  gyro_[0] = GZ_COMPAT_GET_X(y_gyro);
  gyro_[1] = (float) -GZ_COMPAT_GET_Y(y_gyro);
  gyro_[2] = (float) -GZ_COMPAT_GET_Z(y_gyro);

  imu_temperature_ = 27.0 + 273.15;
  // TODO: time manager
  imu_time_us_ = clock_micros();
}

bool GazeboSensors::imu_read(float accel[3], float * temperature, float gyro[3], uint64_t * time_us)
{
  // Return the values computed in the last update step
  for (int i=0; i<3; ++i) {
    accel[i] = accel_[i];
    gyro[i] = gyro_[i];
  }

  (*temperature) = imu_temperature_;
  (*time_us) = imu_time_us_;

  return true;
}

bool GazeboSensors::mag_read(float mag[3])
{
  mag[0] = mag_[0];
  mag[1] = mag_[1];
  mag[2] = mag_[2];

  return true;
}

void GazeboSensors::mag_update(rosflight_msgs::msg::State state)
{
  float T_s = 1.0/mag_update_rate_;
  
  GazeboPose I_to_B = GZ_COMPAT_GET_WORLD_POSE(link_);

  GazeboVector y_mag =
    GZ_COMPAT_GET_ROT(I_to_B).RotateVector(inertial_magnetic_field_) + mag_gauss_markov_eta_;
  
  GazeboVector noise;
  GZ_COMPAT_SET_X(noise, mag_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Y(noise, mag_stdev_ * normal_distribution_(noise_generator_));
  GZ_COMPAT_SET_Z(noise, mag_stdev_ * normal_distribution_(noise_generator_));
  mag_gauss_markov_eta_ = std::exp(-k_mag_*T_s) * mag_gauss_markov_eta_ + T_s*noise;

  // Convert measurement to NED
  mag_[0] = (float) GZ_COMPAT_GET_X(y_mag);
  mag_[1] = (float) -GZ_COMPAT_GET_Y(y_mag);
  mag_[2] = (float) -GZ_COMPAT_GET_Z(y_mag);
}

bool GazeboSensors::baro_read(float * pressure, float * temperature)
{
  (*pressure) = baro_pressure_;
  (*temperature) = baro_temperature_;

  return true;
}

void GazeboSensors::baro_update(rosflight_msgs::msg::State state)
{
  // pull z measurement out of Gazebo
  GazeboPose current_state_NWU = GZ_COMPAT_GET_WORLD_POSE(link_);

  // Invert measurement model for pressure and temperature
  double alt = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(current_state_NWU)) + origin_altitude_;

  // Convert to the true pressure reading
  double y_baro = 101325.0f
    * (float) pow((1 - 2.25694e-5 * alt), 5.2553); // Add these parameters to the parameters.

  // Add noise
  y_baro += baro_stdev_ * normal_distribution_(noise_generator_);

  // Perform bias walk
  baro_bias_ += baro_bias_walk_stdev_ * normal_distribution_(noise_generator_);

  // Add bias walk
  y_baro += baro_bias_;

  baro_pressure_ = (float) y_baro;
  baro_temperature_ = 27.0f + 273.15f;
}

bool GazeboSensors::gnss_read(rosflight_firmware::GNSSData * gnss,
                              rosflight_firmware::GNSSFull * gnss_full)
{
  // Package up the values

  gnss->lat = gnss_.lat;
  gnss->lon = gnss_.lon;
  gnss->height = gnss_.height;

  // For now, we have defined the Gazebo Local Frame as NWU.  This should be fixed in a future
  // commit
  gnss->vel_n = gnss_.vel_n;
  gnss->vel_e = gnss_.vel_e;
  gnss->vel_d = gnss_.vel_d;

  gnss->fix_type = gnss_.fix_type;
  gnss->time_of_week = gnss_.time_of_week;
  gnss->time = gnss_.time;
  // TODO: time manager
  gnss->nanos =
    (uint64_t) std::round((GZ_COMPAT_GET_SIM_TIME(world_).Double() - (double) gnss_->time) * 1e9);

  gnss->h_acc = gnss_.h_acc;
  gnss->v_acc = gnss_.v_acc;

  gnss->ecef.x = gnss_.ecef.x;
  gnss->ecef.y = gnss_.ecef.y;
  gnss->ecef.z = gnss_.ecef.z;
  gnss->ecef.p_acc = gnss_.ecef.p_acc;
  gnss->ecef.vx = gnss_.ecef.vx;
  gnss->ecef.vy = gnss_.ecef.vy;
  gnss->ecef.vz = gnss_.ecef.vz;
  gnss->ecef.s_acc = gnss_.ecef.s_acc;

  // TODO: time manager
  gnss->rosflight_timestamp = clock_micros();

  gnss_full->year = gnss_full_.year;
  gnss_full->month = gnss_full_.month;
  gnss_full->day = gnss_full_.day;
  gnss_full->hour = gnss_full_.hour;
  gnss_full->min = gnss_full_.min;
  gnss_full->sec = gnss_full_.sec;
  gnss_full->valid = gnss_full_.valid;

  gnss_full->lat = gnss_full_.lat;
  gnss_full->lon = gnss_full_.lon;
  gnss_full->height = gnss_full_.height;
  gnss_full->height_msl = gnss_full_.height_msl;

  // For now, we have defined the Gazebo Local Frame as NWU.  This should be
  // fixed in a future commit
  gnss_full.vel_n = gnss_full_.vel_n;
  gnss_full.vel_e = gnss_full_.vel_e;
  gnss_full.vel_d = gnss_full_.vel_d;

  gnss_full.fix_type = gnss_full_.fix_type;
  gnss_full.time_of_week = gnss_full_.time_of_week;
  gnss_full.num_sat = gnss_full_.num_sat;
  // TODO
  gnss_full.t_acc = gnss_full_.t_acc;
  gnss_full.nano = gnss_full_.nano;

  gnss_full.h_acc = gnss_full_.h_acc;
  gnss_full.v_acc = gnss_full_.v_acc;

  gnss_full.g_speed = gnss_full_.g_speed;

  gnss_full.head_mot = gnss_full_.head_mot;
  gnss_full.p_dop = gnss_full_.p_dop;
  gnss_full.rosflight_timestamp = gnss_full_.rosflight_timestamp;
}

void GazeboSensors::gnss_update(rosflight_msgs::msg::State state)
{
  using Vec3 = ignition::math::Vector3d;
  using Coord = gazebo::common::SphericalCoordinates::CoordinateType;

  double T_s = 1.0/gnss_update_rate_;

  GazeboPose local_pose = GZ_COMPAT_GET_WORLD_POSE(link_);
  Vec3 local_pos = GZ_COMPAT_GET_POS(local_pose) + gnss_gauss_markov_eta_;

  Vec3 pos_noise(horizontal_gnss_stdev_ * normal_distribution_(noise_generator_),
                 horizontal_gnss_stdev_ * normal_distribution_(noise_generator_),
                 vertical_gnss_stdev_ * normal_distribution_(noise_generator_));
  gnss_gauss_markov_eta_ = std::exp(-k_gnss_*T_s) * gnss_gauss_markov_eta_ + T_s*pos_noise;


  Vec3 local_vel = GZ_COMPAT_GET_WORLD_LINEAR_VEL(link_);
  Vec3 vel_noise(gnss_velocity_stdev_ * normal_distribution_(noise_generator_),
                 gnss_velocity_stdev_ * normal_distribution_(noise_generator_),
                 gnss_velocity_stdev_ * normal_distribution_(noise_generator_));
  local_vel += vel_noise;

  Vec3 ecef_pos = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::ECEF);
  Vec3 ecef_vel = sph_coord_.VelocityTransform(local_vel, Coord::LOCAL, Coord::ECEF);
  Vec3 lla = sph_coord_.PositionTransform(local_pos, Coord::LOCAL, Coord::SPHERICAL);
  
  gnss_.lat = (int) std::round(rad2Deg(lla.X()) * 1e7);
  gnss_.lon = (int) std::round(rad2Deg(lla.Y()) * 1e7);
  gnss_.height = (int) std::round(lla.Z() * 1e3);

  // For now, we have defined the Gazebo Local Frame as NWU.  This should be fixed in a future
  // commit
  gnss_.vel_n = (int) std::round(local_vel.X() * 1e3);
  gnss_.vel_e = (int) std::round(-local_vel.Y() * 1e3);
  gnss_.vel_d = (int) std::round(-local_vel.Z() * 1e3);

  gnss_.fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_3D_FIX;
  gnss_.time_of_week = GZ_COMPAT_GET_SIM_TIME(world_).Double() * 1000;
  gnss_.time = GZ_COMPAT_GET_SIM_TIME(world_).Double();
  // TODO: time manager
  gnss_.nanos =
    (uint64_t) std::round((GZ_COMPAT_GET_SIM_TIME(world_).Double() - (double) gnss_.time) * 1e9);

  gnss_.h_acc = (int) std::round(horizontal_gnss_stdev_ * 1000.0);
  gnss_.v_acc = (int) std::round(vertical_gnss_stdev_ * 1000.0);

  gnss_.ecef.x = (int) std::round(ecef_pos.X() * 100);
  gnss_.ecef.y = (int) std::round(ecef_pos.Y() * 100);
  gnss_.ecef.z = (int) std::round(ecef_pos.Z() * 100);
  gnss_.ecef.p_acc = (int) std::round(gnss_.h_acc / 10.0);
  gnss_.ecef.vx = (int) std::round(ecef_vel.X() * 100);
  gnss_.ecef.vy = (int) std::round(ecef_vel.Y() * 100);
  gnss_.ecef.vz = (int) std::round(ecef_vel.Z() * 100);
  gnss_.ecef.s_acc = (int) std::round(gnss_velocity_stdev_ * 100);

  // TODO: time manager
  gnss_.rosflight_timestamp = clock_micros();

  using Vec3 = ignition::math::Vector3d;
  using Vec3 = ignition::math::Vector3d;
  using Coord = gazebo::common::SphericalCoordinates::CoordinateType;

  // TODO: Do a better job of simulating the wander of GNSS
  
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  auto now_tm = std::localtime(&now_c);
  
  gnss_full_.year = now_tm->tm_year + 1900;
  gnss_full_.month = now_tm->tm_mon + 1;
  gnss_full_.day = now_tm->tm_mday;
  gnss_full_.hour = now_tm->tm_hour;
  gnss_full_.min = now_tm->tm_min;
  gnss_full_.sec = now_tm->tm_sec;
  gnss_full_.valid = 1;

  gnss_full_.lat = (int) std::round(rad2Deg(lla.X()) * 1e7);
  gnss_full_.lon = (int) std::round(rad2Deg(lla.Y()) * 1e7);
  gnss_full_.height = (int) std::round(lla.Z() * 1e3);
  gnss_full_.height_msl = gnss_full_.height; // TODO

  // For now, we have defined the Gazebo Local Frame as NWU.  This should be
  // fixed in a future commit
  gnss_full_.vel_n = (int) std::round(local_vel.X() * 1e3);
  gnss_full_.vel_e = (int) std::round(-local_vel.Y() * 1e3);
  gnss_full_.vel_d = (int) std::round(-local_vel.Z() * 1e3);

  gnss_full_.fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_3D_FIX;
  gnss_full_.time_of_week = GZ_COMPAT_GET_SIM_TIME(world_).Double() * 1000;
  gnss_full_.num_sat = 15;
  // TODO
  gnss_full_.t_acc = 0;
  gnss_full_.nano = 0;

  gnss_full_.h_acc = (int) std::round(horizontal_gnss_stdev_ * 1000.0);
  gnss_full_.v_acc = (int) std::round(vertical_gnss_stdev_ * 1000.0);

  // Again, TODO switch to using ENU convention per REP
  double vn = local_vel.X();
  double ve = -local_vel.Y();
  double ground_speed = std::sqrt(vn * vn + ve * ve);
  gnss_full_.g_speed = (int) std::round(ground_speed * 1000);

  double head_mot = std::atan2(ve, vn);
  gnss_full_.head_mot = (int) std::round(rad2Deg(head_mot) * 1e5);
  gnss_full_.p_dop = 0.0; // TODO
  gnss_full_.rosflight_timestamp = clock_micros();
}

bool GazeboSensors::sonar_read(float * range)
{
  (*range) = range_;
  return true;
}

void GazeboSensors::sonar_update(rosflight_msgs::msg::State state)
{
  GazeboPose current_state_NWU = GZ_COMPAT_GET_WORLD_POSE(link_);
  double alt = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(current_state_NWU));

  if (alt < sonar_min_range_) {
    range_ = (float) sonar_min_range_;
  } else if (alt > sonar_max_range_) {
    range_ = (float) sonar_max_range_;
  } else {
    range_ = (float) (alt + sonar_stdev_ * normal_distribution_(noise_generator_));
  }
}

bool GazeboSensors::diff_pressure_read(float * diff_pressure, float * temperature)
{
  (*diff_pressure) = diff_pressure_;
  (*temperature) = diff_pressure_temperature_;
  return true;
}

void GazeboSensors::diff_pressure_update(rosflight_msgs::msg::State state)
{
  // Calculate Airspeed
  GazeboVector vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);

  double Va = GZ_COMPAT_GET_LENGTH(vel);

  // Invert Airpseed to get sensor measurement
  // TODO: this uses a constant density right now... We should probably change that 
  double y_as = rho_ * Va * Va / 2.0; // Page 130 in the UAV Book

  // Add noise
  y_as += airspeed_stdev_ * normal_distribution_(noise_generator_);
  airspeed_bias_ += airspeed_bias_walk_stdev_ * normal_distribution_(noise_generator_);
  y_as += airspeed_bias_;

  diff_pressure_ = (float) y_as;
  diff_pressure_temperature_ = 27.0 + 273.15;
}

bool GazeboSensors::battery_read(float * voltage, float * current)
{
  (*voltage) = battery_voltage_;
  (*current) = battery_current_;
  return true;
}

void GazeboSensors::battery_update(rosflight_msgs::msg::State state)
{
  battery_voltage_ = 15 * battery_voltage_multiplier_;
  battery_current_ = 1 * battery_current_multiplier_;
}


} // rosflight_sim
