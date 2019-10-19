/* Copyright (c) 2019 James Jackson, Matt Rydalch
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <signal.h>
#include <fstream>
#include <iostream>

#include <ros/ros.h>

#include <rosflight_utils/ublox_ros.h>

constexpr double deg2rad(double x)
{
    return M_PI / 180.0 * x;
}

namespace ublox_ros
{
UBLOX_ROS::UBLOX_ROS() : nh_(), nh_private_("~")
{
    std::string serial_port = nh_private_.param<std::string>("serial_port", "/dev/ttyACM0");
    int baud_rate = nh_private_.param<int>("baudrate", 921600);

    // Connect ROS topics
    gnss_raw_pub_ = nh_.advertise<rosflight_msgs::GNSSRaw>("gps_raw", 10);
    gnss_pub_ = nh_.advertise<rosflight_msgs::GNSS>("GNSS", 10);
    //    nav_sat_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("NavSatFix");
    //    nav_sat_status_pub_ = nh_.advertise<sensor_msgs::NavSatStatus>("NavSatStatus");

    // Create the serial comm
    ser = new async_comm::Serial(serial_port, baud_rate);
    try {
        ser->init();
    }
    catch(const std::exception &e) {
        ROS_FATAL("%s", e.what());
        ros::shutdown();
    }


    // create the parser
    ublox_ = new ublox::UBLOX(*ser);


    // Create subscriptions
    subscribe(ublox::CLASS_NAV, ublox::NAV_PVT);
    subscribe(ublox::CLASS_NAV, ublox::NAV_POSECEF);
    subscribe(ublox::CLASS_NAV, ublox::NAV_VELECEF);
    ublox_->registerListener(this);
}

UBLOX_ROS::~UBLOX_ROS()
{
    if (ublox_)
        delete ublox_;
    if (ser)
        delete ser;
}

void UBLOX_ROS::got_ubx(const uint8_t cls, const uint8_t id, const ublox::UBX_message_t &msg)
{
    if (cls == ublox::CLASS_NAV)  {
        if(id == ublox::NAV_PVT)
        {
            pvtCB(msg.NAV_PVT);
        } 
        else if (id == ublox::NAV_POSECEF)
        {
            posECEFCB(msg.NAV_POSECEF);
        }
        else if (id == ublox::NAV_VELECEF)
        {
            velECEFCB(msg.NAV_VELECEF);
        }
    }
}


void UBLOX_ROS::pvtCB(const ublox::NAV_PVT_t &msg) {
  pvt_tow_ = msg.iTOW;
  rosflight_msgs::GNSSRaw out;
  // out.iTOW = msg.iTow;
  out.header.stamp = ros::Time::now(); /// TODO: Do this right
  out.year = msg.year;
  out.month = msg.month;
  out.day = msg.day;
  out.hour = msg.hour;
  out.min = msg.min;
  out.sec = msg.sec;
  out.nano = msg.nano;
  out.t_acc = msg.tAcc;
  out.valid = msg.valid;
  out.fix_type = msg.fixType;
  out.num_sat = msg.numSV;
  out.lat = msg.lat;
  out.lon = msg.lon;
  out.height = msg.height;
  out.height_msl = msg.hMSL;
  out.h_acc = msg.hAcc;
  out.v_acc = msg.vAcc;
  out.vel_n = msg.velN;
  out.vel_e = msg.velE;
  out.vel_d = msg.velD;
  out.g_speed = msg.gSpeed;
  out.head_mot = msg.headMot;
  out.s_acc = msg.sAcc;
  out.head_acc = msg.headAcc;
  out.p_dop = msg.pDOP * 0.01;
  out.head_mot = msg.headVeh;

  gnss_raw_pub_.publish(out);

  ecef_msg_.fix = msg.fixType;
  ecef_msg_.header.stamp = ros::Time::now();
  ecef_msg_.horizontal_accuracy = out.h_acc * 1e-3;;
  ecef_msg_.vertical_accuracy = out.v_acc * 1e-3;
  ecef_msg_.speed_accuracy = out.s_acc * 1e-3;
  if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_) 
  {
    gnss_pub_.publish(ecef_msg_);
  }
}


void UBLOX_ROS::posECEFCB(const ublox::NAV_POSECEF_t& msg)
{
    pos_tow_ = msg.iTOW;
    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.position[0] = msg.ecefX * 1e-2;
    ecef_msg_.position[1] = msg.ecefY * 1e-2;
    ecef_msg_.position[2] = msg.ecefZ * 1e-2;
    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
    {
        gnss_pub_.publish(ecef_msg_);
    }
}

void UBLOX_ROS::velECEFCB(const ublox::NAV_VELECEF_t& msg)
{
    vel_tow_ = msg.iTOW;
    ecef_msg_.header.stamp = ros::Time::now();
    ecef_msg_.velocity[0] = msg.ecefVX * 1e-2;
    ecef_msg_.velocity[0] = msg.ecefVY * 1e-2;
    ecef_msg_.velocity[0] = msg.ecefVZ * 1e-2;

    if (pos_tow_ == pvt_tow_ && pos_tow_ == vel_tow_)
    {
      gnss_pub_.publish(ecef_msg_);
    }
}

}  // namespace ublox_ros

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ublox_ros");

    ublox_ros::UBLOX_ROS Thing;

    ros::spin();
}
