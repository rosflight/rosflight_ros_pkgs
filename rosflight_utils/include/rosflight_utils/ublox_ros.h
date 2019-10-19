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

#ifndef UBLOX_ROS_H
#define UBLOX_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "rosflight_utils/ublox.h"
#include "async_comm/serial.h"

#include "rosflight_msgs/GNSS.h"
#include "rosflight_msgs/GNSSRaw.h"

namespace ublox_ros
{
class UBLOX_ROS : public ublox::UBXListener
{
   public:
    UBLOX_ROS();
    ~UBLOX_ROS();

   private:
    ublox::UBLOX* ublox_ = nullptr;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher gnss_raw_pub_;
    ros::Publisher gnss_pub_;

    void pvtCB(const ublox::NAV_PVT_t& msg);
    void posECEFCB(const ublox::NAV_POSECEF_t& msg);
    void velECEFCB(const ublox::NAV_VELECEF_t& msg);

    void got_ubx(const uint8_t cls, const uint8_t id, const ublox::UBX_message_t &) override;

    uint32_t pos_tow_;
    uint32_t vel_tow_;
    uint32_t pvt_tow_;
    uint32_t pvt_week_;
    rosflight_msgs::GNSS ecef_msg_;

    async_comm::Serial* ser;
};

}  // namespace ublox_ros

#endif
