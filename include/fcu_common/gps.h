#ifndef fcu_common_gps_GPS_H_
#define fcu_common_gps_GPS_H_

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <fcu_common/GPS.h>


class GPS{

    private:
     ros::NodeHandle nh_;
     ros::Publisher gps_msg_pub_;
     ros::Subscriber navsatfix_sub_;
     ros::Subscriber twiststamped_sub_;

     void NavSatCallback(const sensor_msgs::NavSatFix& msg);
     void TwistCallback(const geometry_msgs::TwistStamped msg);
//     void Publish();

     geometry_msgs::TwistStamped vel_msgs_;

    public:
        GPS();
};

#endif // fcu_common_gps_GPS_H_
