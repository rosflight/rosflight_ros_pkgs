
# include <fcu_common/gps.h>

GPS::GPS()
{
    nh_ = ros::NodeHandle();
    gps_msg_pub_ = nh_.advertise<fcu_common::GPS>("/gps/data",10);

    navsatfix_sub_ = nh_.subscribe("/fix", 10, &GPS::NavSatCallback, this);
    twiststamped_sub_ = nh_.subscribe("/vel", 10, &GPS::TwistCallback, this);

    vel_msgs_.twist.linear.y = 0; vel_msgs_.twist.linear.x = 0;
}

void GPS::NavSatCallback(const sensor_msgs::NavSatFix &msg)
{
    fcu_common::GPS gps_msg;
    gps_msg.altitude = msg.altitude;
    if(msg.status.status >= 0)
        gps_msg.fix = true;
    gps_msg.latitude = msg.latitude;
    gps_msg.longitude = msg.longitude;
    gps_msg.speed = sqrt(vel_msgs_.twist.linear.x*vel_msgs_.twist.linear.x + vel_msgs_.twist.linear.y*vel_msgs_.twist.linear.y);
    if(vel_msgs_.twist.linear.x != 0)
        gps_msg.ground_course = atan2(vel_msgs_.twist.linear.y,vel_msgs_.twist.linear.x);
    else
        gps_msg.ground_course = 0;
    if(std::isfinite(gps_msg.latitude))
    	gps_msg_pub_.publish(gps_msg);
}

void GPS::TwistCallback(const geometry_msgs::TwistStamped msg)
{
    this->vel_msgs_ = msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fcu_common_gps");
  GPS gps;

  ros::spin();

  return 0;
}
