#ifndef __EXA_ROBOT_TELEOP__
#define __EXA_ROBOT_TELEOP__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class exa_teleop
{
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_, a_scale_;
    int coef;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

public:
    exa_teleop();
};

#endif