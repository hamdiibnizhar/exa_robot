#include "exa_control/exa_teleop.h"

exa_teleop::exa_teleop():linear_(1),angular_(0)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("exa_robot/cmd_vel", 2);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &exa_teleop::joyCallback, this);
}

void exa_teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_ * joy->axes[angular_];
  twist.linear.x = l_scale_ * joy->axes[linear_];
  vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exa_teleop");
  exa_teleop exa_teleopObj;

  ros::spin();
}
