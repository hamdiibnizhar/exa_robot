#include <ros.h>

#include "std_msgs/Float64.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

ros::NodeHandle  nh;
const geometry_msgs::Twist vel;

struct velocity
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
};

velocity veltemp;

void callback(const geometry_msgs::Twist& get_vel)
{
//    veltemp.linear[0] = get_vel.linear.x;
//    veltemp.linear[1] = get_vel.linear.y;
//    veltemp.linear[2] = get_vel.linear.z;
    veltemp.linear = get_vel.linear;
    veltemp.angular = get_vel.linear;
}

void setup()
{
    nh.initNode();
    Serial.begin(115200);
    ros::Subscriber<geometry_msgs::Twist>  ("teleop_test/cmd_vel", &callback);
}

void loop()
{
    Serial.print("value x: ");
    Serial.println(veltemp.linear.x);
    Serial.print("value y: ");
    Serial.println(veltemp.linear.y);
    Serial.print("value z: ");
    Serial.println(veltemp.linear.z);
//    Serial.println("value an x: " + veltemp.angular.x);
//    Serial.println("value an y: " + veltemp.angular.y);
//    Serial.println("value an z: " + veltemp.angular.z);
    nh.spinOnce();
    delay(10);
}