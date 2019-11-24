#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_link_broadcaster");
  ros::NodeHandle n;

  ros::Rate r(5);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.125, 0.0, 0.25)),
        ros::Time::now(),"map", "camera_link"));
    r.sleep();
  }
}