#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "exa_robot_frame_transform");
  ros::NodeHandle exaNode;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(exaNode.ok()){
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)), ros::Time::now(), "base_link", "base_kinect"));
    r.sleep();
  }
}
