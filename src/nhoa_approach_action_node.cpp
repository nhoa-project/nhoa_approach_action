#include <nhoa_approach_action/nhoa_approach_action.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "nhoa_approach_action");

  ROS_INFO("Starting nhoa_approach_action...");
  // tf::TransformListener tf(ros::Duration(10));
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  Nhoa_approach_action approach(&tfBuffer);

  ros::spin();

  return 0;
}
