#include <ros/ros.h>

#include "BaseController.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_controller");

  rrt_planner::Pose2D start_pose(2, 2, 0);
  rrt_planner::Pose2D start_twist(0, 0, 0);
  rrt_planner::BaseController base_controller(start_pose, start_twist);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10);

  while (ros::ok) {
    ros::spinOnce();
    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();

    base_controller.updatePose(dt);
    base_controller.broadcastTransform();
    base_controller.publishOdometry();

    last_time = current_time;
    r.sleep();
  }
}