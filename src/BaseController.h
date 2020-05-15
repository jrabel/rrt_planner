#ifndef BASECONTROLLER_H_
#define BASECONTROLLER_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "Pose2D.h"

namespace rrt_planner {

class BaseController {
 public:
  BaseController();
  BaseController(const Pose2D& pose, const Pose2D& twist);
  ~BaseController();

  void updateVelocity(const geometry_msgs::Twist& cmd_vel);

  void updatePose(double dt);

  void broadcastTransform();

  void publishOdometry();

 private:
  Pose2D pose_;
  Pose2D twist_;

  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
};

};  // namespace rrt_planner

#endif  // BASECONTROLLER_H_