#include "BaseController.h"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace rrt_planner {

BaseController::BaseController()
    : BaseController(Pose2D(0, 0, 0), Pose2D(0, 0, 0)) {}

BaseController::BaseController(const Pose2D& pose, const Pose2D& twist)
    : pose_(pose), twist_(twist) {
  ros::NodeHandle n;
  cmd_vel_sub_ =
      n.subscribe("cmd_vel", 1000, &BaseController::updateVelocity, this);
  odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 50);
}

BaseController::~BaseController(){};

void BaseController::updateVelocity(const geometry_msgs::Twist& cmd_vel) {
  double dx = cmd_vel.linear.x;
  double dy = cmd_vel.linear.y;
  twist_.th = cmd_vel.angular.z;

  twist_.x = dx * cos(pose_.th) - dy * sin(pose_.th);
  twist_.y = dx * sin(pose_.th) + dy * cos(pose_.th);
}

void BaseController::updatePose(double dt) {
  pose_.x += twist_.x * dt;
  pose_.y += twist_.y * dt;
  pose_.th += twist_.th * dt;
}

void BaseController::broadcastTransform() {
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = pose_.x;
  odom_trans.transform.translation.y = pose_.y;
  odom_trans.transform.translation.z = 0.0;

  geometry_msgs::Quaternion odom_quat =
      tf::createQuaternionMsgFromYaw(pose_.th);
  odom_trans.transform.rotation = odom_quat;

  tf_broadcaster_.sendTransform(odom_trans);
}

void BaseController::publishOdometry() {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = pose_.x;
  odom.pose.pose.position.y = pose_.y;
  odom.pose.pose.position.z = 0.0;
  geometry_msgs::Quaternion odom_quat =
      tf::createQuaternionMsgFromYaw(pose_.th);
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = twist_.x;
  odom.twist.twist.linear.y = twist_.y;
  odom.twist.twist.angular.z = twist_.th;

  odom_pub_.publish(odom);
}

};  // namespace rrt_planner