#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace rrt_planner {

class BaseController {
 public:
  struct State {
    double x, y, th = 0;
    double dx, dy, dth = 0;
  };

  BaseController() {
    state_.x = 1.0;
    state_.y = 1.0;

    ros::NodeHandle n;
    cmd_vel_sub_ =
        n.subscribe("cmd_vel", 1000, &BaseController::updateVelocity, this);
    odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 50);
  }

  ~BaseController(){};

  void updateVelocity(const geometry_msgs::Twist& cmd_vel) {
    double dx = cmd_vel.linear.x;
    double dy = cmd_vel.linear.y;
    double dth = cmd_vel.angular.z;

    state_.dx = dx * cos(state_.th) - dy * sin(state_.th);
    state_.dy = dx * sin(state_.th) + dy * cos(state_.th);
    state_.dth = dth;
  }

  void updatePose(double dt) {
    state_.x += state_.dx * dt;
    state_.y += state_.dy * dt;
    state_.th += state_.dth * dt;
  }

  void broadcastTransform() {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = state_.x;
    odom_trans.transform.translation.y = state_.y;
    odom_trans.transform.translation.z = 0.0;

    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(state_.th);
    odom_trans.transform.rotation = odom_quat;

    tf_broadcaster_.sendTransform(odom_trans);
  }

  void publishOdometry() {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = state_.x;
    odom.pose.pose.position.y = state_.y;
    odom.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(state_.th);
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = state_.dx;
    odom.twist.twist.linear.y = state_.dy;
    odom.twist.twist.angular.z = state_.dth;

    odom_pub_.publish(odom);
  }

 private:
  State state_;

  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
};

};  // namespace rrt_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_publisher");

  rrt_planner::BaseController base_controller;

  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10);

  while (ros::ok) {
    ros::spinOnce();  // check for incoming messages
    current_time = ros::Time::now();

    // compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    base_controller.updatePose(dt);
    base_controller.broadcastTransform();
    base_controller.publishOdometry();

    last_time = current_time;
    r.sleep();
  }
}