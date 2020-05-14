#include "rrt_planner.h"

#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>


// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_planner {

RRTPlanner::RRTPlanner() : costmap_ros_(nullptr), initialized_(false) {}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_ros_(nullptr), initialized_(false) {
  initialize(name, costmap_ros);
}

RRTPlanner::~RRTPlanner() {
    if (initialized_) {
        delete world_model_;
    }
}

void RRTPlanner::initialize(std::string name,
                            costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    initialized_ = true;
    ROS_DEBUG("The RRT planner has been initialized.");
  } else {
    ROS_WARN("The RRT planner has already been initialized.");
  }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                          const geometry_msgs::PoseStamped &goal,
                          std::vector<geometry_msgs::PoseStamped> &plan) {
  if (!initialized_) {
    ROS_ERROR(
        "The RRT planner has not been initialized, you must call "
        "initialize().");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f",
            start.pose.position.x, start.pose.position.y, goal.pose.position.x,
            goal.pose.position.y);

  plan.clear();
  costmap_ = costmap_ros_->getCostmap();

  if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
    ROS_ERROR(
        "The RRT planner can only accept goals in the %s frame, "
        "but a goal was sent in the %s frame.",
        costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }

  tf::Stamped<tf::Pose> goal_tf;
  tf::Stamped<tf::Pose> start_tf;

  poseStampedMsgToTF(goal, goal_tf);
  poseStampedMsgToTF(start, start_tf);

  double useless_pitch, useless_roll, goal_yaw, start_yaw;
  start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
  goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

  // we want to step back along the vector created by the robot's position and
  // the goal pose until we find a legal cell
  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;

  double diff_x = goal_x - start_x;
  double diff_y = goal_y - start_y;
  double diff_yaw = angles::normalize_angle(goal_yaw - start_yaw);

  double target_x = goal_x;
  double target_y = goal_y;
  double target_yaw = goal_yaw;

  bool done = false;
  double scale = 1.0;
  double dScale = 0.01;

  while (!done) {
    if (scale < 0) {
      target_x = start_x;
      target_y = start_y;
      target_yaw = start_yaw;
      ROS_WARN("The carrot planner could not find a valid plan for this goal");
      break;
    }
    target_x = start_x + scale * diff_x;
    target_y = start_y + scale * diff_y;
    target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);

    double footprint_cost = footprintCost(target_x, target_y, target_yaw);
    if (footprint_cost >= 0) {
      done = true;
    }
    scale -= dScale;
  }

  plan.push_back(start);
  geometry_msgs::PoseStamped new_goal = goal;
  tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

  new_goal.pose.position.x = target_x;
  new_goal.pose.position.y = target_y;

  new_goal.pose.orientation.x = goal_quat.x();
  new_goal.pose.orientation.y = goal_quat.y();
  new_goal.pose.orientation.z = goal_quat.z();
  new_goal.pose.orientation.w = goal_quat.w();

  plan.push_back(new_goal);
  return (done);
}

double RRTPlanner::footprintCost(double x, double y, double th) {
  if (!initialized_) {
    ROS_ERROR(
        "The RRT Planner has not been initialized, you must call "
        "initialize().");
    return -1.0;
  }

  std::vector<geometry_msgs::Point> footprint =
      costmap_ros_->getRobotFootprint();

  if (footprint.size() < 3) return -1.0;

  double footprint_cost = world_model_->footprintCost(x, y, th, footprint);
  return footprint_cost;
}

};  // namespace rrt_planner