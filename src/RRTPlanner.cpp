
#include "RRTPlanner.h"

#include <angles/angles.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>

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

    ros::NodeHandle private_nh("~/" + name);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

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

  ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f",
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

  double footprint_cost = footprintCost(goal_x, goal_y, goal_yaw);
  if (footprint_cost < 0) {
    ROS_INFO("RRTPlanner::makePlan: Goal pose intersects an obstacle!");
    return false;
  }

  RRTree::Pose start_pose(start_x, start_y, start_yaw);
  RRTree::Pose goal_pose(goal_x, goal_y, goal_yaw);
  rrtree_.initialize(start_pose);

  RRTree::Node::Ptr goal_node = nullptr;
  int max_iterations = 10000;
  int iterations = 0;

  while (iterations < max_iterations) {
    RRTree::Pose random_pose = createRandomValidPose();
    RRTree::Node::Ptr nearest_node = rrtree_.getNearestNode(random_pose);
    RRTree::Pose new_pose = RRTree::createPoseWithinRange(
        nearest_node->pose, random_pose, step_size_);

    if (isValidPathBetweenPoses(nearest_node->pose, new_pose)) {
      RRTree::Node::Ptr new_node =
          std::make_shared<RRTree::Node>(new_pose, nearest_node);
      nearest_node->children.push_back(new_node);

      if (isValidPathBetweenPoses(new_pose, goal_pose)) {
        goal_node = std::make_shared<RRTree::Node>(goal_pose, new_node);
        new_node->children.push_back(goal_node);
        break;
      }
    }

    iterations++;
  }

  rrtree_.publishTree();

  if (goal_node) {
    ROS_INFO("goal found!");
    buildPlanFromGoal(goal_node, plan);
    publishPlan(plan);
    return true;
  } else {
    ROS_INFO("goal not found!");
    return false;
  }
}

double RRTPlanner::footprintCost(double x, double y, double th) const {
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

void RRTPlanner::publishPlan(
    const std::vector<geometry_msgs::PoseStamped> &path) const {
  if (!initialized_) {
    ROS_ERROR(
        "The RRT Planner has not been initialized, you must call "
        "initialize().");
    return;
  }

  nav_msgs::Path path_visual;
  path_visual.poses.resize(path.size());

  if (path.empty()) {
    path_visual.header.frame_id = costmap_ros_->getGlobalFrameID();
    path_visual.header.stamp = ros::Time::now();
  } else {
    path_visual.header.frame_id = path[0].header.frame_id;
    path_visual.header.stamp = path[0].header.stamp;
  }

  for (unsigned int i = 0; i < path.size(); i++) {
    path_visual.poses[i] = path[i];
  }

  plan_pub_.publish(path_visual);
}

RRTree::Pose RRTPlanner::createRandomValidPose() const {
  // get bounds of the costmap in world coordinates
  double wx_min, wy_min;
  costmap_->mapToWorld(0, 0, wx_min, wy_min);

  double wx_max, wy_max;
  unsigned int mx_max = costmap_->getSizeInCellsX();
  unsigned int my_max = costmap_->getSizeInCellsY();
  costmap_->mapToWorld(mx_max, my_max, wx_max, wy_max);

  bool found_pose = false;

  while (!found_pose) {
    double wx_rand = (double)rand() / RAND_MAX;
    wx_rand = wx_min + wx_rand * (wx_max - wx_min);
    double wy_rand = (double)rand() / RAND_MAX;
    wy_rand = wy_min + wy_rand * (wy_max - wy_min);

    double th_rand = (double)rand() / RAND_MAX;
    th_rand = -M_PI + th_rand * (M_PI - -M_PI);

    if (footprintCost(wx_rand, wy_rand, th_rand) >= 0) {
      return RRTree::Pose(wx_rand, wy_rand, th_rand);
    }
  }
}

bool RRTPlanner::isValidPathBetweenPoses(const RRTree::Pose &pose1,
                                         const RRTree::Pose &pose2) const {
  double interp_step_size = 0.05;
  double current_step = interp_step_size;

  double distance = RRTree::distanceBetweenPoses(pose1, pose2);

  while (current_step < distance) {
    RRTree::Pose interp_pose =
        RRTree::createPoseWithinRange(pose1, pose2, current_step);

    if (footprintCost(interp_pose.x, interp_pose.y, interp_pose.th) < 0)
      return false;

    current_step += interp_step_size;
  }

  return true;
}

void RRTPlanner::buildPlanFromGoal(
    const RRTree::Node::Ptr goal_node,
    std::vector<geometry_msgs::PoseStamped> &plan) const {
  RRTree::Node::Ptr node = goal_node;

  while (node) {
    geometry_msgs::PoseStamped waypoint;
    waypoint.header.stamp = ros::Time::now();
    waypoint.header.frame_id = costmap_ros_->getGlobalFrameID();

    waypoint.pose.position.x = node->pose.x;
    waypoint.pose.position.y = node->pose.y;

    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0, 0, node->pose.th);
    waypoint.pose.orientation.x = goal_quat.x();
    waypoint.pose.orientation.y = goal_quat.y();
    waypoint.pose.orientation.z = goal_quat.z();
    waypoint.pose.orientation.w = goal_quat.w();

    plan.push_back(waypoint);

    node = node->parent.lock();
  }
  std::reverse(plan.begin(), plan.end());
}

};  // namespace rrt_planner