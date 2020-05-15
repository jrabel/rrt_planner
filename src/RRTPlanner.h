#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

#include "RRTree.h"

namespace rrt_planner {

class RRTPlanner : public nav_core::BaseGlobalPlanner {
 public:
  RRTPlanner();
  RRTPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  ~RRTPlanner();

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);
  void publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan) const;

 private:
  ros::Publisher plan_pub_;

  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_;
  base_local_planner::WorldModel *world_model_;

  RRTree rrtree_;
  double step_size_ = 0.25;

  bool initialized_;

  double footprintCost(const Pose2D &pose) const;
  bool isValidPose(const Pose2D &pose) const;
  Pose2D createRandomValidPose() const;
  bool isValidPathBetweenPoses(const Pose2D &pose1, const Pose2D &pose2) const;
  void buildPlanFromGoal(const RRTree::Node::Ptr goal_node,
                         std::vector<geometry_msgs::PoseStamped> &plan) const;
};

};  // namespace rrt_planner

#endif  // RRT_PLANNER_H_