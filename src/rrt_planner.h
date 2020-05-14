#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

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

 private:
  costmap_2d::Costmap2DROS *costmap_ros_;
  double step_size_, min_dist_from_robot_;
  costmap_2d::Costmap2D *costmap_;
  base_local_planner::WorldModel *world_model_;
  bool initialized_;

  double footprintCost(double x, double y, double th);
};

};  // namespace rrt_planner

#endif  // RRT_PLANNER_H_