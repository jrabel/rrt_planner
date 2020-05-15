#include "Pose2D.h"

#include <cmath>

namespace rrt_planner {
Pose2D::Pose2D(double x_i, double y_i, double th_i)
    : x(x_i), y(y_i), th(th_i){};

double Pose2D::distanceBetweenPoses(const Pose2D& pose1, const Pose2D& pose2) {
  return std::sqrt((pose1.x - pose2.x) * (pose1.x - pose2.x) +
                   (pose1.y - pose2.y) * (pose1.y - pose2.y));
}

Pose2D Pose2D::createPoseWithinRange(const Pose2D& start_pose,
                                     const Pose2D& end_pose, double range) {
  double x_step = end_pose.x - start_pose.x;
  double y_step = end_pose.y - start_pose.y;
  double mag = std::sqrt((x_step * x_step) + (y_step * y_step));

  if (mag < range) return end_pose;

  x_step /= mag;
  y_step /= mag;

  Pose2D new_pose(start_pose.x + x_step * range, start_pose.y + y_step * range,
                  end_pose.th);

  return new_pose;
}

};  // namespace rrt_planner