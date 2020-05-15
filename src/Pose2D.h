#ifndef POSE2D_H_
#define POSE2D_H_

namespace rrt_planner {

class Pose2D {
 public:
  double x;
  double y;
  double th;

  Pose2D(double x_i, double y_i, double th_i);

  static double distanceBetweenPoses(const Pose2D& pose1, const Pose2D& pose2);
  static Pose2D createPoseWithinRange(const Pose2D& start_pose,
                                      const Pose2D& end_pose, double range);
};

};  // namespace rrt_planner

#endif  // POSE2D_H_