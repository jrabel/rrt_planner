#ifndef RRTREE_H_
#define RRTREE_H_

#include <ros/ros.h>

#include <memory>
#include <vector>

namespace rrt_planner {

class RRTree {
 public:
  struct Pose {
    const double x;
    const double y;
    const double th;

    Pose(double x_i, double y_i, double th_i) : x(x_i), y(y_i), th(th_i){};
  };

  struct Node {
    typedef std::shared_ptr<Node> Ptr;
    typedef std::weak_ptr<Node> WeakPtr;

    const Pose pose;
    Node::WeakPtr parent;
    std::vector<Node::Ptr> children;

    Node(Pose pose_i, Node::Ptr parent_i) : pose(pose_i), parent(parent_i){};
  };

  RRTree();
  ~RRTree();

  void initialize(const Pose& start_pose);
  Node::Ptr getNearestNode(const Pose& pose) const;
  void publishTree() const;

  static double distanceBetweenPoses(const Pose& pose1, const Pose& pose2);
  static RRTree::Pose createPoseWithinRange(const RRTree::Pose& start_pose,
                                            const RRTree::Pose& end_pose,
                                            double range);

 private:
  Node::Ptr root_node_;
  ros::Publisher tree_pub_;
};

};  // namespace rrt_planner

#endif  // RRTREE_H_