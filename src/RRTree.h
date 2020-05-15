#ifndef RRTREE_H_
#define RRTREE_H_

#include <ros/ros.h>

#include <memory>
#include <vector>

#include "Pose2D.h"

namespace rrt_planner {

class RRTree {
 public:
  struct Node {
    typedef std::shared_ptr<Node> Ptr;
    typedef std::weak_ptr<Node> WeakPtr;

    const Pose2D pose;
    const Node::WeakPtr parent;
    std::vector<Node::Ptr> children;

    Node(const Pose2D& pose_i, Node::Ptr parent_i)
        : pose(pose_i), parent(parent_i){};
  };

  RRTree();
  ~RRTree();

  void initialize(const Pose2D& start_pose);
  Node::Ptr getNearestNode(const Pose2D& pose) const;
  void publishTree() const;

 private:
  Node::Ptr root_node_;
  ros::Publisher tree_pub_;
};

};  // namespace rrt_planner

#endif  // RRTREE_H_