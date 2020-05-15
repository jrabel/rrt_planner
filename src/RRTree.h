#ifndef RRTREE_H_
#define RRTREE_H_

#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>

#include <memory>
#include <unordered_map>
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
  void addNodeToTree(Node::Ptr node, Node::Ptr parent);
  Node::Ptr getNearestNode(const Pose2D& pose) const;
  void publishTree() const;

 private:
  Node::Ptr root_node_;
  ros::Publisher tree_pub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_;
  std::unordered_map<unsigned int, Node::Ptr> pointcloud_index_map_;
};

};  // namespace rrt_planner

#endif  // RRTREE_H_