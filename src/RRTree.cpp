#include "RRTree.h"

#include <visualization_msgs/Marker.h>

#include <iostream>
#include <limits>
#include <queue>

namespace rrt_planner {

RRTree::RRTree() {
  ros::NodeHandle private_nh("~/RRTPlanner");
  tree_pub_ = private_nh.advertise<visualization_msgs::Marker>("tree", 1);
}

RRTree::~RRTree(){};

void RRTree::initialize(const Pose2D& start_pose) {
  root_node_ = std::make_shared<Node>(start_pose, nullptr);

  pointcloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pointcloud_index_map_.clear();

  pcl::PointXYZ root_point(start_pose.x, start_pose.y, 0);
  pointcloud_->points.push_back(root_point);
  pointcloud_index_map_.insert({0, root_node_});

  double resolution = 1;
  octree_ =
      boost::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(
          resolution);

  octree_->setInputCloud(pointcloud_);
  octree_->addPointsFromInputCloud();
}

void RRTree::addNodeToTree(Node::Ptr node, Node::Ptr parent) {
  parent->children.push_back(node);

  pcl::PointXYZ point(node->pose.x, node->pose.y, 0);
  octree_->addPointToCloud(point, pointcloud_);

  unsigned int index = pointcloud_index_map_.size();
  pointcloud_index_map_.insert({index, node});
}

RRTree::Node::Ptr RRTree::getNearestNode(const Pose2D& pose) const {
  if (!root_node_) {
    ROS_ERROR(
        "The RRTree has not been initialized, you must call "
        "initialize().");
    return nullptr;
  }

  pcl::PointXYZ search_point;
  search_point.x = pose.x;
  search_point.y = pose.y;
  search_point.z = 0;
  int k = 1;

  std::vector<int> point_indices;
  std::vector<float> point_distances;

  if (octree_->nearestKSearch(search_point, k, point_indices, point_distances) >
      0) {
    return pointcloud_index_map_.at(point_indices[0]);
  } else {
    ROS_ERROR("RRTree::getNearestNode: could not find nearest neighbor.");
  }

  return nullptr;
}

void RRTree::publishTree() const {
  if (!root_node_) {
    ROS_ERROR(
        "The RRTree has not been initialized, you must call "
        "initialize().");
    return;
  }

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "map";
  line_list.header.stamp = ros::Time::now();
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.01;
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;

  std::queue<Node::Ptr> q;
  q.push(root_node_);

  while (!q.empty()) {
    Node::Ptr node = q.front();
    q.pop();

    geometry_msgs::Point start_p;
    start_p.x = node->pose.x;
    start_p.y = node->pose.y;
    start_p.z = 0;

    for (const auto& child : node->children) {
      q.push(child);

      geometry_msgs::Point end_p;
      end_p.x = child->pose.x;
      end_p.y = child->pose.y;
      end_p.z = 0;

      line_list.points.push_back(start_p);
      line_list.points.push_back(end_p);
    }
  }

  tree_pub_.publish(line_list);
}

};  // namespace rrt_planner