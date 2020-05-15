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
}

RRTree::Node::Ptr RRTree::getNearestNode(const Pose2D& pose) const {
  if (!root_node_) {
    ROS_ERROR(
        "The RRTree has not been initialized, you must call "
        "initialize().");
    return nullptr;
  }

  // BFS for closest node
  std::queue<Node::Ptr> q;
  q.push(root_node_);

  double min_distance = std::numeric_limits<double>::max();
  Node::Ptr min_node = nullptr;

  while (!q.empty()) {
    Node::Ptr node = q.front();
    q.pop();

    double distance = Pose2D::distanceBetweenPoses(pose, node->pose);
    if (distance < min_distance) {
      min_distance = distance;
      min_node = node;
    }

    for (const auto& child : node->children) {
      q.push(child);
    }
  }

  return min_node;
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