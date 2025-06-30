// hybrid_a_star_node.cpp

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hybrid_astar/hybrid_astar_node.hpp"
#include "hybrid_astar/planner.hpp"
#include "hybrid_astar/utils.hpp"

#include <optional>

HybridAStarNode::HybridAStarNode() : Node{"hybrid_astar_node"} {
    RCLCPP_INFO(this->get_logger(), "Hybrid A* node initialized.");

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1));
                   
    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/start"
      , latched_qos
      , std::bind(&HybridAStarNode::startCallback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal"
      , latched_qos
      , std::bind(&HybridAStarNode::goalCallback, this, std::placeholders::_1));

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map"
      , latched_qos
      , std::bind(&HybridAStarNode::mapCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", latched_qos);

    has_planned_ = false;
  }

void HybridAStarNode::startCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(get_logger(),
    "Got start: [%.2f, %.2f] θ=%.2f",
     msg->pose.position.x,
     msg->pose.position.y,
     // extract yaw same as planner does
     2*atan2(msg->pose.orientation.z, msg->pose.orientation.w));
  start_ = *msg;
  has_planned_ = false;

  tryPlan();
}

void HybridAStarNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(get_logger(),
    "Got goal: [%.2f, %.2f] θ=%.2f",
     msg->pose.position.x,
     msg->pose.position.y,
     // extract yaw same as planner does
     2*atan2(msg->pose.orientation.z, msg->pose.orientation.w));
  goal_ = *msg;
  has_planned_ = false;

  tryPlan();
}

void HybridAStarNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(get_logger(),
    "Got map: %ux%u @%.2fm",
     msg->info.width,
     msg->info.height,
     msg->info.resolution);
  map_ = *msg;

  tryPlan();
}

void HybridAStarNode::tryPlan(){
  if (!start_ || !goal_ || !map_) {
    return;
  }
  
  if (has_planned_) {
    return;
  }

  has_planned_ = true;  // mark as planned, to avoid re-planning until new data arrives


  if (start_ && goal_ && map_) {
    RCLCPP_INFO(get_logger(), "Planning path...");
    
    VehicleModel vm;
    hybrid_astar::HybridAStarPlanner planner(vm, YAW_RES, YAW_BINS);
    auto maybe_path = planner.plan(*start_, *goal_, *map_);
    if (maybe_path) {
      path_pub_->publish(*maybe_path);
      RCLCPP_INFO(get_logger(), "Published planned path with %zu poses.", maybe_path->poses.size());
    } else {
      RCLCPP_WARN(get_logger(), "Planning failed: no valid path found.");
    }
  }
  else {
    RCLCPP_WARN(get_logger(), "Cannot plan: missing start, goal, or map data.");
  }
}
