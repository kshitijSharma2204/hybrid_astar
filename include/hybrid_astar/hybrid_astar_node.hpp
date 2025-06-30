#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <optional>

static constexpr double YAW_RES = M_PI/12.0; 
static constexpr int    YAW_BINS = int(2*M_PI / YAW_RES);


class HybridAStarNode : public rclcpp::Node {
public:
  HybridAStarNode();
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::optional<geometry_msgs::msg::PoseStamped> start_;
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  std::optional<nav_msgs::msg::OccupancyGrid> map_;

  bool has_planned_;

  void startCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void tryPlan();
};