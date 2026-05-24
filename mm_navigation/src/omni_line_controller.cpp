#include "mm_navigation/omni_line_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mm_navigation
{

void OmniLineController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  plugin_name_ = name;
  logger_ = node_->get_logger();
  clock_ = node_->get_clock();
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  auto declare_parameter = [this](const std::string & param_name, const rclcpp::ParameterValue & value) {
    if (!node_->has_parameter(plugin_name_ + "." + param_name)) {
      node_->declare_parameter(plugin_name_ + "." + param_name, value);
    }
  };

  declare_parameter("desired_linear_vel", rclcpp::ParameterValue(desired_linear_vel_));
  declare_parameter("max_linear_vel", rclcpp::ParameterValue(max_linear_vel_));
  declare_parameter("max_lateral_vel", rclcpp::ParameterValue(max_lateral_vel_));
  declare_parameter("obstacle_slow_cost", rclcpp::ParameterValue(obstacle_slow_cost_));
  declare_parameter("obstacle_stop_cost", rclcpp::ParameterValue(obstacle_stop_cost_));
  declare_parameter("obstacle_check_distance", rclcpp::ParameterValue(obstacle_check_distance_));
  declare_parameter("obstacle_check_width", rclcpp::ParameterValue(obstacle_check_width_));
  declare_parameter("lookahead_distance", rclcpp::ParameterValue(lookahead_distance_));
  declare_parameter("goal_stop_distance", rclcpp::ParameterValue(goal_stop_distance_));

  node_->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node_->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node_->get_parameter(plugin_name_ + ".max_lateral_vel", max_lateral_vel_);
  node_->get_parameter(plugin_name_ + ".obstacle_slow_cost", obstacle_slow_cost_);
  node_->get_parameter(plugin_name_ + ".obstacle_stop_cost", obstacle_stop_cost_);
  node_->get_parameter(plugin_name_ + ".obstacle_check_distance", obstacle_check_distance_);
  node_->get_parameter(plugin_name_ + ".obstacle_check_width", obstacle_check_width_);
  node_->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node_->get_parameter(plugin_name_ + ".goal_stop_distance", goal_stop_distance_);

  RCLCPP_INFO(logger_, "Configured %s for zero-angular path tracking", plugin_name_.c_str());
}

void OmniLineController::cleanup()
{
  global_plan_.poses.clear();
}

void OmniLineController::activate()
{
}

void OmniLineController::deactivate()
{
}

void OmniLineController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  path_index_ = 0;

  if (path.poses.size() >= 2) {
    RCLCPP_INFO(
      logger_, "Received path: start=(%.2f, %.2f), goal=(%.2f, %.2f), poses=%zu",
      path.poses.front().pose.position.x,
      path.poses.front().pose.position.y,
      path.poses.back().pose.position.x,
      path.poses.back().pose.position.y,
      path.poses.size());
  }
}

geometry_msgs::msg::TwistStamped OmniLineController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (global_plan_.poses.empty()) {
    return makeZeroCommand(pose);
  }

  const auto pose_in_plan_frame = transformPoseToPlanFrame(pose);
  return makePathTrackingCommand(pose_in_plan_frame).twist;
}

void OmniLineController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_limit_scale_ = clamp(speed_limit / 100.0, 0.0, 1.0);
    return;
  }

  speed_limit_scale_ = clamp(speed_limit / max_linear_vel_, 0.0, 1.0);
}

OmniLineController::Command OmniLineController::makePathTrackingCommand(
  const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & goal = global_plan_.poses.back().pose.position;
  const double goal_dx = goal.x - pose.pose.position.x;
  const double goal_dy = goal.y - pose.pose.position.y;
  const double goal_distance = hypot2(goal_dx, goal_dy);

  if (goal_distance <= 0.01) {
    return {makeZeroCommand(pose), 0.0, 0.0};
  }

  path_index_ = nearestPathIndex(pose);
  const std::size_t target_index = lookaheadPathIndex(path_index_, pose);
  const auto & target = global_plan_.poses[target_index].pose.position;

  double dx = target.x - pose.pose.position.x;
  double dy = target.y - pose.pose.position.y;
  double distance = hypot2(dx, dy);

  if (distance < 1.0e-4) {
    dx = goal_dx;
    dy = goal_dy;
    distance = std::max(goal_distance, 1.0e-4);
  }

  const double speed = desired_linear_vel_ * speed_limit_scale_;
  const double vx_world = speed * dx / distance;
  const double vy_world = speed * dy / distance;
  Command command;
  command.vx_world = vx_world;
  command.vy_world = vy_world;
  command.twist = makeCommand(pose, vx_world, vy_world, 0.0, 1.0);
  return command;
}

geometry_msgs::msg::PoseStamped OmniLineController::transformPoseToPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  const std::string & target_frame = global_plan_.header.frame_id;
  if (target_frame.empty() || pose.header.frame_id == target_frame) {
    return pose;
  }

  geometry_msgs::msg::PoseStamped transformed_pose;
  try {
    transformed_pose = tf_->transform(pose, target_frame, tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Failed to transform controller pose from %s to %s: %s",
      pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return pose;
  }

  return transformed_pose;
}

std::size_t OmniLineController::nearestPathIndex(const geometry_msgs::msg::PoseStamped & pose) const
{
  std::size_t best_index = std::min(path_index_, global_plan_.poses.size() - 1);
  double best_distance = std::numeric_limits<double>::infinity();

  for (std::size_t i = best_index; i < global_plan_.poses.size(); ++i) {
    const auto & point = global_plan_.poses[i].pose.position;
    const double distance = hypot2(point.x - pose.pose.position.x, point.y - pose.pose.position.y);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  return best_index;
}

std::size_t OmniLineController::lookaheadPathIndex(
  std::size_t start_index,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  for (std::size_t i = start_index; i < global_plan_.poses.size(); ++i) {
    const auto & point = global_plan_.poses[i].pose.position;
    const double distance = hypot2(point.x - pose.pose.position.x, point.y - pose.pose.position.y);
    if (distance >= lookahead_distance_) {
      return i;
    }
  }

  return global_plan_.poses.size() - 1;
}

double OmniLineController::commandSafetyScale(
  const geometry_msgs::msg::PoseStamped & pose, double vx, double vy) const
{
  const double cost = maxCostForCommand(pose, vx, vy);
  if (cost >= obstacle_stop_cost_) {
    return 0.0;
  }
  if (cost <= obstacle_slow_cost_) {
    return 1.0;
  }
  return clamp((obstacle_stop_cost_ - cost) / (obstacle_stop_cost_ - obstacle_slow_cost_), 0.0, 1.0);
}

double OmniLineController::maxCostForCommand(
  const geometry_msgs::msg::PoseStamped & pose, double vx, double vy) const
{
  const double speed = hypot2(vx, vy);
  if (speed < 1.0e-4) {
    return 0.0;
  }

  const double ux = vx / speed;
  const double uy = vy / speed;
  const double nx = -uy;
  const double ny = ux;
  double max_cost = 0.0;

  for (double forward = 0.10; forward <= obstacle_check_distance_; forward += 0.05) {
    for (double side = -obstacle_check_width_ * 0.5; side <= obstacle_check_width_ * 0.5; side += 0.05) {
      const double wx = pose.pose.position.x + ux * forward + nx * side;
      const double wy = pose.pose.position.y + uy * forward + ny * side;
      max_cost = std::max(max_cost, costAtWorld(wx, wy));
    }
  }

  return max_cost;
}

double OmniLineController::costAtWorld(double wx, double wy) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  return static_cast<double>(costmap_->getCost(mx, my));
}

geometry_msgs::msg::TwistStamped OmniLineController::makeZeroCommand(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  return makeCommand(pose, 0.0, 0.0, 0.0);
}

geometry_msgs::msg::TwistStamped OmniLineController::makeCommand(
  const geometry_msgs::msg::PoseStamped & pose,
  double vx_world,
  double vy_world,
  double wz,
  double stamp_scale) const
{
  const double yaw = yawFromPose(pose);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double limited_vx_world = clamp(vx_world * stamp_scale, -max_linear_vel_, max_linear_vel_);
  const double limited_vy_world = clamp(vy_world * stamp_scale, -max_lateral_vel_, max_lateral_vel_);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = pose.header.frame_id;
  cmd.twist.linear.x = cos_yaw * limited_vx_world + sin_yaw * limited_vy_world;
  cmd.twist.linear.y = -sin_yaw * limited_vx_world + cos_yaw * limited_vy_world;
  cmd.twist.angular.z = clamp(wz, 0.0, 0.0);
  return cmd;
}

double OmniLineController::yawFromPose(const geometry_msgs::msg::PoseStamped & pose) const
{
  return tf2::getYaw(pose.pose.orientation);
}

double OmniLineController::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double OmniLineController::clamp(double value, double min_value, double max_value)
{
  return std::min(std::max(value, min_value), max_value);
}

double OmniLineController::hypot2(double x, double y)
{
  return std::hypot(x, y);
}

}  // namespace mm_navigation

PLUGINLIB_EXPORT_CLASS(mm_navigation::OmniLineController, nav2_core::Controller)
