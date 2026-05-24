#ifndef MM_NAVIGATION__OMNI_LINE_CONTROLLER_HPP_
#define MM_NAVIGATION__OMNI_LINE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace mm_navigation
{

class OmniLineController : public nav2_core::Controller
{
public:
  OmniLineController() = default;
  ~OmniLineController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  enum class Mode
  {
    TRACKING,
    AVOIDING
  };

  struct Vec2
  {
    double x{0.0};
    double y{0.0};
  };

  struct LineState
  {
    Vec2 start;
    Vec2 end;
    Vec2 unit;
    Vec2 normal;
    double length{0.0};
    double progress{0.0};
    double cross_track_error{0.0};
    double distance_to_end{0.0};
    double heading{0.0};
  };

  struct Candidate
  {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
  };

  struct Command
  {
    geometry_msgs::msg::TwistStamped twist;
    double vx_world{0.0};
    double vy_world{0.0};
  };

  Command makePathTrackingCommand(const geometry_msgs::msg::PoseStamped & pose);
  geometry_msgs::msg::PoseStamped transformPoseToPlanFrame(
    const geometry_msgs::msg::PoseStamped & pose) const;
  std::size_t nearestPathIndex(const geometry_msgs::msg::PoseStamped & pose) const;
  std::size_t lookaheadPathIndex(std::size_t start_index, const geometry_msgs::msg::PoseStamped & pose) const;
  double commandSafetyScale(const geometry_msgs::msg::PoseStamped & pose, double vx, double vy) const;
  double maxCostForCommand(const geometry_msgs::msg::PoseStamped & pose, double vx, double vy) const;
  double costAtWorld(double wx, double wy) const;

  geometry_msgs::msg::TwistStamped makeZeroCommand(const geometry_msgs::msg::PoseStamped & pose) const;
  geometry_msgs::msg::TwistStamped makeCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    double vx_world,
    double vy_world,
    double wz,
    double stamp_scale = 1.0) const;

  double yawFromPose(const geometry_msgs::msg::PoseStamped & pose) const;
  static double normalizeAngle(double angle);
  static double clamp(double value, double min_value, double max_value);
  static double hypot2(double x, double y);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("OmniLineController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  nav_msgs::msg::Path global_plan_;
  std::size_t path_index_{0};
  double speed_limit_scale_{1.0};

  double desired_linear_vel_{0.18};
  double max_linear_vel_{0.25};
  double max_lateral_vel_{0.20};
  double obstacle_slow_cost_{80.0};
  double obstacle_stop_cost_{200.0};
  double obstacle_check_distance_{0.45};
  double obstacle_check_width_{0.35};
  double lookahead_distance_{0.45};
  double goal_stop_distance_{0.08};
};

}  // namespace mm_navigation

#endif  // MM_NAVIGATION__OMNI_LINE_CONTROLLER_HPP_
