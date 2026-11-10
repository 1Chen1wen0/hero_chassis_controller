// Copyright (c) 2026 1Chen1wen0. All rights reserved.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hero_chassis_controller
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(maximum, value));
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double approach(double current, double target, double maximum_change)
{
  return current + clamp(target - current, -maximum_change, maximum_change);
}

void approachVector(double current_x, double current_y, double target_x, double target_y, double maximum_change,
                    double* result_x, double* result_y)
{
  const double change_x = target_x - current_x;
  const double change_y = target_y - current_y;
  const double change_length = std::hypot(change_x, change_y);
  if (change_length <= maximum_change || change_length == 0.0)
  {
    *result_x = target_x;
    *result_y = target_y;
    return;
  }

  const double scale = maximum_change / change_length;
  *result_x = current_x + change_x * scale;
  *result_y = current_y + change_y * scale;
}
}  // namespace

class SimpleNavigator
{
public:
  SimpleNavigator(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle)
    : tf_listener_(tf_buffer_)
  {
    private_node_handle.param("odom_frame", odom_frame_, std::string("odom"));
    private_node_handle.param("control_rate", control_rate_, 20.0);
    private_node_handle.param("linear_gain", linear_gain_, 1.2);
    private_node_handle.param("angular_gain", angular_gain_, 2.0);
    private_node_handle.param("max_linear_speed", max_linear_speed_, 0.8);
    private_node_handle.param("max_angular_speed", max_angular_speed_, 1.5);
    private_node_handle.param("linear_acceleration", linear_acceleration_, 1.5);
    private_node_handle.param("angular_acceleration", angular_acceleration_, 3.0);
    private_node_handle.param("position_tolerance", position_tolerance_, 0.05);
    private_node_handle.param("yaw_tolerance", yaw_tolerance_, 0.05);
    private_node_handle.param("odom_timeout", odom_timeout_, 0.5);
    private_node_handle.param("transform_timeout", transform_timeout_, 0.2);
    private_node_handle.param("use_scan", use_scan_, false);
    private_node_handle.param("scan_timeout", scan_timeout_, 0.5);
    private_node_handle.param("obstacle_stop_distance", obstacle_stop_distance_, 0.35);
    private_node_handle.param("obstacle_slow_distance", obstacle_slow_distance_, 0.8);
    private_node_handle.param("obstacle_sector_half_angle", obstacle_sector_half_angle_, 0.45);

    validateParameters();

    command_publisher_ = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    odometry_subscriber_ = node_handle.subscribe("odom", 1, &SimpleNavigator::odometryCallback, this);
    goal_subscriber_ = node_handle.subscribe("move_base_simple/goal", 1, &SimpleNavigator::goalCallback, this);
    if (use_scan_)
      scan_subscriber_ = node_handle.subscribe("scan", 1, &SimpleNavigator::scanCallback, this);

    control_timer_ = node_handle.createTimer(ros::Duration(1.0 / control_rate_), &SimpleNavigator::control, this);
    ROS_INFO("Simple navigator ready: goal frame is '%s', laser safety is %s", odom_frame_.c_str(),
             use_scan_ ? "enabled" : "disabled");
  }

  ~SimpleNavigator()
  {
    publishStop();
  }

private:
  static void validatePositive(const std::string& name, double value)
  {
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("~" + name + " must be finite and greater than zero");
  }

  void validateParameters() const
  {
    validatePositive("control_rate", control_rate_);
    validatePositive("linear_gain", linear_gain_);
    validatePositive("angular_gain", angular_gain_);
    validatePositive("max_linear_speed", max_linear_speed_);
    validatePositive("max_angular_speed", max_angular_speed_);
    validatePositive("linear_acceleration", linear_acceleration_);
    validatePositive("angular_acceleration", angular_acceleration_);
    validatePositive("position_tolerance", position_tolerance_);
    validatePositive("yaw_tolerance", yaw_tolerance_);
    validatePositive("odom_timeout", odom_timeout_);
    validatePositive("transform_timeout", transform_timeout_);
    validatePositive("scan_timeout", scan_timeout_);
    validatePositive("obstacle_slow_distance", obstacle_slow_distance_);
    validatePositive("obstacle_sector_half_angle", obstacle_sector_half_angle_);
    if (!std::isfinite(obstacle_stop_distance_) || obstacle_stop_distance_ < 0.0)
      throw std::invalid_argument("~obstacle_stop_distance must be finite and non-negative");
    if (obstacle_slow_distance_ <= obstacle_stop_distance_)
      throw std::invalid_argument("~obstacle_slow_distance must be greater than ~obstacle_stop_distance");
    if (obstacle_sector_half_angle_ > kPi)
      throw std::invalid_argument("~obstacle_sector_half_angle must not exceed pi");
  }

  void odometryCallback(const nav_msgs::OdometryConstPtr& message)
  {
    const auto& position = message->pose.pose.position;
    const double yaw = tf2::getYaw(message->pose.pose.orientation);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(yaw))
    {
      ROS_WARN_THROTTLE(1.0, "Ignoring non-finite odometry");
      return;
    }

    robot_x_ = position.x;
    robot_y_ = position.y;
    robot_yaw_ = yaw;
    odometry_received_ = true;
    last_odometry_time_ = ros::Time::now();
  }

  void scanCallback(const sensor_msgs::LaserScanConstPtr& message)
  {
    latest_scan_ = message;
    last_scan_time_ = ros::Time::now();
  }

  void goalCallback(const geometry_msgs::PoseStampedConstPtr& message)
  {
    geometry_msgs::PoseStamped source = *message;
    if (source.header.frame_id.empty())
      source.header.frame_id = odom_frame_;

    geometry_msgs::PoseStamped goal;
    try
    {
      if (source.header.frame_id == odom_frame_)
        goal = source;
      else
        goal = tf_buffer_.transform(source, odom_frame_, ros::Duration(transform_timeout_));
    }
    catch (const tf2::TransformException& exception)
    {
      ROS_WARN("Rejected goal in frame '%s': %s", source.header.frame_id.c_str(), exception.what());
      return;
    }

    const double yaw = tf2::getYaw(goal.pose.orientation);
    if (!std::isfinite(goal.pose.position.x) || !std::isfinite(goal.pose.position.y) || !std::isfinite(yaw))
    {
      ROS_WARN("Rejected goal containing non-finite values");
      return;
    }

    goal_x_ = goal.pose.position.x;
    goal_y_ = goal.pose.position.y;
    goal_yaw_ = yaw;
    goal_active_ = true;
    command_active_ = true;
    ROS_INFO("Accepted goal: x=%.3f, y=%.3f, yaw=%.3f in %s", goal_x_, goal_y_, goal_yaw_,
             odom_frame_.c_str());
  }

  void control(const ros::TimerEvent& event)
  {
    if (!goal_active_)
      return;

    const ros::Time now = ros::Time::now();
    if (!odometry_received_ || (now - last_odometry_time_).toSec() > odom_timeout_)
    {
      ROS_WARN_THROTTLE(1.0, "Navigation stopped while waiting for fresh odometry");
      publishStop();
      return;
    }
    if (use_scan_ && (!latest_scan_ || (now - last_scan_time_).toSec() > scan_timeout_))
    {
      ROS_WARN_THROTTLE(1.0, "Navigation stopped while waiting for a fresh laser scan");
      publishStop();
      return;
    }

    const double error_x = goal_x_ - robot_x_;
    const double error_y = goal_y_ - robot_y_;
    const double distance = std::hypot(error_x, error_y);
    const double yaw_error = normalizeAngle(goal_yaw_ - robot_yaw_);

    if (distance <= position_tolerance_ && std::abs(yaw_error) <= yaw_tolerance_)
    {
      goal_active_ = false;
      publishStop();
      ROS_INFO("Navigation goal reached");
      return;
    }

    double desired_x = 0.0;
    double desired_y = 0.0;
    if (distance > position_tolerance_)
    {
      const double speed = std::min(max_linear_speed_, linear_gain_ * distance);
      const double cosine = std::cos(robot_yaw_);
      const double sine = std::sin(robot_yaw_);
      const double local_error_x = cosine * error_x + sine * error_y;
      const double local_error_y = -sine * error_x + cosine * error_y;
      desired_x = speed * local_error_x / distance;
      desired_y = speed * local_error_y / distance;

      if (use_scan_)
      {
        const double clearance = clearanceInDirection(std::atan2(desired_y, desired_x));
        const double scale = clamp((clearance - obstacle_stop_distance_) /
                                     (obstacle_slow_distance_ - obstacle_stop_distance_),
                                   0.0, 1.0);
        desired_x *= scale;
        desired_y *= scale;
        if (scale == 0.0)
          ROS_WARN_THROTTLE(1.0, "Obstacle blocks the direction of travel; navigation is stopped");
      }
    }

    const double desired_yaw = clamp(angular_gain_ * yaw_error, -max_angular_speed_, max_angular_speed_);
    double elapsed = (event.current_real - event.last_real).toSec();
    if (!std::isfinite(elapsed) || elapsed <= 0.0)
      elapsed = 1.0 / control_rate_;

    approachVector(current_x_, current_y_, desired_x, desired_y, linear_acceleration_ * elapsed, &current_x_,
                   &current_y_);
    current_yaw_ = approach(current_yaw_, desired_yaw, angular_acceleration_ * elapsed);
    publishCommand();
  }

  double clearanceInDirection(double motion_angle) const
  {
    double minimum_range = std::numeric_limits<double>::infinity();
    double ray_angle = latest_scan_->angle_min;
    for (const float range : latest_scan_->ranges)
    {
      const double angle_error = std::abs(normalizeAngle(ray_angle - motion_angle));
      if (angle_error <= obstacle_sector_half_angle_ && std::isfinite(range) && range >= latest_scan_->range_min &&
          range <= latest_scan_->range_max)
      {
        minimum_range = std::min(minimum_range, static_cast<double>(range));
      }
      ray_angle += latest_scan_->angle_increment;
    }
    return minimum_range;
  }

  void publishCommand()
  {
    geometry_msgs::Twist command;
    command.linear.x = current_x_;
    command.linear.y = current_y_;
    command.angular.z = current_yaw_;
    command_publisher_.publish(command);
  }

  void publishStop()
  {
    if (!command_active_)
      return;

    current_x_ = 0.0;
    current_y_ = 0.0;
    current_yaw_ = 0.0;
    command_publisher_.publish(geometry_msgs::Twist{});
    command_active_ = goal_active_;
  }

  ros::Publisher command_publisher_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber goal_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::Timer control_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  sensor_msgs::LaserScanConstPtr latest_scan_;

  std::string odom_frame_{"odom"};
  double control_rate_{20.0};
  double linear_gain_{1.2};
  double angular_gain_{2.0};
  double max_linear_speed_{0.8};
  double max_angular_speed_{1.5};
  double linear_acceleration_{1.5};
  double angular_acceleration_{3.0};
  double position_tolerance_{0.05};
  double yaw_tolerance_{0.05};
  double odom_timeout_{0.5};
  double transform_timeout_{0.2};
  bool use_scan_{false};
  double scan_timeout_{0.5};
  double obstacle_stop_distance_{0.35};
  double obstacle_slow_distance_{0.8};
  double obstacle_sector_half_angle_{0.45};

  bool odometry_received_{false};
  bool goal_active_{false};
  bool command_active_{false};
  ros::Time last_odometry_time_;
  ros::Time last_scan_time_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
};
}  // namespace hero_chassis_controller

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigator");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  try
  {
    hero_chassis_controller::SimpleNavigator navigator(node_handle, private_node_handle);
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL("Failed to start simple navigator: %s", exception.what());
    return 1;
  }
  return 0;
}
