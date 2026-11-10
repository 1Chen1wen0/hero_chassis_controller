// Copyright (c) 2026 1Chen1wen0. All rights reserved.

#include "hero_chassis_controller/hero_chassis_controller.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hero_chassis_controller
{
namespace
{
//角度范围限制
double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  if (!readParameters(controller_nh))
  {
    return false;
  }

  try//获取轮子句柄和创建控制对象
  {
    wheel_joints_.reserve(joint_names_.size());
    for (const auto& joint_name : joint_names_)
    {
      wheel_joints_.push_back(effort_joint_interface->getHandle(joint_name));
    }
    kinematics_.reset(new MecanumKinematics(wheel_radius_, wheel_base_, wheel_track_));
  }
  catch (const std::exception& exception)
  {
    ROS_ERROR_STREAM("Failed to initialize hero chassis controller: " << exception.what());
    return false;
  }
//pid初始化
  if (!initPids(controller_nh))
  {
    return false;
  }
/*创建发布者和订阅者*/

  command_buffer_.writeFromNonRT(Command{});
  command_subscriber_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::commandCallback, this);

  odometry_publisher_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 10));
  odometry_publisher_->msg_.header.frame_id = odom_frame_;
  odometry_publisher_->msg_.child_frame_id = base_frame_;

  if (publish_tf_)
  {
    tf_publisher_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(root_nh, "/tf", 10));
    tf_publisher_->msg_.transforms.resize(1);
  }

  if (command_mode_ == "global")
  {
    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  }

  ROS_INFO_STREAM("Hero chassis controller initialized in '" << command_mode_ << "' mode");
  return true;
}

bool HeroChassisController::readParameters(ros::NodeHandle& controller_nh)
{
  joint_names_.clear();
  joint_names_.push_back("left_front_wheel_joint");
  joint_names_.push_back("right_front_wheel_joint");
  joint_names_.push_back("left_back_wheel_joint");
  joint_names_.push_back("right_back_wheel_joint");
  controller_nh.getParam("joints", joint_names_);
  if (joint_names_.size() != 4)
  {
    ROS_ERROR("Parameter 'joints' must contain exactly four wheel joint names");
    return false;
  }
//设置参数默认值，给参数赋值
  controller_nh.param("wheel_radius", wheel_radius_, wheel_radius_);
  controller_nh.param("wheel_base", wheel_base_, wheel_base_);
  controller_nh.param("wheel_track", wheel_track_, wheel_track_);
  controller_nh.param("max_wheel_speed", max_wheel_speed_, max_wheel_speed_);
  controller_nh.param("max_effort", max_effort_, max_effort_);
  controller_nh.param("command_timeout", command_timeout_, command_timeout_);
  controller_nh.param("transform_timeout", transform_timeout_, transform_timeout_);
  controller_nh.param("publish_rate", publish_rate_, publish_rate_);
  controller_nh.param("publish_tf", publish_tf_, publish_tf_);
  controller_nh.param("command_mode", command_mode_, command_mode_);
  controller_nh.param("base_frame", base_frame_, base_frame_);
  controller_nh.param("odom_frame", odom_frame_, odom_frame_);
  controller_nh.param("global_frame", global_frame_, global_frame_);

  if (wheel_radius_ <= 0.0 || wheel_base_ <= 0.0 || wheel_track_ <= 0.0 || max_wheel_speed_ <= 0.0 ||
      max_effort_ <= 0.0 || command_timeout_ <= 0.0 || publish_rate_ <= 0.0)
  {
    ROS_ERROR("Chassis dimensions, limits, timeout, and publish rate must be positive");
    return false;
  }
  if (command_mode_ != "base" && command_mode_ != "global")
  {
    ROS_ERROR("Parameter 'command_mode' must be either 'base' or 'global'");
    return false;
  }
  return true;
}

bool HeroChassisController::initPids(ros::NodeHandle& controller_nh)
{
  const std::array<std::string, 4> pid_names = {{"front_left", "front_right", "back_left", "back_right"}};
  for (std::size_t index = 0; index < wheel_pids_.size(); ++index)
  {
    ros::NodeHandle pid_nh(controller_nh, "pid/" + pid_names[index]);
    if (!wheel_pids_[index].init(pid_nh))
    {
      ROS_ERROR_STREAM("Failed to initialize PID for " << pid_names[index]);
      return false;
    }
  }
  return true;
}

void HeroChassisController::starting(const ros::Time& time)
{
  resetPids();
  for (auto& joint : wheel_joints_)
  {
    joint.setCommand(0.0);
  }
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;
  last_publish_time_ = time;
  command_timed_out_ = true;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  std::array<double, 4> actual_wheel_velocities{};
  for (std::size_t index = 0; index < wheel_joints_.size(); ++index)
  {
    actual_wheel_velocities[index] = wheel_joints_[index].getVelocity();
  }

  updateOdometry(time, period, actual_wheel_velocities);

  const Command* command = command_buffer_.readFromRT();
  const bool timed_out = command == nullptr || !command->valid || time < command->stamp ||
                         (time - command->stamp).toSec() > command_timeout_;

  ChassisVelocity target_velocity;
  if (!timed_out)
  {
    target_velocity = command->velocity;
  }
  else if (!command_timed_out_)
  {
    resetPids();
  }
  command_timed_out_ = timed_out;

  auto target_wheel_velocities = kinematics_->inverse(target_velocity);
  limitWheelVelocities(&target_wheel_velocities);

  for (std::size_t index = 0; index < wheel_joints_.size(); ++index)
  {
    const double error = target_wheel_velocities[index] - actual_wheel_velocities[index];
    const double effort = wheel_pids_[index].computeCommand(error, period);
    wheel_joints_[index].setCommand(std::max(-max_effort_, std::min(max_effort_, effort)));
  }
}

void HeroChassisController::stopping(const ros::Time& /*time*/)
{
  for (auto& joint : wheel_joints_)
  {
    joint.setCommand(0.0);
  }
  resetPids();
}

void HeroChassisController::commandCallback(const geometry_msgs::TwistConstPtr& message)
{
  Command command;
  command.velocity.linear_x = message->linear.x;
  command.velocity.linear_y = message->linear.y;
  command.velocity.angular_z = message->angular.z;
  command.stamp = ros::Time::now();
  command.valid = true;

  if (command_mode_ == "global" && !transformGlobalCommand(command.velocity, &command.velocity))
  {
    return;
  }
  command_buffer_.writeFromNonRT(command);
}

bool HeroChassisController::transformGlobalCommand(const ChassisVelocity& global_velocity,
                                                   ChassisVelocity* base_velocity)
{
  geometry_msgs::Vector3Stamped global_vector;
  global_vector.header.frame_id = global_frame_;
  global_vector.header.stamp = ros::Time(0);
  global_vector.vector.x = global_velocity.linear_x;
  global_vector.vector.y = global_velocity.linear_y;

  try
  {
    const auto transform =
        tf_buffer_->lookupTransform(base_frame_, global_frame_, ros::Time(0), ros::Duration(transform_timeout_));
    geometry_msgs::Vector3Stamped base_vector;
    tf2::doTransform(global_vector, base_vector, transform);
    base_velocity->linear_x = base_vector.vector.x;
    base_velocity->linear_y = base_vector.vector.y;
    base_velocity->angular_z = global_velocity.angular_z;
    return true;
  }
  catch (const tf2::TransformException& exception)
  {
    ROS_WARN_THROTTLE(1.0, "Cannot transform cmd_vel from %s to %s: %s", global_frame_.c_str(), base_frame_.c_str(),
                      exception.what());
    return false;
  }
}

void HeroChassisController::limitWheelVelocities(std::array<double, 4>* wheel_velocities) const
{
  double largest_speed = 0.0;
  for (const double velocity : *wheel_velocities)
  {
    largest_speed = std::max(largest_speed, std::abs(velocity));
  }
  if (largest_speed <= max_wheel_speed_)
  {
    return;
  }
  const double scale = max_wheel_speed_ / largest_speed;
  for (double& velocity : *wheel_velocities)
  {
    velocity *= scale;
  }
}

void HeroChassisController::updateOdometry(const ros::Time& time, const ros::Duration& period,
                                           const std::array<double, 4>& wheel_velocities)
{
  const ChassisVelocity velocity = kinematics_->forward(wheel_velocities);
  const double dt = period.toSec();
  const double cosine = std::cos(odom_yaw_);
  const double sine = std::sin(odom_yaw_);
  odom_x_ += (cosine * velocity.linear_x - sine * velocity.linear_y) * dt;
  odom_y_ += (sine * velocity.linear_x + cosine * velocity.linear_y) * dt;
  odom_yaw_ = normalizeAngle(odom_yaw_ + velocity.angular_z * dt);

  if ((time - last_publish_time_).toSec() >= 1.0 / publish_rate_)
  {
    publishOdometry(time, velocity);
    last_publish_time_ = time;
  }
}

void HeroChassisController::publishOdometry(const ros::Time& time, const ChassisVelocity& velocity)
{
  const double quaternion_z = std::sin(odom_yaw_ / 2.0);
  const double quaternion_w = std::cos(odom_yaw_ / 2.0);

  if (odometry_publisher_->trylock())
  {
    auto& message = odometry_publisher_->msg_;
    message.header.stamp = time;
    message.pose.pose.position.x = odom_x_;
    message.pose.pose.position.y = odom_y_;
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation.x = 0.0;
    message.pose.pose.orientation.y = 0.0;
    message.pose.pose.orientation.z = quaternion_z;
    message.pose.pose.orientation.w = quaternion_w;
    message.twist.twist.linear.x = velocity.linear_x;
    message.twist.twist.linear.y = velocity.linear_y;
    message.twist.twist.angular.z = velocity.angular_z;
    odometry_publisher_->unlockAndPublish();
  }

  if (tf_publisher_ && tf_publisher_->trylock())
  {
    auto& transform = tf_publisher_->msg_.transforms[0];
    transform.header.stamp = time;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odom_x_;
    transform.transform.translation.y = odom_y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = quaternion_z;
    transform.transform.rotation.w = quaternion_w;
    tf_publisher_->unlockAndPublish();
  }
}

void HeroChassisController::resetPids()
{
  for (auto& pid : wheel_pids_)
  {
    pid.reset();
  }
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)

}  // namespace hero_chassis_controller
