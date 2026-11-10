// Copyright (c) 2026 1Chen1wen0. All rights reserved.

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mecanum_kinematics.h"
#include "hero_chassis_controller/mecanum_kinematics.h"

namespace hero_chassis_controller
{

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController() = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  struct Command
  {
    ChassisVelocity velocity;
    ros::Time stamp;
    bool valid{false};
  };

  enum WheelIndex
  {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    BACK_LEFT = 2,
    BACK_RIGHT = 3
  };

  bool readParameters(ros::NodeHandle& controller_nh);
  bool initPids(ros::NodeHandle& controller_nh);
  void commandCallback(const geometry_msgs::TwistConstPtr& message);
  bool transformGlobalCommand(const ChassisVelocity& global_velocity, ChassisVelocity* base_velocity);
  void limitWheelVelocities(std::array<double, 4>* wheel_velocities) const;
  void updateOdometry(const ros::Time& time, const ros::Duration& period,
                      const std::array<double, 4>& wheel_velocities);
  void publishOdometry(const ros::Time& time, const ChassisVelocity& velocity);
  void resetPids();

  std::vector<hardware_interface::JointHandle> wheel_joints_;
  std::array<control_toolbox::Pid, 4> wheel_pids_;
  std::unique_ptr<MecanumKinematics> kinematics_;

  ros::Subscriber command_subscriber_;
  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odometry_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> tf_publisher_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<std::string> joint_names_;
  std::string command_mode_{"base"};
  std::string base_frame_{"base_link"};
  std::string odom_frame_{"odom"};
  std::string global_frame_{"odom"};

  double wheel_radius_{0.07625};
  double wheel_base_{0.4};
  double wheel_track_{0.4};
  double max_wheel_speed_{40.0};
  double max_effort_{5.0};
  double command_timeout_{0.5};
  double transform_timeout_{0.02};
  double publish_rate_{50.0};
  bool publish_tf_{true};
  bool command_timed_out_{true};

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  ros::Time last_publish_time_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
