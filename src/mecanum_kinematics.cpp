// Copyright (c) 2026 1Chen1wen0. All rights reserved.

#include "hero_chassis_controller/mecanum_kinematics.h"

#include <stdexcept>
#include <utility>

namespace hero_chassis_controller
{
//参数:轮子半径 r,前后轮之间的轴距 L,左右轮之间的轮距 W
MecanumKinematics::MecanumKinematics(double wheel_radius, double wheel_base, double wheel_track)
    : wheel_radius_(wheel_radius), rotation_radius_((wheel_base + wheel_track) / 2.0)
{
  if (wheel_radius_ <= 0.0)
  {
    throw std::invalid_argument("wheel_radius must be positive");
  }
  if (rotation_radius_ <= 0.0)
  {
    throw std::invalid_argument("wheel_base and wheel_track must be positive");
  }
}
//运动学逆解，根据车体目标速度，算出每个轮子转速
std::array<double, 4> MecanumKinematics::inverse(const ChassisVelocity& velocity) const
{
  const double rotation = rotation_radius_ * velocity.angular_z;
  std::array<double, 4> wheel_velocities;
  wheel_velocities[0] = (velocity.linear_x - velocity.linear_y - rotation) / wheel_radius_;
  wheel_velocities[1] = (velocity.linear_x + velocity.linear_y + rotation) / wheel_radius_;
  wheel_velocities[2] = (velocity.linear_x + velocity.linear_y - rotation) / wheel_radius_;
  wheel_velocities[3] = (velocity.linear_x - velocity.linear_y + rotation) / wheel_radius_;
  return wheel_velocities;
}
//正解，根据四轮速度解算出三个速度分量
ChassisVelocity MecanumKinematics::forward(const std::array<double, 4>& wheel_velocities) const
{
  const double front_left = wheel_velocities[0];
  const double front_right = wheel_velocities[1];
  const double back_left = wheel_velocities[2];
  const double back_right = wheel_velocities[3];

  ChassisVelocity velocity;
  velocity.linear_x = wheel_radius_ * (front_left + front_right + back_left + back_right) / 4.0;
  velocity.linear_y = wheel_radius_ * (-front_left + front_right + back_left - back_right) / 4.0;
  velocity.angular_z = wheel_radius_ * (-front_left + front_right - back_left + back_right) / (4.0 * rotation_radius_);
  return velocity;
}

}  // namespace hero_chassis_controller
