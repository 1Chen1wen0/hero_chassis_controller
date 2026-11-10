// Copyright (c) 2026 1Chen1wen0. All rights reserved.

#ifndef HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H
#define HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H

#include <array>
#include <utility>

namespace hero_chassis_controller
{
//定义x,y轴速度，沿z轴旋转速度（从上向下逆时针为正）
struct ChassisVelocity
{
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

class MecanumKinematics
{
public:
  MecanumKinematics(double wheel_radius, double wheel_base, double wheel_track);

  std::array<double, 4> inverse(const ChassisVelocity& velocity) const;
  ChassisVelocity forward(const std::array<double, 4>& wheel_velocities) const;

private:
  double wheel_radius_;
  double rotation_radius_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H
