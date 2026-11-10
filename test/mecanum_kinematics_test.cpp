// Copyright (c) 2026 1Chen1wen0. All rights reserved.

#include <gtest/gtest.h>

#include "hero_chassis_controller/mecanum_kinematics.h"

namespace hero_chassis_controller
{
namespace
{

constexpr double kTolerance = 1e-9;

TEST(MecanumKinematics, InverseAndForwardAreConsistent)
{
  MecanumKinematics kinematics(0.07625, 0.4, 0.4);
  const ChassisVelocity expected{1.2, -0.35, 0.7};

  const auto wheels = kinematics.inverse(expected);
  const auto actual = kinematics.forward(wheels);

  EXPECT_NEAR(expected.linear_x, actual.linear_x, kTolerance);
  EXPECT_NEAR(expected.linear_y, actual.linear_y, kTolerance);
  EXPECT_NEAR(expected.angular_z, actual.angular_z, kTolerance);
}

TEST(MecanumKinematics, ForwardMotionTurnsAllWheelsForward)
{
  MecanumKinematics kinematics(0.1, 0.4, 0.4);
  const auto wheels = kinematics.inverse(ChassisVelocity{1.0, 0.0, 0.0});

  for (const double velocity : wheels)
  {
    EXPECT_NEAR(10.0, velocity, kTolerance);
  }
}

TEST(MecanumKinematics, LeftMotionUsesExpectedWheelSigns)
{
  MecanumKinematics kinematics(0.1, 0.4, 0.4);
  const auto wheels = kinematics.inverse(ChassisVelocity{0.0, 1.0, 0.0});

  EXPECT_LT(wheels[0], 0.0);
  EXPECT_GT(wheels[1], 0.0);
  EXPECT_GT(wheels[2], 0.0);
  EXPECT_LT(wheels[3], 0.0);
}

}  // namespace
}  // namespace hero_chassis_controller
