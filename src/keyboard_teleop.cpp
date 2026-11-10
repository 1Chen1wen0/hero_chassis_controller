// Copyright (c) 2026 1Chen1wen0. All rights reserved.

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace hero_chassis_controller
{
namespace
{
constexpr char kControlC = 3;

double approach(double current, double target, double maximum_change)
{
  const double difference = target - current;
  return current + std::max(-maximum_change, std::min(maximum_change, difference));
}
//平滑改变二维线速度
void approachVector(double current_x, double current_y, double target_x, double target_y, double maximum_change,
                    double& result_x, double& result_y)
{
  const double delta_x = target_x - current_x;
  const double delta_y = target_y - current_y;
  const double delta_length = std::hypot(delta_x, delta_y);
  if (delta_length <= maximum_change || delta_length == 0.0)
  {
    result_x = target_x;
    result_y = target_y;
    return;
  }

  const double scale = maximum_change / delta_length;
  result_x = current_x + delta_x * scale;
  result_y = current_y + delta_y * scale;
}

void printHelp(double linear_speed, double angular_speed)
{
  std::cout << R"(
Hero mecanum keyboard control
------------------------------
   u    w    o       forward-left / forward / forward-right
   a  SPACE  d       left / emergency stop / right
   m    s    .       back-left / back / back-right

   q / e             rotate counterclockwise / clockwise
   Ctrl-C            exit and stop

Hold a movement key to keep moving. Releasing it triggers a smooth stop.
)" << std::endl;
  std::cout << "linear speed: " << linear_speed << " m/s, angular speed: " << angular_speed << " rad/s"
            << std::endl;
}
}  // namespace

class KeyboardTeleop
{
public:
  KeyboardTeleop(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle)
    : command_publisher_(node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1))
  {
    private_node_handle.param("linear_speed", linear_speed_, 0.6);
    private_node_handle.param("angular_speed", angular_speed_, 1.2);
    private_node_handle.param("linear_acceleration", linear_acceleration_, 1.5);
    private_node_handle.param("angular_acceleration", angular_acceleration_, 3.0);
    private_node_handle.param("publish_rate", publish_rate_, 20.0);
    private_node_handle.param("key_timeout", key_timeout_, 0.6);

    validatePositive("linear_speed", linear_speed_);
    validatePositive("angular_speed", angular_speed_);
    validatePositive("linear_acceleration", linear_acceleration_);
    validatePositive("angular_acceleration", angular_acceleration_);
    validatePositive("publish_rate", publish_rate_);
    validatePositive("key_timeout", key_timeout_);

    openTerminal();
    last_motion_key_ = ros::WallTime::now();
  }

  ~KeyboardTeleop()
  {
    stopImmediately();
    restoreTerminal();
  }

  void run()
  {
    printHelp(linear_speed_, angular_speed_);
    const ros::WallDuration period(1.0 / publish_rate_);
    ros::WallTime previous_time = ros::WallTime::now();

    while (ros::ok())
    {
      const ros::WallTime cycle_start = ros::WallTime::now();
      if (!processAvailableKeys(period))
        break;

      const ros::WallTime current_time = ros::WallTime::now();
      updateCommand((current_time - previous_time).toSec());
      previous_time = current_time;
      ros::spinOnce();

      const ros::WallDuration remaining = period - (ros::WallTime::now() - cycle_start);
      if (remaining.toSec() > 0.0)
        remaining.sleep();
    }
  }

private:
  static void validatePositive(const std::string& name, double value)
  {
    if (value <= 0.0)
      throw std::invalid_argument("~" + name + " must be greater than zero");
  }

  void openTerminal()
  {
    terminal_fd_ = open("/dev/tty", O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    owns_terminal_fd_ = terminal_fd_ >= 0;
    if (terminal_fd_ < 0 && isatty(STDIN_FILENO))
      terminal_fd_ = STDIN_FILENO;
    if (terminal_fd_ < 0)
      throw std::runtime_error("keyboard input needs an interactive terminal; run docker exec with -it");

    if (tcgetattr(terminal_fd_, &original_terminal_settings_) != 0)
      throw std::runtime_error("failed to read terminal settings");

    termios raw_settings = original_terminal_settings_;
    cfmakeraw(&raw_settings);
    if (tcsetattr(terminal_fd_, TCSANOW, &raw_settings) != 0)
      throw std::runtime_error("failed to enable raw keyboard input");
    terminal_is_raw_ = true;
  }

  void restoreTerminal()
  {
    if (terminal_is_raw_)
      tcsetattr(terminal_fd_, TCSANOW, &original_terminal_settings_);
    if (owns_terminal_fd_)
      close(terminal_fd_);
    terminal_is_raw_ = false;
    terminal_fd_ = -1;
  }

  bool processAvailableKeys(const ros::WallDuration& wait)
  {
    pollfd descriptor{};
    descriptor.fd = terminal_fd_;
    descriptor.events = POLLIN;
    const int timeout_ms = static_cast<int>(std::round(wait.toSec() * 1000.0));
    int poll_result = poll(&descriptor, 1, timeout_ms);

    while (poll_result > 0 && (descriptor.revents & POLLIN) != 0)
    {
      char key = 0;
      if (read(terminal_fd_, &key, 1) == 1 && !handleKey(key))
        return false;

      descriptor.revents = 0;
      poll_result = poll(&descriptor, 1, 0);
    }
    return true;
  }
//将字符映射成动作
  bool handleKey(char key)
  {
    if (key == kControlC)
      return false;

    key = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    switch (key)
    {
      case 'w':
        setTarget(1.0, 0.0, 0.0);
        break;
      case 's':
        setTarget(-1.0, 0.0, 0.0);
        break;
      case 'a':
        setTarget(0.0, 1.0, 0.0);
        break;
      case 'd':
        setTarget(0.0, -1.0, 0.0);
        break;
      case 'q':
        setTarget(0.0, 0.0, 1.0);
        break;
      case 'e':
        setTarget(0.0, 0.0, -1.0);
        break;
      case 'u':
        setTarget(1.0, 1.0, 0.0);
        break;
      case 'o':
        setTarget(1.0, -1.0, 0.0);
        break;
      case 'm':
        setTarget(-1.0, 1.0, 0.0);
        break;
      case '.':
        setTarget(-1.0, -1.0, 0.0);
        break;
      case ' ':
      case 'k':
        stopImmediately();
        break;
      default:
        break;
    }
    return true;
  }
//将方向转换成目标速度
  void setTarget(double x, double y, double yaw)
  {
    if (x != 0.0 && y != 0.0)
    {
      constexpr double kDiagonalScale = 0.7071067811865476;
      x *= kDiagonalScale;
      y *= kDiagonalScale;
    }
    target_x_ = x * linear_speed_;
    target_y_ = y * linear_speed_;
    target_yaw_ = yaw * angular_speed_;
    last_motion_key_ = ros::WallTime::now();
  }
//判断接收超时，超时则目标速度设为零
  void updateCommand(double elapsed)
  {
    if ((ros::WallTime::now() - last_motion_key_).toSec() > key_timeout_)
      target_x_ = target_y_ = target_yaw_ = 0.0;

    approachVector(current_x_, current_y_, target_x_, target_y_, linear_acceleration_ * elapsed, current_x_,
                   current_y_);
    current_yaw_ = approach(current_yaw_, target_yaw_, angular_acceleration_ * elapsed);
    publishCommand();
  }
//发送cmd_vel消息
  void publishCommand()
  {
    geometry_msgs::Twist command;
    command.linear.x = current_x_;
    command.linear.y = current_y_;
    command.angular.z = current_yaw_;
    command_publisher_.publish(command);
  }

  void stopImmediately()
  {
    target_x_ = target_y_ = target_yaw_ = 0.0;
    current_x_ = current_y_ = current_yaw_ = 0.0;
    publishCommand();
  }

  ros::Publisher command_publisher_;
  double linear_speed_{ 0.0 };
  double angular_speed_{ 0.0 };
  double linear_acceleration_{ 0.0 };
  double angular_acceleration_{ 0.0 };
  double publish_rate_{ 0.0 };
  double key_timeout_{ 0.0 };
  double target_x_{ 0.0 };
  double target_y_{ 0.0 };
  double target_yaw_{ 0.0 };
  double current_x_{ 0.0 };
  double current_y_{ 0.0 };
  double current_yaw_{ 0.0 };
  ros::WallTime last_motion_key_;
  int terminal_fd_{ -1 };
  bool owns_terminal_fd_{ false };
  bool terminal_is_raw_{ false };
  termios original_terminal_settings_{};
};
}  // namespace hero_chassis_controller

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  try
  {
    hero_chassis_controller::KeyboardTeleop teleop(node_handle, private_node_handle);
    teleop.run();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
  return 0;
}
