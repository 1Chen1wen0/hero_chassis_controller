# hero_chassis_controller

ROS Noetic `ros_control` controller for the four-wheel mecanum Hero chassis.

## Implemented requirements

- Four independent `control_toolbox::Pid` wheel-speed loops.
- `/cmd_vel` inverse kinematics using configurable wheel radius, wheelbase, and track width.
- Forward kinematics, integrated odometry on `/odom`, and `odom -> base_link` TF.
- `base` and `global` command modes. Global commands are transformed from `global_frame` to `base_link` with TF.
- Command timeout, proportional wheel-speed limiting, effort limiting, and realtime-safe command/publisher paths.
- C++ mecanum keyboard teleoperation with configurable speed, acceleration limits, and automatic stopping.

Wheel order is front-left, front-right, back-left, back-right. With `x` forward, `y` left, and positive yaw counterclockwise:

```text
w_fl = (vx - vy - (L + W) / 2 * wz) / r
w_fr = (vx + vy + (L + W) / 2 * wz) / r
w_bl = (vx + vy - (L + W) / 2 * wz) / r
w_br = (vx - vy + (L + W) / 2 * wz) / r
```

## Build

Run inside the ROS Noetic container:

```bash
source /opt/ros/noetic/setup.bash
cd /home/data/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build hero_chassis_controller
source devel/setup.bash
```

## Run

When `command_mode` is omitted, the launch files use `controller/hero_chassis_controller/command_mode`
from `config/controllers.yaml` (`base` or `global`). An explicit `command_mode:=base` or
`command_mode:=global` overrides the YAML value. Restart the controller after changing the mode.

```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch paused:=false
```

Base-frame command mode:

```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch \
  paused:=false command_mode:=base
```

Send test commands from another sourced container terminal:

```bash
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Global-frame command mode interprets `linear.x` and `linear.y` in `global_frame` (default `odom`) and transforms them to `base_link`:

```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch \
  paused:=false command_mode:=global
```

## Gazebo navigation

The navigation simulation starts the dedicated obstacle arena, the Hero chassis controller, a 360-degree laser,
GMapping, `move_base`, the holonomic DWA local planner, and RViz:

```bash
source /opt/ros/noetic/setup.bash
source /home/data/catkin_ws/devel/setup.bash
roslaunch hero_chassis_controller navigation_simulation.launch
```

Wait until the map and laser points appear in RViz, then use **2D Nav Goal** to set a target. GMapping owns
`map -> odom`, the chassis controller owns `odom -> base_link`, and `robot_state_publisher` owns
`base_link -> laser_link`. The DWA configuration samples both `linear.x` and `linear.y` for mecanum motion.

For a headless check:

```bash
roslaunch hero_chassis_controller navigation_simulation.launch gui:=false rviz:=false
```

The launch defaults to `worlds/navigation.world`. Another Gazebo world can be selected with an absolute path:

```bash
roslaunch hero_chassis_controller navigation_simulation.launch \
  world_name:=/absolute/path/to/arena.world
```

The selected world must have collision geometry at the laser height. Do not run `keyboard_teleop` or
`simple_navigator` with `move_base`; all three publish `/cmd_vel`.

## Keyboard control

Start the simulation and controller in one interactive container terminal:

```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch \
  paused:=false command_mode:=base keyboard:=true
```

Alternatively, keep the simulation launch unchanged and start the keyboard node in a second sourced terminal:

```bash
roslaunch hero_chassis_controller keyboard_teleop.launch
```

The keyboard mapping is `W/S` for forward/backward, `A/D` for left/right strafing, `Q/E` for
counterclockwise/clockwise rotation, `U/O/M/.` for diagonal motion, and `Space` (or `K`) for an
immediate stop. Hold a movement key to drive; the node smoothly stops after the key is released.
Run the command from an interactive terminal (`docker exec -it ...`) so the node can read keys.

Keyboard speed, acceleration, publication rate, and release timeout are configured in
`config/keyboard_teleop.yaml`. The node publishes `geometry_msgs/Twist` to `/cmd_vel`, so it uses
the same controller interface as any autonomous command source. Do not run another `/cmd_vel`
publisher at the same time.

## Configuration

Parameters and PID gains are in `config/controllers.yaml`. PID gains appear under:

```text
/controller/hero_chassis_controller/pid/front_left
/controller/hero_chassis_controller/pid/front_right
/controller/hero_chassis_controller/pid/back_left
/controller/hero_chassis_controller/pid/back_right
```

Use `rqt_plot` to inspect the wheel velocities in `/joint_states` while tuning gains. Each
`control_toolbox::Pid` initializes a dynamic-reconfigure server, so gains can be changed while the
controller is running:

```bash
rosrun rqt_plot rqt_plot
rosrun rqt_reconfigure rqt_reconfigure
```

Useful checks:

```bash
rostopic echo /odom
rosrun tf tf_echo odom base_link
rostopic echo /joint_states
rosservice call /controller_manager/list_controllers
```

Run tests with:

```bash
catkin run_tests hero_chassis_controller
catkin_test_results
```
