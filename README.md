# mav_teleop_keyboard
Keyboard position control for an MAV
---

Provides a simple rospy node to control an MAV in Gazebo via position commands.

Requires a position controller which takes position commands.

## Installation

Requires a working ROS [installation](https://wiki.ros.org/Ubuntu/Installation) on Ubuntu 18.04.

Configure a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

Clone this repository, then build it:
```
cd ~/catkin_ws/src
git clone git@github.com:ashwin2802/mav_teleop_keyboard
# use https://github.com/ashwin2802/mav_teleop_keyboard if SSH is not configured
catkin build  # or use catkin_make inside ~/catkin_ws/src
source ~/catkin_ws/devel/setup.bash
```

Sample launch file provided in `default.launch`.

It uses [RotorS](https://github.com/ethz-asl/rotors_simulator) for simulating the quadrotor and [mav_nonlinear_mpc](https://github.com/ethz-asl/mav_control_rw) for the position controller.

Run the sample launch file after building the package using:
```
roslaunch mav_teleop_keyboard default.launch
```

## ROS Connections

### Published Topics

`command/pose` : Topic on which the position command is published for the controller

### Subscribed Topics

`ground_truth/odometry` : Topic on which the MAV odometry gets published

## Credits

Publisher thread and key input support is borrowed from [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard).
