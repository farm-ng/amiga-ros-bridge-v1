# ROS bridge for the Amiga

[![Continuous integration](https://github.com/farm-ng/amiga-ros-bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/farm-ng/amiga-ros-bridge/actions/workflows/ci.yml)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/farm-ng/amiga-ros-bridge)](https://github.com/farm-ng/amiga-ros-bridge/releases/latest)
[![License](https://img.shields.io/badge/License-ADK-blue.svg)](https://github.com/farm-ng/farm-ng-amiga/blob/main/LICENSE)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](https://www.ros.org)
[![Rust](https://img.shields.io/badge/rust-1.66.0+-93450a.svg?logo=rust)](https://www.rust-lang.org)

This repository contains a ROS bridge for the Farm-ng Amiga platform written in [Rust](https://www.rust-lang.org/).

> **Disclaimer:** the Amiga stack does not leverage ROS for control, but rather uses a gRPC service for control. This bridge is provided as a convenience for users who wish to use ROS for control. For performance critical applications, we recommend using the gRPC service directly.

## Overview

The ROS bridge, currently supported for ROS Noetic, interfaces with the [Amiga gRPC services](https://github.com/farm-ng/farm-ng-amiga) for control with ROS [`Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) messages.

The ROS bridge allows you to publish [`Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) commands on the `/amiga/cmd_vel` topic to drive the Amiga and subscribe to [`TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html) measured rates on the  `/amiga/vel` topic.

### Topics

- **Control**
  - Subscribed to by ROS bridge
  - Topic: `/amiga/cmd_vel`
  - Format: [`geometry_msgs/Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)

- **Measured**
  - Published by ROS bridge
  - Topic: `/amiga/vel`
  - Format: [`geometry_msgs/TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)

## Prerequisites

### Install dependencies

- [ROS Noetic install instructions](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Rust install instructions](https://www.rust-lang.org/learn/get-started).


### Catkin workspace

Create a catkin workspace, if you don't already have one, following the instructions at
[ROS Tutorials - create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

### Clone this repository

Clone the repository and initialize the submodule.

<!-- ** [optional] `roscd` to your ROS workspace and clone this repository there. -->

```bash
cd ~/catkin_ws/src
git clone https://github.com/farm-ng/amiga-ros-bridge.git
cd amiga-ros-bridge
git submodule update --init
cd ~/catkin_ws
```

## Run the bridge

### Terminal 1

Start ROS core:

```bash
source ~/catkin_ws/devel/setup.bash
# OR: source /opt/ros/noetic/setup.bash
roscore
```

### Terminal 2

Go to your ROS workspace and source the setup.bash file:

```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
```

#### Build the ROS bridge executables

> Warning: this may take ~10 minutes on the first install on the Amiga brain

```bash
cd ~/catkin_ws
cargo build --manifest-path=src/amiga-ros-bridge/Cargo.toml
# Same as: cd ~/catkin_ws/src/amiga-ros-bridge && cargo build && cd ~/catkin_ws
```

#### Compile the package `amiga_ros_bridge`

```bash
cd ~/catkin_ws
catkin_make clean # optionally, clean the workspace
catkin_make
```

#### Run the package in the robot as follows

```bash
rosrun amiga-ros-bridge amiga-ros-bridge -H localhost -p 50060
```

Alternatively, you can run the package in you computer as follows.
Make sure that the Amiga is connected to the same network as your computer
and that you know the IP address of the Amiga.

```bash
rosrun amiga-ros-bridge amiga-ros-bridge -H 192.168.1.98 -p 50060
```

## Control the Amiga

### Experimental Amiga Joystick

You can start the **experimental** amiga-joystick app to command velocities
to the Amiga (e.g. the mock server) and print received velocity states:

> Warning: This is not stable and may require multiple attempts at launching for this to come up.

```bash
source /opt/ros/noetic/setup.bash
cargo run --example amiga-joystick
```

Use arrow keys (left, right, up, down) to command velocities to the Amiga.

### Stable ROS packages

You can publish `Twist` commands to the ROS bridge on the `/amiga/cmd_vel` topic with the [`rqt_robot_steering`](http://wiki.ros.org/rqt_robot_steering) package.

Install `rqt_robot_steering`, if needed:

```bash
apt-get install ros-noetic-rqt-robot-steering
```

Run from your terminal:

```bash
rosrun rqt_robot_steering rqt_robot_steering
# Then change the topic to `/amiga/cmd_vel`
```

You can subscribe to measured `TwistStamped` states of the amiga with ROS command line tools.

```bash
rostopic echo /amiga/vel
```

## Try the examples

We have provided some examples to help you get started. You will find the examples in the `examples` directory.

### Python

- [`amiga_cmd_vel_publisher.py`](examples/amiga_cmd_vel_publisher.py): Publishes a `TwistStamped` message on the `/amiga/cmd_vel` topic.
- [`amiga_vel_subscriber.py`](examples/amiga_vel_subscriber.py): Subscribes to the `/amiga/vel` topic and prints the received `TwistStamped` messages.

### Rust

- [`amiga_cmd_vel_publisher.rs`](examples/amiga_cmd_vel_publisher.rs): Publishes a `TwistStamped` message on the `/amiga/cmd_vel` topic.
- [`amiga_vel_subscriber.rs`](examples/amiga_vel_subscriber.rs): Subscribes to the `/amiga/vel` topic and prints the received `TwistStamped` messages.
- [`amiga-joystick.rs`](examples/amiga-joystick.rs): A simple joystick to command velocities to the Amiga and print received velocity states.

## Run the mocked bridge

### Terminal 1

Start ROS core:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### Terminal 2

***In a new terminal***, start the Amiga mock server:

```bash
source /opt/ros/noetic/setup.bash
cargo run --example amiga-mock-server
```

This is a stand-in for the actual Amiga gRPC canbus service that echos back the received commands.

### Terminal 3

***In a new terminal***, start the ROS bridge:

> Warning: This will crash immediately if there is no Amiga gRPC (or mock) service to connect to.

```bash
source /opt/ros/noetic/setup.bash
cargo run
```

This will register the bridge to the Vehicle `Twist` stream, forward these as gRPC messages, and republish them as ROS `TwistStamped` messages.

### Terminal 4 (+ 5)

This is where you would launch your ROS nodes for controlling the Amiga and listening to the measured state of the Amiga.
You can also try the following stand-in tools:

- [Experimental Amiga Joystick](#experimental-amiga-joystick)
- [Stable ROS packages](#stable-ros-packages)

## Do you want to know more?

Inspect [`ci_test.sh`](ci_test.sh) for a full example of how to run the bridge in a CI environment.
