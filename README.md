# How to test the amiga-ROS bridge in simulation

***This is currently without Amiga HW in the loop***


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

### Clone this repository

Clone the repository and initialize the submodule.

```bash
git clone https://github.com/farm-ng/amiga-ros-bridge.git
git submodule update --init
```

### Install dependencies

- [ROS Noetic install instructions](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Rust install instructions](https://www.rust-lang.org/learn/get-started).

## Steps to follow

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

#### Experimental Amiga Joystick

You can start the **experimental** amiga-joystick app to command velocities
to the Amiga (e.g. the mock server) and print received velocity states:

> Warning: This is not stable and may require multiple attempts at launching for this to come up.

```bash
source /opt/ros/noetic/setup.bash
cargo run --example amiga-joystick
```

Use arrow keys (left, right, up, down) to command velocities to the Amiga.

#### Stable ROS packages

You can publish `Twist` commands to the ROS bridge on the `/amiga/cmd_vel` topic with the [`rqt_robot_steering`](http://wiki.ros.org/rqt_robot_steering) package.

```bash
rosrun rqt_robot_steering rqt_robot_steering
# Then change the topic to `/amiga/cmd_vel`
```

You can subscribe to measured `TwistStamped` states of the amiga with ROS command line tools.

> Warning: The current implementation of ROS bridge does not stream the measured state on `/amiga/vel` if you are not sending commands on the `/amiga/cmd_vel` topic.
> The `rqt_robot_steering` does not send commands when the linear & angular velocities are both zeroed out, so the measured state stream will pause if they are both zero.

```bash
rostopic echo /amiga/vel
```

## Do you want to know more?

Inspect ci_test.sh.
