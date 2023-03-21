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

Using the ROS bridge with the amiga requires an Amiga OS `>= v1.2.0`.

The ROS bridge allows you to publish [`Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) commands on the `/amiga/cmd_vel` topic to drive the Amiga and subscribe to [`TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html) measured rates on the  `/amiga/vel` topic.

### Usage options

There are currently 3 methods for using the `amiga_ros_bridge`:

1. Running the `amiga_ros_bridge` (and ROS master) on the Amiga, with other ROS nodes on your PC connected to the Amiga ROS master
    - Look for sections tagged as **`[Method 1]`**
   <!-- - See [ROS bridge on the Amiga](#ros-bridge-on-the-amiga) for instructions -->
2. Running the `amiga_ros_bridge` (and ROS master) on your PC, connected to the Amiga canbus service over gRPC
   - Look for sections tagged as **`[Method 2]`**
   <!-- - See [ROS bridge on your PC](#ros-bridge-on-your-pc) for instructions -->
3. Running the `amiga_ros_bridge` (and ROS master) on your PC, using the mock server
   <!-- - See [ROS bridge with mock server](#ros-bridge-with-mock-server) for instructions -->
    - Look for sections tagged as **`[Method 3]`**

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

### Gain access to the ROS bridge container

For: **`[Method 1]`**

If you would like run the `amiga_ros_bridge` on the Amiga,
you will need to gain access to the ROS bridge docker container.

Please contact `info@farm-ng.com` for instructions and credentials.

### Install dependencies

For: **`[Method 2]`**, **`[Method 3]`**

To run the `amiga_ros_bridge` on your PC, you will need:

- [ROS Noetic install instructions](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Rust install instructions](https://www.rust-lang.org/learn/get-started).
  - Rust is only needed if you will run the `amiga_ros_bridge` or any Rust examples on your PC.

> NOTE: Using ROS on your PC means you're either running some flavor of Linux or know how to run ROS in a VM on your other OS.

### `amiga_ros_bridge` installation

For: **`[Method 1]`**, **`[Method 2]`**, **`[Method 3]`**

> We hope to soon package a pre-compiled `amiga_ros_bridge` in the ros bridge docker container
> installed on your Amiga for running with  **`[Method 1]`**.
> In the meantime, you will need to manually install the `amiga_ros_bridge` as you would on your PC.

#### Catkin workspace

Create a catkin workspace, if you don't already have one, following the instructions at
[ROS Tutorials - create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
If this is a new project, it is recommended to create a new, separate catkin workspace.

> The remaining instructions will assume you're using a catkin ws at `~/catkin_ws`.

#### Clone this repository

Clone the repository and initialize the `farm-ng-amiga` submodule.

```bash
cd ~/catkin_ws/src
git clone https://github.com/farm-ng/amiga-ros-bridge.git
cd amiga-ros-bridge
git submodule update --init
cd ~/catkin_ws
```

#### Source your catkin workspace

Go to your ROS workspace and source the setup.bash file:

```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
```

#### Build the ROS bridge executables

> Warning: this may take ~12 minutes on the first install on the Amiga brain.
> It should be much faster on your PC.

```bash
cd ~/catkin_ws
cargo build --manifest-path=src/amiga-ros-bridge/Cargo.toml
# Same as: cd ~/catkin_ws/src/amiga-ros-bridge && cargo build && cd ~/catkin_ws
```

#### Compile the package `amiga_ros_bridge`

> Warning: this may take ~12 minutes on the first install on the Amiga brain.
> It should be much faster on your PC.

```bash
cd ~/catkin_ws
catkin_make clean # optionally, clean the workspace
catkin_make
```

## Run the `amiga_ros_bridge`

### Specifying the ROS Master

For: **`[Method 1]`**

> NOTE: These instructions cover connecting multiple ROS devices to a single ROS master.
> This is only relevant if you are running the `amiga_ros_bridge` on the Amiga,
> and connecting other ROS nodes running on your PC to the `amiga_ros_bridge`.

In order to connect to the same ROS Master when running ROS nodes on multiple devices,
you must specify all nodes to be on the same `ROS_MASTER_URI`, but with their own `ROS_IP`.

If you want to connect to the `roscore` and `amiga_ros_bridge` running on your Amiga,
you must export the `ROS_MASTER_URI` and `ROS_IP` before starting a `roscore`
and/or before launching or running any node that will connect to that ROS Master.
This would be:

```bash
export ROS_MASTER_URI=http://<AMIGA_IP>:11311
export ROS_IP=<AMIGA_IP>
```

Where `<AMIGA_IP>` corresponds to the IP address seen on the bottom right hand corner of your Amiga brain home screen.
So this command would become, e.g.:

```bash
export ROS_MASTER_URI=http://192.168.1.98:11311
export ROS_IP=192.168.1.98
```

Meanwhile, your dev station should be set up to connect to that ROS master.

```bash
export ROS_MASTER_URI=http://<AMIGA_IP>:11311
export ROS_IP=<DEV_STATION_IP>
```

Where `<DEV_STATION_IP>` corresponds to the IP address on your PC running ROS, connected to the same network as the Amiga.

> Hint: `<DEV_STATION_IP>` can be found with `ifconfig` or other basic tools.

So this command would become, e.g.:

```bash
export ROS_MASTER_URI=http://192.168.1.98:11311
export ROS_IP=192.168.1.123
```

For more information see:

- [Running ROS across multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
- [Specify `ROS_IP` for running on multiple machines](https://answers.ros.org/question/349095/ros-on-multiple-machine-not-working-properly/?answer=398784#post-id-398784)

### ROS bridge on the Amiga

For: **`[Method 1]`**

The first method is with the `amiga_ros_bridge` running on the Amiga,
and you connecting other ROS nodes running on your computer to the Amiga's
ROS Master.

Make sure that the Amiga is connected to the same network as your computer
and that you know both the IP address of the Amiga and of your PC.

Launch the `amiga_ros_bridge` on the Amiga by `ssh`-ing into the ros bridge docker container on the Amiga.
Please contact `info@farm-ng.com` for instructions and credentials.

```bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://<AMIGA_IP>:11311
export ROS_IP=<AMIGA_IP>
roslaunch amiga_ros_bridge amiga_ros_bridge.launch
```

Where `<AMIGA_IP>` corresponds to the IP address seen on the bottom right hand corner of your Amiga brain home screen.

### ROS bridge on your PC

For: **`[Method 2]`**

Alternatively, you can run the package in your computer as follows.
Make sure that the Amiga is connected to the same network as your computer
and that you know the IP address of the Amiga.

```bash
roslaunch amiga_ros_bridge amiga_ros_bridge.launch host:=<AMIGA_IP>
```

Where `<AMIGA_IP>` corresponds to the IP address seen on the bottom right hand corner of your Amiga brain home screen.

### ROS bridge with mock server

For: **`[Method 3]`**

Finally, you can run the package in your computer with a mock Amiga canbus server that just echos velocity commands back.
To run this, you must first start the mock server in a separate terminal:

#### Start the mock server

> Warning: This is experimental and is not guaranteed to work.

This is a stand-in for the actual Amiga gRPC canbus service that echos back the received commands.

***In a separate terminal***, start the Amiga mock server:

```bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/amiga-ros-bridge/
cargo run --example amiga-mock-server
```

#### Start the ros bridge

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch amiga_ros_bridge amiga_ros_bridge.launch
```

## Test the `amiga_ros_bridge`

Now you will be able to connect to the ROS topics used by the `amiga_ros_bridge`.
Confirm this in another terminal on your PC with:

```bash
source ~/catkin_ws/devel/setup.bash
rostopic list
```

> NOTE: If running the `amiga_ros_bridge` on the Amiga (as in **`[Method 1]`**) and connecting with ROS over the network,
make sure to follow the [Specifying the ROS Master](#specifying-the-ros-master) instructions.
> Remember to set your `ROS_MASTER_URI` & `ROS_IP` before running. E.g.,
>
> ```bash
> source ~/catkin_ws/devel/setup.bash
> export ROS_MASTER_URI=http://<AMIGA_IP>:11311
> export ROS_IP=<DEV_STATION_IP>
> rostopic list
> ```

And you should see:

```bash
/amiga/cmd_vel
/amiga/vel
/rosout
/rosout_agg
```

## Control the Amiga

> NOTE: If controlling the robot with a ROS connection over the network,
make sure to follow the [Specifying the ROS Master](#specifying-the-ros-master) instructions.

### Experimental Amiga Joystick

You can start the **experimental** amiga-joystick app to command velocities
to the Amiga (e.g. the mock server) and print received velocity states:

> Warning: This is not stable and may require multiple attempts at launching for this to come up.

```bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/amiga-ros-bridge/
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
source ~/catkin_ws/devel/setup.bash
rosrun rqt_robot_steering rqt_robot_steering
# Then change the topic to `/amiga/cmd_vel`
```

You can subscribe to measured `TwistStamped` states of the amiga with ROS command line tools.

```bash
source ~/catkin_ws/devel/setup.bash
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


## Do you want to know more?

Inspect [`ci_test.sh`](ci_test.sh) for a full example of how to run the bridge in a CI environment.
