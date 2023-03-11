# How to test the amiga-ROS bridge in simulation

*without Amiga HW in the loop*

## Prerequisite

Install ROS1 (e.g noetic) and rust
(https://www.rust-lang.org/learn/get-started).


## Steps to follow:

Start ROS core:

```
roslaunch test_launch.launch
```

***In a new terminal***, start the Amiga mock server (this is a stand-in for
the actual Amiga gRPC canbus service or Amiga Simple Sim to be released):

```
source /opt/ros/noetic/setup.bash
cargo run --example amiga-mock-server
```


***In a new terminal***, start the ROS bridge:

```
source /opt/ros/noetic/setup.bash
cargo run
```

This will register the bridge to the Vehicle Twist steam and forward theses
gRPC messages and republish them as ROS TwistStamped messages.


***In a new terminal***, start the amiga-joystick app to command velocities
to the Amiga (e.g. the mock server) and print received velocity states:

```
cargo run --example amiga-joystick
```

Use arrow keys (left, right, up, down) to command velocities to the Amiga.


## Do you want to know more?

Inspect ci_test.sh.
