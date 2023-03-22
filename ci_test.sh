#!/bin/bash
set -e
set -x

cargo build
cargo run --example amiga-mock-server -- --port 50060 &
sleep 2
roslaunch launch/amiga_ros_bridge.launch host:=[::1] &
sleep 2
cargo run --example test_amiga_vel_pubsub
echo DONE
