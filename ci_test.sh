#!/bin/bash
set -e
set -x

# TODO: Reinstate the ci test
# https://github.com/farm-ng/amiga-ros-bridge/issues/19

cargo build
cargo run --example amiga-mock-server -- --port 50050 &
sleep 6
roslaunch launch/amiga_ros_bridge.launch port:=50050 host:=[::1] &
sleep 10
cargo run --example test_amiga_vel_pubsub
echo DONE
