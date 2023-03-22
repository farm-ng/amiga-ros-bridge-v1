#!/bin/bash
set -e
set -x

cargo build
cargo run --example amiga-mock-server -- --port 50050 &
sleep 2
roslaunch amiga_ros_bridge amiga_ros_bridge.launch port:=50050 host:=[::1] &
sleep 2
cargo run --example test_amiga_vel_pubsub
echo DONE
