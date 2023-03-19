#!/bin/bash
set -e
set -x

cargo build
roslaunch launch/test_launch.launch &
sleep 2
cargo run --example amiga-mock-server -- --port 50050 &
sleep 2
cargo run -- --port 50050 &
sleep 2
cargo run --example test_amiga_vel_pubsub
echo DONE
