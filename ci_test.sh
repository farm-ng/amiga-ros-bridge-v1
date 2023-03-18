#!/bin/bash
cargo build
roslaunch launch/test_launch.launch &
sleep 2
cargo run --example amiga-mock-server -- --port 50050 &
cargo run -- --test-mode --port 50050 &
cargo run --example test_amiga_vel_publisher_subscriber
echo DONE
