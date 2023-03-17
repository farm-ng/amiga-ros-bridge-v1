#!/bin/bash
cargo build
roslaunch launch/test_launch.launch &
sleep 2
cargo run --example amiga-mock-server -- --port 50050 &
cargo run --example amiga_cmd_vel_publisher &
cargo run -- --test-mode --port 50050 &
cargo run --example amiga_vel_subscriber
echo DONE
