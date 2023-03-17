#!/bin/bash
cargo build
roslaunch launch/test_launch.launch &
sleep 2
cargo run --example amiga-mock-server &
cargo run --example amiga_cmd_vel_publisher &
cargo run -- --test-mode &
cargo run --example amiga_vel_subscriber
echo DONE
