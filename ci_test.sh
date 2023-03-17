#!/bin/bash
cargo build
roslaunch launch/test_launch.launch &
sleep 2
cargo run --example amiga-mock-server &
cargo run --example test-amiga-cmd_vel &
cargo run -- --test-mode &
cargo run --example test-amiga-vel
echo DONE
