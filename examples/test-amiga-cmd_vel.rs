fn main() {
    // Initialize node
    rosrust::init("amiga_cmd_vel_test");

    // Create publisher
    let cmd_vel_pub = rosrust::publish("/amiga/cmd_vel", 2).unwrap();

    // Create object that maintains 20Hz between sleep requests
    let rate = rosrust::rate(20.0);

    let mut count = 0;

    // Breaks when a shutdown signal is sent
    while rosrust::is_ok() {
        // Create string message
        let msg = rosrust_msg::geometry_msgs::Twist {
            linear: rosrust_msg::geometry_msgs::Vector3 {
                x: count as f64 * 0.001,
                y: 0.0,
                z: 0.0,
            },
            angular: rosrust_msg::geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        };

        // Send string message to topic via publisher
        cmd_vel_pub.send(msg).unwrap();

        // Sleep to maintain 20Hz rate
        rate.sleep();
        count += 1;

        if count > 10 {
            break;
        }
    }
}
