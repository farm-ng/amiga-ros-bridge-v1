use tracing::{info, Level};
use tracing_subscriber::FmtSubscriber;

fn main() {
    tracing::subscriber::set_global_default(
        FmtSubscriber::builder()
            .with_max_level(Level::DEBUG)
            .finish(),
    )
    .expect("setting default subscriber failed");

    // Initialize node
    rosrust::init("amiga_vel_subscriber");

    // Create object that maintains 20Hz between sleep requests
    let rate = rosrust::rate(20.0);

    let shared_state = std::sync::Arc::new(std::sync::Mutex::<
        rosrust_msg::geometry_msgs::TwistStamped,
    >::default());
    let shared_state_clone = shared_state.clone();

    let _subscriber_raii = rosrust::subscribe(
        "/amiga/vel",
        100,
        move |v: rosrust_msg::geometry_msgs::TwistStamped| {
            info!("Received: {:?}", v);
            let mut data = shared_state_clone.lock().unwrap();
            *data = v;
        },
    )
    .unwrap();
    info!("amiga_vel_subscriber launched");
    let cmd_vel_pub = rosrust::publish("/amiga/cmd_vel", 2).unwrap();

    let mut pub_num = 0;
    while rosrust::is_ok() {
        info!("HERE!!!");

        // Create string message
        let msg = rosrust_msg::geometry_msgs::Twist {
            linear: rosrust_msg::geometry_msgs::Vector3 {
                x: pub_num as f64 * 0.1,
                y: 0.0,
                z: 0.0,
            },
            angular: rosrust_msg::geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.5,
            },
        };

        // Send string message to topic via publisher
        cmd_vel_pub.send(msg).unwrap();
        pub_num += 1;

        // Sleep to maintain 20Hz rate
        rate.sleep();
        let state = shared_state.lock().unwrap().clone();
        info!("amiga_vel_subscriber got: {:?}", state);

        let x = state.twist.linear.x;
        let theta = state.twist.angular.z;

        if x >= 1.0 && (theta - 0.5).abs() < 0.001 {
            // We received twist state which achieved linear and angular vel as commanded.
            break;
        }
    }
    info!("amiga_vel_subscriber done");
}
