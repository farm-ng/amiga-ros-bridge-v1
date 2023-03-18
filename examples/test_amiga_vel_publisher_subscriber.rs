use tracing::{debug, info, Level};
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

    let arc_count = std::sync::Arc::new(std::sync::Mutex::<u32>::new(0));
    let arc_count_clone = arc_count.clone();

    let _subscriber_raii = rosrust::subscribe(
        "/amiga/vel",
        100,
        move |v: rosrust_msg::geometry_msgs::TwistStamped| {
            debug!("Received: {:?}", v);
            let mut data = arc_count_clone.lock().unwrap();
            *data += 1;
        },
    )
    .unwrap();
    info!("amiga_vel_subscriber launched");
    let cmd_vel_pub = rosrust::publish("/amiga/cmd_vel", 2).unwrap();

    let mut pub_num = 0;
    while rosrust::is_ok() {
        if pub_num < 10 {
            // Create string message
            let msg = rosrust_msg::geometry_msgs::Twist {
                linear: rosrust_msg::geometry_msgs::Vector3 {
                    x: pub_num as f64 * 0.001,
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
            pub_num += 1;
        }

        // Sleep to maintain 20Hz rate
        rate.sleep();
        let c = *arc_count.lock().unwrap();
        info!("amiga_vel_subscriber got: {}", c);

        if *arc_count.lock().unwrap() > 5 {
            break;
        }
    }
    info!("amiga_vel_subscriber done");
}
