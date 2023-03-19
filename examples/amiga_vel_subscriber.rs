use tracing::{debug, info, Level};
use tracing_subscriber::FmtSubscriber;

fn main() {
    tracing::subscriber::set_global_default(
        FmtSubscriber::builder()
            .with_max_level(Level::INFO)
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

    while rosrust::is_ok() {
        // Sleep to maintain 20Hz rate
        rate.sleep();

        if *arc_count.lock().unwrap() > 10 {
            break;
        }
    }
}
