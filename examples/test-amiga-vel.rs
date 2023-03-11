use tracing::{debug, info, Level};
use tracing_subscriber::FmtSubscriber;
fn main() {
    let subscriber = FmtSubscriber::builder()
        .with_max_level(Level::DEBUG)
        .finish();
    tracing::subscriber::set_global_default(subscriber).expect("setting default subscriber failed");

    // Initialize node
    rosrust::init("amiga/vel listener");
    let arc_count = std::sync::Arc::new(std::sync::Mutex::<u32>::new(0));
    let arc_count_clone = arc_count.clone();
    let _subscriber_raii = rosrust::subscribe(
        "amiga/vel",
        100,
        move |v: rosrust_msg::geometry_msgs::TwistStamped| {
            debug!("Received: {:?}", v);
            let mut data = arc_count_clone.lock().unwrap();
            *data += 1;
        },
    )
    .unwrap();
    info!("/amiga/vel subscriber launched");
    let rate = rosrust::rate(0.0);

    // Block the thread until a count=10

    loop {
        rate.sleep();
        if *arc_count.lock().unwrap() == 10 {
            break;
        }
    }
}
