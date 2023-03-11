use clap::Parser;
use futures::stream::Stream;
use futures::StreamExt;
use ros_bridge::grpc::farm_ng::canbus::proto::canbus_service_client::CanbusServiceClient;
use ros_bridge::grpc::farm_ng::canbus::proto::{
    SendVehicleTwistCommandReply, SendVehicleTwistCommandRequest, Twist2d,
};
use rosrust::Time;
use tokio::sync::mpsc;
use tonic::Status;
use tracing::{debug, info, trace, Level};
use tracing_subscriber::FmtSubscriber;
type StdError = Box<dyn std::error::Error + Send + Sync + 'static>;
type Result<T, E = StdError> = ::std::result::Result<T, E>;

struct AmgigRosBridgeGrpcClient {
    client: CanbusServiceClient<tonic::transport::Channel>,
}

struct RosToGrpcStream {
    rx: tokio::sync::mpsc::Receiver<Result<rosrust_msg::geometry_msgs::Twist, Status>>,
}

// A stream is like a future but can be polled more than once.
impl Stream for RosToGrpcStream {
    type Item = SendVehicleTwistCommandRequest;

    // async way to forward geometry_msgs::Twist to SendVehicleTwistCommandRequest.
    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        // Maps x (geometry_msgs::Twist) to SendVehicleTwistCommandRequest, unless x is
        // Poll::Pending (stream not ready yet).
        self.rx.poll_recv(cx).map(|x| {
            let twist = x.unwrap().unwrap();
            Some(SendVehicleTwistCommandRequest {
                command: Some(Twist2d {
                    linear_velocity_x: twist.linear.x as f32,
                    linear_velocity_y: twist.linear.y as f32,
                    angular_velocity: twist.angular.z as f32,
                }),
            })
        })
    }
}

impl AmgigRosBridgeGrpcClient {
    async fn connect<D>(dst: D) -> Result<Self, tonic::transport::Error>
    where
        D: TryInto<tonic::transport::Endpoint>,
        D::Error: Into<StdError>,
    {
        let client = CanbusServiceClient::connect(dst).await?;
        Ok(Self { client })
    }

    async fn stream(
        &mut self,
        rx: tokio::sync::mpsc::Receiver<Result<rosrust_msg::geometry_msgs::Twist, Status>>,
        is_test_mode: bool,
    ) {
        let vel_pub = rosrust::publish("/amiga/vel", 100).unwrap();

        let mut count = 0;

        let stream = self
            .client
            .send_vehicle_twist_command(RosToGrpcStream { rx });

        let mut stream = stream.await.unwrap().into_inner();

        loop {
            let received = stream.next().await;
            let reply: SendVehicleTwistCommandReply = received.unwrap().unwrap();
            let states: Twist2d = reply.state.unwrap();

            // republish messages to ROS
            let msg = rosrust_msg::geometry_msgs::TwistStamped {
                header: rosrust_msg::std_msgs::Header {
                    seq: count,
                    stamp: Time {
                        sec: reply.stamp as u32,
                        nsec: (reply.stamp.fract() * 1.0e9) as u32,
                    },
                    frame_id: "amiga".to_owned(),
                },
                twist: rosrust_msg::geometry_msgs::Twist {
                    linear: rosrust_msg::geometry_msgs::Vector3 {
                        x: states.linear_velocity_x as f64,
                        y: states.linear_velocity_y as f64,
                        z: 0.0,
                    },
                    angular: rosrust_msg::geometry_msgs::Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: states.angular_velocity as f64,
                    },
                },
            };

            vel_pub.send(msg).unwrap();

            count += 1;
            if is_test_mode && count > 10 {
                info!("Test mode: shutting down");
                break;
            }
        }
    }
}

#[derive(Parser, Debug)]
struct Args {
    #[arg(short, long, default_value_t = 50070)]
    port: u32,
    #[arg(short, long, default_value_t = false)]
    test_mode: bool,
}

fn main() {
    // setting console log level
    let subscriber = FmtSubscriber::builder()
        .with_max_level(Level::DEBUG)
        .finish();
    tracing::subscriber::set_global_default(subscriber).expect("setting default subscriber failed");

    // parse arguments using the popular clap crate.
    let args = Args::parse();
    let port = args.port;
    let is_test_mode = args.test_mode;

    if is_test_mode {
        info!("Test mode: starting up");
    }

    // launching the tokio runtime
    let address: String = format!("http://[::1]:{port}");
    debug!("Starting up tokio runtime");
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(1)
        .enable_all()
        .build()
        .unwrap();

    // Async channels to pass messages across threads in an async tokio context.
    let (tx, rx): (
        tokio::sync::mpsc::Sender<Result<rosrust_msg::geometry_msgs::Twist, Status>>,
        tokio::sync::mpsc::Receiver<Result<rosrust_msg::geometry_msgs::Twist, Status>>,
    ) = mpsc::channel(128);

    debug!("Connecting to gRPC server at {}", address);
    // the gRPC client takes the receiver rx.
    let handle = runtime.spawn(async move {
        AmgigRosBridgeGrpcClient::connect(address)
            .await
            .unwrap()
            .stream(rx, is_test_mode)
            .await
    });
    info!("Connected to gRPC server.");

    debug!("Attempting to connect to roscore");
    rosrust::init("amiga_ros_bridge");
    info!("Connected to roscore");

    debug!("Start thread to subscribe to cmd_vel");
    // The ROS subscriber takes the sender tx so it can forward received ROS messages to the gRPC
    // client.
    let _cmd_vel_sub = rosrust::subscribe(
        "/amiga/cmd_vel",
        100,
        move |v: rosrust_msg::geometry_msgs::Twist| {
            debug!("/amiga/cmd_vel received: {:?}", v);
            // using "blocking_send" since this closure is not "async".
            tx.blocking_send(Ok(v)).unwrap();
            trace!("re-published to channel");
        },
    )
    .unwrap();
    debug!("Subscriber thread started to cmd_vel");

    runtime.block_on(handle).unwrap();
}
