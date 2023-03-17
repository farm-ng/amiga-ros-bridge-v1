use clap::Parser;
use futures::stream::Stream;
use futures::StreamExt;
use amiga_ros_bridge::grpc::farm_ng::canbus::proto::canbus_service_client::CanbusServiceClient;
use amiga_ros_bridge::grpc::farm_ng::canbus::proto::{
    SendVehicleTwistCommandRequest, StreamVehicleTwistStateRequest, Twist2d,
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

// convert StreamVehicleTwistStateReply to TwistStamped
fn twist_stamped_from_state_reply(
    count: u32,
    stamp: f64,
    state: Twist2d,
) -> rosrust_msg::geometry_msgs::TwistStamped {
    rosrust_msg::geometry_msgs::TwistStamped {
        header: rosrust_msg::std_msgs::Header {
            seq: count,
            stamp: Time {
                sec: stamp as u32,
                nsec: (stamp.fract() * 1.0e9) as u32,
            },
            frame_id: "amiga".to_owned(),
        },
        twist: rosrust_msg::geometry_msgs::Twist {
            linear: rosrust_msg::geometry_msgs::Vector3 {
                x: state.linear_velocity_x as f64,
                y: state.linear_velocity_y as f64,
                z: 0.0,
            },
            angular: rosrust_msg::geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: state.angular_velocity as f64,
            },
        },
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

    // async way to forward geometry_msgs::Twist to StreamVehicleTwistStateRequest
    async fn streams(
        &mut self,
        rx: tokio::sync::mpsc::Receiver<Result<rosrust_msg::geometry_msgs::Twist, Status>>,
        is_test_mode: bool,
    ) {
        debug!("Attempting to connect to stream_vehicle_twist_state");

        let mut stream = self
            .client
            .stream_vehicle_twist_state(StreamVehicleTwistStateRequest {})
            .await
            .unwrap()
            .into_inner();
        debug!("stream_vehicle_twist_state connected");

        let handler = tokio::spawn(async move {
            let state_pub = rosrust::publish("/amiga/vel", 100).unwrap();
            let mut count = 0; // message counter

            // iterate over the stream
            while let Some(maybe_reply) = stream.next().await {
                // println!("\treceived: {:?}", item.unwrap());
                let reply = maybe_reply.unwrap();
                let twist_state: Twist2d = reply.state.unwrap();

                // convert to ROS TwistStamped
                let msg = twist_stamped_from_state_reply(count, reply.stamp, twist_state);

                // republish messages to ROS
                state_pub.send(msg).unwrap();
                count += 1; // increment message counter
            }
        });

        debug!("send_vehicle_twist_command connected");

        let stream = self
            .client
            .send_vehicle_twist_command(RosToGrpcStream { rx });
        let mut stream = stream.await.unwrap().into_inner();
        debug!("send_vehicle_twist_command connected");

        let mut count = 0;

        loop {
            // forward the stream to the server and do nothing with the reply
            let _ = stream.next().await;

            count += 1;
            if is_test_mode && count > 10 {
                info!("Test mode: shutting down");
                break;
            }
        }

        handler.await.unwrap();
    }

    // stream is dropped here and the disconnect info is send to server
}

#[derive(Parser, Debug)]
struct Args {
    #[arg(short = 'H', long = "host", default_value_t = String::from("localhost"))]
    host: String,
    #[arg(short, long, default_value_t = 50060)]
    port: u32,
    #[arg(short, long, default_value_t = false)]
    test_mode: bool,
}

#[tokio::main]
async fn main() {
    // setting console log level
    let subscriber = FmtSubscriber::builder()
        .with_max_level(Level::DEBUG)
        .finish();
    tracing::subscriber::set_global_default(subscriber).expect("setting default subscriber failed");

    // parse arguments using the popular clap crate.
    let args: Args = Args::parse();
    let host: String = args.host;
    let port: u32 = args.port;
    let is_test_mode: bool = args.test_mode;

    if is_test_mode {
        info!("Test mode: starting up");
    }

    // launching the tokio runtime
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

    // set the address of the gRPC server
    let address: String = format!("http://{host}:{port}");
    debug!("Connecting to gRPC server at {}", address);

    // the gRPC client takes the receiver rx.
    let _ = runtime.spawn(async move {
        AmgigRosBridgeGrpcClient::connect(address)
            .await
            .unwrap()
            .streams(rx, args.test_mode)
            .await
    });
    info!("Connected to gRPC server.");

    // connect to ROS
    // TODO: add a check to see if roscore is running, if not, exit.
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

    // runtime.block_on(handle).unwrap();

    // NOTE: this will shutdown the runtime when the user presses Ctrl-C
    match tokio::signal::ctrl_c().await {
        Ok(_) => {
            info!("Ctrl-C received, shutting down");
        }
        Err(e) => {
            eprintln!("Error: {}", e);
        }
    }

}
