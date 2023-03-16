use futures::Stream;
use ros_bridge::grpc::farm_ng::canbus::proto::canbus_service_server::{
    CanbusService, CanbusServiceServer,
};
use ros_bridge::grpc::farm_ng::canbus::proto::{
    SendCanbusMessageReply, SendCanbusMessageRequest, SendVehicleTwistCommandReply,
    SendVehicleTwistCommandRequest, StreamCanbusReply, StreamCanbusRequest, StreamMotorStatesReply,
    StreamMotorStatesRequest, StreamVehicleTwistStateReply, StreamVehicleTwistStateRequest,
    Twist2d,
};
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;
use tokio_stream::{wrappers::ReceiverStream, StreamExt};
use tonic::{transport::Server, Response, Status, Streaming};
use tracing::{debug, info, Level};
use tracing_subscriber::FmtSubscriber;

type StreamResult<T> = Result<Response<T>, Status>;
type TwistResponseStream =
    Pin<Box<dyn Stream<Item = Result<SendVehicleTwistCommandReply, Status>> + Send>>;
type MotorResponseStream =
    Pin<Box<dyn Stream<Item = Result<StreamMotorStatesReply, Status>> + Send>>;
type SendCanbusMessageStream =
    Pin<Box<dyn Stream<Item = Result<SendCanbusMessageReply, Status>> + Send>>;
type CanbusResponseStream = Pin<Box<dyn Stream<Item = Result<StreamCanbusReply, Status>> + Send>>;
type VehicleTwistStream =
    Pin<Box<dyn Stream<Item = Result<StreamVehicleTwistStateReply, Status>> + Send>>;

#[derive(Debug, Clone)]
pub struct AmigaMockService {
    twist_state: Arc<Mutex<Twist2d>>,
}

#[tonic::async_trait]
impl CanbusService for AmigaMockService {
    type streamCanbusMessagesStream = CanbusResponseStream;
    type sendCanbusMessageStream = SendCanbusMessageStream;
    type streamMotorStatesStream = MotorResponseStream;
    type sendVehicleTwistCommandStream = TwistResponseStream;
    type streamVehicleTwistStateStream = VehicleTwistStream;

    async fn stream_canbus_messages(
        &self,
        _: tonic::Request<StreamCanbusRequest>,
    ) -> Result<tonic::Response<Self::streamCanbusMessagesStream>, tonic::Status> {
        todo!()
    }

    async fn send_canbus_message(
        &self,
        _: tonic::Request<Streaming<SendCanbusMessageRequest>>,
    ) -> Result<tonic::Response<Self::sendCanbusMessageStream>, tonic::Status> {
        todo!()
    }

    async fn stream_motor_states(
        &self,
        _: tonic::Request<StreamMotorStatesRequest>,
    ) -> Result<tonic::Response<Self::streamMotorStatesStream>, tonic::Status> {
        todo!()
    }

    async fn send_vehicle_twist_command(
        &self,
        request: tonic::Request<Streaming<SendVehicleTwistCommandRequest>>,
    ) -> StreamResult<Self::sendVehicleTwistCommandStream> {
        info!("AmigaMockServer started.");

        let mut in_stream = request.into_inner();
        let (tx, rx): (
            tokio::sync::mpsc::Sender<Result<_, Status>>,
            tokio::sync::mpsc::Receiver<Result<_, Status>>,
        ) = mpsc::channel(128);

        let state_alias = self.twist_state.clone();

        tokio::spawn(async move {
            while let Some(item) = in_stream.next().await {
                match item {
                    Ok(v) => {
                        let state = v.command.clone().unwrap();
                        debug!("state received: {:?}", state);

                        let twist = Twist2d {
                            linear_velocity_x: state.linear_velocity_x,
                            linear_velocity_y: state.linear_velocity_y,
                            angular_velocity: state.angular_velocity,
                        };

                        {
                            let mut s = state_alias.lock().unwrap();
                            *s = twist.clone();
                        }
                        tx.send(Ok(SendVehicleTwistCommandReply {
                            success: true,
                            stamp: tokio::time::Instant::now().elapsed().as_secs_f64(),
                            state: Some(twist),
                        }))
                        .await
                        .expect("working rx")
                    }
                    Err(item) => {
                        info!("error: {:?}", item);
                        break;
                    }
                }
            }
        });

        let output_stream = ReceiverStream::new(rx);
        Ok(Response::new(
            Box::pin(output_stream) as Self::sendVehicleTwistCommandStream
        ))
    }

    async fn stream_vehicle_twist_state(
        &self,
        _: tonic::Request<StreamVehicleTwistStateRequest>,
    ) -> Result<tonic::Response<Self::streamVehicleTwistStateStream>, tonic::Status> {
        let state_alias = self.twist_state.clone();

        let (tx, rx): (
            tokio::sync::mpsc::Sender<Result<StreamVehicleTwistStateReply, Status>>,
            tokio::sync::mpsc::Receiver<Result<StreamVehicleTwistStateReply, Status>>,
        ) = mpsc::channel(128);
        tokio::spawn(async move {
            loop {
                debug!("streaming states");

                let s: Twist2d;
                {
                    s = state_alias.lock().unwrap().clone();
                }
                tx.send(Ok(StreamVehicleTwistStateReply {
                    stamp: tokio::time::Instant::now().elapsed().as_secs_f64(),
                    state: Some(s),
                }))
                .await
                .unwrap();
            }
        });

        let output_stream = ReceiverStream::new(rx);
        Ok(Response::new(
            Box::pin(output_stream) as Self::streamVehicleTwistStateStream
        ))
    }
}

use clap::Parser;

#[derive(Parser, Debug)]
struct Args {
    #[arg(short, long, default_value_t = 50060)]
    port: u32,
    #[arg(short, long, default_value_t = false)]
    test_mode: bool,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let subscriber = FmtSubscriber::builder()
        .with_max_level(Level::DEBUG)
        .finish();
    tracing::subscriber::set_global_default(subscriber).expect("setting default subscriber failed");

    let args: Args = Args::parse();
    let port: u32 = args.port;
    let address: String = format!("[::1]:{port}");

    let server = AmigaMockService {
        twist_state: Arc::new(Mutex::new(Twist2d::default())),
    };
    Server::builder()
        .add_service(CanbusServiceServer::new(server))
        .serve(address.parse()?)
        .await?;
    Ok(())
}
