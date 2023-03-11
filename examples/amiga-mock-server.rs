use futures::Stream;
use ros_bridge::grpc::farm_ng::canbus::proto::canbus_service_server::{
    CanbusService, CanbusServiceServer,
};
use ros_bridge::grpc::farm_ng::canbus::proto::{
    SendCanbusMessageReply, SendCanbusMessageRequest, SendVehicleTwistCommandReply,
    SendVehicleTwistCommandRequest, StreamCanbusReply, StreamCanbusRequest, StreamMotorStatesReply,
    StreamMotorStatesRequest, Twist2d,
};
use std::pin::Pin;
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

#[derive(Debug, Clone)]
pub struct AmigaMockService {}

#[tonic::async_trait]
impl CanbusService for AmigaMockService {
    type streamCanbusMessagesStream = CanbusResponseStream;
    type sendCanbusMessageStream = SendCanbusMessageStream;
    type streamMotorStatesStream = MotorResponseStream;
    type sendVehicleTwistCommandStream = TwistResponseStream;

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

        tokio::spawn(async move {
            while let Some(item) = in_stream.next().await {
                match item {
                    Ok(v) => {
                        let state = v.command.clone().unwrap();
                        debug!("state received: {:?}", state);

                        tx.send(Ok(SendVehicleTwistCommandReply {
                            success: true,
                            stamp: 0.0,
                            state: Some(Twist2d {
                                linear_velocity_x: state.linear_velocity_x,
                                linear_velocity_y: state.linear_velocity_y,
                                angular_velocity: state.angular_velocity,
                            }),
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
}

use clap::Parser;

#[derive(Parser, Debug)]
struct Args {
    #[arg(short, long, default_value_t = 50070)]
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

    let args = Args::parse();
    let port = args.port;
    let address = format!("[::1]:{port}");

    let server = AmigaMockService {};
    Server::builder()
        .add_service(CanbusServiceServer::new(server))
        .serve(address.parse()?)
        .await?;
    Ok(())
}
