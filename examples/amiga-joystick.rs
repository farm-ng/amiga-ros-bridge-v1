#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
use eframe::egui;
use egui::*;
use tracing::{debug, info, Level};
use tracing_subscriber::FmtSubscriber;

const MAX_LINEAR_VEL_MPS: f64 = 0.5;
const MAX_ANGULAR_VEL_RPS: f64 = 0.5;

#[derive(Default)]
struct Shared {
    joystick_angular_accel: f64,
    joystick_linear_x_accel: f64,

    cmd_angular_vel: f64,
    cmd_linear_x_vel: f64,
    state_angular_vel: f64,
    state_linear_x_vel: f64,

    should_exit: bool,
}

#[derive(Default)]
struct Content {
    text: String,
    // Arc<Mutex> is a common pattern in rust to have shared state across threads. Using channels
    // is another options.
    shared: std::sync::Arc<std::sync::Mutex<Shared>>,
}

impl eframe::App for Content {
    fn on_close_event(&mut self) -> bool {
        {
            let mut state = self.shared.lock().unwrap();
            state.should_exit = true;
        }
        true
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ScrollArea::vertical()
                .auto_shrink([false; 2])
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    ui.label(&self.text);
                });

            // on left/right arrow keys
            let mut target_angular_vel = 0.0;
            if ctx.input(|i| i.key_down(Key::ArrowLeft)) {
                target_angular_vel = MAX_ANGULAR_VEL_RPS;
            } else if ctx.input(|i| i.key_down(Key::ArrowRight)) {
                target_angular_vel = -MAX_ANGULAR_VEL_RPS;
            }

            // on up/down arrow keys
            let mut target_linear_x_vel = 0.0;
            if ctx.input(|i| i.key_down(Key::ArrowUp)) {
                target_linear_x_vel = MAX_LINEAR_VEL_MPS;
            } else if ctx.input(|i| i.key_down(Key::ArrowDown)) {
                target_linear_x_vel = -MAX_LINEAR_VEL_MPS;
            }

            {
                // Acquire lock of mutex. This can fail in theory, but should not in this program,
                // unless there is a bug. Hence the unwrap is fine here.
                let mut state = self.shared.lock().unwrap();

                // We are mimicking analogue joystick behavior here. On arrow key downs
                // velocities will be ramped up smoothly using state.joystick_linear_x_accel and
                // state.joystick_angular_accel. The actual command velocities are calculated in
                // the /amiga/cmd_vel publisher thread.
                if (state.cmd_linear_x_vel - target_linear_x_vel).abs() < 0.01 {
                    state.joystick_linear_x_accel = 0.0;
                } else if state.cmd_linear_x_vel < target_linear_x_vel {
                    state.joystick_linear_x_accel = 1.0;
                } else {
                    state.joystick_linear_x_accel = -1.0;
                }

                if (state.cmd_angular_vel - target_angular_vel).abs() < 0.01 {
                    state.joystick_angular_accel = 0.0;
                } else if state.cmd_angular_vel < target_angular_vel {
                    state.joystick_angular_accel = 1.0;
                } else {
                    state.joystick_angular_accel = -1.0;
                }
                self.text = format!(
                    "CMD: LinVel: {} Angular: {}\nState: LinVel: {}, Angular: {}",
                    state.cmd_linear_x_vel,
                    state.cmd_angular_vel,
                    state.state_linear_x_vel,
                    state.state_angular_vel
                );
            }
            ui.ctx().request_repaint(); // make sure we note the holding.
        });
    }
}

fn main() {
    // setting console log level
    let subscriber = FmtSubscriber::builder()
        .with_max_level(Level::DEBUG)
        .finish();
    tracing::subscriber::set_global_default(subscriber).expect("setting default subscriber failed");

    let content = Content::default();

    // We need to clone the Arc (Atomically Reference Counted == rust's shared_ptr) here, so it can
    // be moved into the thread below.
    let content_state = content.shared.clone();

    debug!("Starting /amiga/cmd_vel publisher");
    let handler = std::thread::spawn(move || {
        rosrust::init("amiga_joystick");
        let cmd_vel_pub = rosrust::publish("/amiga/cmd_vel", 2).unwrap();

        let rate_hz = 20.0;
        let delta_t = 1.0 / rate_hz;
        let rate = rosrust::rate(rate_hz);

        loop {
            {
                // Acquire lock of mutex. This can fail in theory, but should not in this program,
                // unless there is a bug. Hence the unwrap is fine here.
                let mut state = content_state.lock().unwrap();

                // Calculating command velocities given virtual joystick accelerations.
                state.cmd_linear_x_vel += delta_t * state.joystick_linear_x_accel;
                state.cmd_linear_x_vel = state
                    .cmd_linear_x_vel
                    .clamp(-MAX_LINEAR_VEL_MPS, MAX_LINEAR_VEL_MPS);
                state.cmd_angular_vel += delta_t * state.joystick_angular_accel;
                state.cmd_angular_vel = state
                    .cmd_angular_vel
                    .clamp(-MAX_ANGULAR_VEL_RPS, MAX_ANGULAR_VEL_RPS);

                let msg = rosrust_msg::geometry_msgs::Twist {
                    linear: rosrust_msg::geometry_msgs::Vector3 {
                        x: state.cmd_linear_x_vel,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: rosrust_msg::geometry_msgs::Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: state.cmd_angular_vel,
                    },
                };

                debug!("Publish twist message on /amiga/cmd_vel topic.");
                cmd_vel_pub.send(msg).unwrap();
                if state.should_exit {
                    break;
                }
            }
            // Sleep to maintain rate_hz
            rate.sleep();
        }
    });
    info!("/amiga/cmd_vel publisher started.");

    // We need to clone shared so we can move it into the subscriber thread.
    let content_state2 = content.shared.clone();

    debug!("Starting /amiga/vel subscriber...");
    let _subscriber = rosrust::subscribe(
        "amiga/vel",
        100,
        move |v: rosrust_msg::geometry_msgs::TwistStamped| {
            let mut data = content_state2.lock().unwrap();
            data.state_linear_x_vel = v.twist.linear.x;
            data.state_angular_vel = v.twist.angular.z;
        },
    )
    .unwrap();
    info!("/amiga/vel subscriber started.");

    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "Amiga Virtual Joystick (ROS1)",
        options,
        Box::new(|_cc| Box::new(content)),
    )
    .unwrap();
    handler.join().unwrap();
}
