use std::{path::Path, sync::mpsc::channel, thread::spawn};

use anyhow::Result;
use control_system::blocks::{
    math::Add,
    producers::Constant,
    siso::{Delay, PIDParams, PID},
};
use control_system::{
    io::{Input, Output},
    numeric::ode::{ODESolver, RungeKutta4},
    Block, ControlSystemParameters, ParameterStore, StepInfo, StepResult,
};
use control_system::{BlockIO, ControlSystemBuilder};
use drone::telemetry::{
    add_protoplotter, rpc::telemetry_connector_server::TelemetryConnectorServer, TelemetryService,
    TelemetryServiceBuilder,
};
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};
use tokio::sync::oneshot::Receiver;
use tonic::transport::Server;

#[derive(Serialize, Deserialize)]
struct CartParams {
    mass: f64,
    pos0: f64,
    vel0: f64,
}

#[derive(BlockIO)]
struct Cart {
    #[blockio(block_name)]
    name: String,

    #[blockio(input)]
    u_force: Input<f64>,

    #[blockio(output)]
    y_pos: Output<f64>,
    #[blockio(output)]
    y_vel: Output<f64>,
    #[blockio(output)]
    y_acc: Output<f64>,

    params: CartParams,
    state: Vector2<f64>,
}

impl Block for Cart {
    fn step(&mut self, k: StepInfo) -> control_system::Result<StepResult> {
        let acc = self.u_force.get() / self.params.mass;

        let odefun = |_, x: Vector2<f64>| Vector2::new(x[1], acc);

        // Propagate state
        self.state = RungeKutta4::solve(odefun, k.t, k.dt, self.state);

        // Assign outputs
        self.y_pos.set(self.state[0]);
        self.y_vel.set(self.state[1]);
        self.y_acc.set(acc);

        Ok(StepResult::Continue)
    }
}

impl Cart {
    fn new(params: CartParams) -> Self {
        Cart {
            name: "cart".to_string(),
            state: Vector2::new(params.pos0, params.vel0),
            params,
            u_force: Input::default(),
            y_pos: Output::default(),
            y_vel: Output::default(),
            y_acc: Output::default(),
        }
    }

    fn from_store(store: &mut ParameterStore, default_params: CartParams) -> Result<Self> {
        let params = store.get_block_params("cart", default_params)?;

        Ok(Self::new(params))
    }
}

fn main() -> Result<()> {
    let (run_end_sender, run_end_receiver) = tokio::sync::oneshot::channel();
    let (ts_builder_sender, ts_builder_receiver) = channel();

    let handle = spawn(|| -> Result<()> {
        run_control_system(ts_builder_sender, run_end_sender)?;
        Ok(())
    });


    let ts_builder = ts_builder_receiver.recv()?;

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_io()
        .build()?;

    let _ = rt.block_on(run_telemetry_server(run_end_receiver, ts_builder));

    handle.join().unwrap()?;
    Ok(())
}

async fn run_telemetry_server(run_end_receiver: Receiver<()>, ts_builder: TelemetryServiceBuilder) -> Result<()> {
    let addr: std::net::SocketAddr = "127.0.0.1:65400".parse().unwrap();
    let telem_service = ts_builder.build()?;

    println!("Telemetry Server listening on {}", addr);

    Server::builder()
        .add_service(TelemetryConnectorServer::new(telem_service))
        .serve(addr)
        .await?;

    Ok(())
}

fn run_control_system(
    ts_builder_sender: std::sync::mpsc::Sender<TelemetryServiceBuilder>,
    run_end_sender: tokio::sync::oneshot::Sender<()>,
) -> Result<()> {
    let mut store = ParameterStore::new(Path::new("cart.toml"), "cart")?;

    let mut builder = ControlSystemBuilder::default();

    // Cart
    builder.add_block(
        Cart::from_store(
            &mut store,
            CartParams {
                mass: 1.0,
                pos0: 0.0,
                vel0: 0.0,
            },
        )?,
        &[("u_force", "/force")],
        &[
            ("y_pos", "/cart/pos"),
            ("y_vel", "/cart/vel"),
            ("y_acc", "/cart/acc"),
        ],
    )?;

    // ** Inner loop **
    // Inputs:
    // - /ref/vel: Velocity reference
    // Outputs:
    // - /force: Commanded force on the cart

    builder.add_block(
        Delay::from_store("vel_delay", &mut store, [0.0].into())?,
        &[("u", "/cart/vel")],
        &[("y", "/cart/vel_delayed")],
    )?;

    builder.add_block(
        Add::<f64, 2>::new("vel_err", [1.0, -1.0].into()),
        &[("u1", "/ref/vel"), ("u2", "/cart/vel_delayed")],
        &[("y", "/err/vel")],
    )?;

    builder.add_block(
        PID::from_store(
            "pid_vel",
            &mut store,
            PIDParams {
                kp: 4.0,
                ..Default::default()
            },
        )?,
        &[("u", "/err/vel")],
        &[("y", "/force")],
    )?;

    // ** Outer Loop **
    // Inputs:
    // - /ref/pos: Position reference
    // Outputs:
    // - /ref/vel: Velocity reference

    builder.add_block(
        Delay::from_store("pos_delay", &mut store, [0.0].into())?,
        &[("u", "/cart/pos")],
        &[("y", "/cart/pos_delayed")],
    )?;

    builder.add_block(
        Add::<f64, 2>::new("pos_err", [1.0, -1.0].into()),
        &[("u1", "/ref/pos"), ("u2", "/cart/pos_delayed")],
        &[("y", "/err/pos")],
    )?;

    builder.add_block(
        PID::from_store(
            "pid_pos",
            &mut store,
            PIDParams {
                kp: 1.0,
                ..Default::default()
            },
        )?,
        &[("u", "/err/pos")],
        &[("y", "/ref/vel")],
    )?;

    // Position reference

    builder.add_block(
        Constant::from_store("pos_ref", &mut store, 15.0.into())?,
        &[],
        &[("y", "/ref/pos")],
    )?;

    // Plotters
    // let mut signals = PlotSignals::default();
    let mut ts_builder = TelemetryServiceBuilder::default();

    let channel_size = 100;
    add_protoplotter::<f64>("/cart/pos", channel_size, &mut builder, &mut ts_builder)?;
    add_protoplotter::<f64>("/cart/vel", channel_size, &mut builder, &mut ts_builder)?;
    add_protoplotter::<f64>("/cart/acc", channel_size, &mut builder, &mut ts_builder)?;
    add_protoplotter::<f64>("/force", channel_size, &mut builder, &mut ts_builder)?;
    add_protoplotter::<f64>("/ref/pos", channel_size, &mut builder, &mut ts_builder)?;
    add_protoplotter::<f64>("/ref/vel", channel_size, &mut builder, &mut ts_builder)?;
    add_protoplotter::<f64>("/err/pos", channel_size, &mut builder, &mut ts_builder)?;
    add_protoplotter::<f64>("/err/vel", channel_size, &mut builder, &mut ts_builder)?;

    let _ = ts_builder_sender.send(ts_builder);

    // Build the control system
    let mut cs = builder.build_from_store(
        "cart",
        &mut store,
        ControlSystemParameters {
            dt: 0.01,
            max_iter: 1000,
        },
    )?;

    store.save()?;

    // Execute
    while cs.step()? != StepResult::Stop {}

    let _ = run_end_sender.send(());

    println!("Control system ended");
    Ok(())
}
