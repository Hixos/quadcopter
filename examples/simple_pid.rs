use std::sync::{Arc, Mutex};

use anyhow::Result;
use control_system::{
    blocks::{Delay, Generator},
    numeric::{ForwardEuler, ODESolver, RungeKutta4},
    ControlBlock, ControlSystemBuilder, InputConnector, Interconnector, OutputConnector, StepInfo,
};
use drone::plottable::Plotter;
use nalgebra::SVector;
use plotter::SignalGroup;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions::default();

    eframe::run_native(
        "Plotter",
        native_options,
        Box::new(|cc| Box::new(plotter::PlotterApp::start(cc, init_control))),
    )
}

pub fn init_control(plot_signals: Arc<Mutex<SignalGroup>>) {
    let gen = Generator::new("p_ref", "p_ref", |k| {
        // Generate reference input
        match k.t {
            t if t <= 1f64 => 0f64,
            t if t > 1f64 && t <= 60f64 => 100f64,
            t if t > 60f64 && t <= 120f64 => 50f64,
            _ => 0f64,
        }
    });

    let pid_pos = PI::new("pid_pos", "p_ref", "p_val", "v_ref", 0.3f64, 0.05f64);
    let pid_vel = PI::new("pid_vel", "v_ref", "v_val", "car/force", 15f64, 0.5f64);
    let car = Car::new("car", 1f64, 10f64);
    let delay_vel = Delay::<f64, 1>::new("vel_delay", 0f64, "car/vel", "v_val");
    let delay_pos = Delay::<f64, 1>::new("pos_delay", 0f64, "car/pos", "p_val");

    let eul_pid_pos = PI::new(
        "eul/pid_pos",
        "p_ref",
        "eul/p_val",
        "eul/v_ref",
        0.3f64,
        0.05f64,
    );
    let eul_pid_vel = PI::new(
        "eul/pid_vel",
        "eul/v_ref",
        "eul/v_val",
        "eul/car/force",
        15f64,
        0.5f64,
    );
    let eul_car = EulerCar::new("eul/car", 1f64, 10f64);
    let eul_delay_vel = Delay::<f64, 1>::new("eul/vel_delay", 0f64, "eul/car/vel", "eul/v_val");
    let eul_delay_pos = Delay::<f64, 1>::new("eul/pos_delay", 0f64, "eul/car/pos", "eul/p_val");

    let mut builder = ControlSystemBuilder::new();

    builder.add_block(gen).unwrap();

    builder.add_block(pid_vel).unwrap();
    builder.add_block(pid_pos).unwrap();
    builder.add_block(car).unwrap();
    builder.add_block(delay_vel).unwrap();
    builder.add_block(delay_pos).unwrap();

    builder.add_block(eul_pid_vel).unwrap();
    builder.add_block(eul_pid_pos).unwrap();
    builder.add_block(eul_car).unwrap();
    builder.add_block(eul_delay_vel).unwrap();
    builder.add_block(eul_delay_pos).unwrap();


    let mut add_probe = |sig_name: &str| {
        builder
            .probe::<f64, _>(sig_name, Plotter::new(sig_name, &plot_signals))
            .unwrap();
    };

    add_probe("p_ref");
    add_probe("v_ref");
    add_probe("car/pos");
    add_probe("car/vel");
    add_probe("car/force");

    add_probe("eul/v_ref");
    add_probe("eul/car/pos");
    add_probe("eul/car/vel");
    add_probe("eul/car/force");



    let dt = 10; // ms
    let n = 200000/dt;

    let mut cs = builder.build(0f64).unwrap();
    for _ in 0..n {
        cs.step(dt as f64 / 1000f64).unwrap();
    }
}

pub struct Car {
    name: String,

    u_f: InputConnector<f64>,
    y_x: OutputConnector<f64>,
    y_v: OutputConnector<f64>,

    // Params
    k: f64,
    m: f64,

    // State
    x: SVector<f64, 2>, // x, v
}

impl Car {
    pub fn new(block_name: &str, k: f64, m: f64) -> Self {
        Car {
            name: block_name.to_string(),
            u_f: InputConnector::new(format!("{}/force", block_name).as_str()),
            y_x: OutputConnector::new(format!("{}/pos", block_name).as_str()),
            y_v: OutputConnector::new(format!("{}/vel", block_name).as_str()),
            k,
            m,
            x: SVector::<f64, 2>::new(0f64, 0f64),
        }
    }
}

impl ControlBlock for Car {
    fn register_inputs(&mut self, interconnector: &mut Interconnector) -> Result<()> {
        interconnector.register_input(&mut self.u_f)
    }

    fn register_outputs(&mut self, interconnector: &mut Interconnector) -> Result<()> {
        interconnector.register_output(&mut self.y_x)?;
        interconnector.register_output(&mut self.y_v)
    }

    #[allow(unused_variables)]
    fn step(&mut self, k: StepInfo) -> Result<()> {
        let f = self.u_f.input().unwrap();
        self.x = RungeKutta4::solve(
            |t0, y0| {
                let v = y0[1];
                let a = (f - self.k * v.abs()) / self.m;
                SVector::<f64, 2>::new(v, a)
            },
            k.t,
            k.dt,
            self.x,
        );

        self.y_x.output(self.x[0]);
        self.y_v.output(self.x[1]);
        Ok(())
    }

    fn name(&self) -> String {
        self.name.clone()
    }
}

pub struct EulerCar {
    name: String,

    u_f: InputConnector<f64>,
    y_x: OutputConnector<f64>,
    y_v: OutputConnector<f64>,

    // Params
    k: f64,
    m: f64,

    // State
    x: SVector<f64, 2>, // x, v
}

impl EulerCar {
    pub fn new(block_name: &str, k: f64, m: f64) -> Self {
        EulerCar {
            name: block_name.to_string(),
            u_f: InputConnector::new(format!("{}/force", block_name).as_str()),
            y_x: OutputConnector::new(format!("{}/pos", block_name).as_str()),
            y_v: OutputConnector::new(format!("{}/vel", block_name).as_str()),
            k,
            m,
            x: SVector::<f64, 2>::new(0f64, 0f64),
        }
    }
}

impl ControlBlock for EulerCar {
    fn register_inputs(&mut self, interconnector: &mut Interconnector) -> Result<()> {
        interconnector.register_input(&mut self.u_f)
    }

    fn register_outputs(&mut self, interconnector: &mut Interconnector) -> Result<()> {
        interconnector.register_output(&mut self.y_x)?;
        interconnector.register_output(&mut self.y_v)
    }

    #[allow(unused_variables)]
    fn step(&mut self, k: StepInfo) -> Result<()> {
        let f = self.u_f.input().unwrap();
        self.x = ForwardEuler::solve(
            |t0, y0| {
                let v = y0[1];
                let a = (f - self.k * v.abs()) / self.m;
                SVector::<f64, 2>::new(v, a)
            },
            k.t,
            k.dt,
            self.x,
        );

        self.y_x.output(self.x[0]);
        self.y_v.output(self.x[1]);
        Ok(())
    }

    fn name(&self) -> String {
        self.name.clone()
    }
}

struct PI {
    name: String,
    u_ref: InputConnector<f64>,
    u_val: InputConnector<f64>,

    y: OutputConnector<f64>,

    // Params
    k: f64,
    ki: f64,

    // State
    acc: f64,
}

impl PI {
    pub fn new(
        block_name: &str,
        ref_name: &str,
        val_name: &str,
        y_name: &str,
        k: f64,
        ki: f64,
    ) -> Self {
        PI {
            name: block_name.to_string(),
            u_ref: InputConnector::new(ref_name),
            u_val: InputConnector::new(val_name),
            y: OutputConnector::new(y_name),
            k,
            ki,
            acc: 0f64,
        }
    }
}

impl ControlBlock for PI {
    fn register_inputs(&mut self, interconnector: &mut Interconnector) -> Result<()> {
        interconnector.register_input(&mut self.u_ref)?;
        interconnector.register_input(&mut self.u_val)
    }

    fn register_outputs(&mut self, interconnector: &mut Interconnector) -> Result<()> {
        interconnector.register_output(&mut self.y)
    }

    #[allow(unused_variables)]
    fn step(&mut self, k: StepInfo) -> Result<()> {
        let err = self.u_ref.input().unwrap() - self.u_val.input().unwrap();
        self.acc += err * k.dt;

        self.y.output(self.k * err + self.ki * self.acc);
        Ok(())
    }

    fn name(&self) -> String {
        self.name.clone()
    }
}
