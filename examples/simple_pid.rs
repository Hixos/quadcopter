use std::sync::{Arc, Mutex};

use anyhow::Result;
use control_system::{
    blocks::Delay, ControlBlock, ControlSystemBuilder, InputConnector, Interconnector,
    OutputConnector, StepInfo,
};
use drone::plottable::{self, Plotter};
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
            t if t > 1f64 && t <= 30f64 => 100f64,
            t if t > 30f64 && t <= 60f64 => 50f64,
            _ => 0f64,
        }
    });

    let pid_pos = PI::new("pid_pos", "p_ref", "p_val", "v_ref", 0.2f64, 0f64);
    let pid_vel = PI::new("pid_vel", "v_ref", "v_val", "car/force", 5f64, 0.5f64);

    let car = Car::new("car", 1f64, 10f64);

    let mut builder = ControlSystemBuilder::new();

    builder.add_block(gen).unwrap();
    builder.add_block(pid_vel).unwrap();
    builder.add_block(pid_pos).unwrap();
    builder.add_block(car).unwrap();

    builder
        .add_block(Delay::<f64, 1>::new("vel_delay", 0f64, "car/vel", "v_val"))
        .unwrap();

    builder
        .add_block(Delay::<f64, 1>::new("pos_delay", 0f64, "car/pos", "p_val"))
        .unwrap();

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

    let mut cs = builder.build(0f64).unwrap();
    for _ in 0..1000 {
        cs.step(0.1).unwrap();
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
    x: f64,
    v: f64,
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
            x: 0f64,
            v: 0f64,
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
        let F = self.u_f.input().unwrap();
        let a = (F - self.k * self.v.abs()) / self.m;

        self.v += a * k.dt;
        self.x += self.v * k.dt;

        self.y_x.output(self.x);
        self.y_v.output(self.v);
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

struct Generator<T, F>
where
    T: Copy,
    F: Fn(StepInfo) -> T,
{
    name: String,
    y: OutputConnector<T>,

    f: F,
}

impl<T, F> Generator<T, F>
where
    T: Copy,
    F: Fn(StepInfo) -> T,
{
    fn new(name: &str, y_name: &str, f: F) -> Self {
        Generator {
            name: name.to_string(),
            y: OutputConnector::new(y_name),
            f: f,
        }
    }
}

impl<T, F> ControlBlock for Generator<T, F>
where
    T: Copy + 'static,
    F: Fn(StepInfo) -> T,
{
    #[allow(unused_variables)]
    fn register_inputs(&mut self, interconnector: &mut Interconnector) -> Result<()> {
        Ok(())
    }

    fn register_outputs(&mut self, interconnector: &mut Interconnector) -> Result<()> {
        interconnector.register_output(&mut self.y)
    }

    #[allow(unused_variables)]
    fn step(&mut self, k: StepInfo) -> Result<()> {
        self.y.output((self.f)(k));
        Ok(())
    }

    fn name(&self) -> String {
        self.name.clone()
    }
}
