use std::sync::{ Arc, Mutex};

use control_system::{blocks, ControlSystemBuilder};
use plotter::{SignalGroup};
use crate::plottable::Plotter;

pub fn init_control(plot_signals: Arc<Mutex<SignalGroup>>) {
    //  c1 -- ADD ---------- PRINT
    //        |          |
    //        \- (z^-1) -/

    let delay = blocks::Delay::<f32, 1>::new("delay", 0f32, "sum", "a2");
    let c1 = blocks::Constant::new("c1", 1f32, "a1");

    let add = blocks::Add::<f32, 2>::new("add", &["a1", "a2"], None, "sum");

    let mut builder = ControlSystemBuilder::new();

    builder.add_block(c1).unwrap();
    builder.add_block(add).unwrap();
    builder.add_block(delay).unwrap();

    builder
        .probe::<f32, _>("a1", Plotter::new("add/a1", &plot_signals))
        .unwrap();

    builder
        .probe::<f32, _>("a2", Plotter::new("add/a2", &plot_signals))
        .unwrap();

    builder
        .probe::<f32, _>("sum", Plotter::new("add/sum", &plot_signals))
        .unwrap();


    let mut control_system = builder.build(0f64).unwrap();

    for k in 0..100 {
        control_system.step(1f64).unwrap(); // Prints 1,2,3,...,10
    }
}
