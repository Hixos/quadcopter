use control_system::{Prober, StepInfo};
use plotter::{SignalGroup, SignalSample};
use std::sync::{mpsc::Sender, Mutex};

pub trait Plottable {
    fn plot(&self, t: f64, plot_signal: &Sender<SignalSample>);
}

pub struct Plotter {
    signal: Sender<SignalSample>,
}

impl Plotter {
    pub fn new(name: &str, signal_group: &Mutex<SignalGroup>) -> Self {
        Plotter {
            signal: signal_group.lock().unwrap().add_signal(name),
        }
    }
}

impl<T: Plottable> Prober<T> for Plotter {
    fn probe(&self, _: &str, v: Option<T>, k: StepInfo) {
        v.unwrap().plot(k.t, &self.signal);
    }
}

impl<T: Into<f64> + Clone> Plottable for T {
    fn plot(&self, t: f64, plot_signal: &Sender<SignalSample>) {
        plot_signal
            .send(SignalSample {
                t: t,
                y: Into::<f64>::into(self.clone()),
            })
            .unwrap();
    }
}
