use control_system::{Prober, StepInfo};
use plotter::{SignalGroup, SignalSample};
use std::sync::{mpsc::Sender, Mutex};

pub trait Plottable<const N: usize> {
    fn plot(&self, t: f64, plot_signals: &[Sender<SignalSample>; N]);
}

pub struct Plotter<const N: usize> {
    signals: [Sender<SignalSample>; N],
}

impl<const N: usize> Plotter<N> {
    pub fn new(name: &str, signal_group: &Mutex<SignalGroup>) -> Self {
        if N == 1 {
            Plotter {
                signals: std::array::from_fn(|_| signal_group.lock().unwrap().add_signal(name)),
            }
        } else {
            Plotter {
                signals: std::array::from_fn(|i| {
                    signal_group
                        .lock()
                        .unwrap()
                        .add_signal(format!("{name}/{name}[{}]", i, name=name).as_str())
                }),
            }
        }
    }
}

impl<const N: usize, T: Plottable<N>> Prober<T> for Plotter<N> {
    fn probe(&self, _: &str, v: Option<T>, k: StepInfo) {
        v.unwrap().plot(k.t, &self.signals);
    }
}

impl<T: Into<f64> + Clone> Plottable<1> for T {
    fn plot(&self, t: f64, plot_signals: &[Sender<SignalSample>; 1]) {
        plot_signals[0]
            .send(SignalSample {
                t: t,
                y: Into::<f64>::into(self.clone()),
            })
            .unwrap();
    }
}
