use control_system::{
    blocks::AsF64Signals, io::Input, Block, BlockIO, ControlSystemBuilder, ControlSystemError,
    StepInfo, StepResult,
};
use tokio::sync::mpsc::Sender;

use super::{
    data::Sample,
    telemetry_server::{TelemetryID, TelemetryServiceBuilder},
};

#[derive(BlockIO)]
pub struct ProtoPlotter<T> {
    #[blockio(block_name)]
    name: String,

    #[blockio(input)]
    u: Input<T>,

    ids: Vec<TelemetryID>,
    senders: Vec<Sender<Sample>>,
}

impl<T: AsF64Signals + Default> ProtoPlotter<T> {
    pub fn new(
        name: &str,
        topic: &str,
        channel_size: usize,
        builder: &mut TelemetryServiceBuilder,
    ) -> Self {
        let (ids, senders) = T::names()
            .into_iter()
            .map(|name| builder.register_signal(&format!("{topic}{name}"), channel_size))
            .unzip();

        // let (id, sender) = , channel_size)
        ProtoPlotter {
            name: name.to_string(),
            u: Input::default(),
            ids,
            senders,
        }
    }
}

impl<T: Clone + AsF64Signals + 'static> Block for ProtoPlotter<T> {
    fn step(&mut self, k: StepInfo) -> Result<StepResult, ControlSystemError> {
        self.u
            .get()
            .values()
            .into_iter()
            .enumerate()
            .for_each(|(i, v)| {
                let _ = self.senders[i].try_send(Sample {
                    id: self.ids[i].as_u64(),
                    time: k.t,
                    value: v,
                });
            });

        Ok(StepResult::Continue)
    }
}

pub fn add_protoplotter<T>(
    signal_name: &str,
    channel_size: usize,
    cs_builder: &mut ControlSystemBuilder,
    ts_builder: &mut TelemetryServiceBuilder,
) -> control_system::Result<()>
where
    T: AsF64Signals + Default + Clone + 'static,
{
    use rand::distributions::Alphanumeric;
    use rand::{thread_rng, Rng};

    let rand_string: String = thread_rng()
        .sample_iter(&Alphanumeric)
        .take(6)
        .map(char::from)
        .collect();

    let name = format!(
        "protoplotter{}_{}",
        signal_name.replace('/', "_"),
        rand_string
    );

    let plotter = ProtoPlotter::<T>::new(name.as_str(), signal_name, channel_size, ts_builder);

    cs_builder.add_block(plotter, &[("u", signal_name)], &[])?;

    Ok(())
}
