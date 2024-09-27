use crate::vector3::Vector3;
use anyhow::{anyhow, Result};
use control_system::{io::Output, Block, BlockIO, StepResult};

#[derive(BlockIO)]
pub struct Lis3mdl {
    #[blockio(block_name)]
    name: String,

    #[blockio(output)]
    dt: Output<f64>,

    #[blockio(output)]
    mag: Output<Vector3>,

    #[blockio(output)]
    mag_norm: Output<f64>,

    buffer_mag: industrial_io::Buffer,

    channels: Channels,

    last_ts: Option<u64>,
}

struct Channels {
    ts_chan: industrial_io::Channel,
    mag_chan: Vec<SensorChannel>,
}

impl Channels {
    fn from_devices(dev: &industrial_io::Device) -> Result<Self> {
        let ts_chan = dev
            .find_channel("timestamp", false)
            .ok_or(anyhow!("Channel '{}' not found", "timestamp"))?;

        let magx_chan = SensorChannel::from_device("magn_x", dev)?;
        let magy_chan = SensorChannel::from_device("magn_y", dev)?;
        let magz_chan = SensorChannel::from_device("magn_z", dev)?;

        Ok(Self {
            ts_chan,
            mag_chan: vec![magx_chan, magy_chan, magz_chan],
        })
    }

    fn enable(&self) {
        self.ts_chan.enable();
        self.mag_chan.iter().for_each(|c| c.chan.enable());
    }

    fn last_timestamp(&self, buf: &industrial_io::Buffer) -> Option<u64> {
        buf.channel_iter::<u64>(&self.ts_chan).last()
    }

    fn last_mag(&self, buf: &industrial_io::Buffer) -> Option<Vector3> {
        let accel = self
            .mag_chan
            .iter()
            .map(|chan| Some(chan.calibrate(buf.channel_iter::<i16>(&chan.chan).last()?)))
            .collect::<Option<Vec<_>>>()?;

        Some(Vector3(nalgebra::Vector3::<f64>::from_column_slice(&accel)))
    }
}

struct SensorChannel {
    chan: industrial_io::Channel,
    scale: f64,
}

impl SensorChannel {
    fn from_device(name: &str, dev: &industrial_io::Device) -> Result<Self> {
        let chan = dev
            .find_channel(name, false)
            .ok_or(anyhow!("Channel '{name}' not found"))?;
        let scale: f64 = chan.attr_read::<f64>("scale")?;

        Ok(SensorChannel { chan, scale })
    }

    fn calibrate(&self, raw: i16) -> f64 {
        (self.chan.convert::<i16>(raw) as f64) * self.scale
    }
}

impl Lis3mdl {
    pub fn new(
        name: &str,
        sampling_frequency: u32,
        iio_ctx: &industrial_io::Context,
    ) -> Result<Self> {
        let dev = iio_ctx
            .find_device("lis3mdl")
            .ok_or(anyhow!("No lis3mdl iio device found"))?;
        let channels = Channels::from_devices(&dev)?;

        channels.enable();

        let trigger = iio_ctx
            .find_device("lis3mdl-trigger2")
            .ok_or(anyhow!("No 'lis3mdl-trigger2' iio trigger found"))?;
        dev.set_trigger(&trigger).unwrap();

        dev.attr_write("in_magn_sampling_frequency", "155.0")?;
        dev.attr_write("current_timestamp_clock", "monotonic")?;

        let buffer_mag = dev.create_buffer(1, false)?;

        Ok(Self {
            name: name.to_string(),
            dt: Output::default(),
            mag: Output::default(),
            mag_norm: Output::default(),
            buffer_mag,
            channels,
            last_ts: None,
        })
    }
}

impl Block for Lis3mdl {
    fn step(&mut self, _: control_system::StepInfo) -> control_system::Result<StepResult> {
        self.buffer_mag
            .refill()
            .map_err(control_system::ControlSystemError::from_boxed)?;

        let ts = self.channels.last_timestamp(&self.buffer_mag).unwrap();

        let mag = self.channels.last_mag(&self.buffer_mag).unwrap();

        if let Some(last_ts) = self.last_ts {
            self.dt.set((ts - last_ts) as f64 / 1000000.0);
        } else {
            self.dt.set(0.0);
        }

        println!("X: {}, Y: {}, Z: {}", mag.0[0], mag.0[1], mag.0[2]);

        self.mag_norm.set(mag.0.norm());
        self.mag.set(mag);

        Ok(StepResult::Continue)
    }
}
