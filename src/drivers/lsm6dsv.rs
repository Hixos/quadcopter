use crate::vector3::Vector3;
use anyhow::{anyhow, Result};
use control_system::{io::Output, Block, BlockIO, StepResult};

#[derive(BlockIO)]
pub struct Lsm6dsv {
    #[blockio(block_name)]
    name: String,

    #[blockio(output)]
    dt_accel: Output<f64>,

    #[blockio(output)]
    dt_gyro: Output<f64>,

    #[blockio(output)]
    accel: Output<Vector3>,

    #[blockio(output)]
    angvel: Output<Vector3>,

    #[blockio(output)]
    accel_norm: Output<f64>,

    #[blockio(output)]
    angvel_norm: Output<f64>,

    buffer_accel: industrial_io::Buffer,
    buffer_gyro: industrial_io::Buffer,

    channels: Channels,

    last_ts_accel: Option<u64>,
    last_ts_gyro: Option<u64>,
}

struct Channels {
    ts_chan_accel: industrial_io::Channel,
    ts_chan_gyro: industrial_io::Channel,
    accel_chan: Vec<SensorChannel>,
    angvel_chan: Vec<SensorChannel>,
}

impl Channels {
    fn from_devices(
        dev_accel: &industrial_io::Device,
        dev_gyro: &industrial_io::Device,
    ) -> Result<Self> {
        let ts_chan_accel = dev_accel
            .find_channel("timestamp", false)
            .ok_or(anyhow!("Accel channel '{}' not found", "timestamp"))?;

        let ts_chan_gyro = dev_gyro
            .find_channel("timestamp", false)
            .ok_or(anyhow!("Gyro channel '{}' not found", "timestamp"))?;

        let accx_chan = SensorChannel::from_device("accel_x", dev_accel)?;
        let accy_chan = SensorChannel::from_device("accel_y", dev_accel)?;
        let accz_chan = SensorChannel::from_device("accel_z", dev_accel)?;

        let angvx_chan = SensorChannel::from_device("anglvel_x", dev_gyro)?;
        let angvy_chan = SensorChannel::from_device("anglvel_y", dev_gyro)?;
        let angvz_chan = SensorChannel::from_device("anglvel_z", dev_gyro)?;

        Ok(Self {
            ts_chan_accel,
            ts_chan_gyro,
            accel_chan: vec![accx_chan, accy_chan, accz_chan],
            angvel_chan: vec![angvx_chan, angvy_chan, angvz_chan],
        })
    }

    fn enable(&self) {
        self.ts_chan_accel.enable();
        self.ts_chan_gyro.enable();
        self.accel_chan.iter().for_each(|c| c.chan.enable());
        self.angvel_chan.iter().for_each(|c| c.chan.enable());
    }

    fn last_timestamp(&self, buf: &industrial_io::Buffer) -> Option<u64> {
        buf.channel_iter::<u64>(&self.ts_chan_accel).last()
    }

    fn last_accel(&self, buf: &industrial_io::Buffer) -> Option<Vector3> {
        let accel = self
            .accel_chan
            .iter()
            .map(|chan| Some(chan.calibrate(buf.channel_iter::<i16>(&chan.chan).last()?)))
            .collect::<Option<Vec<_>>>()?;

        Some(Vector3(nalgebra::Vector3::<f64>::from_column_slice(&accel)))
    }

    fn last_angvel(&self, buf: &industrial_io::Buffer) -> Option<Vector3> {
        let angvel = self
            .angvel_chan
            .iter()
            .map(|chan| Some(chan.calibrate(buf.channel_iter::<i16>(&chan.chan).last()?).to_degrees()))
            .collect::<Option<Vec<_>>>()?;

        Some(Vector3(nalgebra::Vector3::<f64>::from_column_slice(
            &angvel,
        )))
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

impl Lsm6dsv {
    pub fn new(
        name: &str,
        sampling_frequency: u32,
        iio_ctx: &industrial_io::Context,
    ) -> Result<Self> {
        let device_accel = iio_ctx
            .find_device("lsm6dsv16x_accel")
            .ok_or(anyhow!("No lsm6dsv16x_accel iio device found"))?;
        let device_gyro = iio_ctx
            .find_device("lsm6dsv16x_gyro")
            .ok_or(anyhow!("No lsm6dsv16x_gyro iio device found"))?;

        let channels = Channels::from_devices(&device_accel, &device_gyro)?;

        channels.enable();

        device_accel.attr_write("sampling_frequency", sampling_frequency)?;
        device_gyro.attr_write("sampling_frequency", sampling_frequency)?;

        device_accel.attr_write("current_timestamp_clock", "monotonic")?;
        device_gyro.attr_write("current_timestamp_clock", "monotonic")?;

        let buffer_accel = device_accel.create_buffer(1, false)?;
        let buffer_gyro = device_gyro.create_buffer(1, false)?;

        Ok(Self {
            name: name.to_string(),
            dt_accel: Output::default(),
            dt_gyro: Output::default(),
            accel: Output::default(),
            angvel: Output::default(),
            accel_norm: Output::default(),
            angvel_norm: Output::default(),
            buffer_accel,
            buffer_gyro,
            channels,
            last_ts_accel: None,
            last_ts_gyro: None,
        })
    }
}

impl Block for Lsm6dsv {
    fn step(&mut self, _: control_system::StepInfo) -> control_system::Result<StepResult> {
        self.buffer_accel
            .refill()
            .map_err(control_system::ControlSystemError::from_boxed)?;
        self.buffer_gyro
            .refill()
            .map_err(control_system::ControlSystemError::from_boxed)?;

        let ts_accel = self.channels.last_timestamp(&self.buffer_accel).unwrap();
        let ts_gyro = self.channels.last_timestamp(&self.buffer_gyro).unwrap();

        let accel = self.channels.last_accel(&self.buffer_accel).unwrap();
        let angvel = self.channels.last_angvel(&self.buffer_gyro).unwrap();

        if let Some(last_ts_accel) = self.last_ts_accel {
            self.dt_accel.set((ts_accel - last_ts_accel) as f64 / 1000000.0);
        } else {
            self.dt_accel.set(0.0);
        }
        if let Some(last_ts_gyro) = self.last_ts_gyro {
            self.dt_gyro.set((ts_gyro - last_ts_gyro) as f64 / 1000000.0);
        } else {
            self.dt_gyro.set(0.0);
        }

        self.accel_norm.set(accel.0.norm());
        self.angvel_norm.set(angvel.0.norm());

        self.accel.set(accel);
        self.angvel.set(angvel);

        self.last_ts_accel = Some(ts_accel);
        self.last_ts_gyro = Some(ts_gyro);

        Ok(StepResult::Continue)
    }
}
