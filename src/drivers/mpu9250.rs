use std::{
    fs::{read_to_string, File},
    io::{Read, Seek, SeekFrom},
    path::Path,
};

use anyhow::Result;
use control_system::{io::Output, Block, BlockIO, ParameterStore, StepResult};
use serde::{Deserialize, Serialize};

use crate::vector3::Vector3;

#[derive(BlockIO)]
pub struct Mpu9250 {
    #[blockio(block_name)]
    name: String,

    #[blockio(output)]
    acc: Output<Vector3>,

    #[blockio(output)]
    gyro: Output<Vector3>,

    acc_files: Vec<File>,
    acc_biases: Vec<f64>,
    acc_scale: f64,

    gyro_files: Vec<File>,
    gyro_biases: Vec<f64>,
    gyro_scale: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Mpu9250Params {
    pub device: String,
}

impl Mpu9250 {
    pub fn new(name: &str, params: Mpu9250Params) -> Result<Self> {
        let device_dir = Path::new("/sys/bus/iio/devices/").join(params.device);

        let axes = ["x", "y", "z"];

        let device_file_path = |file_template: &str, ax: Option<&str>| -> String {
            let filename = if let Some(ax) = ax {
                file_template.replace('%', ax)
            } else {
                file_template.to_string()
            };

            device_dir
                .join(filename)
                .into_os_string()
                .into_string()
                .unwrap()
        };

        let map_files = |file_template: &str| -> Result<Vec<File>> {
            axes.iter()
                .map(|&ax| {
                    File::open(device_file_path(file_template, Some(ax)))
                        .map_err(anyhow::Error::from)
                })
                .collect::<Result<Vec<_>>>()
        };

        let read_files = |file_template: &str| -> Result<Vec<f64>> {
            let mut files = map_files(file_template)?;

            let mut out = vec![];

            for f in files.iter_mut() {
                let mut buf = String::new();
                f.read_to_string(&mut buf)?;
                out.push(buf.trim().parse::<f64>()?);
            }

            Ok(out)
        };

        let acc_files = map_files("in_accel_%_raw")?;
        let acc_biases: Vec<f64> = read_files("in_accel_%_calibbias")?;
        let acc_scale = read_to_string(device_file_path("in_accel_scale", None))?
            .trim()
            .parse::<f64>()?;

        let gyro_files = map_files("in_anglvel_%_raw")?;
        let gyro_biases: Vec<f64> = read_files("in_anglvel_%_calibbias")?;
        let gyro_scale = read_to_string(device_file_path("in_anglvel_scale", None))?
            .trim()
            .parse::<f64>()?;

        Ok(Self {
            name: name.to_string(),
            acc: Output::<Vector3>::default(),
            gyro: Output::<Vector3>::default(),
            acc_files,
            acc_biases,
            acc_scale,
            gyro_files,
            gyro_biases,
            gyro_scale,
        })
    }

    pub fn from_store(
        name: &str,
        store: &mut ParameterStore,
        default: Mpu9250Params,
    ) -> Result<Self> {
        let params = store.get_block_params(name, default)?;
        Self::new(name, params)
    }
}

impl Block for Mpu9250 {
    fn step(&mut self, _: control_system::StepInfo) -> control_system::Result<StepResult> {
        let acc = Vector3(nalgebra::Vector3::new(
            Self::read_value(&mut self.acc_files[0], self.acc_scale, self.acc_biases[0])?,
            Self::read_value(&mut self.acc_files[1], self.acc_scale, self.acc_biases[1])?,
            Self::read_value(&mut self.acc_files[2], self.acc_scale, self.acc_biases[2])?,
        ));

        let gyro = Vector3(nalgebra::Vector3::new(
            Self::read_value(
                &mut self.gyro_files[0],
                self.gyro_scale,
                self.gyro_biases[0],
            )?,
            Self::read_value(
                &mut self.gyro_files[1],
                self.gyro_scale,
                self.gyro_biases[1],
            )?,
            Self::read_value(
                &mut self.gyro_files[2],
                self.gyro_scale,
                self.gyro_biases[2],
            )?,
        ));

        self.acc.set(acc);
        self.gyro.set(gyro);

        Ok(StepResult::Continue)
    }
}

impl Mpu9250 {
    fn read_value(file: &mut File, scale: f64, offset: f64) -> control_system::Result<f64> {
        let mut buf = String::new();
        file.read_to_string(&mut buf)
            .map_err(control_system::ControlSystemError::from_boxed)?;
        file.seek(SeekFrom::Start(0))
            .map_err(control_system::ControlSystemError::from_boxed)?;

        let val = buf
            .trim()
            .parse::<f64>()
            .map_err(control_system::ControlSystemError::from_boxed)?;

        Ok((val + offset) * scale)
    }
}
