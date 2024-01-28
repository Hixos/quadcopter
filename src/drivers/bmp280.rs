use std::{
    fs::File,
    io::{Read, Seek, SeekFrom},
};

use anyhow::Result;
use control_system::{io::Output, Block, BlockIO, ParameterStore, StepResult};
use serde::{Deserialize, Serialize};

#[derive(BlockIO)]
pub struct Bmp280 {
    #[blockio(block_name)]
    name: String,

    #[blockio(output)]
    press: Output<f64>,

    #[blockio(output)]
    temp: Output<f64>,

    press_file: File,
    temp_file: File,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bmp280Params {
    pub pressure_file: String,
    pub temperature_file: String,
}

impl Bmp280 {
    pub fn new(name: &str, params: Bmp280Params) -> Result<Self> {
        let press_file = File::open(params.pressure_file.as_str())?;
        let temp_file = File::open(params.temperature_file.as_str())?;

        Ok(Self {
            name: name.to_string(),
            press: Output::<f64>::default(),
            temp: Output::<f64>::default(),
            press_file,
            temp_file,
        })
    }

    pub fn from_store(
        name: &str,
        store: &mut ParameterStore,
        default: Bmp280Params,
    ) -> Result<Self> {
        let params = store.get_block_params(name, default)?;
        Self::new(name, params)
    }
}

impl Block for Bmp280 {
    fn step(&mut self, _: control_system::StepInfo) -> control_system::Result<StepResult> {
        self.temp.set(Self::read_value(&mut self.temp_file)?);
        self.press.set(Self::read_value(&mut self.press_file)?);

        Ok(StepResult::Continue)
    }
}

impl Bmp280 {
    fn read_value(file: &mut File) -> control_system::Result<f64> {
        let mut buf = String::new();
        file.read_to_string(&mut buf)
            .map_err(control_system::ControlSystemError::from_boxed)?;
        file.seek(SeekFrom::Start(0))
            .map_err(control_system::ControlSystemError::from_boxed)?;

        let val = buf
            .trim()
            .parse()
            .map_err(control_system::ControlSystemError::from_boxed)?;

        Ok(val)
    }
}
