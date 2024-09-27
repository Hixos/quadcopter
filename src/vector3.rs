use control_system::blocks::AsF64Signals;

#[derive(Clone, Debug, Default)]
pub struct Vector3(pub nalgebra::Vector3<f64>);

impl AsF64Signals for Vector3 {
    fn names() -> Vec<String> {
        vec!["/x".to_string(), "/y".to_string(), "/z".to_string()]
    }

    fn values(&self) -> Vec<f64> {
        vec![self.0[0], self.0[1], self.0[2]]
    }
}

impl From<nalgebra::Vector3<f64>> for Vector3  {
    fn from(value: nalgebra::Vector3<f64>) -> Self {
        Self(value)
    }
}
