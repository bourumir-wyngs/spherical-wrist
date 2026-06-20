use super::*;

/// Parallelogram mechanism.
///
/// The parallelogram mechanism introduces a geometric dependency between two
/// specific joints, typically to maintain the orientation of the end-effector as
/// the robot arm moves. The movement of `joints[driven]` influences
/// `joints[coupled]`, and `scaling` determines the proportional influence.
#[pyclass(frozen)]
#[derive(Clone, Copy)]
pub(crate) struct Parallelogram {
    pub(crate) scaling: f64,
    pub(crate) driven: usize,
    pub(crate) coupled: usize,
}

#[pymethods]
impl Parallelogram {
    #[new]
    #[pyo3(signature = (scaling=1.0, driven=1, coupled=2))]
    fn new(scaling: f64, driven: usize, coupled: usize) -> PyResult<Self> {
        let parallelogram = Parallelogram {
            scaling,
            driven,
            coupled,
        };
        parallelogram.validate()?;
        Ok(parallelogram)
    }

    #[getter]
    fn scaling(&self) -> f64 {
        self.scaling
    }

    #[getter]
    fn driven(&self) -> usize {
        self.driven
    }

    #[getter]
    fn coupled(&self) -> usize {
        self.coupled
    }

    fn __repr__(&self) -> String {
        format!(
            "Parallelogram(scaling={}, driven={}, coupled={})",
            self.scaling, self.driven, self.coupled
        )
    }
}

impl Parallelogram {
    pub(crate) fn validate(&self) -> PyResult<()> {
        if !self.scaling.is_finite() {
            return Err(PyValueError::new_err(
                "parallelogram scaling must be finite",
            ));
        }
        if self.driven >= 6 || self.coupled >= 6 {
            return Err(PyValueError::new_err(
                "parallelogram joint indices must be in the range 0..6",
            ));
        }
        if self.driven == self.coupled {
            return Err(PyValueError::new_err(
                "parallelogram driven and coupled joints must be different",
            ));
        }
        Ok(())
    }
}
