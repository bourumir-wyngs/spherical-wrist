use super::*;

/// Joint constraints that restrict rotations between `from` and `to` limits.
///
/// Wrapping around zero is supported, so limit order is important. The
/// `sorting_weight` controls whether inverse-kinematics solutions are sorted
/// closer to previous joints or closer to the middle of these constraints.
#[pyclass(frozen)]
#[derive(Clone, Copy)]
pub(crate) struct Constraints {
    constraints: RsConstraints,
}

#[pymethods]
impl Constraints {
    #[new]
    #[pyo3(signature = (from_limits, to_limits, sorting_weight=BY_PREV, radians=false))]
    fn new(
        from_limits: [f64; 6],
        to_limits: [f64; 6],
        sorting_weight: f64,
        radians: bool,
    ) -> PyResult<Self> {
        if !sorting_weight.is_finite() {
            return Err(PyValueError::new_err("sorting_weight must be finite"));
        }

        Ok(Self {
            constraints: RsConstraints::new(
                angles_to_radians(from_limits, radians)?,
                angles_to_radians(to_limits, radians)?,
                sorting_weight,
            ),
        })
    }

    #[getter]
    fn sorting_weight(&self) -> f64 {
        self.constraints.sorting_weight
    }

    #[pyo3(signature = (radians=false))]
    #[allow(clippy::wrong_self_convention)]
    fn from_limits(&self, radians: bool) -> [f64; 6] {
        angles_from_radians(self.constraints.from, radians)
    }

    #[pyo3(signature = (radians=false))]
    #[allow(clippy::wrong_self_convention)]
    fn to_limits(&self, radians: bool) -> [f64; 6] {
        angles_from_radians(self.constraints.to, radians)
    }

    #[pyo3(signature = (radians=false))]
    fn limits(&self, radians: bool) -> ([f64; 6], [f64; 6]) {
        (self.from_limits(radians), self.to_limits(radians))
    }

    #[pyo3(signature = (radians=false))]
    fn centers(&self, radians: bool) -> [f64; 6] {
        angles_from_radians(self.constraints.centers, radians)
    }

    #[pyo3(signature = (radians=false))]
    fn tolerances(&self, radians: bool) -> [f64; 6] {
        angles_from_radians(self.constraints.tolerances, radians)
    }

    #[pyo3(signature = (joints, radians=false))]
    fn compliant(&self, joints: [f64; 6], radians: bool) -> PyResult<bool> {
        let joints = angles_to_radians(joints, radians)?;
        Ok(self.constraints.compliant(&joints))
    }

    #[pyo3(signature = (radians=false))]
    fn random_joints(&self, radians: bool) -> [f64; 6] {
        angles_from_radians(self.constraints.random_angles(), radians)
    }

    fn __repr__(&self) -> String {
        format!(
            "Constraints(from_limits={:?}, to_limits={:?}, sorting_weight={})",
            self.from_limits(false),
            self.to_limits(false),
            self.sorting_weight()
        )
    }
}

impl Constraints {
    pub(crate) fn to_rs_constraints(self) -> RsConstraints {
        self.constraints
    }
}
