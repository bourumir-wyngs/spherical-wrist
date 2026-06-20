use super::*;

/// Parameters for the kinematic model of the robot.
///
/// These are OPW geometric parameters, joint offsets, and joint direction
/// corrections used to construct an OPW kinematics solver.
#[pyclass(frozen)]
#[derive(Clone)]
pub(crate) struct KinematicModel {
    a1: f64,
    a2: f64,
    b: f64,
    c1: f64,
    c2: f64,
    c3: f64,
    c4: f64,
    offsets: [f64; 6],
    flip_axes: [bool; 6],
}

impl KinematicModel {
    pub(crate) fn to_parameters(&self, degrees: bool) -> Parameters {
        Parameters {
            a1: self.a1,
            a2: self.a2,
            b: self.b,
            c1: self.c1,
            c2: self.c2,
            c3: self.c3,
            c4: self.c4,
            offsets: if degrees {
                self.offsets.map(f64::to_radians)
            } else {
                self.offsets
            },
            sign_corrections: self.flip_axes.map(|flip| if flip { -1 } else { 1 }),
            dof: 6,
        }
    }
}

#[pymethods]
impl KinematicModel {
    #[new]
    #[pyo3(signature = (
        a1 = 0.0,
        a2 = 0.0,
        b = 0.0,
        c1 = 0.0,
        c2 = 0.0,
        c3 = 0.0,
        c4 = 0.0,
        offsets = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        flip_axes = (false, false, false, false, false, false),
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        a1: f64,
        a2: f64,
        b: f64,
        c1: f64,
        c2: f64,
        c3: f64,
        c4: f64,
        offsets: (f64, f64, f64, f64, f64, f64),
        flip_axes: (bool, bool, bool, bool, bool, bool),
    ) -> PyResult<Self> {
        let model = KinematicModel {
            a1,
            a2,
            b,
            c1,
            c2,
            c3,
            c4,
            offsets: offsets.into(),
            flip_axes: flip_axes.into(),
        };
        model.validate()?;
        Ok(model)
    }

    #[getter]
    fn a1(&self) -> f64 {
        self.a1
    }

    #[getter]
    fn a2(&self) -> f64 {
        self.a2
    }

    #[getter]
    fn b(&self) -> f64 {
        self.b
    }

    #[getter]
    fn c1(&self) -> f64 {
        self.c1
    }

    #[getter]
    fn c2(&self) -> f64 {
        self.c2
    }

    #[getter]
    fn c3(&self) -> f64 {
        self.c3
    }

    #[getter]
    fn c4(&self) -> f64 {
        self.c4
    }

    #[getter]
    fn offsets(&self) -> (f64, f64, f64, f64, f64, f64) {
        array_to_tuple(self.offsets)
    }

    #[getter]
    fn flip_axes(&self) -> (bool, bool, bool, bool, bool, bool) {
        let [a, b, c, d, e, f] = self.flip_axes;
        (a, b, c, d, e, f)
    }

    pub(crate) fn __repr__(&self) -> String {
        format!(
            "KinematicModel(\n    a1={},\n    a2={},\n    b={},\n    c1={},\n    c2={},\n    c3={},\n    c4={},\n    offsets={:?},\n    flip_axes={:?},\n)",
            self.a1,
            self.a2,
            self.b,
            self.c1,
            self.c2,
            self.c3,
            self.c4,
            self.offsets,
            self.flip_axes
        )
    }
}

impl KinematicModel {
    fn validate(&self) -> PyResult<()> {
        let values = [self.a1, self.a2, self.b, self.c1, self.c2, self.c3, self.c4];
        if values
            .iter()
            .chain(self.offsets.iter())
            .any(|v| !v.is_finite())
        {
            return Err(PyValueError::new_err(
                "kinematic parameters and offsets must be finite",
            ));
        }
        Ok(())
    }
}
