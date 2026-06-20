use super::*;

/// Defines tolerance bounds between robot parts, environment objects, and any
/// two parts of the robot.
///
/// Some robot joints may come very close together, so they may require special
/// per-pair distances.
#[pyclass(frozen)]
#[derive(Clone)]
pub(crate) struct SafetyDistances {
    safety: RsSafetyDistances,
}

#[pymethods]
impl SafetyDistances {
    #[new]
    #[pyo3(signature = (to_environment=0.0, to_robot_default=0.0, special_distances=None, mode="all"))]
    fn new(
        to_environment: f64,
        to_robot_default: f64,
        special_distances: Option<Vec<(usize, usize, f64)>>,
        mode: &str,
    ) -> PyResult<Self> {
        let pairs = special_distances
            .unwrap_or_default()
            .into_iter()
            .map(|(a, b, distance)| {
                validate_collision_index(a)?;
                validate_collision_index(b)?;
                Ok((
                    (a, b),
                    validate_safety_distance(distance, "special distance")?,
                ))
            })
            .collect::<PyResult<Vec<_>>>()?;

        Ok(Self {
            safety: RsSafetyDistances {
                to_environment: validate_safety_distance(to_environment, "to_environment")?,
                to_robot_default: validate_safety_distance(to_robot_default, "to_robot_default")?,
                special_distances: RsSafetyDistances::distances(&pairs),
                mode: parse_check_mode(mode)?,
            },
        })
    }

    #[staticmethod]
    #[pyo3(signature = (mode="all"))]
    fn standard(mode: &str) -> PyResult<Self> {
        Ok(Self {
            safety: RsSafetyDistances::standard(parse_check_mode(mode)?),
        })
    }

    #[getter]
    fn to_environment(&self) -> f64 {
        self.safety.to_environment as f64
    }

    #[getter]
    fn to_robot_default(&self) -> f64 {
        self.safety.to_robot_default as f64
    }

    #[getter]
    fn special_distances(&self) -> Vec<(usize, usize, f64)> {
        self.safety
            .special_distances
            .iter()
            .map(|(&(a, b), &distance)| (a as usize, b as usize, distance as f64))
            .collect()
    }

    #[getter]
    fn mode(&self) -> &'static str {
        check_mode_name(self.safety.mode)
    }

    fn __repr__(&self) -> String {
        format!(
            "SafetyDistances(to_environment={}, to_robot_default={}, special_distances={:?}, mode={:?})",
            self.to_environment(),
            self.to_robot_default(),
            self.special_distances(),
            self.mode()
        )
    }
}

impl SafetyDistances {
    pub(crate) fn to_rs_safety(&self) -> RsSafetyDistances {
        self.safety.clone()
    }
}
