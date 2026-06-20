use super::*;

/// Annotated joints specifying the position flags and movement type into this position.
#[pyclass(frozen)]
#[derive(Clone)]
pub(crate) struct AnnotatedJoints {
    pub(crate) joints: [f64; 6],
    pub(crate) flags: u32,
    pub(crate) move_into: String,
}

#[pymethods]
impl AnnotatedJoints {
    #[getter]
    fn joints(&self) -> [f64; 6] {
        self.joints
    }

    #[getter]
    fn flags(&self) -> u32 {
        self.flags
    }

    #[getter]
    fn move_into(&self) -> String {
        self.move_into.clone()
    }

    #[pyo3(signature = (flag))]
    fn has_flag(&self, flag: u32) -> bool {
        self.flags & flag == flag
    }

    fn __repr__(&self) -> String {
        format!(
            "AnnotatedJoints(joints={:?}, flags={}, move_into={:?})",
            self.joints, self.flags, self.move_into
        )
    }
}
