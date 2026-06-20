use super::*;

/// Defines a frame that transforms a robot working area by translating,
/// rotating, and optionally uniformly scaling it.
///
/// The frame can be created from three pairs of tie points: one triplet defining
/// original trajectory points and another triplet defining target points.
#[pyclass(frozen)]
#[derive(Clone, Copy)]
pub(crate) struct Frame {
    frame: RsFrameTransform,
}

#[pymethods]
impl Frame {
    /// Construct a working frame from a tie mapping.
    ///
    /// A tie maps one set of three original trajectory points to the required
    /// set of three target points. The resulting frame may translate, rotate,
    /// and uniformly scale the original coordinate system. Degenerate point
    /// sets, non-uniform scale, and shear are rejected.
    #[staticmethod]
    fn from_tie(original: [[f64; 3]; 3], target: [[f64; 3]; 3]) -> PyResult<Self> {
        let original = tie_points_to_dvec3(original, "original_tie_points")?;
        let target = tie_points_to_dvec3(target, "target_tie_points")?;
        let frame = RsFrame::try_from_tie(original, target).map_err(|error| {
            PyValueError::new_err(format!(
                "could not construct frame from tie points: {error}"
            ))
        })?;

        Ok(Self { frame })
    }

    #[getter]
    fn scale(&self) -> f64 {
        self.frame.scale
    }

    #[getter]
    fn translation(&self) -> [f64; 3] {
        [
            self.frame.translation.x,
            self.frame.translation.y,
            self.frame.translation.z,
        ]
    }

    fn as_matrix(&self) -> [[f64; 4]; 4] {
        frame_transform_to_matrix(self.frame)
    }

    fn __repr__(&self) -> String {
        format!(
            "Frame(translation=({}, {}, {}), scale={})",
            self.frame.translation.x,
            self.frame.translation.y,
            self.frame.translation.z,
            self.frame.scale
        )
    }
}
