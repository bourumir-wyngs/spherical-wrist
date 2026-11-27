use super::*;
use pyo3::buffer::PyBuffer;
use rayon::prelude::*;

pyo3::create_exception!(
    spherical_wrist,
    BatchPoseError,
    PyValueError,
    "A pose matrix rejected by inverse_multiple. The pose_index attribute is the zero-based index within the supplied batch."
);

// Returning Rust tuples creates the public Python joint tuples directly, without
// a Python loop (or temporary Python lists) for every solution in a large batch.
pub(crate) type JointTuple = (f64, f64, f64, f64, f64, f64);

pub(crate) fn inverse_multiple(
    py: Python<'_>,
    matrices: &Bound<'_, PyAny>,
    robot: &dyn Kinematics,
    degrees: bool,
    transform_pose: impl Fn(Pose) -> Pose + Send + Sync,
) -> PyResult<Vec<Vec<JointTuple>>> {
    // Copy the numeric buffer while attached. Rayon must never borrow mutable
    // NumPy storage that another Python thread could change after we detach.
    // The buffer protocol avoids extracting 16 Python floats per input pose.
    let matrices = {
        let buffer = PyBuffer::<f64>::get(matrices)?;
        if buffer.dimensions() != 3 || buffer.shape()[1..] != [4, 4] {
            return Err(PyValueError::new_err("poses must have shape (n, 4, 4)"));
        }
        buffer.to_vec(py)?
    };

    let result = py.detach(move || {
        // This indexed iterator preserves the input order, including poses with
        // no solutions. Validate, solve, filter, and convert units in Rust with
        // the GIL released. All workers share the same immutable solver/meshes.
        matrices
            .par_chunks_exact(16)
            .enumerate()
            .map(|(index, values)| {
                let matrix = std::array::from_fn(|row| {
                    std::array::from_fn(|column| values[row * 4 + column])
                });
                let pose = matrix_to_pose(matrix).map_err(|error| (index, error))?;
                let pose = transform_pose(pose);
                let solutions = robot.inverse(&pose);
                Ok(solutions_from_internal(solutions, degrees)
                    .into_iter()
                    .map(array_to_tuple)
                    .collect())
            })
            .collect::<Result<Vec<_>, _>>()
    });
    // Construct the indexed Python exception only after reacquiring the GIL.
    // If several poses are invalid, Rayon may report any of their indices.
    match result {
        Ok(solutions) => Ok(solutions),
        Err((index, error)) => {
            let indexed_error =
                BatchPoseError::new_err(format!("pose {index}: {}", error.value(py)));
            indexed_error.value(py).setattr("pose_index", index)?;
            indexed_error.set_cause(py, Some(error));
            Err(indexed_error)
        }
    }
}
