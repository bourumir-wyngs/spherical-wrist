use super::*;

/// Handle for a non-blocking visualization window.
///
/// The handle can update the displayed robot joint angles and request that the
/// visualization window closes.
#[pyclass]
pub(crate) struct VisualizationHandle {
    handle: RsVisualizationHandle,
    robot: RsKinematicsWithShape,
    degrees: bool,
}

#[pymethods]
impl VisualizationHandle {
    #[pyo3(signature = (joints))]
    fn set_joints(&self, joints: [f64; 6]) -> PyResult<()> {
        self.handle
            .set_joint_angles(joints_to_visual_degrees(joints, self.degrees)?)
            .map_err(PyValueError::new_err)
    }

    #[pyo3(signature = (joints))]
    fn set_position(&self, joints: [f64; 6]) -> PyResult<()> {
        self.set_joints(joints)
    }

    #[pyo3(signature = (pose, previous_position=None))]
    fn set_pose(
        &self,
        pose: [[f64; 4]; 4],
        previous_position: Option<[f64; 6]>,
    ) -> PyResult<[f64; 6]> {
        let pose = matrix_to_pose(pose)?;
        let solutions = match previous_position {
            Some(previous_position) => {
                let previous_position =
                    previous_joints_to_internal(previous_position, self.degrees)?;
                self.robot.inverse_continuing(&pose, &previous_position)
            }
            None => self.robot.inverse(&pose),
        };
        let Some(joints) = solutions.into_iter().next() else {
            return Err(PyValueError::new_err(
                "pose cannot be resolved to a collision-free joint position",
            ));
        };

        self.handle
            .set_joint_angles(joints_to_visual_degrees(joints, false)?)
            .map_err(PyValueError::new_err)?;

        Ok(if self.degrees {
            joints.map(f64::to_degrees)
        } else {
            joints
        })
    }

    fn close(&self) -> PyResult<()> {
        self.handle.close().map_err(PyValueError::new_err)
    }

    #[getter]
    fn is_running(&self) -> bool {
        self.handle.is_running()
    }

    fn __repr__(&self) -> String {
        format!("VisualizationHandle(is_running={})", self.is_running())
    }
}

#[pyfunction]
#[pyo3(signature = (robot, initial_joints, tcp_box))]
pub(crate) fn visualize_robot(
    robot: &KinematicsWithShape,
    initial_joints: [f64; 6],
    tcp_box: [(f64, f64); 3],
) -> PyResult<VisualizationHandle> {
    let initial_joints = joints_to_visual_degrees(initial_joints, robot.degrees)?;
    let visual_robot = clone_shape_robot(&robot.robot);
    let ik_robot = clone_shape_robot(&robot.robot);
    let tcp_box = tcp_box_to_ranges(tcp_box)?;
    let handle = rs_visualize_robot_async(visual_robot, initial_joints, tcp_box);

    Ok(VisualizationHandle {
        handle,
        robot: ik_robot,
        degrees: robot.degrees,
    })
}

#[pyfunction]
#[pyo3(signature = (robot, initial_joints, tcp_box, safety))]
pub(crate) fn visualize_robot_with_safety(
    robot: &KinematicsWithShape,
    initial_joints: [f64; 6],
    tcp_box: [(f64, f64); 3],
    safety: &SafetyDistances,
) -> PyResult<VisualizationHandle> {
    let initial_joints = joints_to_visual_degrees(initial_joints, robot.degrees)?;
    let visual_robot = clone_shape_robot(&robot.robot);
    let ik_robot = clone_shape_robot(&robot.robot);
    let tcp_box = tcp_box_to_ranges(tcp_box)?;
    let safety = safety.to_rs_safety();
    let handle =
        rs_visualize_robot_with_safety_async(visual_robot, initial_joints, tcp_box, &safety);

    Ok(VisualizationHandle {
        handle,
        robot: ik_robot,
        degrees: robot.degrees,
    })
}
