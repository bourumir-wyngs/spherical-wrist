use super::*;

/// OPW kinematics solver for inverse and direct kinematics.
///
/// This wrapper can also apply optional constraints, tool and base transforms,
/// working frames, parallelogram coupling, and external linear axes.
#[pyclass]
pub(crate) struct Robot {
    robot: OPWKinematics,
    pub(crate) degrees: bool,
    kinematic_model: KinematicModel,
    constraints: Option<Constraints>,
    parallelogram: Option<Parallelogram>,
    gantry_base: Option<Pose>,
    linear_axis: Option<LinearAxisConfig>,
    tool: Option<Pose>,
    base: Option<Pose>,
    frame: Option<RsFrameTransform>,
}

#[derive(Clone, Copy)]
struct LinearAxisConfig {
    axis: usize,
    base: Pose,
}

#[pymethods]
impl Robot {
    #[new]
    #[pyo3(signature = (
        kinematic_model,
        degrees=true,
        tool=None,
        base=None,
        frame=None,
        parallelogram=None,
        constraints=None,
        gantry_base=None,
        linear_axis_axis=None,
        linear_axis_base=None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        kinematic_model: KinematicModel,
        degrees: bool,
        tool: Option<[[f64; 4]; 4]>,
        base: Option<[[f64; 4]; 4]>,
        frame: Option<[[f64; 4]; 4]>,
        parallelogram: Option<Parallelogram>,
        constraints: Option<Constraints>,
        gantry_base: Option<[[f64; 4]; 4]>,
        linear_axis_axis: Option<usize>,
        linear_axis_base: Option<[[f64; 4]; 4]>,
    ) -> PyResult<Self> {
        if let Some(parallelogram) = parallelogram {
            parallelogram.validate()?;
        }
        let parameters = kinematic_model.to_parameters(degrees);
        let robot = match constraints {
            Some(constraints) => {
                OPWKinematics::new_with_constraints(parameters, constraints.to_rs_constraints())
            }
            None => OPWKinematics::new(parameters),
        };

        Ok(Robot {
            robot,
            degrees,
            kinematic_model,
            constraints,
            parallelogram,
            gantry_base: transpose_option(gantry_base, matrix_to_pose)?,
            linear_axis: transpose_option(linear_axis_axis, |axis| {
                validate_linear_axis(axis)?;
                Ok(LinearAxisConfig {
                    axis,
                    base: transpose_option(linear_axis_base, matrix_to_pose)?
                        .unwrap_or_else(Pose::identity),
                })
            })?,
            tool: transpose_option(tool, matrix_to_pose)?,
            base: transpose_option(base, matrix_to_pose)?,
            frame: transpose_option(frame, matrix_to_frame_transform)?,
        })
    }

    #[getter]
    fn degrees(&self) -> bool {
        self.degrees
    }

    #[getter]
    fn constraints(&self) -> Option<Constraints> {
        self.constraints
    }

    fn __repr__(&self) -> String {
        let model = self
            .kinematic_model
            .__repr__()
            .lines()
            .map(|line| format!("    {line}"))
            .collect::<Vec<_>>()
            .join("\n");
        format!(
            "Robot(\n    kinematic_model=\n{},\n    degrees={}\n)",
            model, self.degrees
        )
    }

    #[pyo3(signature = (joints, ee_transform=None, gantry_translation=None, linear_axis_distance=None))]
    fn forward(
        &self,
        joints: [f64; 6],
        ee_transform: Option<[[f64; 4]; 4]>,
        gantry_translation: Option<[f64; 3]>,
        linear_axis_distance: Option<f64>,
    ) -> PyResult<[[f64; 4]; 4]> {
        let joints = joints_to_internal(joints, self.degrees)?;
        let robot = self.build_robot(ee_transform)?;
        let pose = self.apply_external_axes(
            robot.forward(&joints),
            gantry_translation,
            linear_axis_distance,
        )?;

        Ok(pose_to_matrix(pose))
    }

    #[pyo3(signature = (pose, current_joints=None, ee_transform=None, gantry_translation=None, linear_axis_distance=None))]
    fn inverse(
        &self,
        pose: [[f64; 4]; 4],
        current_joints: Option<[f64; 6]>,
        ee_transform: Option<[[f64; 4]; 4]>,
        gantry_translation: Option<[f64; 3]>,
        linear_axis_distance: Option<f64>,
    ) -> PyResult<Vec<[f64; 6]>> {
        let target_pose = self.remove_external_axes(
            matrix_to_pose(pose)?,
            gantry_translation,
            linear_axis_distance,
        )?;
        let robot = self.build_robot(ee_transform)?;

        let solutions = match current_joints {
            Some(joints) => {
                let joints = previous_joints_to_internal(joints, self.degrees)?;
                robot.inverse_continuing(&target_pose, &joints)
            }
            None => robot.inverse(&target_pose),
        };

        Ok(solutions_from_internal(solutions, self.degrees))
    }

    #[pyo3(signature = (pose, previous_joints, ee_transform=None, gantry_translation=None, linear_axis_distance=None))]
    fn inverse_continuing(
        &self,
        pose: [[f64; 4]; 4],
        previous_joints: [f64; 6],
        ee_transform: Option<[[f64; 4]; 4]>,
        gantry_translation: Option<[f64; 3]>,
        linear_axis_distance: Option<f64>,
    ) -> PyResult<Vec<[f64; 6]>> {
        let target_pose = self.remove_external_axes(
            matrix_to_pose(pose)?,
            gantry_translation,
            linear_axis_distance,
        )?;
        let previous_joints = previous_joints_to_internal(previous_joints, self.degrees)?;
        let robot = self.build_robot(ee_transform)?;

        Ok(solutions_from_internal(
            robot.inverse_continuing(&target_pose, &previous_joints),
            self.degrees,
        ))
    }

    #[pyo3(signature = (pose, j6=0.0, ee_transform=None, gantry_translation=None, linear_axis_distance=None))]
    fn inverse_5dof(
        &self,
        pose: [[f64; 4]; 4],
        j6: f64,
        ee_transform: Option<[[f64; 4]; 4]>,
        gantry_translation: Option<[f64; 3]>,
        linear_axis_distance: Option<f64>,
    ) -> PyResult<Vec<[f64; 6]>> {
        let target_pose = self.remove_external_axes(
            matrix_to_pose(pose)?,
            gantry_translation,
            linear_axis_distance,
        )?;
        let j6 = angle_to_internal(j6, self.degrees, "j6")?;
        let robot = self.build_robot(ee_transform)?;

        Ok(solutions_from_internal(
            robot.inverse_5dof(&target_pose, j6),
            self.degrees,
        ))
    }

    #[pyo3(signature = (pose, previous_joints, ee_transform=None, gantry_translation=None, linear_axis_distance=None))]
    fn inverse_continuing_5dof(
        &self,
        pose: [[f64; 4]; 4],
        previous_joints: [f64; 6],
        ee_transform: Option<[[f64; 4]; 4]>,
        gantry_translation: Option<[f64; 3]>,
        linear_axis_distance: Option<f64>,
    ) -> PyResult<Vec<[f64; 6]>> {
        let target_pose = self.remove_external_axes(
            matrix_to_pose(pose)?,
            gantry_translation,
            linear_axis_distance,
        )?;
        let previous_joints = previous_joints_to_internal(previous_joints, self.degrees)?;
        let robot = self.build_robot(ee_transform)?;

        Ok(solutions_from_internal(
            robot.inverse_continuing_5dof(&target_pose, &previous_joints),
            self.degrees,
        ))
    }

    #[pyo3(signature = (joints, ee_transform=None, gantry_translation=None, linear_axis_distance=None))]
    fn forward_with_joint_poses(
        &self,
        joints: [f64; 6],
        ee_transform: Option<[[f64; 4]; 4]>,
        gantry_translation: Option<[f64; 3]>,
        linear_axis_distance: Option<f64>,
    ) -> PyResult<Vec<[[f64; 4]; 4]>> {
        let joints = joints_to_internal(joints, self.degrees)?;
        let robot = self.build_robot(ee_transform)?;
        robot
            .forward_with_joint_poses(&joints)
            .into_iter()
            .map(|pose| {
                self.apply_external_axes(pose, gantry_translation, linear_axis_distance)
                    .map(pose_to_matrix)
            })
            .collect()
    }

    #[pyo3(signature = (joints))]
    fn kinematic_singularity(&self, joints: [f64; 6]) -> PyResult<Option<String>> {
        let joints = joints_to_internal(joints, self.degrees)?;
        let robot = self.build_robot(None)?;

        Ok(robot
            .kinematic_singularity(&joints)
            .map(|Singularity::A| "A".to_string()))
    }
}

impl Robot {
    pub(crate) fn build_robot(
        &self,
        call_tool: Option<[[f64; 4]; 4]>,
    ) -> PyResult<Arc<dyn Kinematics>> {
        let mut robot: Arc<dyn Kinematics> = Arc::new(self.robot);

        if let Some(parallelogram) = self.parallelogram {
            robot = Arc::new(RsParallelogram {
                robot,
                scaling: parallelogram.scaling,
                driven: parallelogram.driven,
                coupled: parallelogram.coupled,
            });
        }

        if let Some(base) = self.base {
            robot = Arc::new(Base { robot, base });
        }

        let tool = match (self.tool, call_tool) {
            (Some(_), Some(_)) => {
                return Err(PyValueError::new_err(
                    "robot already has a constructor tool; pass either constructor tool or per-call tool/ee_transform",
                ));
            }
            (Some(tool), None) => Some(tool),
            (None, Some(tool)) => Some(matrix_to_pose(tool)?),
            (None, None) => None,
        };

        if let Some(tool) = tool {
            robot = Arc::new(Tool { robot, tool });
        }

        if let Some(frame) = self.frame {
            robot = Arc::new(RsFrame { robot, frame });
        }

        Ok(robot)
    }

    fn apply_external_axes(
        &self,
        mut pose: Pose,
        gantry_translation: Option<[f64; 3]>,
        linear_axis_distance: Option<f64>,
    ) -> PyResult<Pose> {
        if let Some(gantry) = self.gantry_transform(gantry_translation)? {
            pose = gantry * pose;
        }

        if let Some(linear_axis) = self.linear_axis_transform(linear_axis_distance)? {
            pose = linear_axis * pose;
        }

        Ok(pose)
    }

    fn remove_external_axes(
        &self,
        mut pose: Pose,
        gantry_translation: Option<[f64; 3]>,
        linear_axis_distance: Option<f64>,
    ) -> PyResult<Pose> {
        if let Some(linear_axis) = self.linear_axis_transform(linear_axis_distance)? {
            pose = linear_axis.inverse() * pose;
        }

        if let Some(gantry) = self.gantry_transform(gantry_translation)? {
            pose = gantry.inverse() * pose;
        }

        Ok(pose)
    }

    fn gantry_transform(&self, translation: Option<[f64; 3]>) -> PyResult<Option<Pose>> {
        match (self.gantry_base, translation) {
            (Some(base), Some(translation)) => Ok(Some(base * translation_pose(translation)?)),
            (Some(base), None) => Ok(Some(base)),
            (None, Some(_)) => Err(PyValueError::new_err(
                "gantry_translation requires a Robot configured with gantry",
            )),
            (None, None) => Ok(None),
        }
    }

    fn linear_axis_transform(&self, distance: Option<f64>) -> PyResult<Option<Pose>> {
        match (self.linear_axis, distance) {
            (Some(config), Some(distance)) => {
                if !distance.is_finite() {
                    return Err(PyValueError::new_err("linear_axis_distance must be finite"));
                }
                Ok(Some(config.base * linear_axis_pose(config.axis, distance)))
            }
            (Some(config), None) => Ok(Some(config.base)),
            (None, Some(_)) => Err(PyValueError::new_err(
                "linear_axis_distance requires a Robot configured with linear_axis",
            )),
            (None, None) => Ok(None),
        }
    }
}
