use super::*;

type PositionedRobotMatrices = (
    Vec<[[f64; 4]; 4]>,
    Option<[[f64; 4]; 4]>,
    Vec<[[f64; 4]; 4]>,
);

/// Combines the kinematic model of a robot with its geometrical shape.
///
/// This provides kinematic functionality for computing joint positions and the
/// physical structure used for collision detection and other geometric checks.
#[pyclass]
pub(crate) struct KinematicsWithShape {
    pub(crate) robot: RsKinematicsWithShape,
    pub(crate) degrees: bool,
    kinematic_model: KinematicModel,
    constraints: Option<Constraints>,
    parallelogram: Option<Parallelogram>,
}

#[pymethods]
impl KinematicsWithShape {
    #[new]
    #[pyo3(signature = (
        kinematic_model,
        degrees,
        joint_meshes,
        constraints=None,
        base=None,
        tool=None,
        parallelogram=None,
        base_mesh=None,
        tool_mesh=None,
        environment=None,
        safety=None,
        first_collision_only=false,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python<'_>,
        kinematic_model: KinematicModel,
        degrees: bool,
        joint_meshes: Vec<Mesh>,
        constraints: Option<Constraints>,
        base: Option<[[f64; 4]; 4]>,
        tool: Option<[[f64; 4]; 4]>,
        parallelogram: Option<Parallelogram>,
        base_mesh: Option<Mesh>,
        tool_mesh: Option<Mesh>,
        environment: Option<Vec<Mesh>>,
        safety: Option<SafetyDistances>,
        first_collision_only: bool,
    ) -> PyResult<Self> {
        if let Some(parallelogram) = parallelogram {
            parallelogram.validate()?;
        }

        let parameters = kinematic_model.to_parameters(degrees);
        let mut kinematics: Arc<dyn Kinematics> = match constraints {
            Some(constraints) => Arc::new(OPWKinematics::new_with_constraints(
                parameters,
                constraints.to_rs_constraints(),
            )),
            None => Arc::new(OPWKinematics::new(parameters)),
        };

        if let Some(parallelogram) = parallelogram {
            kinematics = Arc::new(RsParallelogram {
                robot: kinematics,
                scaling: parallelogram.scaling,
                driven: parallelogram.driven,
                coupled: parallelogram.coupled,
            });
        }

        let base = transpose_option(base, matrix_to_pose)?;
        if let Some(base) = base {
            kinematics = Arc::new(Base {
                robot: kinematics,
                base,
            });
        }

        let tool = transpose_option(tool, matrix_to_pose)?;
        if let Some(tool) = tool {
            kinematics = Arc::new(Tool {
                robot: kinematics,
                tool,
            });
        }

        let body = py.detach(move || {
            let base_body = base_mesh.map(|mesh| {
                let base_pose =
                    base.map(Pose::to_f32).unwrap_or_else(Pose32::identity) * mesh.pose32();
                BaseBody {
                    mesh: mesh.clone_trimesh(),
                    base_pose,
                }
            });
            let tool_body = tool_mesh.map(|mesh| mesh.local_trimesh());
            let collision_environment = environment
                .unwrap_or_default()
                .into_iter()
                .map(|mesh| RsCollisionBody {
                    mesh: mesh.clone_trimesh(),
                    pose: mesh.pose32(),
                })
                .collect();
            let safety = match safety {
                Some(safety) => safety.to_rs_safety(),
                None => RsSafetyDistances::standard(if first_collision_only {
                    CheckMode::FirstCollisionOnly
                } else {
                    CheckMode::AllCollsions
                }),
            };

            Ok::<RobotBody, PyErr>(RobotBody {
                joint_meshes: joint_mesh_array(joint_meshes)?,
                tool: tool_body,
                base: base_body,
                collision_environment,
                safety,
            })
        })?;

        Ok(Self {
            robot: RsKinematicsWithShape { kinematics, body },
            degrees,
            kinematic_model,
            constraints,
            parallelogram,
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

    #[getter]
    fn parallelogram(&self) -> Option<Parallelogram> {
        self.parallelogram
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
            "KinematicsWithShape(\n    kinematic_model=\n{},\n    degrees={},\n    joint_meshes=6,\n    environment={}\n)",
            model,
            self.degrees,
            self.robot.body.collision_environment.len()
        )
    }

    #[pyo3(signature = (joints))]
    fn forward(&self, joints: [f64; 6]) -> PyResult<[[f64; 4]; 4]> {
        let joints = joints_to_internal(joints, self.degrees)?;
        Ok(pose_to_matrix(self.robot.forward(&joints)))
    }

    #[pyo3(signature = (pose, current_joints=None))]
    fn inverse(
        &self,
        pose: [[f64; 4]; 4],
        current_joints: Option<[f64; 6]>,
    ) -> PyResult<Vec<[f64; 6]>> {
        let pose = matrix_to_pose(pose)?;
        let solutions = match current_joints {
            Some(joints) => {
                let joints = previous_joints_to_internal(joints, self.degrees)?;
                self.robot.inverse_continuing(&pose, &joints)
            }
            None => self.robot.inverse(&pose),
        };

        Ok(solutions_from_internal(solutions, self.degrees))
    }

    #[pyo3(signature = (pose, previous_joints))]
    fn inverse_continuing(
        &self,
        pose: [[f64; 4]; 4],
        previous_joints: [f64; 6],
    ) -> PyResult<Vec<[f64; 6]>> {
        let pose = matrix_to_pose(pose)?;
        let previous_joints = previous_joints_to_internal(previous_joints, self.degrees)?;

        Ok(solutions_from_internal(
            self.robot.inverse_continuing(&pose, &previous_joints),
            self.degrees,
        ))
    }

    #[pyo3(signature = (pose, j6=0.0))]
    fn inverse_5dof(&self, pose: [[f64; 4]; 4], j6: f64) -> PyResult<Vec<[f64; 6]>> {
        let pose = matrix_to_pose(pose)?;
        let j6 = angle_to_internal(j6, self.degrees, "j6")?;

        Ok(solutions_from_internal(
            self.robot.inverse_5dof(&pose, j6),
            self.degrees,
        ))
    }

    #[pyo3(signature = (pose, previous_joints))]
    fn inverse_continuing_5dof(
        &self,
        pose: [[f64; 4]; 4],
        previous_joints: [f64; 6],
    ) -> PyResult<Vec<[f64; 6]>> {
        let pose = matrix_to_pose(pose)?;
        let previous_joints = previous_joints_to_internal(previous_joints, self.degrees)?;

        Ok(solutions_from_internal(
            self.robot.inverse_continuing_5dof(&pose, &previous_joints),
            self.degrees,
        ))
    }

    #[pyo3(signature = (joints))]
    fn forward_with_joint_poses(&self, joints: [f64; 6]) -> PyResult<Vec<[[f64; 4]; 4]>> {
        let joints = joints_to_internal(joints, self.degrees)?;
        Ok(self
            .robot
            .forward_with_joint_poses(&joints)
            .into_iter()
            .map(pose_to_matrix)
            .collect())
    }

    #[pyo3(signature = (joints))]
    fn kinematic_singularity(&self, joints: [f64; 6]) -> PyResult<Option<String>> {
        let joints = joints_to_internal(joints, self.degrees)?;

        Ok(self
            .robot
            .kinematic_singularity(&joints)
            .map(|Singularity::A| "A".to_string()))
    }

    #[pyo3(signature = (joints))]
    fn collides(&self, py: Python<'_>, joints: [f64; 6]) -> PyResult<bool> {
        let joints = joints_to_internal(joints, self.degrees)?;
        Ok(py.detach(|| self.robot.collides(&joints)))
    }

    #[pyo3(signature = (joints))]
    fn collision_details(&self, py: Python<'_>, joints: [f64; 6]) -> PyResult<Vec<(usize, usize)>> {
        let joints = joints_to_internal(joints, self.degrees)?;
        Ok(py.detach(|| self.robot.collision_details(&joints)))
    }

    #[pyo3(signature = (joints, safety))]
    fn near(
        &self,
        py: Python<'_>,
        joints: [f64; 6],
        safety: &SafetyDistances,
    ) -> PyResult<Vec<(usize, usize)>> {
        let joints = joints_to_internal(joints, self.degrees)?;
        let safety = safety.to_rs_safety();
        Ok(py.detach(|| self.robot.near(&joints, &safety)))
    }

    #[pyo3(signature = (joints, from_limits, to_limits))]
    fn non_colliding_offsets(
        &self,
        py: Python<'_>,
        joints: [f64; 6],
        from_limits: [f64; 6],
        to_limits: [f64; 6],
    ) -> PyResult<Vec<[f64; 6]>> {
        let joints = joints_to_internal(joints, self.degrees)?;
        let from_limits = joints_to_internal(from_limits, self.degrees)?;
        let to_limits = joints_to_internal(to_limits, self.degrees)?;

        let solutions = py.detach(|| {
            self.robot
                .non_colliding_offsets(&joints, &from_limits, &to_limits)
        });
        Ok(solutions_from_internal(solutions, self.degrees))
    }

    #[pyo3(signature = (joints))]
    fn positioned_robot(&self, joints: [f64; 6]) -> PyResult<PositionedRobotMatrices> {
        let joints = joints_to_internal(joints, self.degrees)?;
        let positioned = self.robot.positioned_robot(&joints);
        let joints = positioned
            .joints
            .iter()
            .map(|joint| pose32_to_matrix(joint.transform))
            .collect();
        let tool = positioned
            .tool
            .map(|joint| pose32_to_matrix(joint.transform));
        let environment = positioned
            .environment
            .into_iter()
            .map(|body| pose32_to_matrix(body.pose))
            .collect();

        Ok((joints, tool, environment))
    }
}
