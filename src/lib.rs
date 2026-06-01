use glam::{DMat3, DQuat, DVec3, Mat3, Vec3};
use parry3d::math::Pose as ParryPose;
use parry3d::shape::TriMesh;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use rs_opw_kinematics::cartesian::{
    AnnotatedJoints as RsAnnotatedJoints, Cartesian as RsCartesian,
    DEFAULT_RECONFIGURATION_PREFIX_CANDIDATES, DEFAULT_TRANSITION_COSTS, MoveKind as RsMoveKind,
    PathFlags,
};
use rs_opw_kinematics::collisions::{
    BaseBody, CheckMode, CollisionBody as RsCollisionBody, NEVER_COLLIDES, RobotBody,
    SafetyDistances as RsSafetyDistances, TOUCH_ONLY, transform_mesh,
};
use rs_opw_kinematics::constraints::{BY_CONSTRAINS, BY_PREV, Constraints as RsConstraints};
use rs_opw_kinematics::frame::Frame;
use rs_opw_kinematics::kinematic_traits::{
    CONSTRAINT_CENTERED, ENV_START_IDX, J_BASE, J_TOOL, J1, J2, J3, J4, J5, J6, Joints, Kinematics,
    Pose, Singularity,
};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape as RsKinematicsWithShape;
use rs_opw_kinematics::parallelogram::Parallelogram as RsParallelogram;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::pose::Pose32;
use rs_opw_kinematics::rrt::RRTPlanner as RsRRTPlanner;
use rs_opw_kinematics::tool::{Base, Tool};
use rs_opw_kinematics::visualization::{
    VisualizationHandle as RsVisualizationHandle,
    visualize_robot_async as rs_visualize_robot_async,
    visualize_robot_with_safety_async as rs_visualize_robot_with_safety_async,
};
use rs_read_trimesh::load_trimesh;
use std::ops::RangeInclusive;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

#[pyclass(frozen)]
#[derive(Clone)]
struct KinematicModel {
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
    fn to_parameters(&self, degrees: bool) -> Parameters {
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

    fn __repr__(&self) -> String {
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

#[pyclass(frozen)]
#[derive(Clone, Copy)]
struct Constraints {
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
    fn from_limits(&self, radians: bool) -> [f64; 6] {
        angles_from_radians(self.constraints.from, radians)
    }

    #[pyo3(signature = (radians=false))]
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
    fn to_rs_constraints(self) -> RsConstraints {
        self.constraints
    }
}

#[pyclass(frozen)]
#[derive(Clone)]
struct SafetyDistances {
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
    fn to_rs_safety(&self) -> RsSafetyDistances {
        self.safety.clone()
    }
}

#[pyclass(frozen)]
#[derive(Clone, Copy)]
struct Parallelogram {
    scaling: f64,
    driven: usize,
    coupled: usize,
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
    fn validate(&self) -> PyResult<()> {
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

#[pyclass(frozen)]
#[derive(Clone)]
struct Mesh {
    mesh: Arc<TriMesh>,
    pose: ParryPose,
}

#[pymethods]
impl Mesh {
    #[new]
    #[pyo3(signature = (path, scale=1.0, pose=None))]
    fn new(path: &str, scale: f64, pose: Option<[[f64; 4]; 4]>) -> PyResult<Self> {
        Self::from_path_with_pose(path, scale, pose)
    }

    #[staticmethod]
    #[pyo3(signature = (path, scale=1.0))]
    fn from_path(path: &str, scale: f64) -> PyResult<Self> {
        Self::from_path_with_pose(path, scale, None)
    }

    #[staticmethod]
    #[pyo3(signature = (vertices, triangles, pose=None))]
    fn from_arrays(
        vertices: Vec<(f64, f64, f64)>,
        triangles: Vec<(u32, u32, u32)>,
        pose: Option<[[f64; 4]; 4]>,
    ) -> PyResult<Self> {
        Self::from_vertices_and_triangles(vertices, triangles, pose)
    }

    #[getter]
    fn vertex_count(&self) -> usize {
        self.mesh.vertices().len()
    }

    #[getter]
    fn triangle_count(&self) -> usize {
        self.mesh.indices().len()
    }

    #[pyo3(signature = (pose))]
    fn transformed(&self, pose: [[f64; 4]; 4]) -> PyResult<Self> {
        Ok(Self {
            mesh: Arc::clone(&self.mesh),
            pose: matrix_to_parry_pose(pose)? * self.pose,
        })
    }

    #[pyo3(signature = (pose))]
    fn transformed_by(&self, pose: [[f64; 4]; 4]) -> PyResult<Self> {
        self.transformed(pose)
    }

    #[pyo3(signature = (other, safety_distance=0.0))]
    fn collides(&self, other: &Mesh, safety_distance: f64) -> PyResult<bool> {
        let safety_distance = validate_nonnegative_f32(safety_distance, "safety_distance")?;
        let intersects = parry3d::query::intersection_test(
            &self.pose,
            self.mesh.as_ref(),
            &other.pose,
            other.mesh.as_ref(),
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        if intersects || safety_distance == 0.0 {
            return Ok(intersects);
        }

        let distance = parry3d::query::distance(
            &self.pose,
            self.mesh.as_ref(),
            &other.pose,
            other.mesh.as_ref(),
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        Ok(distance <= safety_distance)
    }

    fn __repr__(&self) -> String {
        format!(
            "Mesh(vertex_count={}, triangle_count={})",
            self.vertex_count(),
            self.triangle_count()
        )
    }
}

impl Mesh {
    fn from_path_with_pose(path: &str, scale: f64, pose: Option<[[f64; 4]; 4]>) -> PyResult<Self> {
        let scale = validate_positive_f32(scale, "scale")?;
        let mesh = load_trimesh(path, scale).map_err(PyValueError::new_err)?;

        Ok(Self {
            mesh: Arc::new(mesh),
            pose: transpose_option(pose, matrix_to_parry_pose)?.unwrap_or_else(ParryPose::identity),
        })
    }

    fn from_vertices_and_triangles(
        vertices: Vec<(f64, f64, f64)>,
        triangles: Vec<(u32, u32, u32)>,
        pose: Option<[[f64; 4]; 4]>,
    ) -> PyResult<Self> {
        let vertices = vertices
            .into_iter()
            .map(|(x, y, z)| {
                Ok(Vec3::new(
                    validate_f32(x, "vertices")?,
                    validate_f32(y, "vertices")?,
                    validate_f32(z, "vertices")?,
                ))
            })
            .collect::<PyResult<Vec<_>>>()?;
        let triangles = triangles
            .into_iter()
            .map(|(a, b, c)| [a, b, c])
            .collect::<Vec<_>>();
        let mesh = TriMesh::new(vertices, triangles)
            .map_err(|err| PyValueError::new_err(err.to_string()))?;

        Ok(Self {
            mesh: Arc::new(mesh),
            pose: transpose_option(pose, matrix_to_parry_pose)?.unwrap_or_else(ParryPose::identity),
        })
    }

    fn clone_trimesh(&self) -> TriMesh {
        self.mesh.as_ref().clone()
    }

    fn local_trimesh(&self) -> TriMesh {
        transform_mesh(self.mesh.as_ref(), &self.pose32())
    }

    fn pose32(&self) -> Pose32 {
        Pose32::from_parts(self.pose.translation, self.pose.rotation)
    }
}

#[pyclass]
struct KinematicsWithShape {
    robot: RsKinematicsWithShape,
    degrees: bool,
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

        let base_body = base_mesh.map(|mesh| {
            let base_pose = base.map(Pose::to_f32).unwrap_or_else(Pose32::identity) * mesh.pose32();
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

        Ok(Self {
            robot: RsKinematicsWithShape {
                kinematics,
                body: RobotBody {
                    joint_meshes: joint_mesh_array(joint_meshes)?,
                    tool: tool_body,
                    base: base_body,
                    collision_environment,
                    safety,
                },
            },
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

        Ok(match self.robot.kinematic_singularity(&joints) {
            Some(Singularity::A) => Some("A".to_string()),
            None => None,
        })
    }

    #[pyo3(signature = (joints))]
    fn collides(&self, joints: [f64; 6]) -> PyResult<bool> {
        let joints = joints_to_internal(joints, self.degrees)?;
        Ok(self.robot.collides(&joints))
    }

    #[pyo3(signature = (joints))]
    fn collision_details(&self, joints: [f64; 6]) -> PyResult<Vec<(usize, usize)>> {
        let joints = joints_to_internal(joints, self.degrees)?;
        Ok(self.robot.collision_details(&joints))
    }

    #[pyo3(signature = (joints, safety))]
    fn near(&self, joints: [f64; 6], safety: &SafetyDistances) -> PyResult<Vec<(usize, usize)>> {
        let joints = joints_to_internal(joints, self.degrees)?;
        Ok(self.robot.near(&joints, &safety.to_rs_safety()))
    }

    #[pyo3(signature = (joints, from_limits, to_limits))]
    fn non_colliding_offsets(
        &self,
        joints: [f64; 6],
        from_limits: [f64; 6],
        to_limits: [f64; 6],
    ) -> PyResult<Vec<[f64; 6]>> {
        let joints = joints_to_internal(joints, self.degrees)?;
        let from_limits = joints_to_internal(from_limits, self.degrees)?;
        let to_limits = joints_to_internal(to_limits, self.degrees)?;

        Ok(solutions_from_internal(
            self.robot
                .non_colliding_offsets(&joints, &from_limits, &to_limits),
            self.degrees,
        ))
    }

    #[pyo3(signature = (joints))]
    fn positioned_robot(
        &self,
        joints: [f64; 6],
    ) -> PyResult<(
        Vec<[[f64; 4]; 4]>,
        Option<[[f64; 4]; 4]>,
        Vec<[[f64; 4]; 4]>,
    )> {
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

#[pyclass(frozen)]
#[derive(Clone, Copy)]
struct RRTPlanner {
    step_size_joint_space: f64,
    max_try: usize,
    debug: bool,
}

#[pymethods]
impl RRTPlanner {
    #[new]
    #[pyo3(signature = (step_size_joint_space=3.0, max_try=2000, debug=false, radians=false))]
    fn new(
        step_size_joint_space: f64,
        max_try: usize,
        debug: bool,
        radians: bool,
    ) -> PyResult<Self> {
        let step_size_joint_space =
            positive_angle_to_internal(step_size_joint_space, radians, "step_size_joint_space")?;
        if max_try == 0 {
            return Err(PyValueError::new_err("max_try must be positive"));
        }

        Ok(Self {
            step_size_joint_space,
            max_try,
            debug,
        })
    }

    #[pyo3(signature = (radians=false))]
    fn step_size_joint_space(&self, radians: bool) -> f64 {
        angle_from_internal(self.step_size_joint_space, radians)
    }

    #[getter]
    fn max_try(&self) -> usize {
        self.max_try
    }

    #[getter]
    fn debug(&self) -> bool {
        self.debug
    }

    #[pyo3(signature = (robot, start, goal))]
    fn plan_rrt(
        &self,
        robot: &KinematicsWithShape,
        start: [f64; 6],
        goal: [f64; 6],
    ) -> PyResult<Vec<[f64; 6]>> {
        if robot.robot.constraints().is_none() {
            return Err(PyValueError::new_err(
                "RRT planning requires KinematicsWithShape configured with Constraints",
            ));
        }

        let start = joints_to_internal(start, robot.degrees)?;
        let goal = joints_to_internal(goal, robot.degrees)?;
        let stop = AtomicBool::new(false);
        let path = self
            .to_rs_rrt()
            .plan_rrt(&start, &goal, &robot.robot, &stop)
            .map_err(PyValueError::new_err)?;

        Ok(solutions_from_internal(path, robot.degrees))
    }

    fn __repr__(&self) -> String {
        format!(
            "RRTPlanner(step_size_joint_space={}, max_try={}, debug={})",
            self.step_size_joint_space(false),
            self.max_try,
            self.debug
        )
    }
}

impl Default for RRTPlanner {
    fn default() -> Self {
        let planner = RsRRTPlanner::default();
        Self {
            step_size_joint_space: planner.step_size_joint_space,
            max_try: planner.max_try,
            debug: planner.debug,
        }
    }
}

impl RRTPlanner {
    fn to_rs_rrt(self) -> RsRRTPlanner {
        RsRRTPlanner {
            step_size_joint_space: self.step_size_joint_space,
            max_try: self.max_try,
            debug: self.debug,
        }
    }
}

#[pyclass(frozen)]
#[derive(Clone)]
struct CartesianPlanner {
    check_step_m: f64,
    check_step_rad: f64,
    max_transition_cost: f64,
    transition_coefficients: Joints,
    linear_recursion_depth: usize,
    rrt: RRTPlanner,
    allow_reconfigure: bool,
    include_linear_interpolation: bool,
    debug: bool,
}

#[pymethods]
impl CartesianPlanner {
    #[new]
    #[pyo3(signature = (
        check_step_m=0.02,
        check_step_rad=3.0,
        max_transition_cost=3.0,
        transition_coefficients=None,
        linear_recursion_depth=8,
        rrt=None,
        allow_reconfigure=true,
        include_linear_interpolation=true,
        debug=false,
        radians=false,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        check_step_m: f64,
        check_step_rad: f64,
        max_transition_cost: f64,
        transition_coefficients: Option<[f64; 6]>,
        linear_recursion_depth: usize,
        rrt: Option<RRTPlanner>,
        allow_reconfigure: bool,
        include_linear_interpolation: bool,
        debug: bool,
        radians: bool,
    ) -> PyResult<Self> {
        if !check_step_m.is_finite() || check_step_m <= 0.0 {
            return Err(PyValueError::new_err("check_step_m must be positive"));
        }

        let transition_coefficients = transition_coefficients.unwrap_or(DEFAULT_TRANSITION_COSTS);
        validate_joints(&transition_coefficients)?;

        Ok(Self {
            check_step_m,
            check_step_rad: positive_angle_to_internal(check_step_rad, radians, "check_step_rad")?,
            max_transition_cost: positive_angle_to_internal(
                max_transition_cost,
                radians,
                "max_transition_cost",
            )?,
            transition_coefficients,
            linear_recursion_depth,
            rrt: rrt.unwrap_or_default(),
            allow_reconfigure,
            include_linear_interpolation,
            debug,
        })
    }

    #[getter]
    fn check_step_m(&self) -> f64 {
        self.check_step_m
    }

    #[pyo3(signature = (radians=false))]
    fn check_step_rad(&self, radians: bool) -> f64 {
        angle_from_internal(self.check_step_rad, radians)
    }

    #[pyo3(signature = (radians=false))]
    fn max_transition_cost(&self, radians: bool) -> f64 {
        angle_from_internal(self.max_transition_cost, radians)
    }

    #[getter]
    fn transition_coefficients(&self) -> [f64; 6] {
        self.transition_coefficients
    }

    #[getter]
    fn linear_recursion_depth(&self) -> usize {
        self.linear_recursion_depth
    }

    #[getter]
    fn rrt(&self) -> RRTPlanner {
        self.rrt
    }

    #[getter]
    fn allow_reconfigure(&self) -> bool {
        self.allow_reconfigure
    }

    #[getter]
    fn include_linear_interpolation(&self) -> bool {
        self.include_linear_interpolation
    }

    #[getter]
    fn debug(&self) -> bool {
        self.debug
    }

    #[pyo3(signature = (robot, start, land, steps, park))]
    fn plan(
        &self,
        robot: &KinematicsWithShape,
        start: [f64; 6],
        land: [[f64; 4]; 4],
        steps: Vec<[[f64; 4]; 4]>,
        park: [[f64; 4]; 4],
    ) -> PyResult<Vec<AnnotatedJoints>> {
        if robot.robot.constraints().is_none() {
            return Err(PyValueError::new_err(
                "Cartesian planning requires KinematicsWithShape configured with Constraints",
            ));
        }

        let start = joints_to_internal(start, robot.degrees)?;
        let land = matrix_to_pose(land)?;
        let steps = steps
            .into_iter()
            .map(matrix_to_pose)
            .collect::<PyResult<Vec<_>>>()?;
        let park = matrix_to_pose(park)?;
        let planner = RsCartesian {
            robot: &robot.robot,
            check_step_m: self.check_step_m,
            check_step_rad: self.check_step_rad,
            max_transition_cost: self.max_transition_cost,
            transition_coefficients: self.transition_coefficients,
            linear_recursion_depth: self.linear_recursion_depth,
            rrt: self.rrt.to_rs_rrt(),
            allow_reconfigure: self.allow_reconfigure,
            max_reconfiguration_prefix_candidates: DEFAULT_RECONFIGURATION_PREFIX_CANDIDATES,
            include_linear_interpolation: self.include_linear_interpolation,
            debug: self.debug,
        };

        planner
            .plan(&start, &land, steps, &park)
            .map(|path| annotated_joints_from_internal(path, robot.degrees))
            .map_err(PyValueError::new_err)
    }

    fn __repr__(&self) -> String {
        format!(
            "CartesianPlanner(check_step_m={}, check_step_rad={}, max_transition_cost={}, linear_recursion_depth={}, allow_reconfigure={}, include_linear_interpolation={}, debug={})",
            self.check_step_m,
            self.check_step_rad(false),
            self.max_transition_cost(false),
            self.linear_recursion_depth,
            self.allow_reconfigure,
            self.include_linear_interpolation,
            self.debug
        )
    }
}

#[pyclass(frozen)]
#[derive(Clone)]
struct AnnotatedJoints {
    joints: [f64; 6],
    flags: u32,
    move_into: String,
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

#[pyclass]
struct VisualizationHandle {
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
fn visualize_robot(
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
fn visualize_robot_with_safety(
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

#[pyclass]
struct Robot {
    robot: OPWKinematics,
    degrees: bool,
    kinematic_model: KinematicModel,
    constraints: Option<Constraints>,
    parallelogram: Option<Parallelogram>,
    gantry_base: Option<Pose>,
    linear_axis: Option<LinearAxisConfig>,
    tool: Option<Pose>,
    base: Option<Pose>,
    frame: Option<Pose>,
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
            frame: transpose_option(frame, matrix_to_pose)?,
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

        Ok(match robot.kinematic_singularity(&joints) {
            Some(Singularity::A) => Some("A".to_string()),
            None => None,
        })
    }
}

impl Robot {
    fn build_robot(&self, call_tool: Option<[[f64; 4]; 4]>) -> PyResult<Arc<dyn Kinematics>> {
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
            robot = Arc::new(Frame { robot, frame });
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

fn joint_mesh_array(meshes: Vec<Mesh>) -> PyResult<[TriMesh; 6]> {
    if meshes.len() != 6 {
        return Err(PyValueError::new_err(
            "joint_meshes must contain exactly 6 meshes",
        ));
    }

    let meshes = meshes
        .into_iter()
        .map(|mesh| mesh.local_trimesh())
        .collect::<Vec<_>>();
    match meshes.try_into() {
        Ok(meshes) => Ok(meshes),
        Err(_) => unreachable!("joint_meshes length is validated"),
    }
}

fn parse_check_mode(mode: &str) -> PyResult<CheckMode> {
    let normalized = mode.trim().to_ascii_lowercase().replace('-', "_");
    match normalized.as_str() {
        "first" | "first_collision_only" => Ok(CheckMode::FirstCollisionOnly),
        "all" | "all_collisions" | "all_collsions" => Ok(CheckMode::AllCollsions),
        "none" | "no_check" | "off" => Ok(CheckMode::NoCheck),
        _ => Err(PyValueError::new_err(
            "mode must be 'all', 'first_collision_only', or 'no_check'",
        )),
    }
}

fn check_mode_name(mode: CheckMode) -> &'static str {
    match mode {
        CheckMode::FirstCollisionOnly => "first_collision_only",
        CheckMode::AllCollsions => "all",
        CheckMode::NoCheck => "no_check",
    }
}

fn validate_collision_index(index: usize) -> PyResult<()> {
    if index > u16::MAX as usize {
        return Err(PyValueError::new_err(
            "collision indices must fit in an unsigned 16-bit value",
        ));
    }
    Ok(())
}

fn validate_safety_distance(value: f64, name: &str) -> PyResult<f32> {
    let value = validate_f32(value, name)?;
    if value > NEVER_COLLIDES && value < TOUCH_ONLY {
        return Err(PyValueError::new_err(format!(
            "{name} must be non-negative or NEVER_COLLIDES"
        )));
    }
    Ok(value)
}

fn clone_shape_robot(robot: &RsKinematicsWithShape) -> RsKinematicsWithShape {
    RsKinematicsWithShape {
        kinematics: Arc::clone(&robot.kinematics),
        body: RobotBody {
            joint_meshes: robot.body.joint_meshes.clone(),
            tool: robot.body.tool.clone(),
            base: robot.body.base.as_ref().map(|base| BaseBody {
                mesh: base.mesh.clone(),
                base_pose: base.base_pose,
            }),
            collision_environment: robot
                .body
                .collision_environment
                .iter()
                .map(|body| RsCollisionBody {
                    mesh: body.mesh.clone(),
                    pose: body.pose,
                })
                .collect(),
            safety: robot.body.safety.clone(),
        },
    }
}

fn joints_to_visual_degrees(joints: [f64; 6], input_degrees: bool) -> PyResult<[f32; 6]> {
    validate_joints(&joints)?;
    let mut converted = [0.0; 6];
    for (target, value) in converted.iter_mut().zip(joints) {
        let degrees = if input_degrees {
            value
        } else {
            value.to_degrees()
        };
        *target = validate_f32(degrees, "initial_joints")?;
    }
    Ok(converted)
}

fn tcp_box_to_ranges(tcp_box: [(f64, f64); 3]) -> PyResult<[RangeInclusive<f64>; 3]> {
    Ok([
        tcp_range(tcp_box[0], "tcp_box[0]")?,
        tcp_range(tcp_box[1], "tcp_box[1]")?,
        tcp_range(tcp_box[2], "tcp_box[2]")?,
    ])
}

fn tcp_range((from, to): (f64, f64), name: &str) -> PyResult<RangeInclusive<f64>> {
    if !from.is_finite() || !to.is_finite() {
        return Err(PyValueError::new_err(format!(
            "{name} values must be finite"
        )));
    }
    if from > to {
        return Err(PyValueError::new_err(format!(
            "{name} lower bound must not exceed upper bound"
        )));
    }
    Ok(from..=to)
}

fn validate_joints(joints: &Joints) -> PyResult<()> {
    if joints.iter().any(|joint| !joint.is_finite()) {
        return Err(PyValueError::new_err("joint values must be finite"));
    }
    Ok(())
}

fn validate_previous_joints(joints: &Joints) -> PyResult<()> {
    if joints[0].is_nan() {
        if joints[1..].iter().any(|joint| !joint.is_finite()) {
            return Err(PyValueError::new_err(
                "CONSTRAINT_CENTERED previous joints marker must have finite values after J1",
            ));
        }
        return Ok(());
    }

    validate_joints(joints)
}

fn joints_to_internal(mut joints: [f64; 6], degrees: bool) -> PyResult<[f64; 6]> {
    validate_joints(&joints)?;
    if degrees {
        joints
            .iter_mut()
            .for_each(|joint| *joint = joint.to_radians());
    }
    Ok(joints)
}

fn previous_joints_to_internal(mut joints: [f64; 6], degrees: bool) -> PyResult<[f64; 6]> {
    validate_previous_joints(&joints)?;
    if degrees {
        joints
            .iter_mut()
            .for_each(|joint| *joint = joint.to_radians());
    }
    Ok(joints)
}

fn angle_to_internal(angle: f64, degrees: bool, name: &str) -> PyResult<f64> {
    if !angle.is_finite() {
        return Err(PyValueError::new_err(format!("{name} must be finite")));
    }
    Ok(if degrees { angle.to_radians() } else { angle })
}

fn positive_angle_to_internal(angle: f64, radians: bool, name: &str) -> PyResult<f64> {
    if !angle.is_finite() || angle <= 0.0 {
        return Err(PyValueError::new_err(format!("{name} must be positive")));
    }
    Ok(if radians { angle } else { angle.to_radians() })
}

fn angle_from_internal(angle: f64, radians: bool) -> f64 {
    if radians { angle } else { angle.to_degrees() }
}

fn solutions_from_internal(mut solutions: Vec<[f64; 6]>, degrees: bool) -> Vec<[f64; 6]> {
    if degrees {
        for solution in &mut solutions {
            solution
                .iter_mut()
                .for_each(|joint| *joint = joint.to_degrees());
        }
    }
    solutions
}

fn annotated_joints_from_internal(
    joints: Vec<RsAnnotatedJoints>,
    degrees: bool,
) -> Vec<AnnotatedJoints> {
    joints
        .into_iter()
        .map(|step| AnnotatedJoints {
            joints: angles_from_radians(step.joints, !degrees),
            flags: step.flags.bits(),
            move_into: match step.move_into {
                RsMoveKind::Joint => "joint",
                RsMoveKind::Cartesian => "cartesian",
            }
            .to_string(),
        })
        .collect()
}

fn angles_to_radians(mut values: [f64; 6], radians: bool) -> PyResult<[f64; 6]> {
    validate_joints(&values)?;
    if !radians {
        values = values.map(f64::to_radians);
    }
    Ok(values)
}

fn angles_from_radians(values: [f64; 6], radians: bool) -> [f64; 6] {
    if radians {
        values
    } else {
        values.map(f64::to_degrees)
    }
}

fn matrix_to_pose(matrix: [[f64; 4]; 4]) -> PyResult<Pose> {
    if matrix.into_iter().flatten().any(|value| !value.is_finite()) {
        return Err(PyValueError::new_err("pose matrix values must be finite"));
    }

    let rotation = DMat3::from_cols(
        DVec3::new(matrix[0][0], matrix[1][0], matrix[2][0]),
        DVec3::new(matrix[0][1], matrix[1][1], matrix[2][1]),
        DVec3::new(matrix[0][2], matrix[1][2], matrix[2][2]),
    );
    let translation = DVec3::new(matrix[0][3], matrix[1][3], matrix[2][3]);

    Ok(Pose::from_parts(translation, DQuat::from_mat3(&rotation)))
}

fn matrix_to_parry_pose(matrix: [[f64; 4]; 4]) -> PyResult<ParryPose> {
    let rotation = Mat3::from_cols(
        Vec3::new(
            validate_f32(matrix[0][0], "pose matrix")?,
            validate_f32(matrix[1][0], "pose matrix")?,
            validate_f32(matrix[2][0], "pose matrix")?,
        ),
        Vec3::new(
            validate_f32(matrix[0][1], "pose matrix")?,
            validate_f32(matrix[1][1], "pose matrix")?,
            validate_f32(matrix[2][1], "pose matrix")?,
        ),
        Vec3::new(
            validate_f32(matrix[0][2], "pose matrix")?,
            validate_f32(matrix[1][2], "pose matrix")?,
            validate_f32(matrix[2][2], "pose matrix")?,
        ),
    );
    let translation = Vec3::new(
        validate_f32(matrix[0][3], "pose matrix")?,
        validate_f32(matrix[1][3], "pose matrix")?,
        validate_f32(matrix[2][3], "pose matrix")?,
    );

    for value in matrix[3] {
        validate_f32(value, "pose matrix")?;
    }

    Ok(ParryPose::from_parts(
        translation,
        glam::Quat::from_mat3(&rotation),
    ))
}

fn validate_f32(value: f64, name: &str) -> PyResult<f32> {
    let value = value as f32;
    if !value.is_finite() {
        return Err(PyValueError::new_err(format!(
            "{name} values must be finite f32 values"
        )));
    }
    Ok(value)
}

fn validate_positive_f32(value: f64, name: &str) -> PyResult<f32> {
    let value = validate_f32(value, name)?;
    if value <= 0.0 {
        return Err(PyValueError::new_err(format!("{name} must be positive")));
    }
    Ok(value)
}

fn validate_nonnegative_f32(value: f64, name: &str) -> PyResult<f32> {
    let value = validate_f32(value, name)?;
    if value < 0.0 {
        return Err(PyValueError::new_err(format!(
            "{name} must be non-negative"
        )));
    }
    Ok(value)
}

fn validate_linear_axis(axis: usize) -> PyResult<()> {
    if axis >= 3 {
        return Err(PyValueError::new_err(
            "linear axis index must be in the range 0..3",
        ));
    }
    Ok(())
}

fn translation_pose(translation: [f64; 3]) -> PyResult<Pose> {
    if translation.iter().any(|value| !value.is_finite()) {
        return Err(PyValueError::new_err("translation values must be finite"));
    }
    Ok(Pose::from_translation(DVec3::new(
        translation[0],
        translation[1],
        translation[2],
    )))
}

fn linear_axis_pose(axis: usize, distance: f64) -> Pose {
    match axis {
        0 => Pose::from_translation(DVec3::new(distance, 0.0, 0.0)),
        1 => Pose::from_translation(DVec3::new(0.0, distance, 0.0)),
        2 => Pose::from_translation(DVec3::new(0.0, 0.0, distance)),
        _ => unreachable!("linear axis is validated before use"),
    }
}

fn transpose_option<T, U, F>(value: Option<T>, f: F) -> PyResult<Option<U>>
where
    F: FnOnce(T) -> PyResult<U>,
{
    value.map(f).transpose()
}

fn pose_to_matrix(pose: Pose) -> [[f64; 4]; 4] {
    let rotation = DMat3::from_quat(pose.rotation);
    [
        [
            rotation.x_axis.x,
            rotation.y_axis.x,
            rotation.z_axis.x,
            pose.translation.x,
        ],
        [
            rotation.x_axis.y,
            rotation.y_axis.y,
            rotation.z_axis.y,
            pose.translation.y,
        ],
        [
            rotation.x_axis.z,
            rotation.y_axis.z,
            rotation.z_axis.z,
            pose.translation.z,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn pose32_to_matrix(pose: Pose32) -> [[f64; 4]; 4] {
    let rotation = Mat3::from_quat(pose.rotation);
    [
        [
            rotation.x_axis.x as f64,
            rotation.y_axis.x as f64,
            rotation.z_axis.x as f64,
            pose.translation.x as f64,
        ],
        [
            rotation.x_axis.y as f64,
            rotation.y_axis.y as f64,
            rotation.z_axis.y as f64,
            pose.translation.y as f64,
        ],
        [
            rotation.x_axis.z as f64,
            rotation.y_axis.z as f64,
            rotation.z_axis.z as f64,
            pose.translation.z as f64,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn array_to_tuple(values: [f64; 6]) -> (f64, f64, f64, f64, f64, f64) {
    let [a, b, c, d, e, f] = values;
    (a, b, c, d, e, f)
}

#[pymodule(name = "_internal")]
fn spherical_wrist(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("BY_PREV", BY_PREV)?;
    m.add("BY_CONSTRAINTS", BY_CONSTRAINS)?;
    m.add("CONSTRAINT_CENTERED", CONSTRAINT_CENTERED)?;
    m.add("NEVER_COLLIDES", NEVER_COLLIDES)?;
    m.add("TOUCH_ONLY", TOUCH_ONLY)?;
    m.add("J1", J1)?;
    m.add("J2", J2)?;
    m.add("J3", J3)?;
    m.add("J4", J4)?;
    m.add("J5", J5)?;
    m.add("J6", J6)?;
    m.add("J_TOOL", J_TOOL)?;
    m.add("J_BASE", J_BASE)?;
    m.add("ENV_START_IDX", ENV_START_IDX)?;
    m.add("CHECK_MODE_ALL", "all")?;
    m.add("CHECK_MODE_FIRST_COLLISION_ONLY", "first_collision_only")?;
    m.add("CHECK_MODE_NO_CHECK", "no_check")?;
    m.add("DEFAULT_TRANSITION_COSTS", DEFAULT_TRANSITION_COSTS)?;
    m.add("PATH_FLAG_NONE", PathFlags::NONE.bits())?;
    m.add("PATH_FLAG_ONBOARDING", PathFlags::ONBOARDING.bits())?;
    m.add("PATH_FLAG_TRACE", PathFlags::TRACE.bits())?;
    m.add("PATH_FLAG_LIN_INTERP", PathFlags::LIN_INTERP.bits())?;
    m.add("PATH_FLAG_LAND", PathFlags::LAND.bits())?;
    m.add("PATH_FLAG_LANDING", PathFlags::LANDING.bits())?;
    m.add("PATH_FLAG_PARK", PathFlags::PARK.bits())?;
    m.add("PATH_FLAG_PARKING", PathFlags::PARKING.bits())?;
    m.add("PATH_FLAG_FORWARDS", PathFlags::FORWARDS.bits())?;
    m.add("PATH_FLAG_BACKWARDS", PathFlags::BACKWARDS.bits())?;
    m.add("PATH_FLAG_RECONFIGURING", PathFlags::RECONFIGURING.bits())?;
    m.add("PATH_FLAG_ORIGINAL", PathFlags::ORIGINAL.bits())?;
    m.add("PATH_FLAG_DEBUG", PathFlags::DEBUG.bits())?;
    m.add("MOVE_KIND_JOINT", "joint")?;
    m.add("MOVE_KIND_CARTESIAN", "cartesian")?;
    m.add_class::<AnnotatedJoints>()?;
    m.add_class::<CartesianPlanner>()?;
    m.add_class::<Constraints>()?;
    m.add_class::<KinematicsWithShape>()?;
    m.add_class::<KinematicModel>()?;
    m.add_class::<Mesh>()?;
    m.add_class::<Parallelogram>()?;
    m.add_class::<Robot>()?;
    m.add_class::<RRTPlanner>()?;
    m.add_class::<SafetyDistances>()?;
    m.add_class::<VisualizationHandle>()?;
    m.add_function(wrap_pyfunction!(visualize_robot, m)?)?;
    m.add_function(wrap_pyfunction!(visualize_robot_with_safety, m)?)?;
    Ok(())
}
