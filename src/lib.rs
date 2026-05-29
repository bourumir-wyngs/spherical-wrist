use glam::{DMat3, DQuat, DVec3};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use rs_opw_kinematics::frame::Frame;
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parallelogram::Parallelogram as RsParallelogram;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::tool::{Base, Tool};
use std::sync::Arc;

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

#[pyclass]
struct Robot {
    robot: OPWKinematics,
    degrees: bool,
    kinematic_model: KinematicModel,
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
        gantry_base: Option<[[f64; 4]; 4]>,
        linear_axis_axis: Option<usize>,
        linear_axis_base: Option<[[f64; 4]; 4]>,
    ) -> PyResult<Self> {
        if let Some(parallelogram) = parallelogram {
            parallelogram.validate()?;
        }
        Ok(Robot {
            robot: OPWKinematics::new(kinematic_model.to_parameters(degrees)),
            degrees,
            kinematic_model,
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
        mut joints: [f64; 6],
        ee_transform: Option<[[f64; 4]; 4]>,
        gantry_translation: Option<[f64; 3]>,
        linear_axis_distance: Option<f64>,
    ) -> PyResult<[[f64; 4]; 4]> {
        validate_joints(&joints)?;
        if self.degrees {
            joints
                .iter_mut()
                .for_each(|joint| *joint = joint.to_radians());
        }

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
        let target_pose =
            self.remove_external_axes(matrix_to_pose(pose)?, gantry_translation, linear_axis_distance)?;
        let robot = self.build_robot(ee_transform)?;

        let mut solutions = match current_joints {
            Some(mut joints) => {
                validate_joints(&joints)?;
                if self.degrees {
                    joints
                        .iter_mut()
                        .for_each(|joint| *joint = joint.to_radians());
                }
                robot.inverse_continuing(&target_pose, &joints)
            }
            None => robot.inverse(&target_pose),
        };

        if self.degrees {
            for solution in &mut solutions {
                solution
                    .iter_mut()
                    .for_each(|joint| *joint = joint.to_degrees());
            }
        }

        Ok(solutions)
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
                    return Err(PyValueError::new_err(
                        "linear_axis_distance must be finite",
                    ));
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

fn validate_joints(joints: &Joints) -> PyResult<()> {
    if joints.iter().any(|joint| !joint.is_finite()) {
        return Err(PyValueError::new_err("joint values must be finite"));
    }
    Ok(())
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

fn array_to_tuple(values: [f64; 6]) -> (f64, f64, f64, f64, f64, f64) {
    let [a, b, c, d, e, f] = values;
    (a, b, c, d, e, f)
}

#[pymodule(name = "_internal")]
fn spherical_wrist(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<KinematicModel>()?;
    m.add_class::<Parallelogram>()?;
    m.add_class::<Robot>()?;
    Ok(())
}
