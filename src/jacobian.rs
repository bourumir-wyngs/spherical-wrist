use super::*;

/// Holds a fixed 6x6 Jacobian matrix and provides methods to extract velocity
/// and torque information from it.
///
/// The Jacobian matrix maps joint velocities to end-effector linear and angular
/// velocities.
#[pyclass(frozen)]
pub(crate) struct Jacobian {
    jacobian: RsJacobian,
    degrees: bool,
}

#[pymethods]
impl Jacobian {
    #[new]
    #[pyo3(signature = (robot, joints, epsilon=1.0e-6, ee_transform=None))]
    fn new(
        robot: &Robot,
        joints: [f64; 6],
        epsilon: f64,
        ee_transform: Option<[[f64; 4]; 4]>,
    ) -> PyResult<Self> {
        let epsilon = validate_jacobian_epsilon(epsilon)?;
        let joints = joints_to_internal(joints, robot.degrees)?;
        let kinematics = robot.build_robot(ee_transform)?;

        Ok(Self {
            jacobian: RsJacobian::new(kinematics.as_ref(), &joints, epsilon),
            degrees: robot.degrees,
        })
    }

    #[staticmethod]
    #[pyo3(signature = (robot, joints, epsilon=1.0e-6))]
    fn from_shape(robot: &KinematicsWithShape, joints: [f64; 6], epsilon: f64) -> PyResult<Self> {
        let epsilon = validate_jacobian_epsilon(epsilon)?;
        let joints = joints_to_internal(joints, robot.degrees)?;

        Ok(Self {
            jacobian: RsJacobian::new(&robot.robot, &joints, epsilon),
            degrees: robot.degrees,
        })
    }

    #[pyo3(signature = (radians=false))]
    fn matrix(&self, radians: bool) -> [[f64; 6]; 6] {
        let mut rows = *self.jacobian.matrix().rows();
        convert_jacobian_columns_to_output_units(&mut rows, self.degrees, radians);
        rows
    }

    #[pyo3(signature = (linear_velocity, angular_velocity, radians=false))]
    fn velocities(
        &self,
        linear_velocity: [f64; 3],
        angular_velocity: [f64; 3],
        radians: bool,
    ) -> PyResult<[f64; 6]> {
        let twist = Twist::new(
            vector3_to_dvec3(linear_velocity, "linear_velocity")?,
            vector3_to_dvec3(angular_velocity, "angular_velocity")?,
        );
        let velocities = self
            .jacobian
            .velocities(&twist)
            .map_err(PyValueError::new_err)?;

        Ok(joint_rates_from_internal(velocities, self.degrees, radians))
    }

    #[pyo3(signature = (vx, vy, vz, radians=false))]
    fn velocities_fixed(&self, vx: f64, vy: f64, vz: f64, radians: bool) -> PyResult<[f64; 6]> {
        validate_vector3_values(&[vx, vy, vz], "linear velocity")?;
        let velocities = self
            .jacobian
            .velocities_fixed(vx, vy, vz)
            .map_err(PyValueError::new_err)?;

        Ok(joint_rates_from_internal(velocities, self.degrees, radians))
    }

    #[pyo3(signature = (twist, radians=false))]
    fn velocities_from_vector(&self, twist: [f64; 6], radians: bool) -> PyResult<[f64; 6]> {
        validate_vector6(&twist, "twist")?;
        let velocities = self
            .jacobian
            .velocities_from_vector(&twist)
            .map_err(PyValueError::new_err)?;

        Ok(joint_rates_from_internal(velocities, self.degrees, radians))
    }

    #[pyo3(signature = (force, torque, radians=false))]
    fn torques(&self, force: [f64; 3], torque: [f64; 3], radians: bool) -> PyResult<[f64; 6]> {
        let wrench = Wrench::new(
            vector3_to_dvec3(force, "force")?,
            vector3_to_dvec3(torque, "torque")?,
        );
        let torques = self.jacobian.torques(&wrench);

        Ok(generalized_efforts_from_internal(
            torques,
            self.degrees,
            radians,
        ))
    }

    #[pyo3(signature = (wrench, radians=false))]
    fn torques_from_vector(&self, wrench: [f64; 6], radians: bool) -> PyResult<[f64; 6]> {
        validate_vector6(&wrench, "wrench")?;
        let torques = self.jacobian.torques_from_vector(&wrench);

        Ok(generalized_efforts_from_internal(
            torques,
            self.degrees,
            radians,
        ))
    }

    fn __repr__(&self) -> String {
        format!("Jacobian(degrees={})", self.degrees)
    }
}
