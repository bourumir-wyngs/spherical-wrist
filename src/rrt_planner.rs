use super::*;

/// RRT planner that relocates the robot between two positions in a
/// collision-free way.
#[pyclass(frozen)]
#[derive(Clone, Copy)]
pub(crate) struct RRTPlanner {
    step_size_joint_space: f64,
    max_try: usize,
    smooth: usize,
    debug: bool,
}

#[pymethods]
impl RRTPlanner {
    #[new]
    #[pyo3(signature = (step_size_joint_space=3.0, max_try=2000, debug=false, radians=false, smooth=0))]
    fn new(
        step_size_joint_space: f64,
        max_try: usize,
        debug: bool,
        radians: bool,
        smooth: usize,
    ) -> PyResult<Self> {
        let step_size_joint_space =
            positive_angle_to_internal(step_size_joint_space, radians, "step_size_joint_space")?;
        if max_try == 0 {
            return Err(PyValueError::new_err("max_try must be positive"));
        }

        Ok(Self {
            step_size_joint_space,
            max_try,
            smooth,
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
    fn smooth(&self) -> usize {
        self.smooth
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
            "RRTPlanner(step_size_joint_space={}, max_try={}, smooth={}, debug={})",
            self.step_size_joint_space(false),
            self.max_try,
            self.smooth,
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
            smooth: planner.smooth,
            debug: planner.debug,
        }
    }
}

impl RRTPlanner {
    pub(crate) fn to_rs_rrt(self) -> RsRRTPlanner {
        RsRRTPlanner {
            step_size_joint_space: self.step_size_joint_space,
            max_try: self.max_try,
            smooth: self.smooth,
            debug: self.debug,
        }
    }
}
