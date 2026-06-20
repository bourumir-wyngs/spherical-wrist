use super::*;

/// Configurable Cartesian stroke planner for a robot with collision geometry.
///
/// The planner finds a collision-free path that enters a Cartesian stroke,
/// follows the requested TCP poses, can bridge infeasible stroke segments with
/// joint-space RRT reconfiguration, and exits at the park pose.
#[pyclass(frozen)]
#[derive(Clone)]
pub(crate) struct CartesianPlanner {
    check_step_m: f64,
    check_step_rad: f64,
    max_transition_cost: f64,
    transition_coefficients: Joints,
    linear_recursion_depth: usize,
    rrt: RRTPlanner,
    allow_reconfigure: bool,
    max_solutions_await: usize,
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
        max_solutions_await=3,
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
        max_solutions_await: usize,
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
            max_solutions_await,
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
    fn max_solutions_await(&self) -> usize {
        self.max_solutions_await
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
        py: Python<'_>,
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
            preferred_onboarding_suffix_candidates: DEFAULT_PREFERRED_ONBOARDING_SUFFIX_CANDIDATES,
            max_cartesian_layer_states: DEFAULT_CARTESIAN_LAYER_STATES,
            max_solutions_await: self.max_solutions_await,
            include_linear_interpolation: self.include_linear_interpolation,
            debug: self.debug,
        };

        py.detach(|| planner.plan(&start, &land, steps, &park))
            .map(|path| annotated_joints_from_internal(path, robot.degrees))
            .map_err(PyValueError::new_err)
    }

    fn __repr__(&self) -> String {
        format!(
            "CartesianPlanner(check_step_m={}, check_step_rad={}, max_transition_cost={}, linear_recursion_depth={}, allow_reconfigure={}, max_solutions_await={}, include_linear_interpolation={}, debug={})",
            self.check_step_m,
            self.check_step_rad(false),
            self.max_transition_cost(false),
            self.linear_recursion_depth,
            self.allow_reconfigure,
            self.max_solutions_await,
            self.include_linear_interpolation,
            self.debug
        )
    }
}
