use glam::{DMat3, DQuat, DVec3, Mat3, Vec3};
use parry3d::math::Pose as ParryPose;
use parry3d::shape::TriMesh;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use rs_opw_kinematics::cartesian::{
    AnnotatedJoints as RsAnnotatedJoints, Cartesian as RsCartesian, DEFAULT_CARTESIAN_LAYER_STATES,
    DEFAULT_MAX_SOLUTIONS_AWAIT, DEFAULT_PREFERRED_ONBOARDING_SUFFIX_CANDIDATES,
    DEFAULT_RECONFIGURATION_PREFIX_CANDIDATES, DEFAULT_TRANSITION_COSTS, MoveKind as RsMoveKind,
    PathFlags,
};
use rs_opw_kinematics::collisions::{
    BaseBody, CheckMode, CollisionBody as RsCollisionBody, NEVER_COLLIDES, RobotBody,
    SafetyDistances as RsSafetyDistances, TOUCH_ONLY, transform_mesh,
};
use rs_opw_kinematics::constraints::{BY_CONSTRAINS, BY_PREV, Constraints as RsConstraints};
use rs_opw_kinematics::frame::{Frame as RsFrame, FrameTransform as RsFrameTransform};
use rs_opw_kinematics::jacobian::Jacobian as RsJacobian;
use rs_opw_kinematics::kinematic_traits::{
    CONSTRAINT_CENTERED, ENV_START_IDX, J_BASE, J_TOOL, J1, J2, J3, J4, J5, J6, Joints, Kinematics,
    Pose, Singularity,
};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape as RsKinematicsWithShape;
use rs_opw_kinematics::parallelogram::Parallelogram as RsParallelogram;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::pose::{Pose32, Twist, Wrench};
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

mod annotated_joints;
mod cartesian_planner;
mod constraints;
mod frame;
mod helpers;
mod jacobian;
mod kinematic_model;
mod kinematics_with_shape;
mod mesh;
mod parallelogram;
mod robot;
mod rrt_planner;
mod safety_distances;
mod visualization;

pub(crate) use annotated_joints::AnnotatedJoints;
pub(crate) use cartesian_planner::CartesianPlanner;
pub(crate) use constraints::Constraints;
pub(crate) use frame::Frame;
pub(crate) use helpers::*;
pub(crate) use jacobian::Jacobian;
pub(crate) use kinematic_model::KinematicModel;
pub(crate) use kinematics_with_shape::KinematicsWithShape;
pub(crate) use mesh::Mesh;
pub(crate) use parallelogram::Parallelogram;
pub(crate) use robot::Robot;
pub(crate) use rrt_planner::RRTPlanner;
pub(crate) use safety_distances::SafetyDistances;
pub(crate) use visualization::{VisualizationHandle, visualize_robot, visualize_robot_with_safety};

const RADIANS_PER_DEGREE: f64 = std::f64::consts::PI / 180.0;

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
    m.add("DEFAULT_MAX_SOLUTIONS_AWAIT", DEFAULT_MAX_SOLUTIONS_AWAIT)?;
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
    m.add_class::<Frame>()?;
    m.add_class::<Jacobian>()?;
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
