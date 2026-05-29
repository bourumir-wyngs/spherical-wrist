from spherical_wrist import (
    ENV_START_IDX,
    J1,
    KinematicModel,
    KinematicsWithShape,
    Mesh,
    NEVER_COLLIDES,
    Robot,
    SafetyDistances,
)
from scipy.spatial.transform import RigidTransform
import numpy as np


def test_kinematics_with_shape_reports_environment_collision() -> None:
    joints = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_mesh = _unit_cube()
    joint_pose = Robot(_model(), degrees=True).forward_with_joint_poses(joints).as_matrix()[J1]
    environment = _unit_cube().transformed(RigidTransform.from_matrix(joint_pose))
    robot = KinematicsWithShape(
        _model(),
        [joint_mesh] * 6,
        degrees=True,
        environment=[environment],
        safety=SafetyDistances(
            to_environment=0.0,
            to_robot_default=NEVER_COLLIDES,
            mode="all",
        ),
    )

    assert robot.collides(joints)
    assert (J1, ENV_START_IDX) in robot.collision_details(joints)
    assert (J1, ENV_START_IDX) in robot.near(
        joints,
        SafetyDistances(
            to_environment=0.0,
            to_robot_default=NEVER_COLLIDES,
            mode="all",
        ),
    )


def test_kinematics_with_shape_exposes_kinematics_methods() -> None:
    joints = (10.0, 20.0, -70.0, 30.0, 20.0, 10.0)
    robot = KinematicsWithShape(
        _model(),
        [_unit_cube()] * 6,
        degrees=True,
        safety=SafetyDistances(
            to_environment=NEVER_COLLIDES,
            to_robot_default=NEVER_COLLIDES,
        ),
    )

    pose = robot.forward(joints)
    solutions = robot.inverse(pose, current_joints=joints)
    joint_poses = robot.forward_with_joint_poses(joints)
    positioned = robot.positioned_robot(joints)

    assert not robot.collides(joints)
    assert any(np.allclose(solution, joints, atol=1e-6) for solution in solutions)
    assert joint_poses.as_matrix().shape == (6, 4, 4)
    assert positioned.joints.as_matrix().shape == (6, 4, 4)
    assert positioned.tool is None
    assert positioned.environment == ()
    assert robot.non_colliding_offsets(
        joints,
        tuple(value - 1.0 for value in joints),
        tuple(value + 1.0 for value in joints),
    )


def _unit_cube() -> Mesh:
    return Mesh.from_arrays(_cube_vertices(), _cube_triangles())


def _model() -> KinematicModel:
    return KinematicModel(
        a1=400,
        a2=-250,
        b=0,
        c1=830,
        c2=1175,
        c3=1444,
        c4=230,
        offsets=(0, 0, 0, 0, 0, 0),
        flip_axes=(True, False, True, True, False, True),
    )


def _cube_vertices() -> list[tuple[float, float, float]]:
    return [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 1.0),
        (1.0, 1.0, 1.0),
        (0.0, 1.0, 1.0),
    ]


def _cube_triangles() -> list[tuple[int, int, int]]:
    return [
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ]
