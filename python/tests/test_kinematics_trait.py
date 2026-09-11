from spherical_wrist import (
    CONSTRAINT_CENTERED,
    KinematicModel,
    KinematicsWithShape,
    Mesh,
    NEVER_COLLIDES,
    Robot,
    SafetyDistances,
)
from scipy.spatial.transform import RigidTransform, Rotation
import numpy as np
import pytest


@pytest.mark.parametrize("degrees", [True, False], ids=["degrees", "radians"])
def test_forward_with_joint_poses_uses_robot_units(degrees: bool) -> None:
    robot = Robot(_model(), degrees=degrees)
    joints = (10.0, 20.0, -70.0, 30.0, 0.0, 10.0)
    if not degrees:
        joints = tuple(np.deg2rad(joints))

    poses = robot.forward_with_joint_poses(joints)

    assert poses.as_matrix().shape == (6, 4, 4)
    assert np.allclose(
        poses.as_matrix()[-1],
        robot.forward(joints).as_matrix(),
        atol=1e-10,
    )


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped-robot"])
@pytest.mark.parametrize("degrees", [True, False], ids=["degrees", "radians"])
@pytest.mark.parametrize("j5", [0.0, 180.0, -180.0])
def test_wrist_singularity_recovery_preserves_pose_and_continuity(
    shaped: bool, degrees: bool, j5: float
) -> None:
    if shaped:
        mesh = Mesh.from_arrays(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
            [(0, 1, 2)],
        )
        robot = KinematicsWithShape(
            _model(),
            [mesh] * 6,
            degrees=degrees,
            safety=SafetyDistances(
                to_environment=NEVER_COLLIDES,
                to_robot_default=NEVER_COLLIDES,
            ),
        )
    else:
        robot = Robot(_model(), degrees=degrees)

    joints = (10.0, 20.0, -70.0, 30.0, j5, 10.0)
    if not degrees:
        joints = tuple(np.deg2rad(joints))
    pose = robot.forward(joints)

    solutions = robot.inverse(pose)
    continuing = robot.inverse_continuing(pose, joints)

    assert solutions
    assert continuing
    # Recover the original arm configuration without prescribing free wrist angles.
    assert any(
        np.allclose(solution[:3], joints[:3], atol=1e-8, rtol=0)
        for solution in solutions
    )
    # A continuation at the same pose should preserve the supplied wrist angles.
    np.testing.assert_allclose(continuing[0], joints, atol=1e-8, rtol=0)
    for solution in solutions + continuing:
        np.testing.assert_allclose(
            robot.forward(solution).as_matrix(), pose.as_matrix(), atol=1e-9, rtol=0
        )


def test_inverse_continuing_and_constraint_centered_are_exposed() -> None:
    robot = Robot(_model(), degrees=True)
    joints = (10.0, 20.0, -70.0, 30.0, 20.0, 10.0)
    pose = robot.forward(joints, tool=_tool())

    continuing = robot.inverse_continuing(pose, joints, tool=_tool())
    centered = robot.inverse_continuing(pose, CONSTRAINT_CENTERED, tool=_tool())

    assert any(np.allclose(solution, joints, atol=1e-6) for solution in continuing)
    assert centered


def test_5dof_inverse_methods_convert_j6_units() -> None:
    robot = Robot(_model(), degrees=True)
    joints = (10.0, 20.0, -70.0, 30.0, 20.0, 15.0)
    pose = robot.forward(joints)

    inverse_5dof = robot.inverse_5dof(pose, j6=15.0)
    continuing_5dof = robot.inverse_continuing_5dof(pose, joints)

    assert inverse_5dof
    assert continuing_5dof
    assert all(np.isclose(solution[5], 15.0, atol=1e-9) for solution in inverse_5dof)
    assert all(np.isclose(solution[5], 15.0, atol=1e-9) for solution in continuing_5dof)


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


def _tool() -> RigidTransform:
    return RigidTransform.from_components(
        rotation=Rotation.from_euler("xyz", [10, -30, 20], degrees=True),
        translation=[100, 20, -30],
    )
