from spherical_wrist import CONSTRAINT_CENTERED, KinematicModel, Robot
from scipy.spatial.transform import RigidTransform, Rotation
import numpy as np


def test_forward_with_joint_poses_and_singularity_use_robot_units() -> None:
    robot = Robot(_model(), degrees=True)
    joints = (10.0, 20.0, -70.0, 30.0, 0.0, 10.0)

    poses = robot.forward_with_joint_poses(joints)

    assert poses.as_matrix().shape == (6, 4, 4)
    assert np.allclose(
        poses.as_matrix()[-1],
        robot.forward(joints).as_matrix(),
        atol=1e-10,
    )
    assert robot.kinematic_singularity(joints) == "A"
    assert robot.kinematic_singularity((10.0, 20.0, -70.0, 30.0, 20.0, 10.0)) is None


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
