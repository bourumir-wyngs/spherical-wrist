from spherical_wrist import KinematicModel, Parallelogram, Robot
from scipy.spatial.transform import RigidTransform, Rotation
import numpy as np
import pytest


def test_introductory_readme_example() -> None:
    kinematic_model = KinematicModel(
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
    robot = Robot(kinematic_model, degrees=True)

    ee_rotation = Rotation.from_euler("xyz", [0, -90, 0], degrees=True)
    ee_transform = RigidTransform.from_components(
        rotation=ee_rotation,
        translation=[0, 0, 0],
    )

    joints = (10, 0, -90, 0, 0, 0)
    pose = robot.forward(joints, ee_transform=ee_transform)

    assert np.allclose(pose.translation, [2042.49, -360.15, 2255.0], atol=1e-2)
    assert np.allclose(
        pose.rotation.as_euler("XYZ", degrees=True),
        [0.0, 0.0, -10.0],
        atol=1e-2,
    )

    solutions = robot.inverse(pose, ee_transform=ee_transform)

    assert len(solutions) == 2
    for solution in solutions:
        solution_pose = robot.forward(solution, ee_transform=ee_transform)
        assert np.allclose(solution_pose.as_matrix(), pose.as_matrix(), atol=1e-9)


def test_kinematic_model_matches_find_signs_usage() -> None:
    model = KinematicModel(
        a1=400.333,
        a2=-251.449,
        b=0,
        c1=830,
        c2=1177.556,
        c3=1443.593,
        c4=230,
    )

    assert model.a1 == 400.333
    assert model.a2 == -251.449
    assert model.b == 0
    assert model.c4 == 230
    assert model.offsets == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert model.flip_axes == (False, False, False, False, False, False)


def test_constructor_tool_matches_ee_transform_alias() -> None:
    robot = Robot(_model(), degrees=True)
    robot_with_tool = Robot(_model(), degrees=True, tool=_tool())
    joints = (10, 20, -70, 30, 20, 10)

    pose_from_constructor = robot_with_tool.forward(joints)
    pose_from_ee_alias = robot.forward(joints, ee_transform=_tool())
    pose_from_tool_alias = robot.forward(joints, tool=_tool())

    assert np.allclose(
        pose_from_constructor.as_matrix(),
        pose_from_ee_alias.as_matrix(),
        atol=1e-10,
    )
    assert np.allclose(
        pose_from_constructor.as_matrix(),
        pose_from_tool_alias.as_matrix(),
        atol=1e-10,
    )

    solutions = robot_with_tool.inverse(pose_from_constructor, current_joints=joints)
    assert any(np.allclose(solution, joints, atol=1e-6) for solution in solutions)


def test_constructor_base_tool_frame_composition_order() -> None:
    model = _model()
    joints = (10, 20, -70, 30, 20, 10)
    base = RigidTransform.from_components(
        rotation=Rotation.from_euler("Z", 15, degrees=True),
        translation=[100, 200, 300],
    )
    tool = _tool()
    frame = RigidTransform.from_components(
        rotation=Rotation.from_euler("X", -20, degrees=True),
        translation=[-40, 50, 60],
    )

    robot = Robot(model, degrees=True)
    composite_robot = Robot(model, degrees=True, base=base, tool=tool, frame=frame)

    raw_pose = robot.forward(joints)
    expected = base.as_matrix() @ raw_pose.as_matrix() @ tool.as_matrix() @ frame.as_matrix()
    actual = composite_robot.forward(joints).as_matrix()

    assert np.allclose(actual, expected, atol=1e-10)

    solutions = composite_robot.inverse(
        RigidTransform.from_matrix(actual),
        current_joints=joints,
    )
    assert any(np.allclose(solution, joints, atol=1e-6) for solution in solutions)


def test_ee_transform_alias_uses_constructor_tool_order_with_frame() -> None:
    model = _model()
    joints = (10, 20, -70, 30, 20, 10)
    frame = RigidTransform.from_components(
        rotation=Rotation.from_euler("X", -20, degrees=True),
        translation=[-40, 50, 60],
    )

    constructor_tool = Robot(model, degrees=True, tool=_tool(), frame=frame)
    call_tool = Robot(model, degrees=True, frame=frame)

    assert np.allclose(
        constructor_tool.forward(joints).as_matrix(),
        call_tool.forward(joints, ee_transform=_tool()).as_matrix(),
        atol=1e-10,
    )


def test_tool_and_ee_transform_alias_are_mutually_exclusive() -> None:
    robot = Robot(_model(), degrees=True)

    with pytest.raises(ValueError, match="either tool or ee_transform"):
        robot.forward((10, 20, -70, 30, 20, 10), ee_transform=_tool(), tool=_tool())

    robot_with_tool = Robot(_model(), degrees=True, tool=_tool())
    with pytest.raises(ValueError, match="constructor tool"):
        robot_with_tool.forward((10, 20, -70, 30, 20, 10), ee_transform=_tool())


def test_parallelogram_adjusts_coupled_joint_and_roundtrips() -> None:
    model = _model()
    joints = (10, 20, -70, 30, 20, 10)
    parallelogram = Parallelogram(scaling=1.0, driven=1, coupled=2)
    robot = Robot(model, degrees=True)
    robot_with_parallelogram = Robot(
        model,
        degrees=True,
        parallelogram=parallelogram,
    )

    expected_pose = robot.forward((10, 20, -90, 30, 20, 10))
    actual_pose = robot_with_parallelogram.forward(joints)

    assert robot_with_parallelogram.parallelogram is parallelogram
    assert parallelogram.scaling == 1.0
    assert parallelogram.driven == 1
    assert parallelogram.coupled == 2
    assert np.allclose(actual_pose.as_matrix(), expected_pose.as_matrix(), atol=1e-10)

    solutions = robot_with_parallelogram.inverse(actual_pose, current_joints=joints)
    assert any(np.allclose(solution, joints, atol=1e-6) for solution in solutions)


def test_parallelogram_validates_joint_indices() -> None:
    with pytest.raises(ValueError, match="range 0..6"):
        Parallelogram(scaling=1.0, driven=6, coupled=2)

    with pytest.raises(ValueError, match="different"):
        Parallelogram(scaling=1.0, driven=2, coupled=2)


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
