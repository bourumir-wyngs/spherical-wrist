"""Bulk inverse kinematics must preserve the scalar API's solving semantics."""

import numpy as np
import pytest
from scipy.spatial.transform import RigidTransform, Rotation

from spherical_wrist import (
    BatchPoseError,
    Constraints,
    Frame,
    J2,
    KinematicModel,
    KinematicsWithShape,
    Mesh,
    NEVER_COLLIDES,
    Parallelogram,
    Robot,
    SafetyDistances,
)
from spherical_wrist._internal import Robot as _RobotInternal


_JOINTS = (
    (10.0, 20.0, -70.0, 30.0, 20.0, 10.0),
    (-35.0, -20.0, -55.0, 60.0, 45.0, -25.0),
    (50.0, 15.0, -80.0, -20.0, 35.0, 30.0),
)


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped"])
@pytest.mark.parametrize("degrees", [False, True], ids=["radians", "degrees"])
@pytest.mark.parametrize("as_stack", [False, True], ids=["sequence", "stack"])
def test_bulk_matches_scalar_and_preserves_pose_and_solution_order(
    shaped: bool, degrees: bool, as_stack: bool
) -> None:
    robot = _robot(shaped, degrees=degrees)
    joints = [_units(joints, degrees) for joints in _JOINTS]
    reachable = [robot.forward(joint) for joint in joints]
    unreachable = RigidTransform.from_translation([1e9, 1e9, 1e9])
    # A hole and a duplicate detect dropped entries and output reordering.
    poses = [reachable[2], unreachable, reachable[0], reachable[1], reachable[2]]
    expected = [robot.inverse(pose) for pose in poses]

    actual = robot.inverse_multiple(_stack(poses) if as_stack else poses)

    _assert_batches_equal(actual, expected)
    assert actual[1] == []
    for index in (0, 2, 3, 4):
        assert actual[index]
        for solution in actual[index]:
            np.testing.assert_allclose(
                robot.forward(solution).as_matrix(),
                poses[index].as_matrix(),
                atol=1e-8,
                rtol=0,
            )


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped"])
def test_bulk_accepts_empty_and_single_inputs(shaped: bool) -> None:
    robot = _robot(shaped)
    pose = robot.forward(_JOINTS[0])

    assert robot.inverse_multiple([]) == []
    assert robot.inverse_multiple(()) == []
    assert robot.inverse_multiple(RigidTransform.from_matrix(np.empty((0, 4, 4)))) == []
    for poses in (pose, [pose], (pose,), _stack([pose])):
        _assert_batches_equal(robot.inverse_multiple(poses), [robot.inverse(pose)])


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped"])
@pytest.mark.parametrize("degrees", [False, True], ids=["radians", "degrees"])
def test_bulk_applies_constraints_and_preserves_fully_filtered_entries(
    shaped: bool, degrees: bool
) -> None:
    constraints = Constraints(
        tuple(value - 1.0 for value in _JOINTS[0]),
        tuple(value + 1.0 for value in _JOINTS[0]),
    )
    robot = _robot(shaped, degrees=degrees, constraints=constraints)
    plain = Robot(_model(), degrees=degrees)
    original_joints = _units(_JOINTS[0], degrees)
    poses = [plain.forward(_units(joints, degrees)) for joints in _JOINTS[:2]]

    actual = robot.inverse_multiple(_stack(poses))

    _assert_batches_equal(actual, [robot.inverse(pose) for pose in poses])
    assert 0 < len(actual[0]) < len(plain.inverse(poses[0]))
    assert actual[1] == []
    assert all(
        constraints.compliant(solution, radians=not degrees)
        for solution in actual[0]
    )
    assert any(np.allclose(solution, original_joints, atol=1e-8) for solution in actual[0])


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped"])
def test_bulk_solves_singular_poses_independently(shaped: bool) -> None:
    robot = _robot(shaped)
    singular = robot.forward((10.0, 20.0, -70.0, 30.0, 0.0, 10.0))
    # Each singular solve must remain independent of preceding poses.
    poses = [robot.forward(_JOINTS[1]), singular, robot.forward(_JOINTS[2]), singular]

    actual = robot.inverse_multiple(_stack(poses))

    _assert_batches_equal(actual, [robot.inverse(pose) for pose in poses])
    for index in (1, 3):
        assert actual[index]
        for solution in actual[index]:
            np.testing.assert_allclose(
                robot.forward(solution).as_matrix(),
                singular.as_matrix(),
                atol=1e-8,
                rtol=0,
            )


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped"])
def test_bulk_honors_constructor_transforms_and_parallelogram(shaped: bool) -> None:
    robot = _robot(
        shaped,
        base=_transform([100.0, 200.0, 300.0], [0.0, 0.0, 15.0]),
        tool=_transform([100.0, 20.0, -30.0], [10.0, -30.0, 20.0]),
        parallelogram=Parallelogram(scaling=1.0, driven=1, coupled=2),
    )
    poses = [robot.forward(joints) for joints in _JOINTS]

    actual = robot.inverse_multiple(_stack(poses))

    _assert_batches_equal(actual, [robot.inverse(pose) for pose in poses])
    for solutions, joints in zip(actual, _JOINTS):
        assert any(np.allclose(solution, joints, atol=1e-8) for solution in solutions)


@pytest.mark.parametrize("alias", ["tool", "ee_transform"])
def test_bulk_honors_per_call_tool_and_scaled_frame(alias: str) -> None:
    original = np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [0.0, 100.0, 0.0]])
    target = Rotation.from_euler("z", 17.0, degrees=True).apply(original * 1.05)
    target += np.array([10.0, 20.0, 30.0])
    robot = Robot(
        _model(),
        base=_transform([100.0, 200.0, 300.0], [0.0, 0.0, 15.0]),
        frame=Frame.from_tie(original, target),
    )
    kwargs = {alias: _transform([100.0, 20.0, -30.0], [10.0, -30.0, 20.0])}
    poses = [robot.forward(joints, **kwargs) for joints in _JOINTS]

    actual = robot.inverse_multiple(_stack(poses), **kwargs)

    _assert_batches_equal(actual, [robot.inverse(pose, **kwargs) for pose in poses])
    for solutions, joints in zip(actual, _JOINTS):
        assert any(np.allclose(solution, joints, atol=1e-8) for solution in solutions)


def test_bulk_removes_colliding_solutions_but_keeps_other_branches() -> None:
    plain = Robot(_model())
    mesh = _cube()
    obstacle = mesh.transformed(plain.forward_with_joint_poses(_JOINTS[0])[J2])
    robot = KinematicsWithShape(
        _model(),
        [mesh] * 6,
        environment=[obstacle],
        safety=SafetyDistances(
            to_environment=0.1, to_robot_default=NEVER_COLLIDES
        ),
    )
    poses = [plain.forward(joints) for joints in _JOINTS]
    unfiltered = [plain.inverse(pose) for pose in poses]
    expected = [
        [solution for solution in solutions if not robot.collides(solution)]
        for solutions in unfiltered
    ]

    actual = robot.inverse_multiple(_stack(poses))

    # Ensure collision checking actually removes some, but not all, solutions.
    assert 0 < len(actual[0]) < len(unfiltered[0])
    assert any(len(solutions) == len(raw) for solutions, raw in zip(actual, unfiltered))
    _assert_batches_equal(actual, expected)
    _assert_batches_equal(actual, [robot.inverse(pose) for pose in poses])
    assert all(not robot.collides(solution) for solutions in actual for solution in solutions)


def test_bulk_keeps_entries_when_every_solution_collides() -> None:
    plain = Robot(_model())
    poses = [plain.forward(joints) for joints in _JOINTS]
    mesh = _cube()
    # Every branch has the same J6/TCP pose. Offset overlapping cubes slightly
    # so the surfaces intersect instead of coinciding exactly.
    obstacles = [
        mesh.transformed(pose * RigidTransform.from_translation([0.25, 0.25, 0.25]))
        for pose in poses
    ]
    robot = KinematicsWithShape(
        _model(),
        [mesh] * 6,
        environment=obstacles,
        safety=SafetyDistances(
            to_environment=0.1, to_robot_default=NEVER_COLLIDES
        ),
    )

    assert all(plain.inverse(pose) for pose in poses)
    assert all(robot.inverse(pose) == [] for pose in poses)
    assert robot.inverse_multiple(_stack(poses)) == [[], [], []]


def test_bulk_rejects_conflicting_tool_arguments() -> None:
    tool = _transform([100.0, 20.0, -30.0], [10.0, -30.0, 20.0])
    robot = Robot(_model())
    pose = robot.forward(_JOINTS[0])

    with pytest.raises(ValueError, match="either tool or ee_transform"):
        robot.inverse_multiple([pose], tool=tool, ee_transform=tool)
    with pytest.raises(ValueError, match="constructor tool"):
        Robot(_model(), tool=tool).inverse_multiple([pose], tool=tool)


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped"])
def test_bulk_rejects_invalid_input_types_and_current_joints_keyword(shaped: bool) -> None:
    robot = _robot(shaped)
    pose = robot.forward(_JOINTS[0])

    with pytest.raises(TypeError):
        robot.inverse_multiple([pose, np.eye(4)])
    with pytest.raises(TypeError, match="current_joints"):
        robot.inverse_multiple([pose], current_joints=_JOINTS[0])
    with pytest.raises(TypeError, match="current_joints"):
        robot._robot.inverse_multiple(_stack([pose]).as_matrix(), current_joints=_JOINTS[0])


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped"])
@pytest.mark.parametrize("layout", ["reversed", "fortran"])
def test_internal_bulk_respects_numpy_buffer_strides(shaped: bool, layout: str) -> None:
    robot = _robot(shaped)
    poses = [robot.forward(joints) for joints in _JOINTS]
    matrices = np.stack([pose.as_matrix() for pose in poses])
    if layout == "reversed":
        matrices = matrices[::-1]
        poses = poses[::-1]
    else:
        matrices = np.asfortranarray(matrices)
    matrices.flags.writeable = False

    actual = robot._robot.inverse_multiple(matrices)

    _assert_batches_equal(actual, [robot.inverse(pose) for pose in poses])


def test_internal_bulk_honors_external_axis_transform_order() -> None:
    robot = _RobotInternal(
        _model(),
        base=_transform([100.0, 200.0, 300.0], [0.0, 0.0, 15.0]).as_matrix(),
        gantry_base=_transform([50.0, -100.0, 25.0], [10.0, 30.0, 0.0]).as_matrix(),
        linear_axis_axis=1,
        linear_axis_base=_transform([-30.0, 70.0, 10.0], [0.0, 0.0, -20.0]).as_matrix(),
    )
    kwargs = {
        "ee_transform": _transform([100.0, 20.0, -30.0], [10.0, -30.0, 20.0]).as_matrix(),
        "gantry_translation": (150.0, 80.0, -20.0),
        "linear_axis_distance": 175.0,
    }
    matrices = np.asarray([robot.forward(joints, **kwargs) for joints in _JOINTS])

    actual = robot.inverse_multiple(matrices, **kwargs)

    _assert_batches_equal(actual, [robot.inverse(matrix, **kwargs) for matrix in matrices])
    for solutions, joints in zip(actual, _JOINTS):
        assert any(np.allclose(solution, joints, atol=1e-8) for solution in solutions)


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped"])
@pytest.mark.parametrize("invalid", ["shape", "rotation", "nonfinite", "last-row"])
def test_internal_bulk_rejects_malformed_matrices_with_value_error(
    shaped: bool, invalid: str
) -> None:
    robot = _robot(shaped)
    matrices = np.repeat(np.eye(4)[None, :, :], 2, axis=0)
    if invalid == "shape":
        matrices = np.zeros((2, 3, 4))
        expected_message = "poses must have shape (n, 4, 4)"
    elif invalid == "rotation":
        matrices[1, :3, :3] = 0.0
        expected_message = "pose 1: pose matrix rotation axes must be unit length"
    elif invalid == "nonfinite":
        matrices[1, 0, 3] = np.nan
        expected_message = "pose 1: pose matrix values must be finite"
    else:
        matrices[1, 3, 0] = 1.0
        expected_message = "pose 1: pose matrix bottom row must be [0, 0, 0, 1]"

    with pytest.raises(ValueError) as error:
        robot._robot.inverse_multiple(matrices)

    assert str(error.value) == expected_message
    if invalid == "shape":
        assert type(error.value) is ValueError
        assert not hasattr(error.value, "pose_index")
    else:
        assert isinstance(error.value, BatchPoseError)
        assert type(error.value.pose_index) is int
        assert error.value.pose_index == 1
        assert type(error.value.__cause__) is ValueError
        assert str(error.value.__cause__) == expected_message.removeprefix("pose 1: ")


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped"])
@pytest.mark.parametrize("as_stack", [False, True], ids=["sequence", "stack"])
def test_public_bulk_exposes_invalid_pose_index(shaped: bool, as_stack: bool) -> None:
    robot = _robot(shaped)
    # SciPy accepts nonfinite translations; Rust validates them after the public
    # wrapper converts the transforms into a matrix buffer.
    translations = [[1e9, 1e9, 1e9], [np.nan, 0.0, 0.0], [1e9, 1e9, 1e9]]
    poses = (
        RigidTransform.from_translation(translations)
        if as_stack
        else [RigidTransform.from_translation(value) for value in translations]
    )

    with pytest.raises(BatchPoseError) as error:
        robot.inverse_multiple(poses)

    assert isinstance(error.value, ValueError)
    assert type(error.value.pose_index) is int
    assert error.value.pose_index == 1
    assert str(error.value) == "pose 1: pose matrix values must be finite"
    assert type(error.value.__cause__) is ValueError
    assert str(error.value.__cause__) == "pose matrix values must be finite"


@pytest.mark.parametrize("shaped", [False, True], ids=["robot", "shaped"])
@pytest.mark.parametrize("invalid_indices", [(4217,), (51, 4217, 9876)])
def test_internal_bulk_identifies_invalid_pose_in_large_batch(
    shaped: bool, invalid_indices: tuple[int, ...]
) -> None:
    robot = _robot(shaped)
    matrices = np.repeat(np.eye(4)[None, :, :], 10_000, axis=0)
    # Valid but unreachable poses keep the focus on validation, without solving
    # or collision-checking thousands of reachable configurations.
    matrices[:, :3, 3] = 1e9
    matrices[list(invalid_indices), :3, :3] = 0.0

    with pytest.raises(BatchPoseError) as error:
        robot._robot.inverse_multiple(matrices)

    # Rayon may return any failing pose when several workers reject inputs.
    assert isinstance(error.value, ValueError)
    assert type(error.value.pose_index) is int
    assert error.value.pose_index in invalid_indices
    assert str(error.value) == (
        f"pose {error.value.pose_index}: pose matrix rotation axes must be unit length"
    )
    assert type(error.value.__cause__) is ValueError
    assert str(error.value.__cause__) == "pose matrix rotation axes must be unit length"


def _assert_batches_equal(actual: object, expected: list[list[tuple]]) -> None:
    assert isinstance(actual, list)
    assert len(actual) == len(expected)
    for solutions, scalar_solutions in zip(actual, expected):
        assert isinstance(solutions, list)
        assert len(solutions) == len(scalar_solutions)
        assert all(isinstance(solution, tuple) and len(solution) == 6 for solution in solutions)
        np.testing.assert_allclose(solutions, scalar_solutions, atol=1e-10, rtol=0)


def _stack(poses: list[RigidTransform]) -> RigidTransform:
    return RigidTransform.from_matrix(np.stack([pose.as_matrix() for pose in poses]))


def _units(joints: tuple, degrees: bool) -> tuple:
    return joints if degrees else tuple(np.deg2rad(joints))


def _robot(shaped: bool, degrees: bool = True, **kwargs) -> Robot | KinematicsWithShape:
    if shaped:
        return KinematicsWithShape(
            _model(),
            [_cube()] * 6,
            degrees=degrees,
            safety=SafetyDistances(
                to_environment=NEVER_COLLIDES, to_robot_default=NEVER_COLLIDES
            ),
            **kwargs,
        )
    return Robot(_model(), degrees=degrees, **kwargs)


def _model() -> KinematicModel:
    return KinematicModel(
        a1=400,
        a2=-250,
        b=0,
        c1=830,
        c2=1175,
        c3=1444,
        c4=230,
        flip_axes=(True, False, True, True, False, True),
    )


def _transform(translation: list[float], angles: list[float]) -> RigidTransform:
    return RigidTransform.from_components(
        rotation=Rotation.from_euler("xyz", angles, degrees=True),
        translation=translation,
    )


def _cube() -> Mesh:
    return Mesh.from_arrays(
        [
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 1.0, 1.0), (0.0, 1.0, 1.0),
        ],
        [
            (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
            (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
            (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7),
        ],
    )
