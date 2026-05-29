from spherical_wrist import (
    CartesianPlanner,
    Constraints,
    KinematicModel,
    KinematicsWithShape,
    Mesh,
    NEVER_COLLIDES,
    RRTPlanner,
    SafetyDistances,
    VisualizationHandle,
    visualize_robot,
    visualize_robot_with_safety,
)
import pytest


def test_visualization_bindings_validate_tcp_box_without_opening_window() -> None:
    robot = _shape_robot()
    joints = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    invalid_tcp_box = ((1.0, 0.0), (-1.0, 1.0), (-1.0, 1.0))

    with pytest.raises(ValueError, match="tcp_box"):
        visualize_robot(robot, joints, invalid_tcp_box)

    with pytest.raises(ValueError, match="tcp_box"):
        visualize_robot_with_safety(
            robot,
            joints,
            invalid_tcp_box,
            SafetyDistances.standard(),
        )

    with pytest.raises(ValueError, match="tcp_box"):
        robot.visualize(joints, invalid_tcp_box)


def test_visualization_handle_can_play_annotated_path_without_opening_window() -> None:
    robot = _shape_robot()
    start = (10.0, 20.0, -70.0, 30.0, 20.0, 10.0)
    pose = robot.forward(start)
    path = CartesianPlanner(
        rrt=RRTPlanner(step_size_joint_space=720.0, max_try=1),
        debug=False,
    ).plan(robot, start, pose, [], pose)
    fake_internal = _RecordingVisualizationInternal()
    handle = VisualizationHandle._from_internal(fake_internal)

    handle.play_path(path, interval=0.0)

    assert fake_internal.joints == [tuple(step.joints) for step in path]


def test_visualization_handle_rejects_negative_playback_interval() -> None:
    handle = VisualizationHandle._from_internal(_RecordingVisualizationInternal())

    with pytest.raises(ValueError, match="interval"):
        handle.play_path([], interval=-0.1)


class _RecordingVisualizationInternal:
    def __init__(self) -> None:
        self.is_running = True
        self.joints: list[tuple[float, float, float, float, float, float]] = []

    def set_joints(self, joints) -> None:
        self.joints.append(tuple(joints))


def _shape_robot() -> KinematicsWithShape:
    return KinematicsWithShape(
        _model(),
        [_unit_cube()] * 6,
        degrees=True,
        constraints=Constraints(
            (-180.0, -180.0, -180.0, -180.0, -180.0, -180.0),
            (180.0, 180.0, 180.0, 180.0, 180.0, 180.0),
        ),
        safety=SafetyDistances(
            to_environment=NEVER_COLLIDES,
            to_robot_default=NEVER_COLLIDES,
        ),
    )


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


def _unit_cube() -> Mesh:
    return Mesh.from_arrays(_cube_vertices(), _cube_triangles())


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
