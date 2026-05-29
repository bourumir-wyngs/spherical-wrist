from spherical_wrist import (
    Constraints,
    CartesianPlanner,
    KinematicModel,
    KinematicsWithShape,
    Mesh,
    NEVER_COLLIDES,
    PATH_FLAG_LAND,
    PATH_FLAG_PARK,
    RRTPlanner,
    SafetyDistances,
)
import numpy as np


def test_rrt_planner_returns_collision_free_joint_path() -> None:
    robot = _shape_robot()
    start = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    goal = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    planner = RRTPlanner(step_size_joint_space=720.0, max_try=1)

    path = planner.plan_rrt(robot, start, goal)

    assert len(path) >= 2
    assert np.allclose(path[0], start)
    assert np.allclose(path[-1], goal)
    assert all(not robot.collides(step) for step in path)


def test_cartesian_planner_returns_annotated_joints() -> None:
    robot = _shape_robot()
    start = (10.0, 20.0, -70.0, 30.0, 20.0, 10.0)
    pose = robot.forward(start)
    planner = CartesianPlanner(
        rrt=RRTPlanner(step_size_joint_space=720.0, max_try=1),
        debug=False,
    )

    path = planner.plan(robot, start, pose, [], pose)

    assert len(path) >= 2
    assert path[0].has_flag(PATH_FLAG_LAND)
    assert path[-1].has_flag(PATH_FLAG_PARK)
    assert all(len(step.joints) == 6 for step in path)
    assert all(not robot.collides(step.joints) for step in path)


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
