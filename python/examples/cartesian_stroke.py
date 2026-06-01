from __future__ import annotations

from time import perf_counter

import numpy as np
from scipy.spatial.transform import RigidTransform, Rotation

from spherical_wrist import (
    CartesianPlanner,
    PATH_FLAG_BACKWARDS,
    PATH_FLAG_DEBUG,
    PATH_FLAG_FORWARDS,
    PATH_FLAG_LAND,
    PATH_FLAG_LANDING,
    PATH_FLAG_LIN_INTERP,
    PATH_FLAG_ONBOARDING,
    PATH_FLAG_ORIGINAL,
    PATH_FLAG_PARK,
    PATH_FLAG_PARKING,
    PATH_FLAG_RECONFIGURING,
    PATH_FLAG_TRACE,
    RRTPlanner,
    visualize_robot,
)

from _common import DEFAULT_TCP_BOX, create_rx160_robot, wait_for_visualization


FLAG_NAMES = [
    (PATH_FLAG_ONBOARDING, "ONBOARDING"),
    (PATH_FLAG_TRACE, "TRACE"),
    (PATH_FLAG_LIN_INTERP, "LIN_INTERP"),
    (PATH_FLAG_LAND, "LAND"),
    (PATH_FLAG_LANDING, "LANDING"),
    (PATH_FLAG_PARK, "PARK"),
    (PATH_FLAG_PARKING, "PARKING"),
    (PATH_FLAG_FORWARDS, "FORWARDS"),
    (PATH_FLAG_BACKWARDS, "BACKWARDS"),
    (PATH_FLAG_RECONFIGURING, "RECONFIGURING"),
    (PATH_FLAG_ORIGINAL, "ORIGINAL"),
    (PATH_FLAG_DEBUG, "DEBUG"),
]


def describe_flags(flags: int) -> str:
    names = [name for value, name in FLAG_NAMES if flags & value == value]
    return "|".join(names) if names else "NONE"


def main() -> None:
    # FirstCollisionOnly is enough for path planning and saves work while the
    # planner samples and validates many intermediate states.
    robot = create_rx160_robot(
        first_collision_only=True,
        flag_mesh="flag.stl",
        j6_limit=225.0,
    )

    def pose(joints):
        # The Rust example defines land/step/park poses through known joint
        # configurations to keep the example reproducible.
        return robot.forward(joints)

    def tcp_pose(x: float, y: float, z: float) -> RigidTransform:
        return RigidTransform.from_components(
            rotation=Rotation.from_euler("z", -90.0, degrees=True),
            translation=[x, y, z],
        )

    # Initial position of the robot
    start = (20, 50.0, 90, 180, -40, 122)
    # The planner will safely move from "start" to "land"
    # "land" is normally where the tool is activated so there is a special flag
    # "landing" is present in movement between landing pose and stroke (tool warming up)
    land = tcp_pose(1.50, 0.0, 1.6)
    steps = [
        # The actual stroke. This path is rectangle box.
        tcp_pose(1.50, 0.0, 1.7),
        tcp_pose(1.00, 0.0, 1.7),
        tcp_pose(1.00, 1.15, 1.7),
        tcp_pose(1.50, 1.15, 1.7),
        tcp_pose(1.50, 0.0, 1.7),

        tcp_pose(1.50, 0.0, 2.0),
        tcp_pose(1.00, 0.0, 2.0),
        tcp_pose(1.00, 1.15, 2.0),
        tcp_pose(1.50, 1.15, 2.0),
        tcp_pose(1.50, 0.0, 2.0),
    ]
    # "Post last" position with special flag to deactivate the tool
    park = land

    planner = CartesianPlanner(
        check_step_m=0.05,
        check_step_rad=4.0,
        max_transition_cost=3.0,
        linear_recursion_depth=8,
        rrt=RRTPlanner(
            step_size_joint_space=2.0,
            max_try=100,
            debug=False,
        ),
        allow_reconfigure=True,
        include_linear_interpolation=True,
        debug=False,
    )

    started = perf_counter()
    try:
        path = planner.plan(robot, start, land, steps, park)
    except ValueError as exc:
        print("Failed:", exc)
        return

    for index, step in enumerate(path):
        joints = np.round(np.asarray(step.joints), 4).tolist()
        print(f"{index:03d}: {joints}  {step.move_into}  {describe_flags(step.flags)}")

    print(f"Took {perf_counter() - started:.3f} s")

    handle = visualize_robot(robot, path[0].joints, DEFAULT_TCP_BOX)
    try:
        print("Playing planned path...")
        handle.play_path(path)
        wait_for_visualization(handle)
    finally:
        handle.close()


if __name__ == "__main__":
    main()
