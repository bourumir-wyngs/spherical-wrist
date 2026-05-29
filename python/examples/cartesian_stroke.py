from __future__ import annotations

from time import perf_counter

import numpy as np

from spherical_wrist import (
    CartesianPlanner,
    PATH_FLAG_BACKWARDS,
    PATH_FLAG_CARTESIAN,
    PATH_FLAG_DEBUG,
    PATH_FLAG_FORWARDS,
    PATH_FLAG_LAND,
    PATH_FLAG_LANDING,
    PATH_FLAG_LIN_INTERP,
    PATH_FLAG_ONBOARDING,
    PATH_FLAG_ORIGINAL,
    PATH_FLAG_PARK,
    PATH_FLAG_PARKING,
    PATH_FLAG_TRACE,
    RRTPlanner,
)

from _common import create_rx160_robot


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
    (PATH_FLAG_ORIGINAL, "ORIGINAL"),
    (PATH_FLAG_DEBUG, "DEBUG"),
    (PATH_FLAG_CARTESIAN, "CARTESIAN"),
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

    start = (-120.0, -90.0, -92.51, 18.42, 82.23, 189.35)
    land = pose((-120.0, -10.0, -92.51, 18.42, 82.23, 189.35))
    steps = [
        pose((-225.0, -27.61, 88.35, -85.42, 44.61, 138.0)),
        pose((-225.0, -33.02, 134.48, -121.08, 54.82, 191.01)),
    ]
    park = pose((-225.0, -27.61, 88.35, -85.42, 44.61, 110.0))

    planner = CartesianPlanner(
        check_step_m=0.02,
        check_step_rad=3.0,
        max_transition_cost=3.0,
        linear_recursion_depth=8,
        rrt=RRTPlanner(
            step_size_joint_space=2.0,
            max_try=100,
            debug=False,
        ),
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
        print(f"{index:03d}: {joints}  {describe_flags(step.flags)}")

    print(f"Took {perf_counter() - started:.3f} s")


if __name__ == "__main__":
    main()
