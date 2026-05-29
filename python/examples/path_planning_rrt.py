from __future__ import annotations

from time import perf_counter

from spherical_wrist import RRTPlanner

from _common import create_rx160_robot, dump_solutions


def example(name: str, start, goal, robot) -> None:
    print(f"\n** {name} **")
    planner = RRTPlanner(
        step_size_joint_space=3.0,
        max_try=2000,
        debug=True,
    )

    started = perf_counter()
    try:
        path = planner.plan_rrt(robot, start, goal)
    except ValueError as exc:
        print("Planning failed:", exc)
        return

    print(f"Took {perf_counter() - started:.3f} s")
    dump_solutions(path)


def main() -> None:
    # The RRT planner samples random joint configurations from the robot's
    # constraints and rejects configurations that collide with the shaped robot
    # or its environment.
    robot = create_rx160_robot()

    tough_start = (-120.0, -90.0, -92.51, 18.42, 82.23, 189.35)
    tough_goal = (40.0, -90.0, -92.51, 18.42, 82.23, 189.35)
    example("Tough example", tough_start, tough_goal, robot)

    simple_start = (-120.0, -90.0, -92.51, 18.42, 82.23, 189.35)
    simple_goal = (-120.0, -80.0, -90.0, 18.42, 82.23, 189.35)
    example("Simple example", simple_start, simple_goal, robot)


if __name__ == "__main__":
    main()
