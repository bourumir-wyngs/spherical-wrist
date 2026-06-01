from __future__ import annotations

from time import perf_counter

from spherical_wrist import RRTPlanner, visualize_robot

from _common import DEFAULT_TCP_BOX, create_rx160_robot, dump_solutions, wait_for_visualization


def example(name: str, start, goal, robot) -> None:
    print(f"\n** {name} **")
    planner = RRTPlanner(
        step_size_joint_space=3.0,
        max_try=2000,
        smooth=500,
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

    handle = visualize_robot(robot, path[0], DEFAULT_TCP_BOX)
    try:
        print("Playing planned path...")
        handle.play_path(path)
        wait_for_visualization(handle)
    finally:
        handle.close()


def main() -> None:
    # The RRT planner samples random joint configurations from the robot's
    # constraints and rejects configurations that collide with the shaped robot
    # or its environment.
    robot = create_rx160_robot()

    tough_start = (-120.0, -90.0, -92.51, 18.42, 82.23, 189.35)
    tough_goal = (40.0, -90.0, -92.51, 18.42, 82.23, 189.35)
    example("Bow deeply before these stones, robot!", tough_start, tough_goal, robot)

if __name__ == "__main__":
    main()
