from __future__ import annotations

from spherical_wrist import BY_CONSTRAINTS, BY_PREV, CONSTRAINT_CENTERED, Constraints, Robot

from _common import dump_solutions, irb2400_10


def main() -> None:
    # The limits match the Rust example:
    # [-0.1, 0, 0, 0, -pi, -pi] .. [pi, pi, 2pi, pi, pi, pi].
    # Here they are expressed in degrees, which is the Python wrapper default.
    lower = (-5.7296, 0.0, 0.0, 0.0, -180.0, -180.0)
    upper = (180.0, 180.0, 360.0, 180.0, 180.0, 180.0)

    constraints_by_previous = Constraints(lower, upper, sorting_weight=BY_PREV)
    robot = Robot(
        irb2400_10(),
        degrees=True,
        constraints=constraints_by_previous,
    )

    joints = (0.0, 5.7296, 11.4592, 17.1887, 0.0, 28.6479)
    previous_j6_zero = (0.0, 6.3025, 12.6051, 45.8366, 5.7296, 0.0)
    pose = robot.forward(joints)

    print("\nPrefer the solution closest to the center of the constraints:")
    dump_solutions(robot.inverse_continuing(pose, CONSTRAINT_CENTERED))

    print("\nWith constraints, sorted by proximity to the previous pose:")
    dump_solutions(robot.inverse_continuing(pose, previous_j6_zero))

    constraints_by_center = Constraints(lower, upper, sorting_weight=BY_CONSTRAINTS)
    robot = Robot(
        irb2400_10(),
        degrees=True,
        constraints=constraints_by_center,
    )

    print("\nWith constraints, sorted by proximity to the constraint center:")
    dump_solutions(robot.inverse_continuing(pose, previous_j6_zero))


if __name__ == "__main__":
    main()
