from __future__ import annotations

from spherical_wrist import Robot

from _common import dump_solutions, irb2400_10


def main() -> None:
    # Create a robot with the same built-in parameter set used in the Rust
    # README example. Plain Robot has kinematics only; no collision checks.
    robot = Robot(irb2400_10(), degrees=True)

    # The Rust example passes radians. Python scripts normally use degrees, so
    # these are the degree equivalents of [0.0, 0.1, 0.2, 0.3, 0.0, 0.5].
    joints = (0.0, 5.7296, 11.4592, 17.1887, 0.0, 28.6479)
    pose = robot.forward(joints)

    # The continuing variant uses a previous position to sort the IK solutions
    # and to resolve wrist singularities consistently.
    when_continuing_from = (0.0, 6.3025, 12.6051, 17.1887, 5.7296, 28.6479)
    solutions = robot.inverse_continuing(pose, when_continuing_from)

    print("TCP translation:", pose.translation.round(4).tolist())
    dump_solutions(solutions)


if __name__ == "__main__":
    main()
