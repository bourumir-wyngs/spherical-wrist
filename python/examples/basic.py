from __future__ import annotations

import numpy as np

from spherical_wrist import Robot

from _common import dump_joints, dump_solutions, irb2400_10


def main() -> None:
    # The Rust example uses radians directly. The Python wrapper defaults to
    # degrees, which is usually friendlier for scripts and UI work.
    robot = Robot(irb2400_10(), degrees=True)

    # J5 is exactly zero here, so this pose is singular. Plain inverse() can
    # lose one equivalent solution because J4/J6 become coupled at the singularity.
    joints = (0.0, 5.7296, 11.4592, 17.1887, 0.0, 28.6479)
    print("\nInitial joints with singularity J5 = 0:")
    dump_joints(joints)

    pose = robot.forward(joints)

    print("\nSolutions from plain inverse(). The original set may be absent:")
    dump_solutions(robot.inverse(pose))

    print("\nSolutions when continuing from a nearby previous position:")
    near_previous = (0.0, 6.3025, 12.6051, 17.1887, 5.7296, 28.6479)
    dump_solutions(robot.inverse_continuing(pose, near_previous))

    print("\nSame TCP, but previous motion had J4 carrying most of the rotation:")
    previous_j6_zero = (0.0, 6.3025, 12.6051, 45.8366, 5.7296, 0.0)
    dump_solutions(robot.inverse_continuing(pose, previous_j6_zero))

    print("\nIf no previous position is known, zero joints can be used as a neutral hint:")
    dump_solutions(robot.inverse_continuing(pose, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)))

    print("\n5 DOF inverse, forcing J6 to 77 degrees:")
    solutions_5dof = robot.inverse_5dof(pose, j6=77.0)
    dump_solutions(solutions_5dof)

    print("\nAll returned 6-DOF solutions keep the same TCP translation:")
    for solution in robot.inverse_continuing(pose, near_previous):
        translation = robot.forward(solution).translation
        print(f"  x={translation[0]:.3f}, y={translation[1]:.3f}, z={translation[2]:.3f}")

    print("Original TCP:", np.round(pose.translation, 3).tolist())


if __name__ == "__main__":
    main()
