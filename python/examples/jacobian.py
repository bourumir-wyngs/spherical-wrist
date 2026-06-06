from __future__ import annotations

import numpy as np

from spherical_wrist import Jacobian, Robot

from _common import dump_joints, irb2400_10


def main() -> None:
    robot = Robot(irb2400_10(), degrees=True)
    joints = (0.0, 10.0, -45.0, 25.0, 30.0, 40.0)

    jacobian = Jacobian(robot, joints)
    matrix = jacobian.matrix()

    print("Jacobian rows are [vx, vy, vz, wx, wy, wz].")
    print("Columns use the robot's joint unit, so these columns map degree/s to TCP twist.")
    print(np.round(matrix, 6))

    desired_twist = np.array([0.02, 0.0, 0.01, 0.0, 0.0, 0.05])
    joint_rates = jacobian.velocities_from_vector(desired_twist)
    reconstructed_twist = matrix @ np.asarray(joint_rates)

    print("\nJoint rates for desired TCP twist:")
    dump_joints(joint_rates, label="degree/s")
    print("Reconstructed twist:", np.round(reconstructed_twist, 6).tolist())

    wrench = (0.0, 0.0, 25.0, 0.0, 0.0, 2.0)
    efforts = jacobian.torques_from_vector(wrench)
    print("\nGeneralized efforts for wrench [fx, fy, fz, tx, ty, tz]:")
    dump_joints(efforts, label="per degree")

    raw_radian_matrix = jacobian.matrix(radians=True)
    print("\nRaw per-radian matrix norm:", f"{np.linalg.norm(raw_radian_matrix):.6f}")


if __name__ == "__main__":
    main()
