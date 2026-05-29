from __future__ import annotations

import numpy as np

from spherical_wrist import J2, J3, Parallelogram, Robot

from _common import dump_joints, irb2400_10


def euler_xyz_degrees(pose) -> np.ndarray:
    return pose.rotation.as_euler("xyz", degrees=True)


def print_orientation_change(initial: np.ndarray, modified: np.ndarray, label: str) -> None:
    roll, pitch, yaw = modified - initial
    print(
        f"{label}: roll = {roll:.3f}, pitch = {pitch:.3f}, yaw = {yaw:.3f}",
    )


def travel_distance(initial_pose, modified_pose) -> float:
    return float(np.linalg.norm(initial_pose.translation - modified_pose.translation))


def main() -> None:
    # Robot without parallelogram coupling.
    robot_no_parallelogram = Robot(irb2400_10(), degrees=True)

    # Robot with joint 2 driving joint 3 through a parallelogram relation.
    robot_with_parallelogram = Robot(
        irb2400_10(),
        degrees=True,
        parallelogram=Parallelogram(scaling=1.0, driven=J2, coupled=J3),
    )

    joints = (0.0, 5.7, 11.5, 17.2, 0.0, 28.6)
    print("\nInitial joints:")
    dump_joints(joints)

    pose_no_parallelogram = robot_no_parallelogram.forward(joints)
    pose_with_parallelogram = robot_with_parallelogram.forward(joints)

    initial_orientation_no_para = euler_xyz_degrees(pose_no_parallelogram)
    initial_orientation_with_para = euler_xyz_degrees(pose_with_parallelogram)

    modified_joints = list(joints)
    modified_joints[J2] += 10.0
    modified_joints = tuple(modified_joints)
    print("\nModified joints, with joint 2 increased by 10 degrees:")
    dump_joints(modified_joints)

    modified_pose_no_para = robot_no_parallelogram.forward(modified_joints)
    modified_pose_with_para = robot_with_parallelogram.forward(modified_joints)

    print("\nOrientation changes after joint change:")
    print_orientation_change(
        initial_orientation_no_para,
        euler_xyz_degrees(modified_pose_no_para),
        "Robot without parallelogram",
    )
    print_orientation_change(
        initial_orientation_with_para,
        euler_xyz_degrees(modified_pose_with_para),
        "Robot with parallelogram",
    )

    print("\nTravel distances after joint change:")
    print(
        "Robot without parallelogram:",
        f"{travel_distance(pose_no_parallelogram, modified_pose_no_para):.6f}",
    )
    print(
        "Robot with parallelogram:",
        f"{travel_distance(pose_with_parallelogram, modified_pose_with_para):.6f}",
    )


if __name__ == "__main__":
    main()
