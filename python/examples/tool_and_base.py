from __future__ import annotations

from spherical_wrist import Robot

from _common import dump_joints, dump_solutions, staubli_tx2_160l, translation


def main() -> None:
    # Degree equivalent of the Rust joint set [0.0, 0.1, ..., 0.5] radians.
    joints = (0.0, 5.7296, 11.4592, 17.1887, 22.9183, 28.6479)
    dump_joints(joints)

    # Put the robot on a 0.5 m pedestal and attach a 1 m tool in the local
    # Z direction, similar to a long pointer or welding torch.
    robot = Robot(
        staubli_tx2_160l(),
        degrees=True,
        base=translation(0.0, 0.0, 0.5),
        tool=translation(0.0, 0.0, 1.0),
    )

    tcp_pose = robot.forward(joints)
    print("The tool tip translation is:", tcp_pose.translation.round(4).tolist())

    # The configured base and tool are part of the robot, so the usual inverse
    # methods operate on the transformed TCP pose directly.
    dump_solutions(robot.inverse_continuing(tcp_pose, joints))


if __name__ == "__main__":
    main()
