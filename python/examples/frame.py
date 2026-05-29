from __future__ import annotations

import numpy as np

from spherical_wrist import Robot

from _common import compose, dump_joints, dump_solutions, irb2400_10, translation


def main() -> None:
    robot = Robot(irb2400_10(), degrees=True)

    # The Rust Frame example shifts a pose by a small working-frame transform
    # and asks IK for nearby joint values. Python exposes frame support through
    # Robot(frame=...), and this direct composition shows the same underlying
    # operation explicitly.
    frame_transform = translation(0.011, 0.022, 0.033)
    joints_no_frame = (0.0, 6.3025, 12.6051, 17.1887, 5.7296, 28.6479)

    print("No frame transform:")
    dump_joints(joints_no_frame)

    unframed_pose = robot.forward(joints_no_frame)
    transformed_pose = compose(frame_transform, unframed_pose)

    print("\nPossible joint values after the frame transform:")
    solutions = robot.inverse_continuing(transformed_pose, joints_no_frame)
    dump_solutions(solutions)

    if solutions:
        framed_translation = robot.forward(solutions[0]).translation
        unframed_translation = unframed_pose.translation
        delta = framed_translation - unframed_translation
        print(
            "\nDistance between framed and unframed TCP translations:",
            np.round(delta, 4).tolist(),
        )


if __name__ == "__main__":
    main()
