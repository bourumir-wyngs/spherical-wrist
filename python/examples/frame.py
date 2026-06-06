from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from spherical_wrist import Frame, Robot

from _common import dump_joints, dump_solutions, irb2400_10


def main() -> None:
    robot = Robot(irb2400_10(), degrees=True)

    # This is the original, canonical program. In a real cell these joint
    # positions might be taught once against a fixture, pallet, or workpiece at
    # a nominal location.
    canonical_program_rad = np.array(
        [
            [0.0, 0.11, 0.22, 0.30, 0.10, 0.50],
            [0.04, 0.14, 0.19, 0.34, 0.12, 0.45],
            [-0.03, 0.10, 0.25, 0.27, 0.08, 0.56],
        ],
        dtype=np.float64,
    )
    canonical_program = [tuple(np.rad2deg(row)) for row in canonical_program_rad]

    # Tie points are corresponding points between the original program and the
    # required target setup. Three non-collinear tie point pairs define the
    # transport frame: source/original trajectory points -> target/measured
    # points. The frame may include uniform scale in addition to rotation and
    # translation.
    original_tie_points = np.array(
        [robot.forward(joints).translation for joints in canonical_program],
        dtype=np.float64,
    )

    # Simulate the workpiece being moved, slightly rotated, and uniformly scaled
    # around the first tie point. In production these target tie points would be
    # measured or provided by calibration.
    target_rotation = Rotation.from_euler("z", 5.0, degrees=True)
    target_shift = np.array([0.011, 0.022, 0.033])
    target_scale = 1.02
    anchor = original_tie_points[0]
    target_tie_points = anchor + target_shift + target_rotation.apply(
        (original_tie_points - anchor) * target_scale
    )

    print("Tie point pairs used to compute the transport frame:")
    for index, (original, target) in enumerate(
        zip(original_tie_points, target_tie_points),
        start=1,
    ):
        print(
            f"  {index}: original {_format_point(original)} -> target {_format_point(target)}"
        )

    transport_frame = Frame.from_tie(original_tie_points, target_tie_points)
    print(f"Computed frame scale: {transport_frame.scale:.6f}")

    print("\nCanonical trajectory retargeted through the tie-point frame:")
    previous = canonical_program[0]
    for index, canonical_joints in enumerate(canonical_program, start=1):
        print(f"\nWaypoint {index}")
        print("Canonical joints:")
        dump_joints(canonical_joints)

        canonical_pose = robot.forward(canonical_joints)
        target_pose = transport_frame.transform_pose(canonical_pose)
        solutions = robot.inverse_continuing(target_pose, previous)

        print("Retargeted joint solutions:")
        dump_solutions(solutions)

        if solutions:
            selected = solutions[0]
            achieved_pose = robot.forward(selected)
            translation_error = np.linalg.norm(
                achieved_pose.translation - target_pose.translation
            )
            print(
                "Target TCP",
                _format_point(target_pose.translation),
                "achieved",
                _format_point(achieved_pose.translation),
                f"translation error {translation_error:.6f}",
            )
            previous = selected
        else:
            print("No IK solution for this retargeted waypoint")


def _format_point(point) -> str:
    return f"[{point[0]:.4f}, {point[1]:.4f}, {point[2]:.4f}]"


if __name__ == "__main__":
    main()
