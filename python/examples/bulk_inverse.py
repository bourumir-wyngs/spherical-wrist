"""Bulk RX160 IK with constraints and mesh collision filtering, without a GUI.

Run from the package root:
    python python/examples/bulk_inverse.py --count 10000 --chunk-size 2000

Uses the bundled RX160 link, base, tool, and environment meshes through
create_rx160_robot, as in the path planning and visualization examples.
"""

from __future__ import annotations

import argparse
from time import perf_counter

import numpy as np
from scipy.spatial.transform import RigidTransform

from spherical_wrist import Joints, KinematicsWithShape, Robot

from _common import create_rx160_robot, staubli_rx160, translation


def run_bulk(
    name: str,
    solver: Robot | KinematicsWithShape,
    targets: np.ndarray,
    count: int,
    chunk_size: int,
) -> list[list[Joints]]:
    """Process the workload in chunks and return solutions for each distinct target."""
    total_solutions = 0
    empty_poses = 0
    elapsed = 0.0
    for start in range(0, count, chunk_size):
        stop = min(start + chunk_size, count)
        # Repeat the distinct targets to create a large, predictable workload.
        # Generate only this chunk; a real application can instead read
        # distinct pose matrices from a file or generate them with NumPy.
        indices = np.arange(start, stop) % len(targets)

        # A single RigidTransform stack is the fast input form. The wrapper
        # transfers its numeric matrix buffer without a Python loop over
        # individual poses. The last chunk may be smaller than chunk-size.
        poses = RigidTransform.from_matrix(targets[indices])

        # One call dispatches the chunk to Rayon in Rust. The GIL is released
        # during solving and collision filtering. Poses are independent:
        # this does not choose a continuous branch or check motion between
        # targets; use a path planner when those guarantees are needed.
        # Time the bulk call, including result conversion to Python, while
        # excluding mesh loading, target generation, and checks below.
        began = perf_counter()
        results = solver.inverse_multiple(poses)
        elapsed += perf_counter() - began
        # results[i] is a list of J1-J6 tuples for poses[i], in degrees.
        # Solution counts vary: constraints, reachability, and collisions
        # can leave several solutions or none. Input order is preserved
        # even when Rayon workers finish poses in a different order.
        assert len(results) == stop - start
        total_solutions += sum(len(solutions) for solutions in results)
        empty_poses += sum(not solutions for solutions in results)
        # Consume or save each chunk before proceeding. Accumulating every
        # result would defeat the memory benefit of chunking a huge input.
        del results

    # Validate just the distinct targets, outside the timed section.
    # The bulk method should match inverse(pose), including solution order;
    # checking every repeated target would only repeat the same work.
    samples = solver.inverse_multiple(RigidTransform.from_matrix(targets))
    for matrix, solutions in zip(targets, samples):
        expected = solver.inverse(RigidTransform.from_matrix(matrix))
        np.testing.assert_allclose(solutions, expected, atol=1e-8)
    assert samples[-1] == []  # Unreachable input keeps its position.
    counts = [len(solutions) for solutions in samples]

    print(f"{name}: {count:,} poses in {elapsed:.3f} s across chunks")
    print(f"  {total_solutions:,} solutions; {empty_poses:,} poses have no valid solution")
    print(f"  Solution counts for the {len(targets)} distinct targets: {counts}")
    return samples


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    # --count controls the total workload; --chunk-size controls how many
    # poses and their solutions are held in memory for each bulk call.
    parser.add_argument("--count", type=int, default=10_000)
    parser.add_argument("--chunk-size", type=int, default=2_000)
    args = parser.parse_args()
    if args.chunk_size < 1:
        parser.error("--chunk-size must be positive")

    # The shared factory loads all six links, the base, the flag-shaped tool,
    # and four environment objects. It also configures joint limits and safety
    # distances, so collision filtering uses the same setup as other examples.
    # Stop each collision check at the first offending pair, while still
    # returning every collision-free IK solution for each target pose.
    shaped = create_rx160_robot(first_collision_only=True)

    # Match the factory's model, constraints, and base/tool transforms to
    # compare kinematics alone with the same poses filtered by the configured
    # mesh collision rules, including the factory's exemptions for some pairs.
    # The RX160 model uses metres: the base is translated in the world frame,
    # and the TCP is 0.5 m along the flange's local Z axis. Both robots use
    # degrees for joint angles, as configured by create_rx160_robot.
    robot = Robot(
        staubli_rx160(),
        degrees=shaped.degrees,
        constraints=shaped.constraints,
        base=translation(0.4, 0.7, 0.0),
        tool=translation(0.0, 0.0, 0.5),
    )
    # These joint configurations generate reproducible, reachable TCP poses.
    # Forward kinematics itself does not check joint limits or collisions.
    # They are only used to create input data; inverse_multiple receives poses
    # and independently finds their solutions without a joint-position seed.
    configurations = [
        (10.0, 20.0, -70.0, 30.0, 20.0, 10.0),
        (50.0, 25.0, -65.0, 30.0, 25.0, 10.0),
        (-40.0, 15.0, -75.0, 30.0, 15.0, 10.0),
    ]
    # Each forward result is a SciPy RigidTransform. Stacking its homogeneous
    # 4x4 matrix gives the (number_of_poses, 4, 4) layout used for bulk input.
    # Translation is in the last column; the upper-left 3x3 block is rotation.
    matrices = np.stack([shaped.forward(joints).as_matrix() for joints in configurations])

    # Check that both robot setups describe the same TCP before comparing IK.
    for joints, matrix in zip(configurations, matrices):
        np.testing.assert_allclose(robot.forward(joints).as_matrix(), matrix, atol=1e-8)

    # Add a valid transform far outside the workspace. An unreachable pose
    # produces an empty solution list at its input index, not an exception.
    unreachable = np.eye(4)
    unreachable[:3, 3] = (100.0, 100.0, 100.0)
    targets = np.concatenate([matrices, unreachable[None, :, :]])
    # Include every distinct target at least once in the timed workload.
    if args.count < len(targets):
        parser.error(f"--count must be at least {len(targets)} to include every distinct target")

    # Run each solver explicitly, then compare their results for the same
    # distinct targets to see which IK branches collision filtering rejects.
    plain_samples = run_bulk("Robot", robot, targets, args.count, args.chunk_size)
    shaped_samples = run_bulk("KinematicsWithShape", shaped, targets, args.count, args.chunk_size)
    plain_counts = [len(solutions) for solutions in plain_samples]
    shaped_counts = [len(solutions) for solutions in shaped_samples]

    # Collision filtering can only remove solutions from the matching plain
    # solver. These targets exercise rejection of some branches while retaining
    # others. Also check every retained sample directly against the meshes.
    assert all(count <= plain for count, plain in zip(shaped_counts, plain_counts))
    assert any(count < plain for count, plain in zip(shaped_counts, plain_counts))
    assert all(not shaped.collides(joints) for solutions in shaped_samples for joints in solutions)


if __name__ == "__main__":
    main()
