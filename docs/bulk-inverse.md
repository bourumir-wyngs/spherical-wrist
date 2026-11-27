# Bulk Inverse Kinematics

Use `inverse_multiple` to convert many TCP poses in one call. Both `Robot` and
`KinematicsWithShape` solve poses in parallel in Rust with Rayon. Joint
constraints apply to every solution; `KinematicsWithShape` also filters
solutions using its configured collision geometry and safety distances.

## Inputs And Results

The preferred input for large batches is a SciPy `RigidTransform` stack, with
an underlying matrix array of shape `(n, 4, 4)`. A sequence of individual
`RigidTransform` objects is also accepted, but requires Python work to assemble
the matrices. An individual transform is treated as a batch of one, and an
empty sequence returns `[]`.

```python
import numpy as np
from scipy.spatial.transform import RigidTransform
from spherical_wrist import KinematicModel, Robot

model = KinematicModel(
    a1=0.100, a2=-0.135, b=0.0,
    c1=0.615, c2=0.705, c3=0.755, c4=0.085,
    offsets=(0.0, 0.0, -90.0, 0.0, 0.0, 0.0),
)
robot = Robot(model, degrees=True)
joints = (10.0, 20.0, -70.0, 30.0, 20.0, 10.0)

# Build 10,000 targets with a fixed orientation and varying TCP x position.
# Some offsets can leave the reachable workspace and yield empty solution lists.
matrices = np.repeat(robot.forward(joints).as_matrix()[None, :, :], 10_000, axis=0)
matrices[:, 0, 3] += np.linspace(-0.02, 0.02, len(matrices))
poses = RigidTransform.from_matrix(matrices)
solutions_by_pose = robot.inverse_multiple(poses)

print(len(solutions_by_pose))             # 10,000
print(len(solutions_by_pose[0]))          # number of valid IK solutions for pose 0
if solutions_by_pose[0]:
    print(solutions_by_pose[0][0])        # one six-float joint tuple
```

The return type is `list[list[Joints]]`. Output item `i` contains all returned
solutions for input pose `i`, preserving input order even when worker threads
finish out of order. Solution counts can differ between poses. An unreachable
pose, or one whose solutions are all rejected by constraints or collision
checks, has an empty list at its original index. Malformed input raises an
exception instead of being treated as an unreachable pose.

Joint input and output use degrees when the robot has `degrees=True`, and
radians when it has `degrees=False`. Pose translations use the model's distance
unit. Base, tool, frame, and parallelogram settings apply as they do for scalar
`inverse` calls. `Robot.inverse_multiple` also accepts a per-call `tool`
transform, or its compatibility alias `ee_transform`. As with scalar calls,
passing a per-call tool when the robot already has a configured tool raises an
error.

Each pose uses the same solving and solution ordering behavior as
`inverse(pose)` with its default `current_joints=None`. The bulk method does not
accept current joints; every pose is solved independently. For sequential
continuity, use `inverse_continuing` with the selected previous solution; for a
collision-free motion through poses, use
[CartesianPlanner](path-planning.md).

## Validation Errors

When Rust rejects an individual pose matrix, `inverse_multiple` raises
`BatchPoseError`, a subclass of `ValueError`. Its integer `pose_index` attribute
gives the zero-based index within the supplied batch, without parsing the error
message. The message also includes the index and validation reason, and
`__cause__` preserves the original `ValueError`.

```python
from spherical_wrist import BatchPoseError

try:
    solutions_by_pose = robot.inverse_multiple(poses)
except BatchPoseError as exc:
    print(exc.pose_index)
    print(exc.__cause__)  # original matrix validation error
```

For a chunk beginning at `start`, the index in the complete input is
`start + exc.pose_index`. If several matrices are invalid, parallel validation
may report any one of their indices. Whole-batch shape or type errors remain
ordinary `ValueError` or `TypeError` exceptions, without `pose_index`. SciPy may
also reject malformed matrices when constructing `RigidTransform` objects,
before `inverse_multiple` is called.

## Collision Filtering

A shaped robot uses the same call and return format:

```python
solutions_by_pose = shaped_robot.inverse_multiple(poses)
```

Every returned configuration satisfies the shaped robot's joint constraints
and passes its configured collision policy. This includes self-collision,
base, tool, and environment checks for the geometry and pairs that are enabled.
`NEVER_COLLIDES` excludes configured pairs; `SafetyDistances(mode="no_check")`
disables collision filtering, matching scalar `inverse` behavior.

Collision checks apply to the returned joint configurations. They do not check
the motions between poses or choose a continuous branch. See
[Meshes And Collisions](meshes-and-collisions.md) for configuring geometry.

## Large Batches

The binding copies the matrix data into an owned contiguous buffer before
releasing Python's GIL. Validation, IK solving, constraint and collision
filtering, and joint-unit conversion run in Rust while the GIL is released.
Python result objects are created after reacquiring the GIL. A transform stack
avoids a Python loop over input poses, and returned tuples are assembled by the
native binding.

All results are materialized for a call. For very large inputs, process chunks
and consume or write each chunk before requesting the next one:

```python
chunk_size = 10_000
for start in range(0, len(poses), chunk_size):
    chunk = poses[start:start + chunk_size]
    solutions = robot.inverse_multiple(chunk)
    # Replace this with writing or processing these results; do not retain
    # every chunk if the goal is to limit result memory.
    print(start, sum(len(per_pose) for per_pose in solutions))
```

If the poses themselves are too large to hold in memory, read matrix chunks
from the data source and construct one `RigidTransform` stack per chunk.

Rayon uses its global worker pool. To limit worker threads, set
`RAYON_NUM_THREADS` before the first parallel operation, for example:

```bash
RAYON_NUM_THREADS=4 python python/examples/bulk_inverse.py --count 100000 --chunk-size 10000
```

Throughput depends on batch size, geometry complexity, solution counts, and
available CPU cores. For tiny inputs, input conversion, scheduling, and Python
result allocation can dominate. The runnable
[bulk_inverse.py](../python/examples/bulk_inverse.py) example times both robot
types without opening a visualization window. It uses `create_rx160_robot`
from [`_common.py`](../python/examples/_common.py) to load the bundled RX160
meshes for all six links, the base, the tool, and four environment objects.
The comparison `Robot` uses the same RX160 model, base, tool, and joint
constraints, so the shaped robot's collision filtering is the difference
between their results.
