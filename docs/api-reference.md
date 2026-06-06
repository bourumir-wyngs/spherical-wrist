# API Reference

This page is a compact map of the public Python API. Use IDE tooltips and the
Python wrapper docstrings for exact signatures.

## Types

`Joints`
: Tuple of six floats: J1 through J6.

`TcpBox`
: Three `(min, max)` pairs for visualization TCP controls: x, y, z.

`KinematicModel`
: OPW geometry parameters, offsets, and axis flips.

`Robot`
: Kinematics-only robot.

`Frame`
: Working-frame retargeting transform. Build with `Frame.from_tie(...)` when
  the measured target tie points may include uniform scale.

`KinematicsWithShape`
: Robot with six joint meshes and optional collision geometry.

`Mesh`
: Triangle mesh with loading, transforms, and pairwise collision checks.

`Constraints`
: Joint limits and IK sorting preference.

`SafetyDistances`
: Collision/proximity policy for robot, tool, base, and environment pairs.

`RRTPlanner`
: Joint-space collision-free planner.

`CartesianPlanner`
: Cartesian TCP stroke planner.

`AnnotatedJoints`
: Cartesian planner output item with `joints`, `flags`, and `move_into`.

`VisualizationHandle`
: Non-blocking visualization window control handle.

`Jacobian`
: Numerical 6x6 TCP Jacobian for one robot joint configuration.

## Constants

Joint and collision object angle indices in a pose array:

- `J1`, `J2`, `J3`, `J4`, `J5`, `J6`
- `J_TOOL`
- `J_BASE`

Constraint sorting:

- `BY_PREV`
- `BY_CONSTRAINTS`
- `CONSTRAINT_CENTERED`

Safety:

- `NEVER_COLLIDES`
- `TOUCH_ONLY`
- `CHECK_MODE_ALL`
- `CHECK_MODE_FIRST_COLLISION_ONLY`
- `CHECK_MODE_NO_CHECK`

Cartesian path flags:

- `PATH_FLAG_ONBOARDING`
- `PATH_FLAG_TRACE`
- `PATH_FLAG_LIN_INTERP`
- `PATH_FLAG_LAND`
- `PATH_FLAG_LANDING`
- `PATH_FLAG_PARK`
- `PATH_FLAG_PARKING`
- `PATH_FLAG_FORWARDS`
- `PATH_FLAG_BACKWARDS`
- `PATH_FLAG_RECONFIGURING`
- `PATH_FLAG_ORIGINAL`
- `PATH_FLAG_DEBUG`

Move kinds:

- `MOVE_KIND_JOINT`
- `MOVE_KIND_CARTESIAN`

## `Robot`

```python
Robot(
    kinematic_model,
    degrees=True,
    tool=None,
    base=None,
    frame=None,
    parallelogram=None,
    constraints=None,
)
```

Main methods:

- `forward(joints, tool=None, ee_transform=None)`
- `inverse(pose, current_joints=None, tool=None, ee_transform=None)`
- `inverse_continuing(pose, previous_joints, tool=None, ee_transform=None)`
- `inverse_5dof(pose, j6=0.0, tool=None, ee_transform=None)`
- `inverse_continuing_5dof(pose, previous_joints, tool=None, ee_transform=None)`
- `forward_with_joint_poses(joints)`
- `kinematic_singularity(joints)`

`frame` accepts either a SciPy `RigidTransform` or `Frame.from_tie(...)`.
Base and tool transforms are rigid. A `Frame` may include uniform scale and is
applied to the final TCP pose.

## `Frame`

```python
Frame.from_tie(
    original_tie_points,
    target_tie_points,
)
```

`original_tie_points` and `target_tie_points` must each have shape `(3, 3)`.
Each row is one `[x, y, z]` tie point. The target points may be translated,
rotated, and uniformly scaled relative to the original points. Non-uniform
scale and shear are rejected.

Main properties and methods:

- `scale`
- `translation`
- `as_matrix()`
- `transform_pose(pose)`
- `inverse_transform_pose(pose)`

## `KinematicsWithShape`

```python
KinematicsWithShape(
    kinematic_model,
    joint_meshes,
    degrees=True,
    constraints=None,
    base=None,
    tool=None,
    parallelogram=None,
    base_mesh=None,
    tool_mesh=None,
    environment=(),
    safety=None,
    first_collision_only=False,
)
```

Main methods:

- `forward`
- `inverse`
- `inverse_continuing`
- `inverse_5dof`
- `inverse_continuing_5dof`
- `forward_with_joint_poses`
- `collides`
- `collision_details`
- `near`
- `non_colliding_offsets`
- `positioned_robot`
- `visualize`

## Planners

```python
RRTPlanner(
    step_size_joint_space=3.0,
    max_try=2000,
    debug=False,
    radians=False,
    smooth=0,
)
```

```python
CartesianPlanner(
    check_step_m=0.02,
    check_step_rad=3.0,
    max_transition_cost=3.0,
    transition_coefficients=None,
    linear_recursion_depth=8,
    rrt=None,
    allow_reconfigure=True,
    max_solutions_await=3,
    include_linear_interpolation=True,
    debug=False,
    radians=False,
)
```

See [Path Planning](path-planning.md) for how to use them.

## `Jacobian`

```python
Jacobian(robot, joints, epsilon=1e-6, tool=None, ee_transform=None)
```

Main methods:

- `matrix(radians=False)`
- `velocities(linear_velocity, angular_velocity=(0.0, 0.0, 0.0), radians=False)`
- `velocities_fixed(vx, vy, vz, radians=False)`
- `velocities_from_vector(twist, radians=False)`
- `torques(force, torque=(0.0, 0.0, 0.0), radians=False)`
- `torques_from_vector(wrench, radians=False)`

See [Jacobian](jacobian.md) for row order, units, and examples.
