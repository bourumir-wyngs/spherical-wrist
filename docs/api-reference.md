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

## Constants

Joint and collision object indices:

- `J1`, `J2`, `J3`, `J4`, `J5`, `J6`
- `J_TOOL`
- `J_BASE`
- `ENV_START_IDX`

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
