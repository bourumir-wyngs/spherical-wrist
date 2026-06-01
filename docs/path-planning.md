# Path Planning

Path planning requires `KinematicsWithShape`, because the planner must reject
colliding robot states.

Use [path_planning_rrt.py](../python/examples/path_planning_rrt.py) for
joint-space planning and [cartesian_stroke.py](../python/examples/cartesian_stroke.py)
for Cartesian TCP stroke planning.

## RRT Joint-Space Planning

Use `RRTPlanner` when you have start and goal joint configurations.

```python
from spherical_wrist import RRTPlanner

planner = RRTPlanner(
    step_size_joint_space=3.0,
    max_try=2000,
    smooth=500,
)

path = planner.plan_rrt(
    robot,
    start=(-120.0, -90.0, -92.51, 18.42, 82.23, 189.35),
    goal=(40.0, -90.0, -92.51, 18.42, 82.23, 189.35),
)
```

The result is a list of joint tuples. Each tuple is in the robot's joint unit.

Important settings:

- `step_size_joint_space`: extension step, degrees by default
- `max_try`: maximum expansion attempts
- `smooth`: shortcut smoothing budget
- `debug`: print upstream planner diagnostics

## Cartesian Stroke Planning

Use `CartesianPlanner` when the TCP must follow Cartesian poses.

```python
from spherical_wrist import CartesianPlanner, RRTPlanner

planner = CartesianPlanner(
    check_step_m=0.05,
    check_step_rad=3.0,
    max_transition_cost=15,
    linear_recursion_depth=8,
    rrt=RRTPlanner(step_size_joint_space=2.0, max_try=100),
    allow_reconfigure=False,
    include_linear_interpolation=True,
)

path = planner.plan(
    robot,
    start=start_joints,
    land=landing_pose,
    steps=stroke_poses,
    park=parking_pose,
)
```

The result is a list of `AnnotatedJoints`.

Each item has:

- `joints`: the six joint values
- `flags`: bit flags describing the path segment
- `move_into`: `"joint"` or `"cartesian"`

## Path Flags

Common flags:

- `PATH_FLAG_ONBOARDING`: movement from the start configuration toward landing
- `PATH_FLAG_LAND`: the landing pose itself
- `PATH_FLAG_LANDING`: movement between landing and the stroke
- `PATH_FLAG_TRACE`: the user-requested stroke
- `PATH_FLAG_LIN_INTERP`: generated linear interpolation point
- `PATH_FLAG_PARKING`: movement from stroke toward park
- `PATH_FLAG_PARK`: the final park pose
- `PATH_FLAG_RECONFIGURING`: fallback joint-space reconfiguration

`cartesian_stroke.py` contains a small `describe_flags` helper for printing
flag names.

## Choosing The Planner

Use RRT if:

- the start and goal are joint configurations
- the path does not need to follow a specific TCP line or shape
- you only need collision-free relocation

Use Cartesian planning if:

- the TCP must follow a defined path
- you need landing, stroke, and parking phases
- you need to know which output points belong to each phase

## Failure Handling

Planning methods raise `ValueError` when no path can be found with the current
settings. Common fixes are:

- increase `max_try`
- reduce `step_size_joint_space`
- relax `max_transition_cost`
- allow reconfiguration in `CartesianPlanner`
- inspect collision details for the start, goal, land, and stroke poses
