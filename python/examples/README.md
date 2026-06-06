# Examples

Run examples from the repository root:

```bash
python python/examples/readme_intro.py
```

If your environment has ROS or another global Python stack on `PYTHONPATH`, use:

```bash
env -u PYTHONPATH python python/examples/readme_intro.py
```

## Start Here

`readme_intro.py`
: Minimal `KinematicModel` plus `Robot.forward` and `Robot.inverse`.

`basic_readme.py`
: Compact version of the Rust README singularity example, translated to the
Python API.

`basic.py`
: More complete singularity and 5-DOF IK demonstration.

## Important Planning Examples

`cartesian_stroke.py`
: The main Cartesian planning example. It builds a shaped RX160 robot, plans a
TCP stroke through Cartesian poses, prints annotated path steps, and plays the
result in the visualization window.

`path_planning_rrt.py`
: Collision-aware joint-space RRT planning from start joints to goal joints.
Use this when the TCP does not need to follow a specific Cartesian line.

## Geometry And Robot Setup

`complete_visible_robot.py`
: Builds the shaped RX160 robot, checks collisions, solves IK, and opens the
visualization window.

`tool_and_base.py`
: Shows persistent base and tool transforms.

`frame.py`
: Computes a transport frame, including uniform scale, from three tie point
pairs and retargets a canonical trajectory.

`parallelogram.py`
: Shows parallelogram coupling configuration.

`constraints.py`
: Shows joint constraints, constraint-centered solving, and sorting behavior.

`jacobian.py`
: Shows the TCP Jacobian matrix, joint-rate solving, and wrench-to-effort
mapping.

`_common.py`
: Shared helper functions and the reference `create_rx160_robot` setup used by
the shaped robot and planning examples.

## Windowed Examples

These examples open the Bevy visualization window:

- `complete_visible_robot.py`
- `path_planning_rrt.py`
- `cartesian_stroke.py`

They require a graphical desktop session.
