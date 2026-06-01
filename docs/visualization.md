# Visualization

Visualization opens a Bevy window in a background thread and returns a
`VisualizationHandle`.

Visualization requires a shaped robot, so use `KinematicsWithShape`.

```python
from spherical_wrist import visualize_robot

handle = visualize_robot(
    robot,
    initial_joints=(0, 0, 0, 0, 0, 0),
    tcp_box=((-2.0, 2.0), (-2.0, 2.0), (1.0, 2.0)),
)
```

## Controlling The Window

```python
handle.set_joints((10, 0, -90, 0, 0, 0))
selected = handle.set_pose(target_pose, previous_position=(10, 0, -90, 0, 0, 0))
handle.close()
```

`set_pose` solves IK, selects a collision-free solution, updates the display,
and returns the selected joints.

## Playing A Path

```python
handle.play_path(path, interval=0.05)
```

The path may contain raw joint tuples or `AnnotatedJoints` from
`CartesianPlanner.plan`.

## Keeping Examples Open

Examples use `wait_for_visualization` from
[python/examples/_common.py](../python/examples/_common.py). In an interactive
terminal it waits for Enter. In non-interactive runs it waits until the window
is closed.

## Display Requirements

The visualization window needs a graphical desktop session. On Linux, make sure
`DISPLAY` or the appropriate Wayland environment is available.

If visualization fails but non-visual examples work, see
[Troubleshooting](troubleshooting.md).
