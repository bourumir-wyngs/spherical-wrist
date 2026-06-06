# Transforms And Units

Robotics bugs often come from unit or frame confusion. This page documents the
Python conventions used by `spherical-wrist`.

## Joint Units

`Robot(..., degrees=True)` and `KinematicsWithShape(..., degrees=True)` use
degrees for all joint input and output.

```python
robot = Robot(model, degrees=True)
pose = robot.forward((10, 0, -90, 0, 0, 0))
```

Use radians explicitly when needed:

```python
robot = Robot(model, degrees=False)
pose = robot.forward((0.1745, 0, -1.5708, 0, 0, 0))
```

Planner angle-like settings are also degrees by default. Pass `radians=True` to
planner constructors when you provide radians.

## Distance Units

The solver does not force meters or millimeters. Choose one distance unit for a
model and keep it consistent:

- model parameters
- mesh scale
- base/tool translations
- frame translations and frame scale
- target pose translations
- safety distances

The shaped RX160 examples use meters.

## Pose Type

Python poses are SciPy `RigidTransform` values.

```python
from scipy.spatial.transform import RigidTransform, Rotation

pose = RigidTransform.from_components(
    rotation=Rotation.from_euler("zyx", [90, 0, 0], degrees=True),
    translation=[1.0, 0.0, 1.7],
)
```

## Tool Transform

The tool transform is the flange-to-TCP transform. If the same tool is always
attached, put it on the robot:

```python
robot = Robot(model, degrees=True, tool=tool_transform)
pose = robot.forward(joints)
```

If the transform is only needed for one call, pass it as `tool`.

```python
pose = robot.forward(joints, tool=tool_transform)
solutions = robot.inverse(pose, tool=tool_transform)
```

`ee_transform` is accepted as a compatibility alias for `tool`.

## Base And Frame

`base` places the robot in the world. `frame` retargets the final TCP pose into
a measured working frame. Base and tool transforms must be rigid. A frame
created with `Frame.from_tie(...)` may include uniform scale.

```python
from spherical_wrist import Frame, Robot

work_frame = Frame.from_tie(
    original_tie_points=[
        [0.10, 0.00, 0.20],
        [0.20, 0.00, 0.20],
        [0.10, 0.10, 0.20],
    ],
    target_tie_points=[
        [0.12, 0.02, 0.21],
        [0.222, 0.02, 0.21],
        [0.12, 0.122, 0.21],
    ],
)

robot = Robot(
    model,
    degrees=True,
    base=base_transform,
    tool=tool_transform,
    frame=work_frame,
)
```

For `Robot.forward`, transforms are applied as:

```text
frame.transform_pose(base * robot.forward(joints) * tool)
```

If `frame` is supplied as a SciPy `RigidTransform`, it is treated as a frame
with scale 1. If it is supplied as `Frame.from_tie(...)`, the scale changes TCP
translations before IK/Jacobian calculations. TCP orientation is rotated by the
frame but is not scaled.

## Joint Poses

`forward_with_joint_poses` returns the six joint poses. For a configured tool,
the sixth pose is still the J6 pose, not the TCP pose. Use `forward` when you
need the TCP.
