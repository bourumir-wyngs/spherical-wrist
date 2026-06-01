# Quickstart

This guide shows the minimal kinematics workflow.

## Create A Robot

`KinematicModel` stores the OPW geometry parameters. `Robot` exposes plain
kinematics without mesh collision checks.

```python
from spherical_wrist import KinematicModel, Robot

model = KinematicModel(
    a1=0.100,
    a2=-0.135,
    b=0.000,
    c1=0.615,
    c2=0.705,
    c3=0.755,
    c4=0.085,
    offsets=(0.0, 0.0, -90.0, 0.0, 0.0, 0.0),
)

robot = Robot(model, degrees=True)
```

With `degrees=True`, all joint values you pass in and receive back are degrees.
Use `degrees=False` if your application works in radians.

## Forward Kinematics

```python
joints = (0.0, 5.7296, 11.4592, 17.1887, 0.0, 28.6479)
pose = robot.forward(joints)

print(pose.translation)
print(pose.rotation.as_quat())
```

The returned pose is a SciPy `RigidTransform`.

## Inverse Kinematics

```python
solutions = robot.inverse(pose)

for solution in solutions:
    print(solution)
```

Inverse kinematics returns all valid joint solutions after constraints are
applied.

## Continuing Through Singularities

At wrist singularities, several joint combinations may represent the same tool
pose. If you know the previous joint position, pass it to
`inverse_continuing` so the solver can choose a consistent J4/J6 result.

```python
previous = (0.0, 6.3025, 12.6051, 17.1887, 5.7296, 28.6479)
solutions = robot.inverse_continuing(pose, previous)
```

Use `CONSTRAINT_CENTERED` when you do not have a real previous position and
want the solution closest to the configured constraint center.

```python
from spherical_wrist import CONSTRAINT_CENTERED

solutions = robot.inverse_continuing(pose, CONSTRAINT_CENTERED)
```

## Joint Constraints

Constraints filter IK results and can also affect sorting.

```python
from spherical_wrist import BY_PREV, Constraints, Robot

constraints = Constraints(
    from_limits=(-180, -90, -120, -180, -120, -360),
    to_limits=(180, 90, 120, 180, 120, 360),
    sorting_weight=BY_PREV,
)

robot = Robot(model, degrees=True, constraints=constraints)
```

See [Transforms And Units](transforms-and-units.md) for pose conventions and
[Path Planning](path-planning.md) for collision-aware movement.
