# Jacobian

`Jacobian` computes a numerical 6x6 Jacobian for a robot at one joint
configuration. Use it when you need local differential kinematics:

- map joint rates to TCP twist
- solve joint rates for a desired TCP velocity
- map a TCP wrench to generalized joint efforts

For a runnable script, see
[python/examples/jacobian.py](../python/examples/jacobian.py).

## Create A Jacobian

Use `Robot` when you only need kinematics:

```python
from spherical_wrist import Jacobian, Robot

robot = Robot(model, degrees=True)
joints = (0.0, 10.0, -45.0, 25.0, 30.0, 40.0)

jacobian = Jacobian(robot, joints)
```

Configured robot transforms are included. If the `Robot` has `base`, `tool`,
or `frame`, the Jacobian is computed for the same TCP pose as `Robot.forward`.
Base and tool transforms are rigid. The frame may be a rigid SciPy
`RigidTransform` or a scale-aware `Frame` built from tie points.

```python
robot = Robot(
    model,
    degrees=True,
    base=base_transform,
    tool=tool_transform,
    frame=frame_transform,
)

jacobian = Jacobian(robot, joints)
```

This follows the same composition documented in
[Transforms And Units](transforms-and-units.md):

```text
frame.transform_pose(base * robot.forward(joints) * tool)
```

The base transform rotates the reported linear and angular velocity axes. Tool
and frame transforms move the differentiated TCP, so they affect the linear
velocity rows as well as the orientation rows. If the frame has scale, that
scale is included in the translational part of the finite differences. TCP
orientation is rotated by the frame but not scaled.

If the robot uses a per-call tool transform, pass it when constructing the
Jacobian. The Jacobian is then computed for that TCP, matching
`robot.forward(joints, tool=tool)`.

```python
jacobian = Jacobian(robot, joints, tool=tool_transform)
```

`ee_transform` is accepted as a compatibility alias for `tool`.

Use `KinematicsWithShape` when the robot is already built with collision
geometry:

```python
jacobian = Jacobian(shaped_robot, joints)
```

The matrix itself is kinematic only. Collision geometry does not change the
Jacobian, but using `KinematicsWithShape` keeps the same tool, base, and
parallelogram setup as the shaped robot.

## Matrix Layout

`matrix()` returns a NumPy array with shape `(6, 6)`.

Rows are ordered as TCP twist components:

```text
[vx, vy, vz, wx, wy, wz]
```

Columns are ordered by joint:

```text
[J1, J2, J3, J4, J5, J6]
```

Example:

```python
import numpy as np

matrix = jacobian.matrix()
joint_rates = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
tcp_twist = matrix @ joint_rates
```

`tcp_twist[:3]` is linear velocity in the same distance unit as the robot model
per second. `tcp_twist[3:]` is angular velocity in radians per second.

## Joint Units

By default, `Jacobian` follows the robot's configured joint unit.

For a degree-based robot:

```python
robot = Robot(model, degrees=True)
jacobian = Jacobian(robot, joints_in_degrees)
matrix = jacobian.matrix()
```

`matrix` maps degree/second joint rates to TCP twist.

For a radian-based robot:

```python
robot = Robot(model, degrees=False)
jacobian = Jacobian(robot, joints_in_radians)
matrix = jacobian.matrix()
```

`matrix` maps radian/second joint rates to TCP twist.

Pass `radians=True` to force raw radian joint coordinates even when the robot
uses degrees:

```python
raw = jacobian.matrix(radians=True)
rates_rad_s = jacobian.velocities_from_vector(twist, radians=True)
efforts_per_rad = jacobian.torques_from_vector(wrench, radians=True)
```

This is useful when comparing with textbooks, controllers, or code that assumes
Jacobian columns are derivatives per radian.

## Solve Joint Rates

Use `velocities_from_vector` with a full TCP twist:

```python
twist = (0.02, 0.0, 0.01, 0.0, 0.0, 0.05)
joint_rates = jacobian.velocities_from_vector(twist)
```

The twist order is:

```text
[vx, vy, vz, wx, wy, wz]
```

For split linear and angular vectors:

```python
joint_rates = jacobian.velocities(
    linear_velocity=(0.02, 0.0, 0.01),
    angular_velocity=(0.0, 0.0, 0.05),
)
```

For pure translation:

```python
joint_rates = jacobian.velocities_fixed(0.02, 0.0, 0.01)
```

Returned joint rates use the robot's joint unit by default. For a degree-based
robot they are degrees/second. Pass `radians=True` for radians/second.

## Wrenches And Joint Efforts

Use `torques_from_vector` with a TCP wrench:

```python
wrench = (0.0, 0.0, 25.0, 0.0, 0.0, 2.0)
efforts = jacobian.torques_from_vector(wrench)
```

The wrench order is:

```text
[fx, fy, fz, tx, ty, tz]
```

For split force and torque vectors:

```python
efforts = jacobian.torques(
    force=(0.0, 0.0, 25.0),
    torque=(0.0, 0.0, 2.0),
)
```

Mathematically this computes:

```text
efforts = J.T @ wrench
```

When the robot uses degrees, default efforts are scaled to be consistent with
degree-based generalized coordinates. Pass `radians=True` if you need the raw
per-radian convention.

## Numerical Differentiation

The Jacobian is computed numerically by perturbing each joint by `epsilon`
radians. The default is usually appropriate:

```python
jacobian = Jacobian(robot, joints, epsilon=1e-6)
```

Use a smaller or larger value only if you have a concrete numerical reason.
Too small can amplify floating-point noise. Too large can smear local curvature.

## Singularities

Near singular configurations, a direct inverse Jacobian solve may be unstable.
The implementation falls back to damped least squares for velocity solves.

This means `velocities_from_vector` can still return a finite least-squares
joint-rate vector at wrist singularities, but the result may not reproduce the
requested twist exactly if the twist is not achievable at that configuration.

Check reconstruction when this matters:

```python
matrix = jacobian.matrix()
rates = np.asarray(jacobian.velocities_from_vector(twist))
reconstructed = matrix @ rates
error = np.linalg.norm(reconstructed - np.asarray(twist))
```
