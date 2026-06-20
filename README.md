# spherical-wrist

`spherical-wrist` is a Python package for six-axis industrial robot
kinematics, collision checking, path planning, and visualization.

`spherical-wrist` is backed by the [rs-opw-kinematics](https://github.com/bourumir-wyngs/rs-opw-kinematics) crate. It wraps the
same Rust library as [py-opw-kinematics](https://github.com/CEAD-group/py-opw-kinematics), but exposes a broader API surface:
`py-opw-kinematics` intentionally publishes only a selected subset of
`rs-opw-kinematics`.

The goal of `spherical-wrist` is to make the full potential of
`rs-opw-kinematics` available from Python, including collision-free path and
trajectory planning. Collision checking is separate from inverse kinematics,
but collision-aware stroke planning needs both capabilities close together:
available from multiple threads and without repeated Python-Rust boundary
crossing overhead.

The public API is Python first: joint positions are tuples or arrays, poses are SciPy
`RigidTransform` objects, and examples are plain Python scripts.

## Install

```bash
python -m pip install spherical-wrist
```

The package requires Python 3.11 or newer.

For local development, source builds, and maturin, see
[Development With Maturin](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/development-maturin.md). The Rust dependencies
are resolved from crates.io during the build.

## Hello World

Create a kinematic model, compute a TCP pose from joint angles, then ask inverse
kinematics for joint solutions that reach the same pose.

```python
from scipy.spatial.transform import RigidTransform, Rotation
from spherical_wrist import KinematicModel, Robot

model = KinematicModel(
    a1=400,
    a2=-250,
    b=0,
    c1=830,
    c2=1175,
    c3=1444,
    c4=230,
    offsets=(0, 0, 0, 0, 0, 0),
    flip_axes=(True, False, True, True, False, True),
)

robot = Robot(model, degrees=True)

tool = RigidTransform.from_components(
    rotation=Rotation.from_euler("xyz", [0, -90, 0], degrees=True),
    translation=[0, 0, 0],
)

joints = (10, 0, -90, 0, 0, 0)
pose = robot.forward(joints, tool=tool)
solutions = robot.inverse(pose, tool=tool)

print("TCP position:", pose.translation)
print("IK solutions:", solutions)
```

The same script is in [python/examples/readme_intro.py](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/python/examples/readme_intro.py).

## What It Does

- analytical forward and inverse kinematics for spherical-wrist industrial arms
- continuing inverse kinematics that handles wrist singularities with a
  previous joint position
- optional joint constraints and solution ranking
- 5-DOF inverse kinematics for tools that do not care about J6 rotation
- base, tool, frame, and parallelogram transforms
- mesh loading and collision/proximity checks
- shaped robot models with tool, base, and environment geometry
- RRT joint-space planning
- Cartesian stroke planning with annotated path segments
- Bevy-based visualization and path playback

## Main Examples

The most useful examples are:

- [cartesian_stroke.py](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/python/examples/cartesian_stroke.py): builds a shaped
  Staubli RX160 robot, plans a Cartesian TCP stroke, annotates the path, and
  plays it in the visualization window.
- [path_planning_rrt.py](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/python/examples/path_planning_rrt.py): plans a
  collision-free joint-space path between start and goal configurations.

Run them with:

```bash
python python/examples/cartesian_stroke.py
python python/examples/path_planning_rrt.py
```

If your shell has ROS or another global Python stack on `PYTHONPATH`, unset it
for these commands:

```bash
env -u PYTHONPATH python python/examples/cartesian_stroke.py
```

See [Examples](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/python/examples/README.md) for the full example catalog.

## Documentation

- [Installation](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/installation.md)
- [Quickstart](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/quickstart.md)
- [Core Concepts](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/concepts.md)
- [Robot Models](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/robots.md)
- [Transforms And Units](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/transforms-and-units.md)
- [Jacobian](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/jacobian.md)
- [Meshes And Collisions](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/meshes-and-collisions.md)
- [Path Planning](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/path-planning.md)
- [Visualization](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/visualization.md)
- [API Reference](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/api-reference.md)
- [Troubleshooting](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/troubleshooting.md)
- [Development With Maturin](https://github.com/bourumir-wyngs/spherical-wrist/blob/master/docs/development-maturin.md)

## Project Links

Please use the [GitHub repository](https://github.com/bourumir-wyngs/spherical-wrist)
to raise issues and provide merge requests. The core Rust implementation is
[rs-opw-kinematics](https://github.com/bourumir-wyngs/rs-opw-kinematics). A useful
background article is
[An Analytical Solution of the Inverse Kinematics Problem of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist](https://www.researchgate.net/publication/264212870_An_Analytical_Solution_of_the_Inverse_Kinematics_Problem_of_Industrial_Serial_Manipulators_with_an_Ortho-parallel_Basis_and_a_Spherical_Wrist).

## Use cases

Use `Robot` when you only need kinematics. It provides forward
kinematics for all roboto joints, not only the tool-tip pose.

Use `KinematicsWithShape` when robot geometry matters: collision checks,
distance queries, path planning, or visualization.

Use `RRTPlanner` when you want to move the robot between the two known poses.
It plans a collision-free joint-space path between them and can
enforce a configurable safety distance around geometry. That margin matters in
real cells: calibration error can otherwise turn a mathematically valid
near-contact path into brief surface contact, resulting in scratches, or robot damage.

Use `CartesianPlanner` when the tool center point must follow a Cartesian
stroke defined by poses. It can start from an arbitrary robot pose, plan the landing into
best suitable joint configuration for the stroke when alternatives exist, and plan a
collision-free path that follows the Cartesian line. Simple joint interpolation
does not provide that guarantee.
