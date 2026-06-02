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

For local development, source builds, maturin, and sibling Rust repositories,
see [Development With Maturin](docs/development-maturin.md).

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

The same script is in [python/examples/readme_intro.py](python/examples/readme_intro.py).

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

- [cartesian_stroke.py](python/examples/cartesian_stroke.py): builds a shaped
  Staubli RX160 robot, plans a Cartesian TCP stroke, annotates the path, and
  plays it in the visualization window.
- [path_planning_rrt.py](python/examples/path_planning_rrt.py): plans a
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

See [Examples](python/examples/README.md) for the full example catalog.

## Documentation

- [Installation](docs/installation.md)
- [Quickstart](docs/quickstart.md)
- [Core Concepts](docs/concepts.md)
- [Robot Models](docs/robots.md)
- [Transforms And Units](docs/transforms-and-units.md)
- [Meshes And Collisions](docs/meshes-and-collisions.md)
- [Path Planning](docs/path-planning.md)
- [Visualization](docs/visualization.md)
- [API Reference](docs/api-reference.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Development With Maturin](docs/development-maturin.md)

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
