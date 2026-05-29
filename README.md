# spherical-wrist

`spherical-wrist` is a PyO3/maturin Python package backed by the local
`rs-opw-kinematics` crate. The first API slice mirrors the introductory
`py-opw-kinematics` workflow:

- `KinematicModel`
- `Robot.forward`
- `Robot.inverse`
- `Parallelogram`
- constructor-level `tool`, `base`, and `frame` transforms
- SciPy `RigidTransform` inputs and outputs

```python
from spherical_wrist import KinematicModel, Robot
from scipy.spatial.transform import RigidTransform, Rotation

kinematic_model = KinematicModel(
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

robot = Robot(kinematic_model, degrees=True)
ee_transform = RigidTransform.from_components(
    rotation=Rotation.from_euler("xyz", [0, -90, 0], degrees=True),
    translation=[0, 0, 0],
)

pose = robot.forward((10, 0, -90, 0, 0, 0), ee_transform=ee_transform)
solutions = robot.inverse(pose, ee_transform=ee_transform)
```

`ee_transform` is kept as a compatibility alias for a per-call `tool` transform.
For persistent transforms, pass them to the robot constructor:

```python
robot = Robot(
    kinematic_model,
    degrees=True,
    parallelogram=Parallelogram(scaling=1.0, driven=1, coupled=2),
    tool=tool_transform,
    base=base_transform,
    frame=work_frame_transform,
)
```

The constructor composes transforms in this order:

```text
base * robot.forward(joints) * tool * frame
```

`Parallelogram` uses zero-based joint indices, matching the Rust crate
constants: `J1=0`, `J2=1`, ..., `J6=5`. The common J2/J3 parallelogram is:

```python
from spherical_wrist import Parallelogram

parallelogram = Parallelogram(scaling=1.0, driven=1, coupled=2)
```
