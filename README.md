# spherical-wrist

`spherical-wrist` is a PyO3/maturin Python package backed by the local
`rs-opw-kinematics` crate. The first API slice mirrors the introductory
`py-opw-kinematics` workflow:

- `KinematicModel`
- `Constraints`
- `Robot.forward`
- `Robot.inverse`
- `Robot.inverse_continuing`
- `Robot.inverse_5dof`
- `Robot.inverse_continuing_5dof`
- `Robot.forward_with_joint_poses`
- `Robot.kinematic_singularity`
- `Mesh` loading and Parry collision/proximity checks
- `KinematicsWithShape` collision-aware kinematics
- `SafetyDistances` collision/proximity configuration
- `RRTPlanner` and `CartesianPlanner` path planning
- Bevy visualization wrappers
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

`Robot` exposes the Rust `Kinematics` trait methods. Joint arrays are accepted
and returned in degrees when `Robot(degrees=True)` and radians when
`Robot(degrees=False)`. `CONSTRAINT_CENTERED` can be used as the previous joint
state for `inverse_continuing`.

Joint constraints are passed to the robot constructor and are used internally by
`rs-opw-kinematics` to filter inverse-kinematics solutions. Limits are degrees
by default; pass `radians=True` when constructing or reading them as radians:

```python
from spherical_wrist import BY_PREV, Constraints

constraints = Constraints(
    from_limits=(-180, -90, -120, -180, -120, -360),
    to_limits=(180, 90, 120, 180, 120, 360),
    sorting_weight=BY_PREV,
)

robot = Robot(kinematic_model, degrees=True, constraints=constraints)
lower_rad, upper_rad = constraints.limits(radians=True)
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

Meshes are loaded with `rs-read-trimesh` and checked with Parry 0.26:

```python
from spherical_wrist import Mesh
from scipy.spatial.transform import RigidTransform, Rotation

fixture = Mesh.from_path("fixture.obj", scale=0.001)
part = Mesh.from_path("part.stl").transformed(
    RigidTransform.from_components(
        rotation=Rotation.identity(),
        translation=[0.5, 0, 0],
    )
)

if fixture.collides(part, safety_distance=0.01):
    print("collision or safety-distance violation")
```

For small procedural meshes, construct directly from vertex and triangle arrays:

```python
tetrahedron = Mesh.from_arrays(
    vertices=[
        (0, 0, 0),
        (1, 0, 0),
        (0, 1, 0),
        (0, 0, 1),
    ],
    triangles=[
        (0, 2, 1),
        (0, 1, 3),
        (1, 2, 3),
        (2, 0, 3),
    ],
)
```

`KinematicsWithShape` combines the kinematic model with six joint meshes and
optional base, tool, and environment meshes. Joint and tool mesh poses are local
to their robot frames; environment mesh poses are global. Its inverse methods
filter out colliding solutions:

```python
from spherical_wrist import KinematicsWithShape, NEVER_COLLIDES, SafetyDistances

shape_robot = KinematicsWithShape(
    kinematic_model,
    joint_meshes=[link_1, link_2, link_3, link_4, link_5, link_6],
    degrees=True,
    tool=tool_transform,
    tool_mesh=tool_mesh,
    environment=[fixture, workpiece],
    safety=SafetyDistances(
        to_environment=0.05,
        to_robot_default=NEVER_COLLIDES,
        mode="all",
    ),
)

if shape_robot.collides((10, 0, -90, 0, 0, 0)):
    print(shape_robot.collision_details((10, 0, -90, 0, 0, 0)))

safe_solutions = shape_robot.inverse(target_pose)
```

Path planning uses `KinematicsWithShape` because collision checks and joint
constraints are required. RRT step sizes are degrees by default, matching the
usual Python-side joint unit convention:

```python
from spherical_wrist import CartesianPlanner, RRTPlanner

rrt = RRTPlanner(step_size_joint_space=3.0, max_try=2000, smooth=100)
joint_path = rrt.plan_rrt(
    shape_robot,
    start=(0, 0, 0, 0, 0, 0),
    goal=(20, -10, 15, 0, 0, 30),
)

cartesian = CartesianPlanner(rrt=rrt, check_step_m=0.02, check_step_rad=3.0)
annotated_path = cartesian.plan(
    shape_robot,
    start=(0, 0, 0, 0, 0, 0),
    land=landing_pose,
    steps=[stroke_pose_1, stroke_pose_2],
    park=parking_pose,
)
```

Visualization opens the upstream Bevy window in the background and returns a
handle. It uses `KinematicsWithShape`, initial joints in the robot's configured
angle unit, and TCP slider limits for x, y, and z:

```python
from spherical_wrist import visualize_robot

view = visualize_robot(
    shape_robot,
    initial_joints=(0, 0, 0, 0, 0, 0),
    tcp_box=((-2.0, 2.0), (-2.0, 2.0), (0.0, 2.0)),
)

view.set_joints((10, 0, -90, 0, 0, 0))
selected_joints = view.set_pose(target_pose, previous_position=(10, 0, -90, 0, 0, 0))
view.close()
```
