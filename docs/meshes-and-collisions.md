# Meshes And Collisions

Use meshes when you need collision checks, safety distances, path planning, or
realistic visualization.

## Loading Meshes

`Mesh.from_path` loads triangle meshes from common formats supported by the
Rust mesh reader.

```python
from spherical_wrist import Mesh

fixture = Mesh.from_path("fixture.stl")
scaled = Mesh.from_path("part.obj", scale=0.001)
```

For small procedural meshes, use arrays. This example creates a pyramid (tetrahedron):

```python
mesh = Mesh.from_arrays(
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

## Moving Meshes

Use SciPy `RigidTransform` values to place a mesh.

```python
from scipy.spatial.transform import RigidTransform, Rotation

placed = fixture.transformed(
    RigidTransform.from_components(
        rotation=Rotation.identity(),
        translation=[0.5, 0.0, 0.0],
    )
)
```

For environment meshes in `KinematicsWithShape`, the mesh pose is global.

## Pairwise Collision Checks

```python
if fixture.collides(placed, safety_distance=0.01):
    print("collision or safety-distance violation")
```

With a positive `safety_distance`, `collides` is true if the meshes intersect or
are closer than that distance.

## Shaped Robots

`KinematicsWithShape` needs exactly six joint meshes:

```python
from spherical_wrist import KinematicsWithShape

robot = KinematicsWithShape(
    model,
    joint_meshes=[link_1, link_2, link_3, link_4, link_5, link_6],
    degrees=True,
    base=base_transform,
    tool=tool_transform,
    base_mesh=base_mesh,
    tool_mesh=tool_mesh,
    environment=[fixture, workpiece],
    safety=safety,
)
```

Joint and tool meshes are local to the robot frames. Environment meshes are
already in world coordinates.

## Safety Distances

```python
from spherical_wrist import J2, J3, J4, J_BASE, NEVER_COLLIDES, SafetyDistances

safety = SafetyDistances(
    to_environment=0.05,
    to_robot_default=0.05,
    special_distances=[
        (J2, J_BASE, NEVER_COLLIDES),
        (J3, J_BASE, NEVER_COLLIDES),
        (J2, J4, NEVER_COLLIDES),
    ],
)
```

Use `special_distances` for pairs that need a different rule than the default.

## Collision Results

```python
if robot.collides(joints):
    print(robot.collision_details(joints))
```

Collision pair indices use constants such as `J1`, `J2`, `J_TOOL` and `J_BASE`.

The helper `collision_name` in [python/examples/_common.py](../python/examples/_common.py)
shows how to turn those indices into readable names.
