# Robot Models

`KinematicModel` describes a six-axis OPW robot. The same model can be used
with plain `Robot` kinematics or with `KinematicsWithShape`.

## Parameters

The seven OPW geometry parameters are:

- `a1`
- `a2`
- `b`
- `c1`
- `c2`
- `c3`
- `c4`

They describe the link lengths and offsets in the OPW convention. Use one
consistent unit for all distances. The examples use meters for real robot
models and millimeters in the small hello-world model.

## Offsets

`offsets` are six joint offsets. They are used to align the robot manufacturer's
zero position with the OPW zero position.

When `Robot(..., degrees=True)`, offsets are degrees. With
`Robot(..., degrees=False)`, offsets are radians.

```python
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
```

## Axis Flips

`flip_axes` is a tuple of six booleans. Use it when a robot axis rotates in the
opposite direction from the OPW convention.

```python
model = KinematicModel(
    a1=400,
    a2=-250,
    b=0,
    c1=830,
    c2=1175,
    c3=1444,
    c4=230,
    flip_axes=(True, False, True, True, False, True),
)
```

## Built-In Example Models

The Python examples define reusable model helpers in
[python/examples/_common.py](../python/examples/_common.py):

- `irb2400_10()`
- `staubli_tx2_160l()`
- `staubli_rx160()`

Those helpers are examples, not a complete robot database.

## Shaped Robot Models

A shaped robot is a `KinematicsWithShape` with one mesh per joint plus optional
base, tool, and environment meshes.

Use [create_rx160_robot](../python/examples/_common.py) as the reference shape
setup. It shows the expected mesh order, safety distances, tool transform, base
transform, and environment object placement.
