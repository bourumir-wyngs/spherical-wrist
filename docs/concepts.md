# Core Concepts

`spherical-wrist` has a small set of core types. Understanding which one to use
is more important than memorizing every method.

## `KinematicModel`

`KinematicModel` stores the OPW geometry:

- `a1`, `a2`, `b`, `c1`, `c2`, `c3`, `c4`
- six joint offsets
- six axis flip flags

The OPW convention assumes a six-axis arm with an ortho-parallel base and a
spherical wrist. Offsets and axis flips adapt a manufacturer zero position to
that convention.

See [Robot Models](robots.md).

## `Robot`

Use `Robot` for kinematics only:

- `forward`
- `inverse`
- `inverse_continuing`
- `inverse_5dof`
- `forward_with_joint_poses`
- `kinematic_singularity`

`Robot` does not know about meshes, self-collision, or environment objects.

## `KinematicsWithShape`

Use `KinematicsWithShape` when geometry matters. It combines:

- one `KinematicModel`
- six joint meshes
- optional base mesh
- optional tool mesh
- optional environment meshes
- optional constraints
- optional safety distances

Its IK methods filter out colliding solutions. Path planning and visualization
use this type.

## `Mesh`

`Mesh` represents a triangle mesh loaded from a file or small arrays. It can be
used directly for pairwise collision/proximity checks, or as part of a shaped
robot.

## `SafetyDistances`

`SafetyDistances` controls how close robot parts, tools, and environment
objects may come to each other. A positive distance means "treat this as a
violation even before surfaces touch."

`NEVER_COLLIDES` disables checks for a specific pair. This is useful for
neighboring links or designed contacts that are always close.

## `RRTPlanner`

`RRTPlanner` plans a collision-free joint-space path from start joints to goal
joints. Use it when the desired output is simply "move safely from this joint
configuration to that one."

## `CartesianPlanner`

`CartesianPlanner` plans a tool path through Cartesian TCP poses. It returns
`AnnotatedJoints` so you can tell which output steps belong to onboarding,
landing, the stroke, parking, or fallback reconfiguration.

## `VisualizationHandle`

Visualization opens a non-blocking Bevy window and returns a handle. The handle
can set joints, solve and display a target pose, play paths, and close the
window.
