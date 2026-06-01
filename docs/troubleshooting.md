# Troubleshooting

## Import Works In One Shell But Not Another

Check that the shell uses the same Python where the package was installed:

```bash
python -m pip show spherical-wrist
python - <<'PY'
import sys
import spherical_wrist
print(sys.executable)
print(spherical_wrist.__file__)
PY
```

## ROS Or Global Python Packages Interfere

ROS often sets `PYTHONPATH` to a different Python version. If tests or examples
load unrelated packages, unset it:

```bash
env -u PYTHONPATH python python/examples/readme_intro.py
```

For pytest:

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

## Visualization Window Does Not Open

Check that a desktop display is available:

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
```

If you are on a remote server, run non-visual examples first. Visualization
requires a graphical session.

## No IK Solutions

Common causes:

- target pose is outside the robot workspace
- constraints filter all solutions
- the tool or base transform is not what you expect
- units are mixed, for example meters in the model but millimeters in the pose
- shaped robot IK filters all solutions because they collide

Try plain `Robot` first. Then add constraints. Then add shape and collision
checks.

## Planner Fails

Check the start and goal first:

```python
print(robot.collides(start))
print(robot.collides(goal))
```

For Cartesian planning, also check the landing, stroke, and parking poses by
calling `robot.inverse(...)`.

Then tune planner settings:

- increase `max_try`
- reduce joint step size
- relax `max_transition_cost`
- allow reconfiguration
- review safety distances

## Meshes Are In The Wrong Place

Confirm which frame a mesh pose uses:

- joint meshes are local to their joint frames
- tool mesh is local to J6/tool setup
- base mesh is composed with the robot base transform
- environment meshes are global

Use `positioned_robot(joints)` or visualization to inspect the result.
