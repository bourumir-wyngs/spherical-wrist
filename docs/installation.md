# Installation

Install the package with pip:

```bash
python -m pip install spherical-wrist
```

The package requires Python 3.11 or newer.

## Check The Install

```bash
python - <<'PY'
from spherical_wrist import KinematicModel, Robot

robot = Robot(KinematicModel(), degrees=True)
print(robot)
PY
```

## Virtual Environment

A virtual environment is recommended for applications:

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install spherical-wrist
```

On Windows:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install spherical-wrist
```

## Native Extension Note

`spherical-wrist` contains a native Rust extension. Normal users should install
a prebuilt wheel with pip. If pip has to build from source, see
[Development With Maturin](development-maturin.md).

## Running Examples From A Source Checkout

When running examples from this repository, use the interpreter where the
package is installed:

```bash
python python/examples/readme_intro.py
```

If your environment has ROS or another global Python stack on `PYTHONPATH`, run:

```bash
env -u PYTHONPATH python python/examples/readme_intro.py
```
