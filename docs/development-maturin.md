# Development With Maturin

This page is for contributors and source builds. Normal users should install
with pip:

```bash
python -m pip install spherical-wrist
```

## Repository Layout

The current source checkout expects sibling Rust repositories:

```text
py-rs/
  rs-py-opw/
  rs-opw-kinematics/
  rs-read-trimesh/
```

`rs-py-opw/Cargo.toml` uses path dependencies for the Rust crates.

The current Python bindings expect `rs-opw-kinematics` from the
`bw/glam-opw-kinematics` branch.

```bash
cd ../rs-opw-kinematics
git checkout bw/glam-opw-kinematics
```

## Build A Development Environment

```bash
cd ../rs-py-opw
uv venv --python 3.12 .venv
uv pip install maturin pytest patchelf
uv run maturin develop --release --extras test
```

Without `uv`, activate any Python 3.11 or newer virtual environment and run:

```bash
python -m pip install maturin pytest patchelf
maturin develop --release --extras test
```

## Test

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q
```

## Rebuild After Rust Changes

```bash
env -u PYTHONPATH .venv/bin/maturin develop --release --extras test
```

## Common Source Build Issues

If maturin reports a missing `patchelf`, install it in the virtual environment:

```bash
python -m pip install patchelf
```

If Cargo cannot find a local crate, confirm the sibling directories exist and
match the path dependencies in `Cargo.toml`.

If `rs-opw-kinematics` API symbols are missing, confirm the branch:

```bash
git -C ../rs-opw-kinematics branch --show-current
```
