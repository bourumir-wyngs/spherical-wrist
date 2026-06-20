# Development With Maturin

This page is for contributors and source builds. Normal users should install
with pip:

```bash
python -m pip install spherical-wrist
```

## Source Layout

The source checkout is self-contained. `Cargo.toml` uses published crates.io
dependencies for the Rust crates, and Cargo resolves them during source builds:

```text
rs-py-opw/
  Cargo.toml
  pyproject.toml
  python/
  src/
```

## Build A Development Environment

```bash
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


