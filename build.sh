#!/usr/bin/env bash
set -euo pipefail

# Run from the repository root, regardless of the caller's working directory.
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

for tool in uv cargo; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        printf 'Error: %s must be installed and available on PATH.\n' "$tool" >&2
        exit 1
    fi
done

# Reuse the local environment; PYTHON_VERSION selects Python for a new one.
if [[ ! -d .venv ]]; then
    uv venv --python "${PYTHON_VERSION:-3.12}" .venv
fi

export VIRTUAL_ENV="$PWD/.venv"
export PATH="$VIRTUAL_ENV/bin:$PATH"

# Honor the project's build requirements instead of using a system Maturin.
build_requirements=$("$VIRTUAL_ENV/bin/python" -c '
import tomllib
with open("pyproject.toml", "rb") as config:
    print("\n".join(tomllib.load(config)["build-system"]["requires"]))
')
uv pip install --python "$VIRTUAL_ENV/bin/python" --requirement - patchelf <<< "$build_requirements"

# Extra arguments are forwarded, e.g. ./build.sh --extras test
exec "$VIRTUAL_ENV/bin/maturin" develop --release --uv "$@"
