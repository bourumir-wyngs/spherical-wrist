#!/usr/bin/env python3
from __future__ import annotations

import argparse
from email.parser import Parser
import json
import os
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import venv
import zipfile
from pathlib import Path, PurePosixPath

try:
    from packaging.tags import sys_tags
    from packaging.utils import (
        canonicalize_name,
        parse_sdist_filename,
        parse_wheel_filename,
    )
except ImportError as exc:
    raise SystemExit(
        "Missing release-check dependency. Install with: "
        "python -m pip install packaging twine pytest"
    ) from exc


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_DIR = "spherical_wrist"
REQUIRED_PACKAGE_FILES = (
    f"{PACKAGE_DIR}/_internal.pyi",
    f"{PACKAGE_DIR}/py.typed",
)
NATIVE_SUFFIXES = (".pyd", ".dll", ".dylib")
STAUBLI_ASSET_PREFIX = "python/examples/assets/staubli/"
STAUBLI_LICENSE_FILE = f"{STAUBLI_ASSET_PREFIX}LICENSE"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build and validate Python release artifacts."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    build = subparsers.add_parser(
        "build",
        help=(
            "Build sdist/wheel artifacts with maturin build --sdist, then "
            "check them."
        ),
    )
    add_common_args(build)
    build.add_argument(
        "--compatibility",
        help=(
            "Optional maturin compatibility tag, for example 'linux' for local "
            "Linux CI checks or 'pypi' for upload-compatible artifacts."
        ),
    )
    build.add_argument(
        "--auditwheel",
        choices=("repair", "check", "warn", "skip"),
        help="Optional maturin auditwheel mode for Linux wheel builds.",
    )
    build.add_argument(
        "--interpreter",
        default=sys.executable,
        help="Python interpreter passed to maturin -i. Defaults to this Python.",
    )
    build.add_argument(
        "--maturin",
        default=sys.executable,
        help=("Python executable used to run `-m maturin`. Defaults to this Python."),
    )
    build.add_argument(
        "--keep-dist",
        action="store_true",
        help="Do not clear --dist-dir before building.",
    )

    check = subparsers.add_parser(
        "check",
        help="Check artifacts that already exist in --dist-dir.",
    )
    add_common_args(check)

    args = parser.parse_args()
    dist_dir = resolve_path(args.dist_dir)
    report_dir = resolve_path(args.report_dir)

    if args.command == "build":
        if not args.keep_dist and dist_dir.exists():
            shutil.rmtree(dist_dir)
        dist_dir.mkdir(parents=True, exist_ok=True)
        build_artifacts(args, dist_dir)

    check_artifacts(args, dist_dir, report_dir)


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--dist-dir",
        default="target/release-check/dist",
        help="Directory containing release artifacts.",
    )
    parser.add_argument(
        "--report-dir",
        default="target/release-check/report",
        help="Directory for release check reports.",
    )
    parser.add_argument(
        "--venv-dir",
        default="target/release-check/install-venv",
        help="Temporary virtualenv used for installed-wheel tests.",
    )
    parser.add_argument(
        "--tests-dir",
        default="python/tests",
        help="Python test directory to copy and run outside the checkout.",
    )
    parser.add_argument(
        "--skip-installed-tests",
        action="store_true",
        help="Skip clean-venv install and installed-wheel pytest run.",
    )
    parser.add_argument(
        "--skip-native-deps",
        action="store_true",
        help="Skip ldd/otool/dumpbin native dependency inspection.",
    )


def resolve_path(path: str) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return PROJECT_ROOT / candidate


def resolve_executable_path(path: str) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return PROJECT_ROOT / candidate


def build_artifacts(args: argparse.Namespace, dist_dir: Path) -> None:
    maturin_python = resolve_executable_path(args.maturin)
    interpreter = resolve_executable_path(args.interpreter)
    command = [
        str(maturin_python),
        "-m",
        "maturin",
        "build",
        "--sdist",
        "--release",
        "--locked",
        "--out",
        str(dist_dir),
        "-i",
        str(interpreter),
    ]
    if args.compatibility:
        command.extend(["--compatibility", args.compatibility])
    if args.auditwheel:
        command.extend(["--auditwheel", args.auditwheel])

    run(command, cwd=PROJECT_ROOT, env=python_path_env(maturin_python))


def check_artifacts(
    args: argparse.Namespace,
    dist_dir: Path,
    report_dir: Path,
) -> None:
    artifacts = sorted(path for path in dist_dir.iterdir() if path.is_file())
    wheels = [path for path in artifacts if path.suffix == ".whl"]
    sdists = [path for path in artifacts if path.name.endswith(".tar.gz")]

    if not wheels:
        raise SystemExit(f"No wheels found in {dist_dir}")
    if not sdists:
        raise SystemExit(f"No source distributions found in {dist_dir}")

    report_dir.mkdir(parents=True, exist_ok=True)

    run(
        [sys.executable, "-m", "twine", "check", *map(str, artifacts)],
        cwd=PROJECT_ROOT,
    )

    report: dict[str, object] = {
        "dist_dir": str(dist_dir),
        "wheels": [],
        "sdists": [],
    }

    sdist_reports = []
    for sdist in sdists:
        sdist_report = inspect_sdist(sdist)
        sdist_reports.append(sdist_report)
        report["sdists"].append(sdist_report)  # type: ignore[index]

    wheel_reports = []
    for wheel in wheels:
        wheel_report = inspect_wheel(
            wheel,
            report_dir,
            skip_native=args.skip_native_deps,
        )
        wheel_reports.append(wheel_report)
        report["wheels"].append(wheel_report)  # type: ignore[index]

    report["version"] = require_single_artifact_version(sdist_reports, wheel_reports)

    write_wheel_sizes(wheel_reports, report_dir / "wheel-sizes.tsv")

    selected_wheel = select_compatible_wheel(wheels)
    report["installed_test_wheel"] = str(selected_wheel)

    if not args.skip_installed_tests:
        run_installed_tests(
            wheel=selected_wheel,
            venv_dir=resolve_path(args.venv_dir),
            tests_dir=resolve_path(args.tests_dir),
        )

    write_json(report_dir / "release-artifact-report.json", report)
    print(f"Wrote release artifact report to {report_dir}")


def inspect_sdist(path: Path) -> dict[str, object]:
    with tarfile.open(path, "r:gz") as archive:
        names = archive.getnames()
        pkg_info = next((name for name in names if name.endswith("/PKG-INFO")), None)
        if pkg_info is None:
            raise SystemExit(f"{path.name}: missing PKG-INFO")
        pkg_info_text = archive.extractfile(pkg_info).read().decode("utf-8")

    require_metadata(pkg_info_text, path.name)
    staubli_assets = require_staubli_asset_policy(names, pkg_info_text, path.name)
    distribution, filename_version = parse_sdist_filename(path.name)
    metadata_version_text = metadata_version(pkg_info_text, path.name)

    if canonicalize_name(str(distribution)) != canonicalize_name("spherical-wrist"):
        raise SystemExit(f"{path.name}: unexpected distribution name {distribution!r}")

    if str(filename_version) != metadata_version_text:
        raise SystemExit(
            f"{path.name}: sdist filename version {filename_version!s} does not "
            f"match PKG-INFO version {metadata_version_text!r}"
        )

    return {
        "file": path.name,
        "bytes": path.stat().st_size,
        "mebibytes": round(path.stat().st_size / (1024 * 1024), 3),
        "version": metadata_version_text,
        "staubli_assets": staubli_assets,
    }


def inspect_wheel(
    path: Path,
    report_dir: Path,
    *,
    skip_native: bool,
) -> dict[str, object]:
    name, version, build, tags = parse_wheel_filename(path.name)
    tag_strings = sorted(str(tag) for tag in tags)

    with zipfile.ZipFile(path) as archive:
        names = set(archive.namelist())
        missing = [name for name in REQUIRED_PACKAGE_FILES if name not in names]
        if missing:
            raise SystemExit(f"{path.name}: missing package files: {missing}")

        metadata_name = next(
            (name for name in names if name.endswith(".dist-info/METADATA")),
            None,
        )
        if metadata_name is None:
            raise SystemExit(f"{path.name}: missing dist-info/METADATA")
        metadata = archive.read(metadata_name).decode("utf-8")
        require_metadata(metadata, path.name)
        staubli_assets = require_staubli_asset_policy(names, metadata, path.name)
        metadata_version_text = metadata_version(metadata, path.name)
        version_text = str(version)
        if metadata_version_text != version_text:
            raise SystemExit(
                f"{path.name}: wheel filename version {version_text!r} does not "
                f"match METADATA version {metadata_version_text!r}"
            )

        native_members = sorted(name for name in names if is_native_member(name))
        if not native_members:
            raise SystemExit(f"{path.name}: no native extension or library found")

        native_reports = []
        if not skip_native:
            if is_compatible(tags):
                inspected_members = [
                    name for name in native_members if name.startswith(f"{PACKAGE_DIR}/")
                ]
                if not inspected_members:
                    inspected_members = native_members
                native_reports = inspect_native_dependencies(
                    path,
                    archive,
                    inspected_members,
                    report_dir,
                )
            else:
                native_reports = [
                    f"Skipped native dependency inspection for {path.name} on "
                    f"{platform.system()}"
                ]

    return {
        "file": path.name,
        "bytes": path.stat().st_size,
        "mebibytes": round(path.stat().st_size / (1024 * 1024), 3),
        "name": str(name),
        "version": version_text,
        "build": build,
        "tags": tag_strings,
        "required_files": list(REQUIRED_PACKAGE_FILES),
        "native_files": native_members,
        "native_dependency_reports": native_reports,
        "staubli_assets": staubli_assets,
    }


def is_native_member(name: str) -> bool:
    if not (name.startswith(f"{PACKAGE_DIR}/") or ".libs/" in name):
        return False

    filename = PurePosixPath(name).name
    return (
        filename.endswith(NATIVE_SUFFIXES)
        or filename.endswith(".so")
        or ".so." in filename
    )


def metadata_version(metadata: str, artifact_name: str) -> str:
    version = Parser().parsestr(metadata).get("Version")
    if version is None:
        raise SystemExit(f"{artifact_name}: missing metadata header 'Version'")
    return version


def require_single_artifact_version(
    sdist_reports: list[dict[str, object]],
    wheel_reports: list[dict[str, object]],
) -> str:
    artifacts = [
        (str(report["file"]), str(report["version"]))
        for report in [*sdist_reports, *wheel_reports]
    ]
    versions = {version for _, version in artifacts}
    if len(versions) != 1:
        details = "\n".join(
            f"  {artifact}: {version}" for artifact, version in sorted(artifacts)
        )
        raise SystemExit(f"Release artifacts contain multiple versions:\n{details}")

    version = artifacts[0][1]
    print(f"All release artifacts have version {version}")
    return version


def require_staubli_asset_policy(
    names: list[str] | set[str],
    metadata: str,
    artifact_name: str,
) -> dict[str, object]:
    source_paths = {
        source_path
        for name in names
        if (source_path := normalized_source_path(name)) is not None
    }
    staubli_members = {
        name for name in source_paths if name.startswith(STAUBLI_ASSET_PREFIX)
    }
    asset_members = sorted(
        name
        for name in staubli_members
        if name != STAUBLI_LICENSE_FILE and not name.endswith("/")
    )
    source_license_present = STAUBLI_LICENSE_FILE in staubli_members
    metadata_license_present = STAUBLI_LICENSE_FILE in metadata_license_files(metadata)

    if asset_members and not source_license_present:
        raise SystemExit(
            f"{artifact_name}: Staubli assets are distributed without "
            f"{STAUBLI_LICENSE_FILE}"
        )
    if asset_members and not metadata_license_present:
        raise SystemExit(
            f"{artifact_name}: Staubli assets are distributed but metadata lacks "
            f"License-File: {STAUBLI_LICENSE_FILE}"
        )

    return {
        "asset_files": asset_members,
        "source_license_file": source_license_present,
        "metadata_license_file": metadata_license_present,
    }


def normalized_source_path(name: str) -> str | None:
    parts = PurePosixPath(name).parts
    if not parts or ".." in parts:
        return None
    if parts[0] == "python":
        return "/".join(parts)
    if len(parts) > 1 and parts[1] == "python":
        return "/".join(parts[1:])
    return None


def metadata_license_files(metadata: str) -> set[str]:
    return set(Parser().parsestr(metadata).get_all("License-File", []))


def require_metadata(metadata: str, artifact_name: str) -> None:
    required_headers = (
        "Name: spherical-wrist",
        "Summary: Python bindings for spherical-wrist industrial robot kinematics",
        "Description-Content-Type: text/markdown",
        "Project-URL: Repository, https://github.com/bourumir-wyngs/spherical-wrist",
        "Project-URL: Issues, https://github.com/bourumir-wyngs/spherical-wrist/issues",
    )
    for header in required_headers:
        if header not in metadata:
            raise SystemExit(f"{artifact_name}: missing metadata header {header!r}")

    if "# spherical-wrist" not in metadata:
        raise SystemExit(f"{artifact_name}: README content missing from metadata")


def inspect_native_dependencies(
    wheel: Path,
    archive: zipfile.ZipFile,
    native_members: list[str],
    report_dir: Path,
) -> list[str]:
    system = platform.system()
    if system == "Linux":
        tool = shutil.which("ldd")
        command_template = [tool] if tool else []
    elif system == "Darwin":
        tool = shutil.which("otool")
        command_template = [tool, "-L"] if tool else []
    elif system == "Windows":
        tool = shutil.which("dumpbin")
        command_template = [tool, "/DEPENDENTS"] if tool else []
    else:
        command_template = []

    if not command_template:
        note_path = report_dir / f"native-deps-{wheel.stem}.txt"
        note_path.write_text(
            f"No native dependency inspection tool found for {system}\n",
            encoding="utf-8",
        )
        return [str(note_path)]

    native_report_dir = report_dir / "native-deps" / wheel.stem
    native_report_dir.mkdir(parents=True, exist_ok=True)
    report_paths = []

    with tempfile.TemporaryDirectory(prefix="spherical-wrist-native-") as tmp:
        tmp_dir = Path(tmp)
        member_targets = {}
        for member in archive.namelist():
            target = archive_member_path(tmp_dir, member)
            if member.endswith("/"):
                target.mkdir(parents=True, exist_ok=True)
                continue

            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.read(member))
            if member in native_members:
                member_targets[member] = target

        for member, target in member_targets.items():
            command = [*command_template, str(target)]
            completed = subprocess.run(
                command,
                cwd=tmp_dir,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            report_path = archive_member_path(native_report_dir, f"{member}.txt")
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(
                "$ " + " ".join(command) + "\n" + completed.stdout,
                encoding="utf-8",
            )
            report_paths.append(str(report_path))
            output = completed.stdout.lower()
            if completed.returncode != 0 or "not found" in output:
                raise SystemExit(
                    f"Unresolved native dependency for {member} in {wheel.name}; "
                    f"see {report_path}"
                )

    return report_paths


def archive_member_path(root: Path, member: str) -> Path:
    member_path = PurePosixPath(member)
    if member_path.is_absolute() or ".." in member_path.parts:
        raise SystemExit(f"Unsafe archive member path: {member!r}")
    return root.joinpath(*member_path.parts)


def select_compatible_wheel(wheels: list[Path]) -> Path:
    supported = list(sys_tags())
    ranking = {tag: index for index, tag in enumerate(supported)}
    candidates = []

    for wheel in wheels:
        try:
            _, _, _, tags = parse_wheel_filename(wheel.name)
        except Exception:
            continue
        matching = [ranking[tag] for tag in tags if tag in ranking]
        if matching:
            candidates.append((min(matching), wheel))

    if not candidates:
        supported_preview = ", ".join(str(tag) for tag in supported[:5])
        raise SystemExit(
            "No wheel is compatible with this Python. "
            f"First supported tags: {supported_preview}"
        )

    candidates.sort(key=lambda item: (item[0], item[1].name))
    selected = candidates[0][1]
    print(f"Selected compatible wheel for installed tests: {selected.name}")
    return selected


def is_compatible(tags) -> bool:
    supported = set(sys_tags())
    return any(tag in supported for tag in tags)


def run_installed_tests(wheel: Path, venv_dir: Path, tests_dir: Path) -> None:
    if not tests_dir.is_dir():
        raise SystemExit(f"Tests directory not found: {tests_dir}")

    if venv_dir.exists():
        shutil.rmtree(venv_dir)
    venv_dir.parent.mkdir(parents=True, exist_ok=True)
    venv.EnvBuilder(with_pip=True, clear=True).create(venv_dir)

    python = venv_python(venv_dir)
    env = clean_python_env()
    run([str(python), "-m", "pip", "install", "--upgrade", "pip"], env=env)
    run([str(python), "-m", "pip", "install", "pytest"], env=env)
    run([str(python), "-m", "pip", "install", str(wheel)], env=env)

    with tempfile.TemporaryDirectory(prefix="spherical-wrist-installed-tests-") as tmp:
        tmp_dir = Path(tmp)
        copied_tests = tmp_dir / "tests"
        shutil.copytree(
            tests_dir,
            copied_tests,
            ignore=shutil.ignore_patterns("__pycache__"),
        )

        verify_code = "\n".join(
            [
                "from importlib.resources import files",
                "import spherical_wrist",
                "import spherical_wrist._internal",
                "root = files('spherical_wrist')",
                "assert (root / '_internal.pyi').is_file()",
                "assert (root / 'py.typed').is_file()",
                "print(spherical_wrist.__file__)",
            ]
        )
        run([str(python), "-c", verify_code], cwd=tmp_dir, env=env)
        run(
            [str(python), "-m", "pytest", "-q", str(copied_tests)],
            cwd=tmp_dir,
            env={**env, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"},
        )


def write_wheel_sizes(wheel_reports: list[dict[str, object]], path: Path) -> None:
    lines = ["wheel\tbytes\tMiB\ttags"]
    for report in sorted(wheel_reports, key=lambda item: str(item["file"])):
        tags = ",".join(report["tags"])  # type: ignore[arg-type]
        lines.append(
            f"{report['file']}\t{report['bytes']}\t{report['mebibytes']}\t{tags}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: dict[str, object]) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run(
    command: list[str],
    *,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
) -> None:
    print("$ " + " ".join(command), flush=True)
    subprocess.run(command, cwd=cwd, env=env, check=True)


def python_path_env(python: Path) -> dict[str, str]:
    env = clean_python_env()
    env["PATH"] = str(python.parent) + os.pathsep + env.get("PATH", "")
    return env


def clean_python_env() -> dict[str, str]:
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    return env


def venv_python(venv_dir: Path) -> Path:
    if platform.system() == "Windows":
        return venv_dir / "Scripts" / "python.exe"
    return venv_dir / "bin" / "python"


if __name__ == "__main__":
    main()
