from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.spatial.transform import RigidTransform, Rotation

from spherical_wrist import (
    BY_PREV,
    CHECK_MODE_ALL,
    CHECK_MODE_FIRST_COLLISION_ONLY,
    Constraints,
    ENV_START_IDX,
    J2,
    J3,
    J4,
    J6,
    J_BASE,
    J_TOOL,
    KinematicModel,
    KinematicsWithShape,
    Mesh,
    NEVER_COLLIDES,
    SafetyDistances,
    TcpBox,
)
from spherical_wrist import Joints


DEFAULT_TCP_BOX: TcpBox = ((-2.0, 2.0), (-2.0, 2.0), (1.0, 2.0))
WORKSPACE = Path(__file__).resolve().parents[3]
RS_OPW_KINEMATICS = WORKSPACE / "rs-opw-kinematics"
DATA = RS_OPW_KINEMATICS / "src" / "tests" / "data"
RX160_MESHES = DATA / "staubli" / "rx160"


def irb2400_10() -> KinematicModel:
    """ABB IRB 2400/10 parameters matching Parameters::irb2400_10()."""
    return KinematicModel(
        a1=0.100,
        a2=-0.135,
        b=0.000,
        c1=0.615,
        c2=0.705,
        c3=0.755,
        c4=0.085,
        offsets=(0.0, 0.0, -90.0, 0.0, 0.0, 0.0),
    )


def staubli_tx2_160l() -> KinematicModel:
    """Staubli TX2-160L parameters matching Parameters::staubli_tx2_160l()."""
    return KinematicModel(
        a1=0.150,
        a2=0.000,
        b=0.000,
        c1=0.550,
        c2=0.825,
        c3=0.925,
        c4=0.110,
    )


def staubli_rx160() -> KinematicModel:
    """Staubli RX160 parameters used by the collision/visualization examples."""
    return KinematicModel(
        a1=0.150,
        a2=0.000,
        b=0.000,
        c1=0.550,
        c2=0.825,
        c3=0.625,
        c4=0.110,
    )


def translation(x: float, y: float, z: float) -> RigidTransform:
    """Create a translation-only SciPy RigidTransform."""
    return RigidTransform.from_components(
        rotation=Rotation.identity(),
        translation=[x, y, z],
    )


def compose(left: RigidTransform, right: RigidTransform) -> RigidTransform:
    """Compose transforms in matrix order, left * right."""
    return RigidTransform.from_matrix(left.as_matrix() @ right.as_matrix())


def dump_joints(joints: Joints, *, label: str | None = None) -> None:
    """Print joints in degrees, similar to rs-opw-kinematics dump_joints."""
    prefix = "" if label is None else f"{label}: "
    print(f"{prefix}{np.round(np.asarray(joints), 4).tolist()}")


def dump_solutions(solutions: Iterable[Joints], *, limit: int | None = None) -> None:
    """Print IK/path solutions compactly."""
    solutions = list(solutions)
    print(f"{len(solutions)} solution(s)")
    for index, joints in enumerate(solutions):
        if limit is not None and index >= limit:
            print(f"  ... {len(solutions) - limit} more")
            break
        print(f"  {index:02d}: {np.round(np.asarray(joints), 4).tolist()}")


def wait_for_visualization(handle) -> None:
    """Keep an example alive while the visualization window is open."""
    if sys.stdin.isatty():
        input("Window is running. Press Enter here to close it... ")
        return

    print("Window is running. Close the window to exit.")
    while handle.is_running:
        time.sleep(0.1)


def create_rx160_robot(
    *,
    first_collision_only: bool = False,
    flag_mesh: str = "flag.ply",
    j6_limit: float = 360.0,
) -> KinematicsWithShape:
    """
    Build the shaped RX160 robot used by visualization and path planning examples.

    This mirrors the Rust examples: six link meshes, base mesh, tool mesh, and
    four static environment objects are all loaded from rs-opw-kinematics test
    data. The meshes are Apache/BSD-compatible test assets from the upstream
    repository.
    """
    monolith = Mesh.from_path(DATA / "object.stl")

    # Static environment meshes use their Mesh pose as a global transform.
    environment = [
        monolith.transformed(translation(1.0, 0.0, 0.0)),
        monolith.transformed(translation(-1.0, 0.0, 0.0)),
        monolith.transformed(translation(0.0, 1.0, 0.0)),
        monolith.transformed(translation(0.0, -1.0, 0.0)),
    ]

    mode = CHECK_MODE_FIRST_COLLISION_ONLY if first_collision_only else CHECK_MODE_ALL
    constraints = Constraints(
        (-225.0, -225.0, -225.0, -225.0, -225.0, -j6_limit),
        (225.0, 225.0, 225.0, 225.0, 225.0, j6_limit),
        sorting_weight=BY_PREV,
    )
    safety = SafetyDistances(
        to_environment=0.05,
        to_robot_default=0.05,
        special_distances=[
            # These pairs are close by construction and are allowed to touch
            # less, or ignored entirely, matching the Rust examples.
            (J2, J_BASE, NEVER_COLLIDES),
            (J3, J_BASE, NEVER_COLLIDES),
            (J2, J4, NEVER_COLLIDES),
            (J3, J4, NEVER_COLLIDES),
            (J4, J_TOOL, 0.02),
            (J4, J6, 0.02),
        ],
        mode=mode,
    )

    return KinematicsWithShape(
        staubli_rx160(),
        joint_meshes=[
            Mesh.from_path(RX160_MESHES / "link_1.stl"),
            Mesh.from_path(RX160_MESHES / "link_2.stl"),
            Mesh.from_path(RX160_MESHES / "link_3.stl"),
            Mesh.from_path(RX160_MESHES / "link_4.stl"),
            Mesh.from_path(RX160_MESHES / "link_5.stl"),
            Mesh.from_path(RX160_MESHES / "link_6.stl"),
        ],
        degrees=True,
        constraints=constraints,
        base=translation(0.4, 0.7, 0.0),
        base_mesh=Mesh.from_path(RX160_MESHES / "base_link.stl"),
        tool=translation(0.0, 0.0, 0.5),
        tool_mesh=Mesh.from_path(DATA / flag_mesh),
        environment=environment,
        safety=safety,
    )


def collision_name(index: int) -> str:
    """Human-readable collision object name for example output."""
    if 0 <= index <= 5:
        return f"J{index + 1}"
    if index == J_TOOL:
        return "tool"
    if index == J_BASE:
        return "base"
    if index >= ENV_START_IDX:
        return f"environment[{index - ENV_START_IDX}]"
    return str(index)
