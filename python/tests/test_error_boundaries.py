from collections.abc import Callable
import re

import numpy as np
import pytest

from spherical_wrist import Frame, KinematicModel
from spherical_wrist._internal import Mesh as _MeshInternal
from spherical_wrist._internal import Robot as _RobotInternal


def test_invalid_robot_pose_matrix_surfaces_as_value_error_not_panic() -> None:
    invalid_tool = np.eye(4)
    invalid_tool[:3, :3] = 0.0

    assert_value_error_not_panic(
        lambda: _RobotInternal(_model(), True, invalid_tool, None, None, None, None),
        "rotation axes must be unit length",
    )


def test_invalid_mesh_pose_matrix_surfaces_as_value_error_not_panic() -> None:
    invalid_pose = np.eye(4)
    invalid_pose[:3, :3] = 0.0

    assert_value_error_not_panic(
        lambda: _MeshInternal.from_arrays(_cube_vertices(), _cube_triangles(), invalid_pose),
        "rotation axes must be unit length",
    )


def test_degenerate_frame_tie_points_surface_as_value_error_not_panic() -> None:
    original_tie_points = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    target_tie_points = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 2.0, 0.0],
        ],
        dtype=np.float64,
    )

    assert_value_error_not_panic(
        lambda: Frame.from_tie(original_tie_points, target_tie_points),
        "colinear source points",
    )


def assert_value_error_not_panic(call: Callable[[], object], pattern: str) -> None:
    try:
        call()
    except ValueError as exc:
        assert type(exc) is ValueError
        assert re.search(pattern, str(exc)), str(exc)
    except BaseException as exc:
        pytest.fail(
            "expected ValueError, not "
            f"{type(exc).__module__}.{type(exc).__name__}: {exc}"
        )
    else:
        pytest.fail("expected ValueError")


def _model() -> KinematicModel:
    return KinematicModel(
        a1=400,
        a2=-250,
        b=0,
        c1=830,
        c2=1175,
        c3=1444,
        c4=230,
    )


def _cube_vertices() -> list[tuple[float, float, float]]:
    return [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 1.0),
        (1.0, 1.0, 1.0),
        (0.0, 1.0, 1.0),
    ]


def _cube_triangles() -> list[tuple[int, int, int]]:
    return [
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ]
