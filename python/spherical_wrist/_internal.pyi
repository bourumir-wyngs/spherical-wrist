from typing import List, Tuple, Optional

import numpy as np
import numpy.typing as npt

Matrix4 = npt.NDArray[np.float64]
Joints = Tuple[float, float, float, float, float, float]


class KinematicModel:
    a1: float
    a2: float
    b: float
    c1: float
    c2: float
    c3: float
    c4: float
    offsets: Joints
    flip_axes: Tuple[bool, bool, bool, bool, bool, bool]

    def __init__(
        self,
        a1: float = 0,
        a2: float = 0,
        b: float = 0,
        c1: float = 0,
        c2: float = 0,
        c3: float = 0,
        c4: float = 0,
        offsets: Joints = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        flip_axes: Tuple[bool, bool, bool, bool, bool, bool] = (
            False,
            False,
            False,
            False,
            False,
            False,
        ),
    ) -> None: ...


class Parallelogram:
    scaling: float
    driven: int
    coupled: int

    def __init__(self, scaling: float = 1.0, driven: int = 1, coupled: int = 2) -> None: ...

    def __repr__(self) -> str: ...


class Robot:
    degrees: bool

    def __init__(
        self,
        kinematic_model: KinematicModel,
        degrees: bool = True,
        tool: Optional[Matrix4] = None,
        base: Optional[Matrix4] = None,
        frame: Optional[Matrix4] = None,
        parallelogram: Optional[Parallelogram] = None,
    ) -> None: ...

    def __repr__(self) -> str: ...

    def forward(
        self,
        joints: Joints,
        ee_transform: Optional[Matrix4] = None,
    ) -> Matrix4: ...

    def inverse(
        self,
        pose: Matrix4,
        current_joints: Optional[Joints] = None,
        ee_transform: Optional[Matrix4] = None,
    ) -> List[Joints]: ...
