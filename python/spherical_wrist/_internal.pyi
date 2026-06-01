from typing import List, Tuple, Optional

import numpy as np
import numpy.typing as npt

Matrix4 = npt.NDArray[np.float64]
Joints = Tuple[float, float, float, float, float, float]
TcpBox = Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]
BY_PREV: float
BY_CONSTRAINTS: float
CONSTRAINT_CENTERED: Joints
NEVER_COLLIDES: float
TOUCH_ONLY: float
J1: int
J2: int
J3: int
J4: int
J5: int
J6: int
J_TOOL: int
J_BASE: int
ENV_START_IDX: int
CHECK_MODE_ALL: str
CHECK_MODE_FIRST_COLLISION_ONLY: str
CHECK_MODE_NO_CHECK: str
DEFAULT_TRANSITION_COSTS: Joints
PATH_FLAG_NONE: int
PATH_FLAG_ONBOARDING: int
PATH_FLAG_TRACE: int
PATH_FLAG_LIN_INTERP: int
PATH_FLAG_LAND: int
PATH_FLAG_LANDING: int
PATH_FLAG_PARK: int
PATH_FLAG_PARKING: int
PATH_FLAG_FORWARDS: int
PATH_FLAG_BACKWARDS: int
PATH_FLAG_RECONFIGURING: int
PATH_FLAG_ORIGINAL: int
PATH_FLAG_DEBUG: int
MOVE_KIND_JOINT: str
MOVE_KIND_CARTESIAN: str


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


class Constraints:
    sorting_weight: float

    def __init__(
        self,
        from_limits: Joints,
        to_limits: Joints,
        sorting_weight: float = 0.0,
        radians: bool = False,
    ) -> None: ...

    def from_limits(self, radians: bool = False) -> Joints: ...

    def to_limits(self, radians: bool = False) -> Joints: ...

    def limits(self, radians: bool = False) -> Tuple[Joints, Joints]: ...

    def centers(self, radians: bool = False) -> Joints: ...

    def tolerances(self, radians: bool = False) -> Joints: ...

    def compliant(self, joints: Joints, radians: bool = False) -> bool: ...

    def random_joints(self, radians: bool = False) -> Joints: ...

    def __repr__(self) -> str: ...


class SafetyDistances:
    to_environment: float
    to_robot_default: float
    special_distances: List[Tuple[int, int, float]]
    mode: str

    def __init__(
        self,
        to_environment: float = 0.0,
        to_robot_default: float = 0.0,
        special_distances: Optional[List[Tuple[int, int, float]]] = None,
        mode: str = "all",
    ) -> None: ...

    @staticmethod
    def standard(mode: str = "all") -> "SafetyDistances": ...

    def __repr__(self) -> str: ...


class Parallelogram:
    scaling: float
    driven: int
    coupled: int

    def __init__(self, scaling: float = 1.0, driven: int = 1, coupled: int = 2) -> None: ...

    def __repr__(self) -> str: ...


class Mesh:
    vertex_count: int
    triangle_count: int

    def __init__(
        self,
        path: str,
        scale: float = 1.0,
        pose: Optional[Matrix4] = None,
    ) -> None: ...

    @staticmethod
    def from_path(path: str, scale: float = 1.0) -> "Mesh": ...

    @staticmethod
    def from_arrays(
        vertices: List[Tuple[float, float, float]],
        triangles: List[Tuple[int, int, int]],
        pose: Optional[Matrix4] = None,
    ) -> "Mesh": ...

    def transformed(self, pose: Matrix4) -> "Mesh": ...

    def transformed_by(self, pose: Matrix4) -> "Mesh": ...

    def collides(self, other: "Mesh", safety_distance: float = 0.0) -> bool: ...

    def __repr__(self) -> str: ...


class KinematicsWithShape:
    degrees: bool
    constraints: Optional[Constraints]
    parallelogram: Optional[Parallelogram]

    def __init__(
        self,
        kinematic_model: KinematicModel,
        degrees: bool,
        joint_meshes: List[Mesh],
        constraints: Optional[Constraints] = None,
        base: Optional[Matrix4] = None,
        tool: Optional[Matrix4] = None,
        parallelogram: Optional[Parallelogram] = None,
        base_mesh: Optional[Mesh] = None,
        tool_mesh: Optional[Mesh] = None,
        environment: Optional[List[Mesh]] = None,
        safety: Optional[SafetyDistances] = None,
        first_collision_only: bool = False,
    ) -> None: ...

    def __repr__(self) -> str: ...

    def forward(self, joints: Joints) -> Matrix4: ...

    def inverse(
        self,
        pose: Matrix4,
        current_joints: Optional[Joints] = None,
    ) -> List[Joints]: ...

    def inverse_continuing(
        self,
        pose: Matrix4,
        previous_joints: Joints,
    ) -> List[Joints]: ...

    def inverse_5dof(
        self,
        pose: Matrix4,
        j6: float = 0.0,
    ) -> List[Joints]: ...

    def inverse_continuing_5dof(
        self,
        pose: Matrix4,
        previous_joints: Joints,
    ) -> List[Joints]: ...

    def forward_with_joint_poses(self, joints: Joints) -> List[Matrix4]: ...

    def kinematic_singularity(self, joints: Joints) -> Optional[str]: ...

    def collides(self, joints: Joints) -> bool: ...

    def collision_details(self, joints: Joints) -> List[Tuple[int, int]]: ...

    def near(
        self,
        joints: Joints,
        safety: SafetyDistances,
    ) -> List[Tuple[int, int]]: ...

    def non_colliding_offsets(
        self,
        joints: Joints,
        from_limits: Joints,
        to_limits: Joints,
    ) -> List[Joints]: ...

    def positioned_robot(
        self,
        joints: Joints,
    ) -> Tuple[List[Matrix4], Optional[Matrix4], List[Matrix4]]: ...


def visualize_robot(
    robot: KinematicsWithShape,
    initial_joints: Joints,
    tcp_box: TcpBox,
) -> "VisualizationHandle": ...


def visualize_robot_with_safety(
    robot: KinematicsWithShape,
    initial_joints: Joints,
    tcp_box: TcpBox,
    safety: SafetyDistances,
) -> "VisualizationHandle": ...


class VisualizationHandle:
    is_running: bool

    def set_joints(self, joints: Joints) -> None: ...

    def set_position(self, joints: Joints) -> None: ...

    def set_pose(
        self,
        pose: Matrix4,
        previous_position: Optional[Joints] = None,
    ) -> Joints: ...

    def close(self) -> None: ...

    def __repr__(self) -> str: ...


class RRTPlanner:
    max_try: int
    debug: bool

    def __init__(
        self,
        step_size_joint_space: float = 3.0,
        max_try: int = 2000,
        debug: bool = False,
        radians: bool = False,
    ) -> None: ...

    def step_size_joint_space(self, radians: bool = False) -> float: ...

    def plan_rrt(
        self,
        robot: KinematicsWithShape,
        start: Joints,
        goal: Joints,
    ) -> List[Joints]: ...

    def __repr__(self) -> str: ...


class CartesianPlanner:
    check_step_m: float
    transition_coefficients: Joints
    linear_recursion_depth: int
    rrt: RRTPlanner
    allow_reconfigure: bool
    include_linear_interpolation: bool
    debug: bool

    def __init__(
        self,
        check_step_m: float = 0.02,
        check_step_rad: float = 3.0,
        max_transition_cost: float = 3.0,
        transition_coefficients: Optional[Joints] = None,
        linear_recursion_depth: int = 8,
        rrt: Optional[RRTPlanner] = None,
        allow_reconfigure: bool = True,
        include_linear_interpolation: bool = True,
        debug: bool = False,
        radians: bool = False,
    ) -> None: ...

    def check_step_rad(self, radians: bool = False) -> float: ...

    def max_transition_cost(self, radians: bool = False) -> float: ...

    def plan(
        self,
        robot: KinematicsWithShape,
        start: Joints,
        land: Matrix4,
        steps: List[Matrix4],
        park: Matrix4,
    ) -> List["AnnotatedJoints"]: ...

    def __repr__(self) -> str: ...


class AnnotatedJoints:
    joints: Joints
    flags: int
    move_into: str

    def has_flag(self, flag: int) -> bool: ...

    def __repr__(self) -> str: ...


class Robot:
    degrees: bool
    constraints: Optional[Constraints]

    def __init__(
        self,
        kinematic_model: KinematicModel,
        degrees: bool = True,
        tool: Optional[Matrix4] = None,
        base: Optional[Matrix4] = None,
        frame: Optional[Matrix4] = None,
        parallelogram: Optional[Parallelogram] = None,
        constraints: Optional[Constraints] = None,
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

    def inverse_continuing(
        self,
        pose: Matrix4,
        previous_joints: Joints,
        ee_transform: Optional[Matrix4] = None,
    ) -> List[Joints]: ...

    def inverse_5dof(
        self,
        pose: Matrix4,
        j6: float = 0.0,
        ee_transform: Optional[Matrix4] = None,
    ) -> List[Joints]: ...

    def inverse_continuing_5dof(
        self,
        pose: Matrix4,
        previous_joints: Joints,
        ee_transform: Optional[Matrix4] = None,
    ) -> List[Joints]: ...

    def forward_with_joint_poses(
        self,
        joints: Joints,
        ee_transform: Optional[Matrix4] = None,
    ) -> List[Matrix4]: ...

    def kinematic_singularity(self, joints: Joints) -> Optional[str]: ...
