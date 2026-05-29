"""
spherical-wrist: Python bindings for spherical-wrist industrial robot kinematics.

The initial API mirrors the introductory py-opw-kinematics workflow while using
the local rs-opw-kinematics crate internally.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from os import PathLike
from typing import Iterable, Optional, Sequence, Tuple, cast

import numpy as np
from numpy.typing import ArrayLike
from scipy.spatial.transform import RigidTransform

from ._internal import BY_CONSTRAINTS
from ._internal import BY_PREV
from ._internal import CHECK_MODE_ALL
from ._internal import CHECK_MODE_FIRST_COLLISION_ONLY
from ._internal import CHECK_MODE_NO_CHECK
from ._internal import CONSTRAINT_CENTERED
from ._internal import Constraints
from ._internal import AnnotatedJoints
from ._internal import CartesianPlanner as _CartesianPlannerInternal
from ._internal import DEFAULT_TRANSITION_COSTS
from ._internal import ENV_START_IDX
from ._internal import J1
from ._internal import J2
from ._internal import J3
from ._internal import J4
from ._internal import J5
from ._internal import J6
from ._internal import J_BASE
from ._internal import J_TOOL
from ._internal import KinematicsWithShape as _KinematicsWithShapeInternal
from ._internal import KinematicModel
from ._internal import Mesh as _MeshInternal
from ._internal import NEVER_COLLIDES
from ._internal import PATH_FLAG_ALTERED
from ._internal import PATH_FLAG_BACKWARDS
from ._internal import PATH_FLAG_CARTESIAN
from ._internal import PATH_FLAG_DEBUG
from ._internal import PATH_FLAG_FORWARDS
from ._internal import PATH_FLAG_LAND
from ._internal import PATH_FLAG_LANDING
from ._internal import PATH_FLAG_LIN_INTERP
from ._internal import PATH_FLAG_NONE
from ._internal import PATH_FLAG_ONBOARDING
from ._internal import PATH_FLAG_ORIGINAL
from ._internal import PATH_FLAG_PARK
from ._internal import PATH_FLAG_PARKING
from ._internal import PATH_FLAG_TRACE
from ._internal import Parallelogram
from ._internal import RRTPlanner as _RRTPlannerInternal
from ._internal import Robot as _RobotInternal
from ._internal import SafetyDistances
from ._internal import TOUCH_ONLY
from ._internal import VisualizationHandle as _VisualizationHandleInternal
from ._internal import visualize_robot as _visualize_robot_internal
from ._internal import visualize_robot_with_safety as _visualize_robot_with_safety_internal

Joints = Tuple[float, float, float, float, float, float]
PathStep = Joints | AnnotatedJoints
PathInput = str | PathLike[str]
TcpBox = Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]


@dataclass(frozen=True)
class PositionedRobot:
    """World poses for the robot collision shapes at a joint configuration."""

    joints: RigidTransform
    tool: Optional[RigidTransform]
    environment: tuple[RigidTransform, ...]


class Mesh:
    """Triangle mesh with Parry collision checks."""

    def __init__(
        self,
        path: PathInput,
        scale: float = 1.0,
        pose: Optional[RigidTransform] = None,
    ) -> None:
        """
        Load a triangle mesh from a file.

        :param path: Mesh path supported by rs-read-trimesh (.stl, .ply, .obj, .dae).
        :param scale: Optional vertex scale applied while loading.
        :param pose: Optional mesh pose.
        """
        self._mesh = _MeshInternal(
            str(path),
            scale,
            None if pose is None else pose.as_matrix(),
        )

    @classmethod
    def from_path(cls, path: PathInput, scale: float = 1.0) -> Mesh:
        """Load a mesh from a path."""
        return cls(path, scale=scale)

    @classmethod
    def from_arrays(
        cls,
        vertices: ArrayLike,
        triangles: ArrayLike,
        pose: Optional[RigidTransform] = None,
    ) -> Mesh:
        """
        Build a small triangle mesh directly from arrays.

        :param vertices: Array-like shape ``(n, 3)`` with x, y, z coordinates.
        :param triangles: Array-like shape ``(m, 3)`` with vertex indices.
        :param pose: Optional mesh pose.
        """
        vertex_array = np.asarray(vertices, dtype=np.float64)
        if vertex_array.ndim != 2 or vertex_array.shape[1] != 3:
            raise ValueError("vertices must have shape (n, 3)")
        if not np.isfinite(vertex_array).all():
            raise ValueError("vertices must contain finite values")

        triangle_array = np.asarray(triangles)
        if triangle_array.ndim != 2 or triangle_array.shape[1] != 3:
            raise ValueError("triangles must have shape (m, 3)")
        if not np.issubdtype(triangle_array.dtype, np.integer):
            raise ValueError("triangles must contain integer indices")
        if (triangle_array < 0).any():
            raise ValueError("triangles must contain non-negative indices")

        max_u32 = np.iinfo(np.uint32).max
        if (triangle_array > max_u32).any():
            raise ValueError("triangles must fit in uint32")

        vertex_triples = [tuple(row) for row in vertex_array.tolist()]
        triangle_triples = [
            tuple(int(index) for index in row)
            for row in np.ascontiguousarray(triangle_array, dtype=np.uint32).tolist()
        ]
        mesh = _MeshInternal.from_arrays(
            vertex_triples,
            triangle_triples,
            None if pose is None else pose.as_matrix(),
        )
        return cls._from_internal(mesh)

    @classmethod
    def _from_internal(cls, mesh: _MeshInternal) -> Mesh:
        instance = cls.__new__(cls)
        instance._mesh = mesh
        return instance

    @property
    def vertex_count(self) -> int:
        """Number of mesh vertices."""
        return int(self._mesh.vertex_count)

    @property
    def triangle_count(self) -> int:
        """Number of mesh triangles."""
        return int(self._mesh.triangle_count)

    def transformed(self, pose: RigidTransform) -> Mesh:
        """Return a copy transformed by ``pose``."""
        return self._from_internal(self._mesh.transformed(pose.as_matrix()))

    def transformed_by(self, pose: RigidTransform) -> Mesh:
        """Alias for :meth:`transformed`."""
        return self.transformed(pose)

    def collides(self, other: Mesh, safety_distance: float = 0.0) -> bool:
        """
        Check collision or proximity with another mesh.

        With ``safety_distance > 0``, this returns true when the meshes intersect
        or their Parry distance is at most the requested safety distance.
        """
        return bool(self._mesh.collides(other._mesh, safety_distance))

    def __repr__(self) -> str:
        return self._mesh.__repr__()


class Robot:
    """Robot kinematics with SciPy RigidTransform integration."""

    def __init__(
        self,
        kinematic_model: KinematicModel,
        degrees: bool = True,
        tool: Optional[RigidTransform] = None,
        base: Optional[RigidTransform] = None,
        frame: Optional[RigidTransform] = None,
        parallelogram: Optional[Parallelogram] = None,
        constraints: Optional[Constraints] = None,
    ) -> None:
        """
        Initialize robot kinematics.

        :param kinematic_model: OPW kinematic model.
        :param degrees: Whether joint angles are in degrees.
        :param tool: Optional flange-to-TCP transform.
        :param base: Optional world-to-robot-base transform.
        :param frame: Optional working-frame transform.
        :param parallelogram: Optional parallelogram coupling configuration.
        :param constraints: Optional joint constraints used by inverse kinematics.
        """
        self._robot = _RobotInternal(
            kinematic_model,
            degrees,
            None if tool is None else tool.as_matrix(),
            None if base is None else base.as_matrix(),
            None if frame is None else frame.as_matrix(),
            parallelogram,
            constraints,
        )
        self._degrees = degrees
        self._kinematic_model = kinematic_model
        self._tool = tool
        self._base = base
        self._frame = frame
        self._parallelogram = parallelogram
        self._constraints = constraints

    @property
    def degrees(self) -> bool:
        """Whether joint angles are in degrees."""
        return self._degrees

    @property
    def parallelogram(self) -> Optional[Parallelogram]:
        """Parallelogram coupling configured for this robot, if any."""
        return self._parallelogram

    @property
    def constraints(self) -> Optional[Constraints]:
        """Joint constraints configured for this robot, if any."""
        return self._constraints

    def __repr__(self) -> str:
        return self._robot.__repr__()

    def forward(
        self,
        joints: Joints,
        ee_transform: Optional[RigidTransform] = None,
        *,
        tool: Optional[RigidTransform] = None,
    ) -> RigidTransform:
        """
        Compute forward kinematics for a joint configuration.

        :param joints: Joint angles J1-J6.
        :param ee_transform: Optional per-call flange-to-TCP transform.
            This is a compatibility alias for ``tool``.
        :param tool: Optional per-call flange-to-TCP transform.
        :return: TCP pose as a SciPy RigidTransform.
        """
        tool_transform = _resolve_tool(tool=tool, ee_transform=ee_transform)
        tool_matrix = None if tool_transform is None else tool_transform.as_matrix()
        matrix = self._robot.forward(joints, tool_matrix)
        return RigidTransform.from_matrix(matrix)

    def inverse(
        self,
        pose: RigidTransform,
        current_joints: Optional[Joints] = None,
        ee_transform: Optional[RigidTransform] = None,
        *,
        tool: Optional[RigidTransform] = None,
    ) -> list[Joints]:
        """
        Compute inverse kinematics for a TCP pose.

        :param pose: Target TCP pose.
        :param current_joints: Optional current joint configuration used to rank
            and continue through singularities.
        :param ee_transform: Optional per-call flange-to-TCP transform.
            This is a compatibility alias for ``tool``.
        :param tool: Optional per-call flange-to-TCP transform.
        :return: Possible joint configurations.
        """
        tool_transform = _resolve_tool(tool=tool, ee_transform=ee_transform)
        tool_matrix = None if tool_transform is None else tool_transform.as_matrix()
        solutions = self._robot.inverse(pose.as_matrix(), current_joints, tool_matrix)
        return [cast(Joints, tuple(solution)) for solution in solutions]

    def inverse_continuing(
        self,
        pose: RigidTransform,
        previous_joints: Joints,
        ee_transform: Optional[RigidTransform] = None,
        *,
        tool: Optional[RigidTransform] = None,
    ) -> list[Joints]:
        """
        Compute inverse kinematics while sorting near a previous joint state.

        :param pose: Target TCP pose.
        :param previous_joints: Previous joint values, or ``CONSTRAINT_CENTERED``.
        :param ee_transform: Optional per-call flange-to-TCP transform.
            This is a compatibility alias for ``tool``.
        :param tool: Optional per-call flange-to-TCP transform.
        """
        tool_transform = _resolve_tool(tool=tool, ee_transform=ee_transform)
        tool_matrix = None if tool_transform is None else tool_transform.as_matrix()
        solutions = self._robot.inverse_continuing(
            pose.as_matrix(),
            previous_joints,
            tool_matrix,
        )
        return [cast(Joints, tuple(solution)) for solution in solutions]

    def inverse_5dof(
        self,
        pose: RigidTransform,
        j6: float = 0.0,
        ee_transform: Optional[RigidTransform] = None,
        *,
        tool: Optional[RigidTransform] = None,
    ) -> list[Joints]:
        """
        Compute position-accurate 5-DOF inverse kinematics.

        Rotation around joint 6 is ignored and the returned J6 value is set from
        ``j6`` in this robot's angle unit.
        """
        tool_transform = _resolve_tool(tool=tool, ee_transform=ee_transform)
        tool_matrix = None if tool_transform is None else tool_transform.as_matrix()
        solutions = self._robot.inverse_5dof(pose.as_matrix(), j6, tool_matrix)
        return [cast(Joints, tuple(solution)) for solution in solutions]

    def inverse_continuing_5dof(
        self,
        pose: RigidTransform,
        previous_joints: Joints,
        ee_transform: Optional[RigidTransform] = None,
        *,
        tool: Optional[RigidTransform] = None,
    ) -> list[Joints]:
        """
        Compute 5-DOF inverse kinematics while sorting near previous joints.

        The returned J6 value is taken from ``previous_joints``.
        """
        tool_transform = _resolve_tool(tool=tool, ee_transform=ee_transform)
        tool_matrix = None if tool_transform is None else tool_transform.as_matrix()
        solutions = self._robot.inverse_continuing_5dof(
            pose.as_matrix(),
            previous_joints,
            tool_matrix,
        )
        return [cast(Joints, tuple(solution)) for solution in solutions]

    def forward_with_joint_poses(
        self,
        joints: Joints,
        ee_transform: Optional[RigidTransform] = None,
        *,
        tool: Optional[RigidTransform] = None,
    ) -> RigidTransform:
        """
        Compute the six joint poses exposed by the Rust Kinematics trait.

        For a configured tool, the sixth pose remains the J6 pose, matching
        ``rs-opw-kinematics`` trait behavior.
        """
        tool_transform = _resolve_tool(tool=tool, ee_transform=ee_transform)
        tool_matrix = None if tool_transform is None else tool_transform.as_matrix()
        return RigidTransform.from_matrix(
            np.asarray(self._robot.forward_with_joint_poses(joints, tool_matrix))
        )

    def kinematic_singularity(self, joints: Joints) -> Optional[str]:
        """Return the singularity kind, currently ``"A"``, or ``None``."""
        return self._robot.kinematic_singularity(joints)


class KinematicsWithShape:
    """Robot kinematics with collision geometry."""

    def __init__(
        self,
        kinematic_model: KinematicModel,
        joint_meshes: Sequence[Mesh],
        degrees: bool = True,
        constraints: Optional[Constraints] = None,
        base: Optional[RigidTransform] = None,
        tool: Optional[RigidTransform] = None,
        parallelogram: Optional[Parallelogram] = None,
        base_mesh: Optional[Mesh] = None,
        tool_mesh: Optional[Mesh] = None,
        environment: Sequence[Mesh] = (),
        safety: Optional[SafetyDistances] = None,
        first_collision_only: bool = False,
    ) -> None:
        """
        Initialize kinematics and robot shape for collision-aware solving.

        :param joint_meshes: Exactly six joint meshes in their local joint frames.
        :param base: Optional world-to-robot-base transform.
        :param tool: Optional J6-to-TCP transform.
        :param base_mesh: Optional base mesh. Its pose is composed with ``base``.
        :param tool_mesh: Optional tool mesh in the J6 frame.
        :param environment: Static collision meshes. Each mesh pose is global.
        :param safety: Optional safety distance configuration.
        """
        joint_mesh_list = list(joint_meshes)
        if len(joint_mesh_list) != 6:
            raise ValueError("joint_meshes must contain exactly 6 meshes")

        self._robot = _KinematicsWithShapeInternal(
            kinematic_model,
            degrees,
            [_mesh_internal(mesh) for mesh in joint_mesh_list],
            constraints,
            None if base is None else base.as_matrix(),
            None if tool is None else tool.as_matrix(),
            parallelogram,
            None if base_mesh is None else _mesh_internal(base_mesh),
            None if tool_mesh is None else _mesh_internal(tool_mesh),
            [_mesh_internal(mesh) for mesh in environment],
            safety,
            first_collision_only,
        )
        self._degrees = degrees
        self._kinematic_model = kinematic_model
        self._constraints = constraints
        self._parallelogram = parallelogram

    @property
    def degrees(self) -> bool:
        """Whether joint angles are in degrees."""
        return self._degrees

    @property
    def constraints(self) -> Optional[Constraints]:
        """Joint constraints configured for this robot, if any."""
        return self._constraints

    @property
    def parallelogram(self) -> Optional[Parallelogram]:
        """Parallelogram coupling configured for this robot, if any."""
        return self._parallelogram

    def __repr__(self) -> str:
        return self._robot.__repr__()

    def forward(self, joints: Joints) -> RigidTransform:
        """Compute forward kinematics for a joint configuration."""
        return RigidTransform.from_matrix(self._robot.forward(joints))

    def inverse(
        self,
        pose: RigidTransform,
        current_joints: Optional[Joints] = None,
    ) -> list[Joints]:
        """Compute collision-filtered inverse kinematics for a TCP pose."""
        solutions = self._robot.inverse(pose.as_matrix(), current_joints)
        return [cast(Joints, tuple(solution)) for solution in solutions]

    def inverse_continuing(
        self,
        pose: RigidTransform,
        previous_joints: Joints,
    ) -> list[Joints]:
        """Compute collision-filtered inverse kinematics near previous joints."""
        solutions = self._robot.inverse_continuing(pose.as_matrix(), previous_joints)
        return [cast(Joints, tuple(solution)) for solution in solutions]

    def inverse_5dof(
        self,
        pose: RigidTransform,
        j6: float = 0.0,
    ) -> list[Joints]:
        """Compute collision-filtered 5-DOF inverse kinematics."""
        solutions = self._robot.inverse_5dof(pose.as_matrix(), j6)
        return [cast(Joints, tuple(solution)) for solution in solutions]

    def inverse_continuing_5dof(
        self,
        pose: RigidTransform,
        previous_joints: Joints,
    ) -> list[Joints]:
        """Compute collision-filtered 5-DOF inverse kinematics near previous joints."""
        solutions = self._robot.inverse_continuing_5dof(
            pose.as_matrix(),
            previous_joints,
        )
        return [cast(Joints, tuple(solution)) for solution in solutions]

    def forward_with_joint_poses(self, joints: Joints) -> RigidTransform:
        """Compute world poses for the six robot joints."""
        return RigidTransform.from_matrix(
            np.asarray(self._robot.forward_with_joint_poses(joints))
        )

    def kinematic_singularity(self, joints: Joints) -> Optional[str]:
        """Return the singularity kind, currently ``"A"``, or ``None``."""
        return self._robot.kinematic_singularity(joints)

    def collides(self, joints: Joints) -> bool:
        """Return whether this joint configuration collides or violates safety."""
        return bool(self._robot.collides(joints))

    def collision_details(self, joints: Joints) -> list[tuple[int, int]]:
        """Return collision pairs reported by rs-opw-kinematics."""
        return [tuple(pair) for pair in self._robot.collision_details(joints)]

    def near(
        self,
        joints: Joints,
        safety: SafetyDistances,
    ) -> list[tuple[int, int]]:
        """Run collision/proximity checks with an override safety configuration."""
        return [tuple(pair) for pair in self._robot.near(joints, safety)]

    def non_colliding_offsets(
        self,
        joints: Joints,
        from_limits: Joints,
        to_limits: Joints,
    ) -> list[Joints]:
        """Return single-joint limit offsets that remain collision free."""
        solutions = self._robot.non_colliding_offsets(joints, from_limits, to_limits)
        return [cast(Joints, tuple(solution)) for solution in solutions]

    def positioned_robot(self, joints: Joints) -> PositionedRobot:
        """Return current poses for joint, tool, and environment collision shapes."""
        joint_matrices, tool_matrix, environment_matrices = self._robot.positioned_robot(
            joints
        )
        return PositionedRobot(
            joints=RigidTransform.from_matrix(np.asarray(joint_matrices)),
            tool=None
            if tool_matrix is None
            else RigidTransform.from_matrix(tool_matrix),
            environment=tuple(
                RigidTransform.from_matrix(matrix) for matrix in environment_matrices
            ),
        )

    def visualize(
        self,
        initial_joints: Joints,
        tcp_box: TcpBox,
        safety: Optional[SafetyDistances] = None,
    ) -> VisualizationHandle:
        """Open the visualization window and return a control handle."""
        if safety is None:
            handle = _visualize_robot_internal(self._robot, initial_joints, tcp_box)
        else:
            handle = _visualize_robot_with_safety_internal(
                self._robot,
                initial_joints,
                tcp_box,
                safety,
            )
        return VisualizationHandle._from_internal(handle)


class VisualizationHandle:
    """Control handle for a non-blocking visualization window."""

    def __init__(self, *_: object, **__: object) -> None:
        raise TypeError("VisualizationHandle is returned by visualize_robot")

    @classmethod
    def _from_internal(
        cls,
        handle: _VisualizationHandleInternal,
    ) -> VisualizationHandle:
        instance = cls.__new__(cls)
        instance._handle = handle
        return instance

    @property
    def is_running(self) -> bool:
        """Whether the visualization thread is still running."""
        return bool(self._handle.is_running)

    def set_joints(self, joints: Joints) -> None:
        """Set the displayed robot joint angles."""
        self._handle.set_joints(joints)

    def set_position(self, joints: Joints) -> None:
        """Alias for :meth:`set_joints`."""
        self.set_joints(joints)

    def play_path(self, path: Iterable[PathStep], interval: float = 0.05) -> None:
        """
        Play a planned path in the visualization window.

        ``path`` may contain raw joint tuples or ``AnnotatedJoints`` values as
        returned by :meth:`CartesianPlanner.plan`.
        """
        if interval < 0.0:
            raise ValueError("interval must be non-negative")

        for step in path:
            if not self.is_running:
                return
            self.set_joints(_path_step_joints(step))
            if interval > 0.0:
                time.sleep(interval)

    def set_pose(
        self,
        pose: RigidTransform,
        previous_position: Optional[Joints] = None,
    ) -> Joints:
        """
        Resolve ``pose`` with IK, update the window, and return selected joints.

        If ``previous_position`` is provided it is used to pick the closest
        continuing IK solution. If no collision-free solution exists, this
        raises ``ValueError`` and leaves the window running.
        """
        solution = self._handle.set_pose(
            pose.as_matrix(),
            previous_position,
        )
        return cast(Joints, tuple(solution))

    def close(self) -> None:
        """Close the visualization window."""
        self._handle.close()

    def __repr__(self) -> str:
        return self._handle.__repr__()


class RRTPlanner:
    """Bidirectional RRT-Connect planner for collision-free joint relocation."""

    def __init__(
        self,
        step_size_joint_space: float = 3.0,
        max_try: int = 2000,
        debug: bool = False,
        radians: bool = False,
    ) -> None:
        """
        Initialize an RRT planner.

        :param step_size_joint_space: Joint-space extension step. Degrees by
            default; pass ``radians=True`` to provide radians.
        :param max_try: Maximum RRT expansion attempts.
        :param debug: Whether upstream planner diagnostics are printed.
        """
        self._planner = _RRTPlannerInternal(
            step_size_joint_space,
            max_try,
            debug,
            radians,
        )

    @classmethod
    def _from_internal(cls, planner: _RRTPlannerInternal) -> RRTPlanner:
        instance = cls.__new__(cls)
        instance._planner = planner
        return instance

    @property
    def max_try(self) -> int:
        """Maximum RRT expansion attempts."""
        return int(self._planner.max_try)

    @property
    def debug(self) -> bool:
        """Whether upstream planner diagnostics are printed."""
        return bool(self._planner.debug)

    def step_size_joint_space(self, radians: bool = False) -> float:
        """Return the joint-space extension step in degrees or radians."""
        return float(self._planner.step_size_joint_space(radians))

    def plan_rrt(
        self,
        robot: KinematicsWithShape,
        start: Joints,
        goal: Joints,
    ) -> list[Joints]:
        """Plan a collision-free joint-space path from ``start`` to ``goal``."""
        path = self._planner.plan_rrt(robot._robot, start, goal)
        return [cast(Joints, tuple(step)) for step in path]

    def __repr__(self) -> str:
        return self._planner.__repr__()


class CartesianPlanner:
    """Collision-aware Cartesian stroke planner."""

    def __init__(
        self,
        check_step_m: float = 0.02,
        check_step_rad: float = 3.0,
        max_transition_cost: float = 3.0,
        transition_coefficients: Optional[Joints] = None,
        linear_recursion_depth: int = 8,
        rrt: Optional[RRTPlanner] = None,
        include_linear_interpolation: bool = True,
        debug: bool = False,
        radians: bool = False,
    ) -> None:
        """
        Initialize Cartesian stroke planning settings.

        Angle-like parameters are degrees by default; pass ``radians=True`` to
        provide radians.
        """
        self._planner = _CartesianPlannerInternal(
            check_step_m,
            check_step_rad,
            max_transition_cost,
            transition_coefficients,
            linear_recursion_depth,
            None if rrt is None else rrt._planner,
            include_linear_interpolation,
            debug,
            radians,
        )

    @property
    def check_step_m(self) -> float:
        """Translation interpolation check step in meters."""
        return float(self._planner.check_step_m)

    @property
    def transition_coefficients(self) -> Joints:
        """Joint transition cost weights."""
        return cast(Joints, tuple(self._planner.transition_coefficients))

    @property
    def linear_recursion_depth(self) -> int:
        """Maximum recursive split depth for Cartesian transitions."""
        return int(self._planner.linear_recursion_depth)

    @property
    def rrt(self) -> RRTPlanner:
        """RRT planner used for non-Cartesian fallback segments."""
        return RRTPlanner._from_internal(self._planner.rrt)

    @property
    def include_linear_interpolation(self) -> bool:
        """Whether linear interpolation steps are included in the output."""
        return bool(self._planner.include_linear_interpolation)

    @property
    def debug(self) -> bool:
        """Whether upstream planner diagnostics are printed."""
        return bool(self._planner.debug)

    def check_step_rad(self, radians: bool = False) -> float:
        """Return the angular interpolation check step in degrees or radians."""
        return float(self._planner.check_step_rad(radians))

    def max_transition_cost(self, radians: bool = False) -> float:
        """Return the maximum weighted joint transition cost."""
        return float(self._planner.max_transition_cost(radians))

    def plan(
        self,
        robot: KinematicsWithShape,
        start: Joints,
        land: Optional[RigidTransform],
        steps: Sequence[RigidTransform],
        park: Optional[RigidTransform],
    ) -> list[AnnotatedJoints]:
        """
        Plan a collision-free Cartesian path with annotated joint steps.

        If ``land`` is ``None``, planning goes directly into the first
        Cartesian stroke pose. If ``park`` is ``None``, the returned path stops
        at the final stroke pose.
        """
        step_matrices = [step.as_matrix() for step in steps]
        if land is None:
            if not step_matrices:
                raise ValueError("land=None requires at least one Cartesian step")
            land_matrix = step_matrices[0]
            trim_land = True
        else:
            land_matrix = land.as_matrix()
            trim_land = False

        if park is None:
            park_matrix = step_matrices[-1] if step_matrices else land_matrix
            trim_park = True
        else:
            park_matrix = park.as_matrix()
            trim_park = False

        path = list(
            self._planner.plan(
                robot._robot,
                start,
                land_matrix,
                step_matrices,
                park_matrix,
            )
        )
        if trim_land:
            path = _without_first_land_marker(path)
        if trim_park:
            path = _without_final_park(path)
        return path

    def __repr__(self) -> str:
        return self._planner.__repr__()


def visualize_robot(
    robot: KinematicsWithShape,
    initial_joints: Joints,
    tcp_box: TcpBox,
) -> VisualizationHandle:
    """Open the visualization window and return a control handle."""
    return VisualizationHandle._from_internal(
        _visualize_robot_internal(robot._robot, initial_joints, tcp_box)
    )


def visualize_robot_with_safety(
    robot: KinematicsWithShape,
    initial_joints: Joints,
    tcp_box: TcpBox,
    safety: SafetyDistances,
) -> VisualizationHandle:
    """Open the visualization window with an override safety configuration."""
    return VisualizationHandle._from_internal(
        _visualize_robot_with_safety_internal(
            robot._robot,
            initial_joints,
            tcp_box,
            safety,
        )
    )


def _resolve_tool(
    *,
    tool: Optional[RigidTransform],
    ee_transform: Optional[RigidTransform],
) -> Optional[RigidTransform]:
    if tool is not None and ee_transform is not None:
        raise ValueError("pass either tool or ee_transform, not both")
    return tool if tool is not None else ee_transform


def _mesh_internal(mesh: Mesh) -> _MeshInternal:
    return mesh._mesh


def _path_step_joints(step: PathStep) -> Joints:
    joints = step.joints if isinstance(step, AnnotatedJoints) else step
    return cast(Joints, tuple(joints))


def _without_first_land_marker(path: list[AnnotatedJoints]) -> list[AnnotatedJoints]:
    for index, step in enumerate(path):
        if step.has_flag(PATH_FLAG_LAND):
            return path[:index] + path[index + 1 :]
    return path


def _without_final_park(path: list[AnnotatedJoints]) -> list[AnnotatedJoints]:
    if path and path[-1].has_flag(PATH_FLAG_PARK):
        return path[:-1]
    return path


__all__ = [
    "AnnotatedJoints",
    "BY_CONSTRAINTS",
    "BY_PREV",
    "CHECK_MODE_ALL",
    "CHECK_MODE_FIRST_COLLISION_ONLY",
    "CHECK_MODE_NO_CHECK",
    "CONSTRAINT_CENTERED",
    "Constraints",
    "CartesianPlanner",
    "DEFAULT_TRANSITION_COSTS",
    "ENV_START_IDX",
    "J1",
    "J2",
    "J3",
    "J4",
    "J5",
    "J6",
    "J_BASE",
    "J_TOOL",
    "Joints",
    "KinematicsWithShape",
    "KinematicModel",
    "Mesh",
    "NEVER_COLLIDES",
    "PATH_FLAG_ALTERED",
    "PATH_FLAG_BACKWARDS",
    "PATH_FLAG_CARTESIAN",
    "PATH_FLAG_DEBUG",
    "PATH_FLAG_FORWARDS",
    "PATH_FLAG_LAND",
    "PATH_FLAG_LANDING",
    "PATH_FLAG_LIN_INTERP",
    "PATH_FLAG_NONE",
    "PATH_FLAG_ONBOARDING",
    "PATH_FLAG_ORIGINAL",
    "PATH_FLAG_PARK",
    "PATH_FLAG_PARKING",
    "PATH_FLAG_TRACE",
    "PathStep",
    "PathInput",
    "Parallelogram",
    "PositionedRobot",
    "RRTPlanner",
    "RigidTransform",
    "Robot",
    "SafetyDistances",
    "TOUCH_ONLY",
    "TcpBox",
    "VisualizationHandle",
    "visualize_robot",
    "visualize_robot_with_safety",
]
