"""
spherical-wrist: Python bindings for spherical-wrist industrial robot kinematics.

The initial API mirrors the introductory py-opw-kinematics workflow while using
the local rs-opw-kinematics crate internally.
"""

from __future__ import annotations

from typing import Optional, Tuple, cast

from scipy.spatial.transform import RigidTransform

from ._internal import KinematicModel
from ._internal import Parallelogram
from ._internal import Robot as _RobotInternal

Joints = Tuple[float, float, float, float, float, float]


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
    ) -> None:
        """
        Initialize robot kinematics.

        :param kinematic_model: OPW kinematic model.
        :param degrees: Whether joint angles are in degrees.
        :param tool: Optional flange-to-TCP transform.
        :param base: Optional world-to-robot-base transform.
        :param frame: Optional working-frame transform.
        :param parallelogram: Optional parallelogram coupling configuration.
        """
        self._robot = _RobotInternal(
            kinematic_model,
            degrees,
            None if tool is None else tool.as_matrix(),
            None if base is None else base.as_matrix(),
            None if frame is None else frame.as_matrix(),
            parallelogram,
        )
        self._degrees = degrees
        self._kinematic_model = kinematic_model
        self._tool = tool
        self._base = base
        self._frame = frame
        self._parallelogram = parallelogram

    @property
    def degrees(self) -> bool:
        """Whether joint angles are in degrees."""
        return self._degrees

    @property
    def parallelogram(self) -> Optional[Parallelogram]:
        """Parallelogram coupling configured for this robot, if any."""
        return self._parallelogram

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


def _resolve_tool(
    *,
    tool: Optional[RigidTransform],
    ee_transform: Optional[RigidTransform],
) -> Optional[RigidTransform]:
    if tool is not None and ee_transform is not None:
        raise ValueError("pass either tool or ee_transform, not both")
    return tool if tool is not None else ee_transform


__all__ = [
    "Joints",
    "KinematicModel",
    "Parallelogram",
    "RigidTransform",
    "Robot",
]
