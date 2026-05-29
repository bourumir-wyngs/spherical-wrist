from spherical_wrist import BY_CONSTRAINTS, BY_PREV, Constraints, KinematicModel, Robot
from scipy.spatial.transform import RigidTransform, Rotation
import numpy as np


def test_constraints_present_limits_in_degrees_or_radians() -> None:
    from_limits = (-10.0, -90.0, -45.0, 0.0, -180.0, 0.0)
    to_limits = (20.0, 90.0, 45.0, 180.0, 180.0, 360.0)
    constraints = Constraints(
        from_limits,
        to_limits,
        sorting_weight=BY_CONSTRAINTS,
    )

    assert constraints.sorting_weight == BY_CONSTRAINTS
    assert np.allclose(constraints.from_limits(), from_limits)
    assert np.allclose(constraints.to_limits(), to_limits)
    assert np.allclose(constraints.from_limits(radians=True), np.deg2rad(from_limits))
    assert np.allclose(constraints.to_limits(radians=True), np.deg2rad(to_limits))
    lower, upper = constraints.limits()
    assert np.allclose(lower, from_limits)
    assert np.allclose(upper, to_limits)
    assert constraints.compliant((0.0, 0.0, 0.0, 90.0, 0.0, 180.0))
    assert constraints.compliant(
        tuple(np.deg2rad((0.0, 0.0, 0.0, 90.0, 0.0, 180.0))),
        radians=True,
    )
    assert not constraints.compliant((30.0, 0.0, 0.0, 90.0, 0.0, 180.0))


def test_robot_uses_internal_constraints_for_inverse_filtering() -> None:
    model = _model()
    joints = (10.0, 20.0, -70.0, 30.0, 20.0, 10.0)
    plain_robot = Robot(model, degrees=True)
    pose = plain_robot.forward(joints, ee_transform=_tool())

    constraints = Constraints(
        (9.0, 19.0, -71.0, 29.0, 19.0, 9.0),
        (11.0, 21.0, -69.0, 31.0, 21.0, 11.0),
        sorting_weight=BY_PREV,
    )
    constrained_robot = Robot(
        model,
        degrees=True,
        tool=_tool(),
        constraints=constraints,
    )

    assert constrained_robot.constraints is constraints
    solutions = constrained_robot.inverse(pose, current_joints=joints)

    assert solutions
    assert all(constraints.compliant(solution) for solution in solutions)
    assert any(np.allclose(solution, joints, atol=1e-6) for solution in solutions)


def _model() -> KinematicModel:
    return KinematicModel(
        a1=400,
        a2=-250,
        b=0,
        c1=830,
        c2=1175,
        c3=1444,
        c4=230,
        offsets=(0, 0, 0, 0, 0, 0),
        flip_axes=(True, False, True, True, False, True),
    )


def _tool() -> RigidTransform:
    return RigidTransform.from_components(
        rotation=Rotation.from_euler("xyz", [10, -30, 20], degrees=True),
        translation=[100, 20, -30],
    )
