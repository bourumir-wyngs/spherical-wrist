import numpy as np

from spherical_wrist import Frame, Jacobian, KinematicModel, Robot
from scipy.spatial.transform import RigidTransform, Rotation


def test_jacobian_matrix_velocities_and_torques_use_radian_convention() -> None:
    robot = Robot(_model(), degrees=False)
    joints = tuple(np.deg2rad((10.0, 20.0, -70.0, 30.0, 20.0, 10.0)))
    jacobian = Jacobian(robot, joints)

    matrix = jacobian.matrix(radians=True)
    twist = np.array([0.2, -0.1, 0.05, 0.01, -0.02, 0.03])
    wrench = np.array([0.0, 0.0, 25.0, 0.0, 0.0, 2.0])

    rates = np.asarray(jacobian.velocities_from_vector(twist, radians=True))
    fixed_rates = np.asarray(jacobian.velocities_fixed(0.2, -0.1, 0.05, radians=True))
    split_rates = np.asarray(
        jacobian.velocities(twist[:3], twist[3:], radians=True)
    )
    efforts = np.asarray(jacobian.torques_from_vector(wrench, radians=True))
    split_efforts = np.asarray(jacobian.torques(wrench[:3], wrench[3:], radians=True))

    assert matrix.shape == (6, 6)
    assert np.isfinite(matrix).all()
    assert np.isfinite(rates).all()
    assert np.isfinite(fixed_rates).all()
    assert np.allclose(split_rates, rates)
    assert np.allclose(split_efforts, efforts)
    assert np.allclose(efforts, matrix.T @ wrench)


def test_degree_robot_jacobian_defaults_to_degree_joint_units() -> None:
    joints_degrees = (10.0, 20.0, -70.0, 30.0, 20.0, 10.0)
    joints_radians = tuple(np.deg2rad(joints_degrees))
    twist = np.array([0.2, -0.1, 0.05, 0.01, -0.02, 0.03])
    wrench = np.array([0.0, 0.0, 25.0, 0.0, 0.0, 2.0])

    degree_jacobian = Jacobian(Robot(_model(), degrees=True), joints_degrees)
    radian_jacobian = Jacobian(Robot(_model(), degrees=False), joints_radians)

    degree_matrix = degree_jacobian.matrix()
    raw_matrix = radian_jacobian.matrix(radians=True)
    degree_rates = np.asarray(degree_jacobian.velocities_from_vector(twist))
    raw_rates = np.asarray(radian_jacobian.velocities_from_vector(twist, radians=True))
    degree_efforts = np.asarray(degree_jacobian.torques_from_vector(wrench))
    raw_efforts = np.asarray(degree_jacobian.torques_from_vector(wrench, radians=True))

    assert np.allclose(degree_matrix, raw_matrix * np.pi / 180.0)
    assert np.allclose(degree_rates, np.rad2deg(raw_rates))
    assert np.allclose(degree_matrix @ degree_rates, raw_matrix @ raw_rates)
    assert np.allclose(degree_efforts, raw_efforts * np.pi / 180.0)


def test_jacobian_accounts_for_constructor_base_tool_and_frame() -> None:
    joints = tuple(np.deg2rad((10.0, 20.0, -70.0, 30.0, 20.0, 10.0)))
    epsilon = 1e-6
    base = RigidTransform.from_components(
        rotation=Rotation.from_euler("zyx", [15.0, -5.0, 10.0], degrees=True),
        translation=[100.0, 200.0, 300.0],
    )
    tool = RigidTransform.from_components(
        rotation=Rotation.from_euler("xyz", [10.0, -30.0, 20.0], degrees=True),
        translation=[100.0, 20.0, -30.0],
    )
    original_tie_points = np.array(
        [
            [0.0, 0.0, 0.0],
            [100.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
        ],
        dtype=np.float64,
    )
    frame_rotation = Rotation.from_euler("xyz", [-20.0, 5.0, 15.0], degrees=True)
    frame_translation = np.array([-40.0, 50.0, 60.0])
    frame_scale = 1.03
    target_tie_points = frame_translation + frame_rotation.apply(
        original_tie_points * frame_scale
    )
    frame = Frame.from_tie(original_tie_points, target_tie_points)
    robot = Robot(_model(), degrees=False, base=base, tool=tool, frame=frame)

    matrix = Jacobian(robot, joints, epsilon=epsilon).matrix(radians=True)
    expected = _finite_difference_jacobian(robot, joints, epsilon)

    assert np.allclose(matrix, expected, atol=1e-5)


def _finite_difference_jacobian(
    robot: Robot,
    joints: tuple[float, float, float, float, float, float],
    epsilon: float,
) -> np.ndarray:
    current_pose = robot.forward(joints)
    current_translation = np.asarray(current_pose.translation)
    current_rotation = current_pose.rotation
    matrix = np.zeros((6, 6))

    for index in range(6):
        perturbed = list(joints)
        perturbed[index] += epsilon
        perturbed_pose = robot.forward(tuple(perturbed))
        delta_translation = np.asarray(perturbed_pose.translation) - current_translation
        delta_rotation = perturbed_pose.rotation * current_rotation.inv()

        matrix[:3, index] = delta_translation / epsilon
        matrix[3:, index] = delta_rotation.as_rotvec() / epsilon

    return matrix


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
