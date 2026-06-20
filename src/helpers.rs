use super::*;

pub(crate) fn joint_mesh_array(meshes: Vec<Mesh>) -> PyResult<[TriMesh; 6]> {
    if meshes.len() != 6 {
        return Err(PyValueError::new_err(
            "joint_meshes must contain exactly 6 meshes",
        ));
    }

    let meshes = meshes
        .into_iter()
        .map(|mesh| mesh.local_trimesh())
        .collect::<Vec<_>>();
    match meshes.try_into() {
        Ok(meshes) => Ok(meshes),
        Err(_) => unreachable!("joint_meshes length is validated"),
    }
}

pub(crate) fn parse_check_mode(mode: &str) -> PyResult<CheckMode> {
    let normalized = mode.trim().to_ascii_lowercase().replace('-', "_");
    match normalized.as_str() {
        "first" | "first_collision_only" => Ok(CheckMode::FirstCollisionOnly),
        "all" | "all_collisions" | "all_collsions" => Ok(CheckMode::AllCollsions),
        "none" | "no_check" | "off" => Ok(CheckMode::NoCheck),
        _ => Err(PyValueError::new_err(
            "mode must be 'all', 'first_collision_only', or 'no_check'",
        )),
    }
}

pub(crate) fn check_mode_name(mode: CheckMode) -> &'static str {
    match mode {
        CheckMode::FirstCollisionOnly => "first_collision_only",
        CheckMode::AllCollsions => "all",
        CheckMode::NoCheck => "no_check",
    }
}

pub(crate) fn validate_collision_index(index: usize) -> PyResult<()> {
    if index > u16::MAX as usize {
        return Err(PyValueError::new_err(
            "collision indices must fit in an unsigned 16-bit value",
        ));
    }
    Ok(())
}

pub(crate) fn validate_safety_distance(value: f64, name: &str) -> PyResult<f32> {
    let value = validate_f32(value, name)?;
    if value > NEVER_COLLIDES && value < TOUCH_ONLY {
        return Err(PyValueError::new_err(format!(
            "{name} must be non-negative or NEVER_COLLIDES"
        )));
    }
    Ok(value)
}

pub(crate) fn clone_shape_robot(robot: &RsKinematicsWithShape) -> RsKinematicsWithShape {
    RsKinematicsWithShape {
        kinematics: Arc::clone(&robot.kinematics),
        body: RobotBody {
            joint_meshes: robot.body.joint_meshes.clone(),
            tool: robot.body.tool.clone(),
            base: robot.body.base.as_ref().map(|base| BaseBody {
                mesh: base.mesh.clone(),
                base_pose: base.base_pose,
            }),
            collision_environment: robot
                .body
                .collision_environment
                .iter()
                .map(|body| RsCollisionBody {
                    mesh: body.mesh.clone(),
                    pose: body.pose,
                })
                .collect(),
            safety: robot.body.safety.clone(),
        },
    }
}

pub(crate) fn joints_to_visual_degrees(
    joints: [f64; 6],
    input_degrees: bool,
) -> PyResult<[f32; 6]> {
    validate_joints(&joints)?;
    let mut converted = [0.0; 6];
    for (target, value) in converted.iter_mut().zip(joints) {
        let degrees = if input_degrees {
            value
        } else {
            value.to_degrees()
        };
        *target = validate_f32(degrees, "initial_joints")?;
    }
    Ok(converted)
}

pub(crate) fn tcp_box_to_ranges(tcp_box: [(f64, f64); 3]) -> PyResult<[RangeInclusive<f64>; 3]> {
    Ok([
        tcp_range(tcp_box[0], "tcp_box[0]")?,
        tcp_range(tcp_box[1], "tcp_box[1]")?,
        tcp_range(tcp_box[2], "tcp_box[2]")?,
    ])
}

pub(crate) fn tcp_range((from, to): (f64, f64), name: &str) -> PyResult<RangeInclusive<f64>> {
    if !from.is_finite() || !to.is_finite() {
        return Err(PyValueError::new_err(format!(
            "{name} values must be finite"
        )));
    }
    if from > to {
        return Err(PyValueError::new_err(format!(
            "{name} lower bound must not exceed upper bound"
        )));
    }
    Ok(from..=to)
}

pub(crate) fn validate_jacobian_epsilon(epsilon: f64) -> PyResult<f64> {
    if !epsilon.is_finite() || epsilon <= 0.0 {
        return Err(PyValueError::new_err("epsilon must be positive"));
    }
    Ok(epsilon)
}

pub(crate) fn validate_vector6(values: &[f64; 6], name: &str) -> PyResult<()> {
    if values.iter().any(|value| !value.is_finite()) {
        return Err(PyValueError::new_err(format!(
            "{name} values must be finite"
        )));
    }
    Ok(())
}

pub(crate) fn validate_vector3_values(values: &[f64; 3], name: &str) -> PyResult<()> {
    if values.iter().any(|value| !value.is_finite()) {
        return Err(PyValueError::new_err(format!(
            "{name} values must be finite"
        )));
    }
    Ok(())
}

pub(crate) fn vector3_to_dvec3(values: [f64; 3], name: &str) -> PyResult<DVec3> {
    validate_vector3_values(&values, name)?;
    Ok(DVec3::new(values[0], values[1], values[2]))
}

pub(crate) fn tie_points_to_dvec3(points: [[f64; 3]; 3], name: &str) -> PyResult<[DVec3; 3]> {
    let [p1, p2, p3] = points;
    Ok([
        vector3_to_dvec3(p1, &format!("{name}[0]"))?,
        vector3_to_dvec3(p2, &format!("{name}[1]"))?,
        vector3_to_dvec3(p3, &format!("{name}[2]"))?,
    ])
}

pub(crate) fn convert_jacobian_columns_to_output_units(
    rows: &mut [[f64; 6]; 6],
    degrees: bool,
    radians: bool,
) {
    if degrees && !radians {
        for value in rows.iter_mut().flatten() {
            *value *= RADIANS_PER_DEGREE;
        }
    }
}

pub(crate) fn joint_rates_from_internal(
    mut rates: [f64; 6],
    degrees: bool,
    radians: bool,
) -> [f64; 6] {
    if degrees && !radians {
        rates.iter_mut().for_each(|rate| *rate = rate.to_degrees());
    }
    rates
}

pub(crate) fn generalized_efforts_from_internal(
    mut efforts: [f64; 6],
    degrees: bool,
    radians: bool,
) -> [f64; 6] {
    if degrees && !radians {
        efforts
            .iter_mut()
            .for_each(|effort| *effort *= RADIANS_PER_DEGREE);
    }
    efforts
}

pub(crate) fn validate_joints(joints: &Joints) -> PyResult<()> {
    if joints.iter().any(|joint| !joint.is_finite()) {
        return Err(PyValueError::new_err("joint values must be finite"));
    }
    Ok(())
}

pub(crate) fn validate_previous_joints(joints: &Joints) -> PyResult<()> {
    if joints[0].is_nan() {
        if joints[1..].iter().any(|joint| !joint.is_finite()) {
            return Err(PyValueError::new_err(
                "CONSTRAINT_CENTERED previous joints marker must have finite values after J1",
            ));
        }
        return Ok(());
    }

    validate_joints(joints)
}

pub(crate) fn joints_to_internal(mut joints: [f64; 6], degrees: bool) -> PyResult<[f64; 6]> {
    validate_joints(&joints)?;
    if degrees {
        joints
            .iter_mut()
            .for_each(|joint| *joint = joint.to_radians());
    }
    Ok(joints)
}

pub(crate) fn previous_joints_to_internal(
    mut joints: [f64; 6],
    degrees: bool,
) -> PyResult<[f64; 6]> {
    validate_previous_joints(&joints)?;
    if degrees {
        joints
            .iter_mut()
            .for_each(|joint| *joint = joint.to_radians());
    }
    Ok(joints)
}

pub(crate) fn angle_to_internal(angle: f64, degrees: bool, name: &str) -> PyResult<f64> {
    if !angle.is_finite() {
        return Err(PyValueError::new_err(format!("{name} must be finite")));
    }
    Ok(if degrees { angle.to_radians() } else { angle })
}

pub(crate) fn positive_angle_to_internal(angle: f64, radians: bool, name: &str) -> PyResult<f64> {
    if !angle.is_finite() || angle <= 0.0 {
        return Err(PyValueError::new_err(format!("{name} must be positive")));
    }
    Ok(if radians { angle } else { angle.to_radians() })
}

pub(crate) fn angle_from_internal(angle: f64, radians: bool) -> f64 {
    if radians { angle } else { angle.to_degrees() }
}

pub(crate) fn solutions_from_internal(
    mut solutions: Vec<[f64; 6]>,
    degrees: bool,
) -> Vec<[f64; 6]> {
    if degrees {
        for solution in &mut solutions {
            solution
                .iter_mut()
                .for_each(|joint| *joint = joint.to_degrees());
        }
    }
    solutions
}

pub(crate) fn annotated_joints_from_internal(
    joints: Vec<RsAnnotatedJoints>,
    degrees: bool,
) -> Vec<AnnotatedJoints> {
    joints
        .into_iter()
        .map(|step| AnnotatedJoints {
            joints: angles_from_radians(step.joints, !degrees),
            flags: step.flags.bits(),
            move_into: match step.move_into {
                RsMoveKind::Joint => "joint",
                RsMoveKind::Cartesian => "cartesian",
            }
            .to_string(),
        })
        .collect()
}

pub(crate) fn angles_to_radians(mut values: [f64; 6], radians: bool) -> PyResult<[f64; 6]> {
    validate_joints(&values)?;
    if !radians {
        values = values.map(f64::to_radians);
    }
    Ok(values)
}

pub(crate) fn angles_from_radians(values: [f64; 6], radians: bool) -> [f64; 6] {
    if radians {
        values
    } else {
        values.map(f64::to_degrees)
    }
}

pub(crate) fn matrix_to_pose(matrix: [[f64; 4]; 4]) -> PyResult<Pose> {
    let rotation = validate_pose_matrix(matrix, "pose matrix")?;
    let translation = DVec3::new(matrix[0][3], matrix[1][3], matrix[2][3]);

    Pose::try_from_parts(translation, DQuat::from_mat3(&rotation))
        .map_err(|error| PyValueError::new_err(format!("invalid pose matrix: {error}")))
}

pub(crate) fn matrix_to_frame_transform(matrix: [[f64; 4]; 4]) -> PyResult<RsFrameTransform> {
    if matrix.into_iter().flatten().any(|value| !value.is_finite()) {
        return Err(PyValueError::new_err("frame matrix values must be finite"));
    }

    let expected_bottom = [0.0, 0.0, 0.0, 1.0];
    if matrix[3]
        .iter()
        .zip(expected_bottom.iter())
        .any(|(actual, expected)| (actual - expected).abs() > 1.0e-9)
    {
        return Err(PyValueError::new_err(
            "frame matrix bottom row must be [0, 0, 0, 1]",
        ));
    }

    let c0 = DVec3::new(matrix[0][0], matrix[1][0], matrix[2][0]);
    let c1 = DVec3::new(matrix[0][1], matrix[1][1], matrix[2][1]);
    let c2 = DVec3::new(matrix[0][2], matrix[1][2], matrix[2][2]);
    let lengths = [c0.length(), c1.length(), c2.length()];
    if lengths.iter().any(|length| *length <= 1.0e-12) {
        return Err(PyValueError::new_err(
            "frame matrix scale must be finite and positive",
        ));
    }

    let scale = lengths.iter().sum::<f64>() / 3.0;
    let tolerance = scale.max(1.0) * 1.0e-8;
    if lengths
        .iter()
        .any(|length| (length - scale).abs() > tolerance)
    {
        return Err(PyValueError::new_err(
            "frame matrix must use one uniform scale value",
        ));
    }

    let r0 = c0 / scale;
    let r1 = c1 / scale;
    let r2 = c2 / scale;
    if r0.dot(r1).abs() > 1.0e-8 || r0.dot(r2).abs() > 1.0e-8 || r1.dot(r2).abs() > 1.0e-8 {
        return Err(PyValueError::new_err(
            "frame matrix scaled rotation axes must be orthogonal",
        ));
    }

    let rotation = DMat3::from_cols(r0, r1, r2);
    if rotation.determinant() <= 0.0 {
        return Err(PyValueError::new_err(
            "frame matrix must not contain reflection",
        ));
    }
    let translation = DVec3::new(matrix[0][3], matrix[1][3], matrix[2][3]);

    RsFrameTransform::try_from_parts(translation, DQuat::from_mat3(&rotation), scale)
        .map_err(|error| PyValueError::new_err(format!("invalid frame matrix: {error}")))
}

pub(crate) fn matrix_to_parry_pose(matrix: [[f64; 4]; 4]) -> PyResult<ParryPose> {
    validate_pose_matrix(matrix, "pose matrix")?;

    let rotation = Mat3::from_cols(
        Vec3::new(
            validate_f32(matrix[0][0], "pose matrix")?,
            validate_f32(matrix[1][0], "pose matrix")?,
            validate_f32(matrix[2][0], "pose matrix")?,
        ),
        Vec3::new(
            validate_f32(matrix[0][1], "pose matrix")?,
            validate_f32(matrix[1][1], "pose matrix")?,
            validate_f32(matrix[2][1], "pose matrix")?,
        ),
        Vec3::new(
            validate_f32(matrix[0][2], "pose matrix")?,
            validate_f32(matrix[1][2], "pose matrix")?,
            validate_f32(matrix[2][2], "pose matrix")?,
        ),
    );
    let translation = Vec3::new(
        validate_f32(matrix[0][3], "pose matrix")?,
        validate_f32(matrix[1][3], "pose matrix")?,
        validate_f32(matrix[2][3], "pose matrix")?,
    );

    for value in matrix[3] {
        validate_f32(value, "pose matrix")?;
    }

    let pose = Pose32::try_from_parts(translation, glam::Quat::from_mat3(&rotation))
        .map_err(|error| PyValueError::new_err(format!("invalid pose matrix: {error}")))?;

    Ok(ParryPose::from_parts(pose.translation, pose.rotation))
}

pub(crate) fn validate_pose_matrix(matrix: [[f64; 4]; 4], name: &str) -> PyResult<DMat3> {
    if matrix.iter().flatten().any(|value| !value.is_finite()) {
        return Err(PyValueError::new_err(format!(
            "{name} values must be finite"
        )));
    }

    let expected_bottom = [0.0, 0.0, 0.0, 1.0];
    if matrix[3]
        .iter()
        .zip(expected_bottom.iter())
        .any(|(actual, expected)| (actual - expected).abs() > 1.0e-9)
    {
        return Err(PyValueError::new_err(format!(
            "{name} bottom row must be [0, 0, 0, 1]"
        )));
    }

    let c0 = DVec3::new(matrix[0][0], matrix[1][0], matrix[2][0]);
    let c1 = DVec3::new(matrix[0][1], matrix[1][1], matrix[2][1]);
    let c2 = DVec3::new(matrix[0][2], matrix[1][2], matrix[2][2]);
    validate_unit_rotation_axes(c0, c1, c2, name)?;

    Ok(DMat3::from_cols(c0, c1, c2))
}

pub(crate) fn validate_unit_rotation_axes(
    c0: DVec3,
    c1: DVec3,
    c2: DVec3,
    name: &str,
) -> PyResult<()> {
    let lengths = [c0.length(), c1.length(), c2.length()];
    if lengths
        .iter()
        .any(|length| *length <= 1.0e-12 || (length - 1.0).abs() > 1.0e-8)
    {
        return Err(PyValueError::new_err(format!(
            "{name} rotation axes must be unit length"
        )));
    }

    if c0.dot(c1).abs() > 1.0e-8 || c0.dot(c2).abs() > 1.0e-8 || c1.dot(c2).abs() > 1.0e-8 {
        return Err(PyValueError::new_err(format!(
            "{name} rotation axes must be orthogonal"
        )));
    }

    let determinant = DMat3::from_cols(c0, c1, c2).determinant();
    if determinant <= 0.0 {
        return Err(PyValueError::new_err(format!(
            "{name} rotation must not contain reflection"
        )));
    }

    Ok(())
}

pub(crate) fn validate_f32(value: f64, name: &str) -> PyResult<f32> {
    let value = value as f32;
    if !value.is_finite() {
        return Err(PyValueError::new_err(format!(
            "{name} values must be finite f32 values"
        )));
    }
    Ok(value)
}

pub(crate) fn validate_positive_f32(value: f64, name: &str) -> PyResult<f32> {
    let value = validate_f32(value, name)?;
    if value <= 0.0 {
        return Err(PyValueError::new_err(format!("{name} must be positive")));
    }
    Ok(value)
}

pub(crate) fn validate_nonnegative_f32(value: f64, name: &str) -> PyResult<f32> {
    let value = validate_f32(value, name)?;
    if value < 0.0 {
        return Err(PyValueError::new_err(format!(
            "{name} must be non-negative"
        )));
    }
    Ok(value)
}

pub(crate) fn validate_linear_axis(axis: usize) -> PyResult<()> {
    if axis >= 3 {
        return Err(PyValueError::new_err(
            "linear axis index must be in the range 0..3",
        ));
    }
    Ok(())
}

pub(crate) fn translation_pose(translation: [f64; 3]) -> PyResult<Pose> {
    if translation.iter().any(|value| !value.is_finite()) {
        return Err(PyValueError::new_err("translation values must be finite"));
    }
    Pose::try_from_translation(DVec3::new(translation[0], translation[1], translation[2]))
        .map_err(|error| PyValueError::new_err(error.to_string()))
}

pub(crate) fn linear_axis_pose(axis: usize, distance: f64) -> Pose {
    match axis {
        0 => Pose::from_translation(DVec3::new(distance, 0.0, 0.0)),
        1 => Pose::from_translation(DVec3::new(0.0, distance, 0.0)),
        2 => Pose::from_translation(DVec3::new(0.0, 0.0, distance)),
        _ => unreachable!("linear axis is validated before use"),
    }
}

pub(crate) fn transpose_option<T, U, F>(value: Option<T>, f: F) -> PyResult<Option<U>>
where
    F: FnOnce(T) -> PyResult<U>,
{
    value.map(f).transpose()
}

pub(crate) fn pose_to_matrix(pose: Pose) -> [[f64; 4]; 4] {
    let rotation = DMat3::from_quat(pose.rotation);
    [
        [
            rotation.x_axis.x,
            rotation.y_axis.x,
            rotation.z_axis.x,
            pose.translation.x,
        ],
        [
            rotation.x_axis.y,
            rotation.y_axis.y,
            rotation.z_axis.y,
            pose.translation.y,
        ],
        [
            rotation.x_axis.z,
            rotation.y_axis.z,
            rotation.z_axis.z,
            pose.translation.z,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

pub(crate) fn frame_transform_to_matrix(frame: RsFrameTransform) -> [[f64; 4]; 4] {
    let rotation = DMat3::from_quat(frame.rotation);
    let x_axis = rotation.x_axis * frame.scale;
    let y_axis = rotation.y_axis * frame.scale;
    let z_axis = rotation.z_axis * frame.scale;
    [
        [x_axis.x, y_axis.x, z_axis.x, frame.translation.x],
        [x_axis.y, y_axis.y, z_axis.y, frame.translation.y],
        [x_axis.z, y_axis.z, z_axis.z, frame.translation.z],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

pub(crate) fn pose32_to_matrix(pose: Pose32) -> [[f64; 4]; 4] {
    let rotation = Mat3::from_quat(pose.rotation);
    [
        [
            rotation.x_axis.x as f64,
            rotation.y_axis.x as f64,
            rotation.z_axis.x as f64,
            pose.translation.x as f64,
        ],
        [
            rotation.x_axis.y as f64,
            rotation.y_axis.y as f64,
            rotation.z_axis.y as f64,
            pose.translation.y as f64,
        ],
        [
            rotation.x_axis.z as f64,
            rotation.y_axis.z as f64,
            rotation.z_axis.z as f64,
            pose.translation.z as f64,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

pub(crate) fn array_to_tuple(values: [f64; 6]) -> (f64, f64, f64, f64, f64, f64) {
    let [a, b, c, d, e, f] = values;
    (a, b, c, d, e, f)
}
