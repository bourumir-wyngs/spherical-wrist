use super::super::*;
use super::assert_value_error;

const IDENTITY: [[f64; 4]; 4] = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

// A positive quarter-turn around Z, followed by an asymmetric translation.
const RIGID_TRANSFORM: [[f64; 4]; 4] = [
    [0.0, -1.0, 0.0, 1.25],
    [1.0, 0.0, 0.0, -2.5],
    [0.0, 0.0, 1.0, 3.75],
    [0.0, 0.0, 0.0, 1.0],
];

fn assert_matrix_close(actual: [[f64; 4]; 4], expected: [[f64; 4]; 4], tolerance: f64) {
    for row in 0..4 {
        for column in 0..4 {
            assert!(
                (actual[row][column] - expected[row][column]).abs() <= tolerance,
                "entry [{row}][{column}]: expected {}, got {}",
                expected[row][column],
                actual[row][column],
            );
        }
    }
}

fn assert_rigid_value_error(matrix: [[f64; 4]; 4], fragment: &str) {
    assert_value_error(matrix_to_pose(matrix), fragment);
    assert_value_error(matrix_to_parry_pose(matrix), fragment);
}

#[test]
fn rigid_matrix_preserves_translation_and_rotation_direction() {
    let pose = matrix_to_pose(RIGID_TRANSFORM).unwrap();

    assert_eq!(pose.translation, DVec3::new(1.25, -2.5, 3.75));
    let transformed = pose.transform_point(DVec3::new(2.0, 3.0, 4.0));
    assert!((transformed - DVec3::new(-1.75, -0.5, 7.75)).length() < 1.0e-12);
    assert_matrix_close(pose_to_matrix(pose), RIGID_TRANSFORM, 1.0e-12);
}

#[test]
fn frame_matrix_preserves_uniform_scale_without_scaling_translation() {
    for scale in [0.25, 1.0, 3.0, 1000.0] {
        let mut matrix = RIGID_TRANSFORM;
        for row in &mut matrix[..3] {
            for value in &mut row[..3] {
                *value *= scale;
            }
        }

        let frame = matrix_to_frame_transform(matrix).unwrap();
        assert_eq!(frame.scale, scale);
        assert_eq!(frame.translation, DVec3::new(1.25, -2.5, 3.75));
        let transformed = frame.transform_point(DVec3::new(2.0, 3.0, 4.0));
        let expected = DVec3::new(1.25 - 3.0 * scale, -2.5 + 2.0 * scale, 3.75 + 4.0 * scale);
        assert!((transformed - expected).length() < 1.0e-12 * scale.max(1.0));
        assert_matrix_close(frame_transform_to_matrix(frame), matrix, 1.0e-12 * scale);
    }
}

#[test]
fn parry_matrix_preserves_translation_and_rotation_direction() {
    let pose = matrix_to_parry_pose(RIGID_TRANSFORM).unwrap();

    assert_eq!(pose.translation, Vec3::new(1.25, -2.5, 3.75));
    let transformed = pose.transform_point(Vec3::new(2.0, 3.0, 4.0));
    assert!((transformed - Vec3::new(-1.75, -0.5, 7.75)).length() < 1.0e-6);
    assert_matrix_close(
        pose32_to_matrix(Pose32::from_parts(pose.translation, pose.rotation)),
        RIGID_TRANSFORM,
        1.0e-6,
    );
}

#[test]
fn matrix_converters_reject_nonfinite_values_in_every_entry() {
    for row in 0..4 {
        for column in 0..4 {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut matrix = IDENTITY;
                matrix[row][column] = value;

                assert_rigid_value_error(matrix, "pose matrix values must be finite");
                assert_value_error(
                    matrix_to_frame_transform(matrix),
                    "frame matrix values must be finite",
                );
            }
        }
    }
}

#[test]
fn bottom_row_tolerance_applies_to_all_four_entries() {
    for column in 0..4 {
        for sign in [-1.0, 1.0] {
            let mut matrix = IDENTITY;
            matrix[3][column] += sign * 0.5e-9;
            assert!(matrix_to_pose(matrix).is_ok());
            assert!(matrix_to_parry_pose(matrix).is_ok());
            assert!(matrix_to_frame_transform(matrix).is_ok());

            matrix[3][column] = IDENTITY[3][column] + sign * 2.0e-9;
            assert_rigid_value_error(matrix, "bottom row must be [0, 0, 0, 1]");
            assert_value_error(
                matrix_to_frame_transform(matrix),
                "bottom row must be [0, 0, 0, 1]",
            );
        }
    }
}

#[test]
fn matrix_converters_reject_reflections_about_each_axis() {
    for axis in 0..3 {
        let mut matrix = IDENTITY;
        matrix[axis][axis] = -1.0;

        assert_rigid_value_error(matrix, "must not contain reflection");
        assert_value_error(
            matrix_to_frame_transform(matrix),
            "must not contain reflection",
        );
    }
}

#[test]
fn matrix_converters_reject_nonorthogonal_unit_axes() {
    for (first, second) in [(0, 1), (0, 2), (1, 2)] {
        let mut matrix = IDENTITY;
        // Both columns retain unit length, so rejection must check their angle.
        matrix[first][second] = 0.6;
        matrix[second][second] = 0.8;

        assert_rigid_value_error(matrix, "axes must be orthogonal");
        assert_value_error(matrix_to_frame_transform(matrix), "axes must be orthogonal");
    }
}

#[test]
fn matrix_converters_reject_nonuniform_and_zero_scale() {
    for axis in 0..3 {
        for (scale, frame_error) in [
            (2.0, "must use one uniform scale value"),
            (0.0, "scale must be finite and positive"),
            (1.0e-13, "scale must be finite and positive"),
        ] {
            let mut matrix = IDENTITY;
            matrix[axis][axis] = scale;

            assert_rigid_value_error(matrix, "rotation axes must be unit length");
            assert_value_error(matrix_to_frame_transform(matrix), frame_error);
        }
    }
}

#[test]
fn rigid_rotation_axes_must_have_unit_length_within_tolerance() {
    for axis in 0..3 {
        for sign in [-1.0, 1.0] {
            let mut matrix = IDENTITY;
            matrix[axis][axis] += sign * 0.5e-8;
            assert!(matrix_to_pose(matrix).is_ok());
            assert!(matrix_to_parry_pose(matrix).is_ok());

            matrix[axis][axis] = 1.0 + sign * 2.0e-8;
            assert_rigid_value_error(matrix, "rotation axes must be unit length");
        }
    }

    for scale in [0.25, 2.0] {
        let mut matrix = IDENTITY;
        for (axis, row) in matrix.iter_mut().enumerate().take(3) {
            row[axis] = scale;
        }
        assert_rigid_value_error(matrix, "rotation axes must be unit length");
    }
}

#[test]
fn rotation_axis_orthogonality_tolerance_checks_each_pair() {
    for (first, second) in [(0, 1), (0, 2), (1, 2)] {
        for sign in [-1.0, 1.0] {
            let mut matrix = IDENTITY;
            matrix[first][second] = sign * 0.5e-8;
            assert!(matrix_to_pose(matrix).is_ok());
            assert!(matrix_to_parry_pose(matrix).is_ok());
            assert!(matrix_to_frame_transform(matrix).is_ok());

            matrix[first][second] = sign * 2.0e-8;
            assert_rigid_value_error(matrix, "axes must be orthogonal");
            assert_value_error(matrix_to_frame_transform(matrix), "axes must be orthogonal");
        }
    }
}

#[test]
fn frame_uniform_scale_tolerance_handles_small_and_large_scales() {
    for scale in [0.25_f64, 4.0] {
        for axis in 0..3 {
            let mut matrix = IDENTITY;
            for (diagonal, row) in matrix.iter_mut().enumerate().take(3) {
                row[diagonal] = scale;
            }

            let perturbation = scale.max(1.0) * 0.5e-8;
            matrix[axis][axis] += perturbation;
            let frame = matrix_to_frame_transform(matrix).unwrap();
            assert!((frame.scale - (scale + perturbation / 3.0)).abs() < 1.0e-15);

            matrix[axis][axis] = scale + scale.max(1.0) * 3.0e-8;
            assert_value_error(
                matrix_to_frame_transform(matrix),
                "must use one uniform scale value",
            );
        }
    }
}

#[test]
fn parry_matrix_rejects_translation_that_overflows_f32() {
    for axis in 0..3 {
        for sign in [-1.0, 1.0] {
            let mut matrix = IDENTITY;
            matrix[axis][3] = sign * f64::from(f32::MAX);
            let pose = matrix_to_parry_pose(matrix).unwrap();
            assert_eq!(pose.translation[axis], sign as f32 * f32::MAX);

            matrix[axis][3] *= 2.0;
            assert!(matrix_to_pose(matrix).is_ok());
            assert_value_error(
                matrix_to_parry_pose(matrix),
                "pose matrix values must be finite f32 values",
            );
        }
    }
}
