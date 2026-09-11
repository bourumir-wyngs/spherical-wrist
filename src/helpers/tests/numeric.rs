use super::super::*;
use super::assert_value_error;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

#[test]
fn f32_conversion_accepts_finite_boundaries_and_preserves_signed_zero() {
    for expected in [
        -f32::MAX,
        -1.5,
        -0.0,
        0.0,
        f32::from_bits(1),
        f32::MIN_POSITIVE,
        1.5,
        f32::MAX,
    ] {
        let actual = validate_f32(f64::from(expected), "distance").unwrap();
        assert_eq!(actual.to_bits(), expected.to_bits());
    }

    // Finite values may underflow when narrowing unless positivity is required.
    assert_eq!(validate_f32(f64::MIN_POSITIVE, "distance").unwrap(), 0.0);
}

#[test]
fn f32_conversion_rejects_nonfinite_values_and_overflow() {
    for value in [
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::from(f32::MAX) * 2.0,
        -f64::from(f32::MAX) * 2.0,
        f64::MAX,
        -f64::MAX,
    ] {
        assert_value_error(
            validate_f32(value, "distance"),
            "distance values must be finite f32 values",
        );
    }
}

#[test]
fn positive_f32_accepts_smallest_subnormal_normal_and_largest_values() {
    for expected in [f32::from_bits(1), f32::MIN_POSITIVE, 1.0, f32::MAX] {
        assert_eq!(
            validate_positive_f32(f64::from(expected), "radius").unwrap(),
            expected,
        );
    }
}

#[test]
fn positive_f32_rejects_zero_negative_and_values_that_underflow_to_zero() {
    for value in [
        0.0,
        -0.0,
        -1.0,
        -f64::from(f32::from_bits(1)),
        f64::MIN_POSITIVE,
        f64::from(f32::from_bits(1)) / 4.0,
    ] {
        assert_value_error(
            validate_positive_f32(value, "radius"),
            "radius must be positive",
        );
    }

    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::MAX] {
        assert_value_error(
            validate_positive_f32(value, "radius"),
            "radius values must be finite f32 values",
        );
    }
}

#[test]
fn nonnegative_f32_accepts_zero_and_positive_values() {
    for expected in [-0.0, 0.0, f32::from_bits(1), 0.25, f32::MAX] {
        assert_eq!(
            validate_nonnegative_f32(f64::from(expected), "margin")
                .unwrap()
                .to_bits(),
            expected.to_bits(),
        );
    }
}

#[test]
fn nonnegative_f32_rejects_negative_nonfinite_and_overflowing_values() {
    for value in [-f64::from(f32::from_bits(1)), -1.0, -f64::from(f32::MAX)] {
        assert_value_error(
            validate_nonnegative_f32(value, "margin"),
            "margin must be non-negative",
        );
    }

    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::MAX] {
        assert_value_error(
            validate_nonnegative_f32(value, "margin"),
            "margin values must be finite f32 values",
        );
    }
}

#[test]
fn joint_conversion_handles_degrees_and_preserves_radians() {
    let degrees = [0.0, 45.0, -90.0, 180.0, -180.0, 360.0];
    let radians = [0.0, FRAC_PI_4, -FRAC_PI_2, PI, -PI, TAU];
    assert!(validate_joints(&degrees).is_ok());
    assert!(validate_joints(&[-f64::MAX, f64::MAX, -0.0, 0.0, 1.0, -1.0]).is_ok());
    assert_eq!(joints_to_internal(radians, false).unwrap(), radians);
    assert_eq!(
        previous_joints_to_internal(radians, false).unwrap(),
        radians
    );

    for converted in [
        joints_to_internal(degrees, true).unwrap(),
        previous_joints_to_internal(degrees, true).unwrap(),
    ] {
        for (actual, expected) in converted.into_iter().zip(radians) {
            assert!((actual - expected).abs() < 1.0e-14);
        }
    }
}

#[test]
fn ordinary_joints_reject_nonfinite_values_in_every_position() {
    for position in 0..6 {
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut joints = [0.0; 6];
            joints[position] = invalid;
            assert_value_error(validate_joints(&joints), "joint values must be finite");
            for degrees in [false, true] {
                assert_value_error(
                    joints_to_internal(joints, degrees),
                    "joint values must be finite",
                );
            }
        }
    }
}

#[test]
fn previous_joints_accept_constraint_centered_and_nan_with_finite_tail() {
    for joints in [
        CONSTRAINT_CENTERED,
        [f64::NAN, 45.0, -90.0, 180.0, -180.0, 360.0],
        [f64::NAN, -f64::MAX, f64::MAX, -0.0, 0.0, 1.0],
    ] {
        assert!(validate_previous_joints(&joints).is_ok());
    }

    // The NaN marker is only valid for previous joints.
    assert_value_error(
        validate_joints(&CONSTRAINT_CENTERED),
        "joint values must be finite",
    );
}

#[test]
fn previous_joints_reject_nonfinite_values_outside_the_j1_nan_marker() {
    for position in 0..6 {
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            if position == 0 && invalid.is_nan() {
                continue;
            }
            let mut joints = [0.0; 6];
            joints[position] = invalid;
            assert_value_error(
                validate_previous_joints(&joints),
                "joint values must be finite",
            );

            for degrees in [false, true] {
                assert_value_error(
                    previous_joints_to_internal(joints, degrees),
                    "joint values must be finite",
                );
            }
        }
    }
}

#[test]
fn previous_nan_marker_does_not_bypass_validation_of_the_remaining_joints() {
    for position in 1..6 {
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut joints = CONSTRAINT_CENTERED;
            joints[position] = invalid;
            let message =
                "CONSTRAINT_CENTERED previous joints marker must have finite values after J1";
            assert_value_error(validate_previous_joints(&joints), message);
            for degrees in [false, true] {
                assert_value_error(previous_joints_to_internal(joints, degrees), message);
            }
        }
    }
}

#[test]
fn previous_joint_conversion_preserves_marker_and_converts_finite_tail() {
    let degrees = [f64::NAN, 45.0, -90.0, 180.0, -180.0, 360.0];
    let radians = [f64::NAN, FRAC_PI_4, -FRAC_PI_2, PI, -PI, TAU];
    for input_degrees in [false, true] {
        let converted_marker =
            previous_joints_to_internal(CONSTRAINT_CENTERED, input_degrees).unwrap();
        assert!(converted_marker[0].is_nan());
        assert_eq!(converted_marker[1..], CONSTRAINT_CENTERED[1..]);

        let input = if input_degrees { degrees } else { radians };
        let converted = previous_joints_to_internal(input, input_degrees).unwrap();
        assert!(converted[0].is_nan());
        for (actual, expected) in converted[1..].iter().zip(&radians[1..]) {
            assert!((actual - expected).abs() < 1.0e-14);
        }
    }
}

#[test]
fn visual_joint_conversion_returns_degrees_for_both_input_units() {
    let degrees = [0.0, 45.0, -90.0, 180.0, -180.0, 360.0];
    let radians = [0.0, FRAC_PI_4, -FRAC_PI_2, PI, -PI, TAU];
    let expected = [0.0, 45.0, -90.0, 180.0, -180.0, 360.0];
    assert_eq!(joints_to_visual_degrees(degrees, true).unwrap(), expected);
    assert_eq!(joints_to_visual_degrees(radians, false).unwrap(), expected);
}

#[test]
fn visual_joint_conversion_rejects_overflow_after_converting_radians() {
    for (value, input_degrees) in [
        (f64::from(f32::MAX) * 2.0, true),
        // This fits in f32 as radians, but the degree value does not.
        (f64::from(f32::MAX), false),
        // Conversion to degrees can also overflow f64 before narrowing.
        (f64::MAX, false),
    ] {
        for position in 0..6 {
            let mut joints = [0.0; 6];
            joints[position] = value;
            assert_value_error(
                joints_to_visual_degrees(joints, input_degrees),
                "initial_joints values must be finite f32 values",
            );
        }
    }

    for input_degrees in [false, true] {
        assert_value_error(
            joints_to_visual_degrees(CONSTRAINT_CENTERED, input_degrees),
            "joint values must be finite",
        );
    }
}
