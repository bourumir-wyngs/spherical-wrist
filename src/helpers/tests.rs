use super::*;

mod matrices;
mod numeric;

fn assert_value_error<T>(result: PyResult<T>, expected_message: &str) {
    let error = match result {
        Ok(_) => panic!("expected ValueError containing {expected_message:?}"),
        Err(error) => error,
    };

    // Exception inspection needs Python; initialize once safely across test threads.
    Python::initialize();
    Python::attach(|py| {
        assert!(error.is_instance_of::<PyValueError>(py), "{error}");
        let message = error.to_string();
        assert!(
            message.contains(expected_message),
            "expected {message:?} to contain {expected_message:?}"
        );
    });
}
