//! Contract tests shared by every V6 geometry backend.
//! These tests define behavior independently of backend implementation details.

use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, ToleranceContext, ValidationResult};

const TOLERANCE: ToleranceContext = ToleranceContext {
    modeling: 1e-9,
    validation: 1e-9,
};

pub fn assert_backend_identity<B: GeometryBackend>(backend: &B, expected: &'static str) {
    assert_eq!(backend.backend_name(), expected);
}

pub fn assert_box_requires_positive_dimensions<B: GeometryBackend>(backend: &B) {
    for dimensions in [
        (0.0, 20.0, 30.0),
        (-1.0, 20.0, 30.0),
        (20.0, 0.0, 30.0),
        (20.0, 30.0, -1.0),
        (f64::NAN, 20.0, 30.0),
        (20.0, f64::INFINITY, 30.0),
        (20.0, 30.0, f64::NEG_INFINITY),
    ] {
        match backend.box_solid(dimensions.0, dimensions.1, dimensions.2, TOLERANCE) {
            Err(GeometryError::InvalidInput(_)) => {}
            Err(GeometryError::Unsupported(_)) => {
                panic!("backend does not yet implement this contract")
            }
            Err(other) => panic!("unexpected geometry error: {other}"),
            Ok(_) => panic!("invalid box dimensions unexpectedly succeeded"),
        }
    }
}

pub fn assert_invalid_tolerance_is_rejected<B: GeometryBackend>(backend: &B) {
    let invalid = [
        ToleranceContext {
            modeling: -1e-9,
            validation: 1e-9,
        },
        ToleranceContext {
            modeling: f64::NAN,
            validation: 1e-9,
        },
        ToleranceContext {
            modeling: 1e-9,
            validation: f64::INFINITY,
        },
    ];

    for tolerance in invalid {
        match backend.box_solid(10.0, 20.0, 30.0, tolerance) {
            Err(GeometryError::InvalidTolerance) => {}
            Err(other) => panic!("unexpected error for invalid tolerance: {other}"),
            Ok(_) => panic!("invalid tolerance unexpectedly succeeded"),
        }
    }
}

pub fn assert_numeric_scale_survives_validation<B: GeometryBackend>(backend: &B) {
    for edge in [1e-12, 1e-9, 1e-6, 1e3, 1e6] {
        let result = backend
            .box_solid(edge, edge * 2.0, edge * 3.0, TOLERANCE)
            .unwrap_or_else(|error| panic!("failed scale {edge:e}: {error}"));
        let validation = backend
            .validate(&result.shape, TOLERANCE)
            .unwrap_or_else(|error| panic!("validation failed at scale {edge:e}: {error}"));
        assert_eq!(validation, ValidationResult {
            valid: true,
            manifold: true,
            message: None,
        });
    }
}

pub fn assert_translation_preserves_validation<B: GeometryBackend>(backend: &B) {
    let solid = backend
        .box_solid(10.0, 20.0, 30.0, TOLERANCE)
        .expect("box construction should succeed")
        .shape;

    let translated = backend
        .translate(&solid, 1000.0, -2000.0, 3000.0, TOLERANCE)
        .expect("translation should succeed")
        .shape;

    let before = backend
        .validate(&solid, TOLERANCE)
        .expect("validation should succeed");
    let after = backend
        .validate(&translated, TOLERANCE)
        .expect("validation should succeed");

    assert_eq!(before, ValidationResult { valid: true, manifold: true, message: None });
    assert_eq!(after, before);
}

pub fn assert_deterministic_validation<B: GeometryBackend>(backend: &B) {
    let mut reference = None;
    for _ in 0..32 {
        let shape = backend
            .box_solid(37.0, 11.0, 5.0, TOLERANCE)
            .expect("deterministic box should construct")
            .shape;
        let current = backend
            .validate(&shape, TOLERANCE)
            .expect("deterministic validation should succeed");
        match &reference {
            Some(expected) => assert_eq!(&current, expected),
            None => reference = Some(current),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use umlcad_v6_occt_backend::OcctBackend;

    #[test]
    fn occt_identity_contract_is_stable() {
        assert_backend_identity(&OcctBackend::new(), "occt");
    }

    #[test]
    fn occt_box_contract() {
        assert_box_requires_positive_dimensions(&OcctBackend::new());
    }

    #[test]
    fn occt_invalid_tolerance_contract() {
        assert_invalid_tolerance_is_rejected(&OcctBackend::new());
    }

    #[test]
    fn occt_numeric_scale_contract() {
        assert_numeric_scale_survives_validation(&OcctBackend::new());
    }

    #[test]
    fn occt_translation_validation_contract() {
        assert_translation_preserves_validation(&OcctBackend::new());
    }

    #[test]
    fn occt_determinism_contract() {
        assert_deterministic_validation(&OcctBackend::new());
    }
}
