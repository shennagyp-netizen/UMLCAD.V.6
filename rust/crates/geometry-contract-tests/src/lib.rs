//! Contract tests shared by every V6 geometry backend.
//! These tests intentionally describe behavior before the OCCT implementation exists.

use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, ToleranceContext, ValidationResult};

pub fn assert_backend_identity<B: GeometryBackend>(backend: &B, expected: &'static str) {
    assert_eq!(backend.backend_name(), expected);
}

pub fn assert_box_requires_positive_dimensions<B: GeometryBackend>(backend: &B) {
    let tolerance = ToleranceContext {
        modeling: 1e-9,
        validation: 1e-9,
    };

    let result = backend.box_solid(0.0, 20.0, 30.0, tolerance);
    match result {
        Err(GeometryError::InvalidInput(_)) => {}
        Err(GeometryError::Unsupported(_)) => panic!("backend does not yet implement this contract"),
        Err(other) => panic!("unexpected geometry error: {other}"),
        Ok(_) => panic!("zero-width box unexpectedly succeeded"),
    }
}

pub fn assert_translation_preserves_validation<B: GeometryBackend>(backend: &B) {
    let tolerance = ToleranceContext {
        modeling: 1e-9,
        validation: 1e-9,
    };
    let solid = backend
        .box_solid(10.0, 20.0, 30.0, tolerance)
        .expect("box construction should succeed")
        .shape;

    let translated = backend
        .translate(&solid, 1000.0, -2000.0, 3000.0)
        .expect("translation should succeed")
        .shape;

    let before = backend.validate(&solid, tolerance).expect("validation should succeed");
    let after = backend.validate(&translated, tolerance).expect("validation should succeed");

    assert_eq!(before, ValidationResult { valid: true, manifold: true, message: None });
    assert_eq!(after, before);
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
    #[ignore = "RED: becomes active when the first OCCT primitive is implemented"]
    fn occt_box_contract() {
        assert_box_requires_positive_dimensions(&OcctBackend::new());
    }

    #[test]
    #[ignore = "RED: becomes active when the first OCCT primitive is implemented"]
    fn occt_translation_validation_contract() {
        assert_translation_preserves_validation(&OcctBackend::new());
    }
}
