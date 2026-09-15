use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, GeometryKind, ToleranceContext, ValidationResult};
use umlcad_v6_occt_backend::OcctBackend;

const T: ToleranceContext = ToleranceContext { modeling: 1e-9, validation: 1e-6 };

#[test]
fn circle_curve_is_explicitly_a_curve_and_geometrically_valid() {
    let backend = OcctBackend::new();
    let result = backend.circle_curve(10.0, T).unwrap();
    assert_eq!(result.kind, GeometryKind::Curve);
    assert_eq!(result.evidence.backend, "occt");
    assert_eq!(result.evidence.tolerance, T);
    assert_eq!(backend.validate(&result.shape, T).unwrap(), ValidationResult { valid: true, manifold: false, message: None });
}

#[test]
fn circle_curve_has_expected_bounds() {
    let backend = OcctBackend::new();
    let shape = backend.circle_curve(10.0, T).unwrap().shape;
    let bounds = backend.bounding_box(&shape, T).unwrap();
    assert!((bounds.min_x + 10.0).abs() <= T.validation);
    assert!((bounds.max_x - 10.0).abs() <= T.validation);
    assert!((bounds.min_y + 10.0).abs() <= T.validation);
    assert!((bounds.max_y - 10.0).abs() <= T.validation);
    assert!((bounds.min_z).abs() <= T.validation);
    assert!((bounds.max_z).abs() <= T.validation);
}

#[test]
fn circle_curve_rejects_invalid_radius() {
    let backend = OcctBackend::new();
    for radius in [0.0, -1.0, 1e-7, f64::NAN, f64::INFINITY] {
        assert!(matches!(backend.circle_curve(radius, T), Err(GeometryError::InvalidInput(_))));
    }
}

#[test]
fn circle_curve_is_deterministic_and_immutable() {
    let backend = OcctBackend::new();
    let source = backend.circle_curve(10.0, T).unwrap().shape;
    let first = backend.translate(&source, 20.0, -30.0, 40.0, T).unwrap();
    let second = backend.translate(&source, 20.0, -30.0, 40.0, T).unwrap();
    assert_eq!(first.kind, GeometryKind::Solid);
    assert_eq!(second.kind, GeometryKind::Solid);
    assert_eq!(backend.bounding_box(&first.shape, T).unwrap(), backend.bounding_box(&second.shape, T).unwrap());
    assert_eq!(backend.bounding_box(&source, T).unwrap(), backend.bounding_box(&backend.circle_curve(10.0, T).unwrap().shape, T).unwrap());
}
