use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, GeometryKind, ToleranceContext, ValidationResult};
use umlcad_v6_occt_backend::OcctBackend;

const T: ToleranceContext = ToleranceContext { modeling: 1e-9, validation: 1e-6 };

#[test]
fn ring_torus_is_a_valid_manifold_solid() {
    let backend = OcctBackend::new();
    let result = backend.torus_solid(20.0, 5.0, T).unwrap();
    assert_eq!(result.kind, GeometryKind::Solid);
    assert_eq!(result.evidence.backend, "occt");
    assert_eq!(backend.validate(&result.shape, T).unwrap(), ValidationResult { valid: true, manifold: true, message: None });
}

#[test]
fn ring_torus_has_expected_axis_aligned_bounds() {
    let backend = OcctBackend::new();
    let shape = backend.torus_solid(20.0, 5.0, T).unwrap().shape;
    let bounds = backend.bounding_box(&shape, T).unwrap();
    assert!((bounds.min_x + 25.0).abs() <= 1e-9, "x bounds: {:?}", bounds);
    assert!((bounds.max_x - 25.0).abs() <= 1e-9, "x bounds: {:?}", bounds);
    assert!((bounds.min_y + 25.0).abs() <= 1e-9, "y bounds: {:?}", bounds);
    assert!((bounds.max_y - 25.0).abs() <= 1e-9, "y bounds: {:?}", bounds);
    assert!((bounds.min_z + 5.0).abs() <= 1e-9, "z bounds: {:?}", bounds);
    assert!((bounds.max_z - 5.0).abs() <= 1e-9, "z bounds: {:?}", bounds);
}

#[test]
fn ring_torus_rejects_degenerate_self_intersecting_or_non_finite_parameters() {
    let backend = OcctBackend::new();
    for (major, minor) in [(0.0,5.0),(-1.0,5.0),(20.0,0.0),(20.0,-1.0),(5.0,5.0),(4.0,5.0),(f64::NAN,5.0),(20.0,f64::INFINITY)] {
        match backend.torus_solid(major, minor, T) {
            Err(GeometryError::InvalidInput(_)) => {}
            Err(error) => panic!("unexpected torus error: {error:?}"),
            Ok(_) => panic!("invalid torus parameters unexpectedly succeeded"),
        }
    }
}

#[test]
fn ring_torus_is_immutable_and_deterministic() {
    let backend = OcctBackend::new();
    let source = backend.torus_solid(20.0, 5.0, T).unwrap().shape;
    let first = backend.translate(&source, 100.0, -200.0, 300.0, T).unwrap().shape;
    let second = backend.translate(&source, 100.0, -200.0, 300.0, T).unwrap().shape;
    assert_eq!(backend.bounding_box(&first,T).unwrap(), backend.bounding_box(&second,T).unwrap());
    assert_eq!(backend.topology_counts(&first,T).unwrap(), backend.topology_counts(&second,T).unwrap());
    assert_eq!(backend.bounding_box(&source,T).unwrap(), backend.bounding_box(&backend.torus_solid(20.0,5.0,T).unwrap().shape,T).unwrap());
    assert!(backend.validate(&source,T).unwrap().valid);
    assert!(backend.validate(&first,T).unwrap().valid);
}
