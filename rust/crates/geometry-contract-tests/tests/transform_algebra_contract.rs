use umlcad_v6_geometry_api::{GeometryBackend, ToleranceContext};
use umlcad_v6_occt_backend::OcctBackend;

const TOLERANCE: f64 = 1e-6;
const T: ToleranceContext = ToleranceContext { modeling: 1e-9, validation: TOLERANCE };

#[test]
fn zero_translation_is_identity_and_source_is_unchanged() {
    let backend = OcctBackend::new();
    let source = backend.box_solid(12.0, 20.0, 30.0, T).unwrap().shape;
    let before = backend.bounding_box(&source, T).unwrap();
    let counts_before = backend.topology_counts(&source, T).unwrap();

    let translated = backend.translate(&source, 0.0, 0.0, 0.0, T).unwrap().shape;

    assert_eq!(before, backend.bounding_box(&translated, T).unwrap());
    assert_eq!(before, backend.bounding_box(&source, T).unwrap());
    assert_eq!(counts_before, backend.topology_counts(&source, T).unwrap());
}

#[test]
fn translation_composition_equals_vector_sum() {
    let backend = OcctBackend::new();
    let source = backend.box_solid(12.0, 20.0, 30.0, T).unwrap().shape;

    let first = backend.translate(&source, 3.0, -4.0, 5.0, T).unwrap().shape;
    let composed = backend.translate(&first, -1.0, 7.0, 2.0, T).unwrap().shape;
    let direct = backend.translate(&source, 2.0, 3.0, 7.0, T).unwrap().shape;

    assert_eq!(
        backend.bounding_box(&composed, T).unwrap(),
        backend.bounding_box(&direct, T).unwrap()
    );
    assert_eq!(
        backend.topology_counts(&composed, T).unwrap(),
        backend.topology_counts(&direct, T).unwrap()
    );
}

#[test]
fn full_turn_rotation_preserves_geometry_within_measurement_tolerance() {
    let backend = OcctBackend::new();
    let source = backend.box_solid(12.0, 20.0, 30.0, T).unwrap().shape;
    let rotated = backend
        .rotate(&source, 0.0, 0.0, 1.0, std::f64::consts::TAU, T)
        .unwrap()
        .shape;

    let original = backend.bounding_box(&source, T).unwrap();
    let result = backend.bounding_box(&rotated, T).unwrap();
    assert!((original.min_x - result.min_x).abs() <= TOLERANCE);
    assert!((original.min_y - result.min_y).abs() <= TOLERANCE);
    assert!((original.min_z - result.min_z).abs() <= TOLERANCE);
    assert!((original.max_x - result.max_x).abs() <= TOLERANCE);
    assert!((original.max_y - result.max_y).abs() <= TOLERANCE);
    assert!((original.max_z - result.max_z).abs() <= TOLERANCE);
    assert_eq!(
        backend.topology_counts(&source, T).unwrap(),
        backend.topology_counts(&rotated, T).unwrap()
    );
}

#[test]
fn transform_order_is_observable_and_source_remains_unchanged() {
    let backend = OcctBackend::new();
    let source = backend.box_solid(8.0, 12.0, 16.0, T).unwrap().shape;
    let original = backend.bounding_box(&source, T).unwrap();

    let translated_then_rotated = backend
        .translate(&source, 20.0, 0.0, 0.0, T)
        .and_then(|r| backend.rotate(&r.shape, 0.0, 0.0, 1.0, std::f64::consts::FRAC_PI_2, T))
        .unwrap()
        .shape;

    let rotated_then_translated = backend
        .rotate(&source, 0.0, 0.0, 1.0, std::f64::consts::FRAC_PI_2, T)
        .and_then(|r| backend.translate(&r.shape, 20.0, 0.0, 0.0, T))
        .unwrap()
        .shape;

    let first = backend.bounding_box(&translated_then_rotated, T).unwrap();
    let second = backend.bounding_box(&rotated_then_translated, T).unwrap();
    assert_ne!(first, second);
    assert_eq!(original, backend.bounding_box(&source, T).unwrap());
}

#[test]
fn valid_transforms_preserve_solid_validation() {
    let backend = OcctBackend::new();
    let source = backend.box_solid(12.0, 20.0, 30.0, T).unwrap().shape;
    let translated = backend.translate(&source, 3.0, 4.0, 5.0, T).unwrap().shape;
    let rotated = backend
        .rotate(&translated, 1.0, 2.0, 3.0, 0.37, T)
        .unwrap()
        .shape;

    assert!(backend.validate(&source, T).unwrap().valid);
    assert!(backend.validate(&translated, T).unwrap().valid);
    assert!(backend.validate(&rotated, T).unwrap().valid);
}
