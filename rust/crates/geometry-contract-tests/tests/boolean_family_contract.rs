use umlcad_v6_geometry_api::{GeometryBackend, ToleranceContext};
use umlcad_v6_occt_backend::OcctBackend;

const TOLERANCE: ToleranceContext = ToleranceContext { modeling: 1e-9, validation: 1e-9 };

#[test]
fn common_and_cut_produce_valid_results() {
    let backend = OcctBackend::new();
    let left = backend.box_solid(10.0, 10.0, 10.0, TOLERANCE).unwrap().shape;
    let right_base = backend.box_solid(10.0, 10.0, 10.0, TOLERANCE).unwrap().shape;
    let right = backend.translate(&right_base, 5.0, 0.0, 0.0, TOLERANCE).unwrap().shape;

    let common = backend.common(&left, &right, TOLERANCE).unwrap().shape;
    let cut = backend.cut(&left, &right, TOLERANCE).unwrap().shape;

    assert!(backend.validate(&common, TOLERANCE).unwrap().valid);
    assert!(backend.validate(&cut, TOLERANCE).unwrap().valid);

    let common_bounds = backend.bounding_box(&common, TOLERANCE).unwrap();
    assert!((common_bounds.min_x - 5.0).abs() <= 1e-9);
    assert!((common_bounds.max_x - 10.0).abs() <= 1e-9);

    let cut_bounds = backend.bounding_box(&cut, TOLERANCE).unwrap();
    assert!((cut_bounds.min_x - 0.0).abs() <= 1e-9);
    assert!((cut_bounds.max_x - 5.0).abs() <= 1e-9);
}

#[test]
fn Boolean_results_are_deterministic_for_identical_operands() {
    let backend = OcctBackend::new();
    let left = backend.box_solid(10.0, 10.0, 10.0, TOLERANCE).unwrap().shape;
    let right = backend.translate(
        &backend.box_solid(10.0, 10.0, 10.0, TOLERANCE).unwrap().shape,
        5.0,
        0.0,
        0.0,
        TOLERANCE,
    ).unwrap().shape;

    for operation in [0_u8, 1_u8, 2_u8] {
        let first = match operation {
            0 => backend.fuse(&left, &right, TOLERANCE).unwrap().shape,
            1 => backend.common(&left, &right, TOLERANCE).unwrap().shape,
            _ => backend.cut(&left, &right, TOLERANCE).unwrap().shape,
        };
        let second = match operation {
            0 => backend.fuse(&left, &right, TOLERANCE).unwrap().shape,
            1 => backend.common(&left, &right, TOLERANCE).unwrap().shape,
            _ => backend.cut(&left, &right, TOLERANCE).unwrap().shape,
        };
        assert_eq!(backend.bounding_box(&first, TOLERANCE).unwrap(), backend.bounding_box(&second, TOLERANCE).unwrap());
        assert_eq!(backend.topology_counts(&first, TOLERANCE).unwrap(), backend.topology_counts(&second, TOLERANCE).unwrap());
    }
}
