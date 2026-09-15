use umlcad_v6_geometry_api::{GeometryBackend, GeometryKind, ToleranceContext};
use umlcad_v6_occt_backend::OcctBackend;

const T: ToleranceContext = ToleranceContext { modeling: 1e-9, validation: 1e-6 };

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance * a.abs().max(b.abs()).max(1.0)
}

#[test]
fn box_topology_satisfies_independent_euler_and_incidence_invariants() {
    let backend = OcctBackend::new();
    let shape = backend.box_solid(10.0, 20.0, 30.0, T).unwrap().shape;
    let c = backend.topology_counts(&shape, T).unwrap();
    assert_eq!(c.solids, 1);
    assert_eq!(c.shells, 1);
    assert_eq!(c.faces, 6);
    assert_eq!(c.edges, 12);
    assert_eq!(c.vertices, 8);
    assert_eq!(c.vertices as i64 - c.edges as i64 + c.faces as i64, 2);
    let faces = backend.face_descriptors(&shape, T).unwrap();
    let edges = backend.edge_descriptors(&shape, T).unwrap();
    let vertices = backend.vertex_descriptors(&shape, T).unwrap();
    assert_eq!(faces.len() as u32, c.faces);
    assert_eq!(edges.len() as u32, c.edges);
    assert_eq!(vertices.len() as u32, c.vertices);
    let face_boundary_sum: u32 = faces.iter().map(|f| f.boundary_edge_count).sum();
    let edge_vertex_sum: u32 = edges.iter().map(|e| e.vertex_use_count).sum();
    let vertex_edge_sum: u32 = vertices.iter().map(|v| v.edge_use_count).sum();
    assert_eq!(face_boundary_sum, 2 * c.edges);
    assert_eq!(edge_vertex_sum, 2 * c.edges);
    assert_eq!(vertex_edge_sum, 2 * c.edges);
}

#[test]
fn primitive_bounds_match_independent_analytic_expectations() {
    let backend = OcctBackend::new();
    let box_shape = backend.box_solid(10.0, 20.0, 30.0, T).unwrap().shape;
    let b = backend.bounding_box(&box_shape, T).unwrap();
    assert!(close(b.min_x, 0.0, T.validation));
    assert!(close(b.min_y, 0.0, T.validation));
    assert!(close(b.min_z, 0.0, T.validation));
    assert!(close(b.max_x, 10.0, T.validation));
    assert!(close(b.max_y, 20.0, T.validation));
    assert!(close(b.max_z, 30.0, T.validation));

    let cylinder = backend.cylinder_solid(5.0, 12.0, T).unwrap().shape;
    let c = backend.bounding_box(&cylinder, T).unwrap();
    assert!(close(c.min_x, -5.0, T.validation));
    assert!(close(c.max_x, 5.0, T.validation));
    assert!(close(c.min_y, -5.0, T.validation));
    assert!(close(c.max_y, 5.0, T.validation));
    assert!(close(c.min_z, 0.0, T.validation));
    assert!(close(c.max_z, 12.0, T.validation));

    let sphere = backend.sphere_solid(7.0, T).unwrap().shape;
    let s = backend.bounding_box(&sphere, T).unwrap();
    assert!(close(s.min_x, -7.0, T.validation));
    assert!(close(s.max_x, 7.0, T.validation));
    assert!(close(s.min_y, -7.0, T.validation));
    assert!(close(s.max_y, 7.0, T.validation));
    assert!(close(s.min_z, -7.0, T.validation));
    assert!(close(s.max_z, 7.0, T.validation));

    let cone = backend.cone_solid(8.0, 3.0, 11.0, T).unwrap().shape;
    let k = backend.bounding_box(&cone, T).unwrap();
    assert!(close(k.min_x, -8.0, T.validation));
    assert!(close(k.max_x, 8.0, T.validation));
    assert!(close(k.min_y, -8.0, T.validation));
    assert!(close(k.max_y, 8.0, T.validation));
    assert!(close(k.min_z, 0.0, T.validation));
    assert!(close(k.max_z, 11.0, T.validation));
}

#[test]
fn translation_is_compositional_against_an_independent_vector_oracle() {
    let backend = OcctBackend::new();
    let source = backend.box_solid(10.0, 20.0, 30.0, T).unwrap().shape;
    let first = backend.translate(&source, 11.0, -7.0, 4.0, T).unwrap().shape;
    let composed = backend.translate(&first, -3.0, 5.0, 9.0, T).unwrap().shape;
    let direct = backend.translate(&source, 8.0, -2.0, 13.0, T).unwrap().shape;
    let a = backend.bounding_box(&composed, T).unwrap();
    let b = backend.bounding_box(&direct, T).unwrap();
    assert!(close(a.min_x, b.min_x, T.validation));
    assert!(close(a.min_y, b.min_y, T.validation));
    assert!(close(a.min_z, b.min_z, T.validation));
    assert!(close(a.max_x, b.max_x, T.validation));
    assert!(close(a.max_y, b.max_y, T.validation));
    assert!(close(a.max_z, b.max_z, T.validation));
}

#[test]
fn full_turn_rotation_is_an_independent_identity_oracle_for_solid_geometry() {
    let backend = OcctBackend::new();
    let source = backend.box_solid(10.0, 20.0, 30.0, T).unwrap().shape;
    let rotated = backend.rotate(&source, 0.0, 0.0, 1.0, std::f64::consts::TAU, T).unwrap();
    assert_eq!(rotated.kind, GeometryKind::Solid);
    let source_bounds = backend.bounding_box(&source, T).unwrap();
    let rotated_bounds = backend.bounding_box(&rotated.shape, T).unwrap();
    assert!(close(source_bounds.min_x, rotated_bounds.min_x, T.validation));
    assert!(close(source_bounds.min_y, rotated_bounds.min_y, T.validation));
    assert!(close(source_bounds.min_z, rotated_bounds.min_z, T.validation));
    assert!(close(source_bounds.max_x, rotated_bounds.max_x, T.validation));
    assert!(close(source_bounds.max_y, rotated_bounds.max_y, T.validation));
    assert!(close(source_bounds.max_z, rotated_bounds.max_z, T.validation));
    assert_eq!(backend.topology_counts(&source, T).unwrap(), backend.topology_counts(&rotated.shape, T).unwrap());
}

#[test]
fn analytic_bounds_scale_covariantly_for_a_small_but_realizable_model() {
    let backend = OcctBackend::new();
    let shape = backend.box_solid(1e-3, 2e-3, 3e-3, T).unwrap().shape;
    let bounds = backend.bounding_box(&shape, T).unwrap();
    assert!(close(bounds.min_x, 0.0, T.validation));
    assert!(close(bounds.min_y, 0.0, T.validation));
    assert!(close(bounds.min_z, 0.0, T.validation));
    assert!(close(bounds.max_x, 1e-3, T.validation));
    assert!(close(bounds.max_y, 2e-3, T.validation));
    assert!(close(bounds.max_z, 3e-3, T.validation));
}
