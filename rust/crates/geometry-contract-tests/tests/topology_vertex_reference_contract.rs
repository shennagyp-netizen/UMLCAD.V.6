use umlcad_v6_geometry_api::{GeometryBackend, ReferenceResolution, ToleranceContext, VertexDescriptor};
use umlcad_v6_occt_backend::OcctBackend;

const T: ToleranceContext = ToleranceContext { modeling: 1e-9, validation: 1e-6 };

#[test]
fn box_vertex_evidence_has_eight_valid_vertices() {
    let backend = OcctBackend::new();
    let shape = backend.box_solid(10.0, 20.0, 30.0, T).unwrap().shape;
    let vertices = backend.vertex_descriptors(&shape, T).unwrap();

    assert_eq!(vertices.len(), 8);
    for vertex in &vertices {
        assert!(vertex.x.is_finite() && vertex.y.is_finite() && vertex.z.is_finite());
        assert!(vertex.edge_use_count >= 3);
        assert!(vertex.face_use_count >= 3);
        assert!(vertex.validate().is_ok());
    }
}

#[test]
fn repeated_vertex_evidence_is_deterministic() {
    let backend = OcctBackend::new();
    let shape = backend.box_solid(10.0, 20.0, 30.0, T).unwrap().shape;
    assert_eq!(backend.vertex_descriptors(&shape, T).unwrap(), backend.vertex_descriptors(&shape, T).unwrap());
}

#[test]
fn unique_geometric_vertex_query_resolves_without_traversal_identity() {
    let backend = OcctBackend::new();
    let shape = backend.box_solid(10.0, 20.0, 30.0, T).unwrap().shape;
    let vertices = backend.vertex_descriptors(&shape, T).unwrap();
    let query = vertices.iter().find(|v| v.x == 10.0 && v.y == 20.0 && v.z == 30.0).copied().unwrap();

    assert_eq!(backend.resolve_vertex_descriptor(&shape, &query, T).unwrap(), ReferenceResolution::Unique(query));
}

#[test]
fn tolerance_small_vertex_change_resolves_but_large_change_does_not() {
    let backend = OcctBackend::new();
    let shape = backend.box_solid(10.0, 20.0, 30.0, T).unwrap().shape;
    let original = backend.vertex_descriptors(&shape, T).unwrap().iter().find(|v| v.x == 10.0 && v.y == 20.0 && v.z == 30.0).copied().unwrap();

    let near = VertexDescriptor { x: original.x + 5e-7, ..original };
    assert!(matches!(backend.resolve_vertex_descriptor(&shape, &near, T).unwrap(), ReferenceResolution::Unique(_)));

    let far = VertexDescriptor { x: original.x + 5e-4, ..original };
    assert_eq!(backend.resolve_vertex_descriptor(&shape, &far, T).unwrap(), ReferenceResolution::NotFound);
}

#[test]
fn unmatched_geometric_vertex_query_is_not_found() {
    let backend = OcctBackend::new();
    let shape = backend.box_solid(10.0, 20.0, 30.0, T).unwrap().shape;
    let mut query = backend.vertex_descriptors(&shape, T).unwrap()[0];
    query.z = 1234.0;
    assert_eq!(backend.resolve_vertex_descriptor(&shape, &query, T).unwrap(), ReferenceResolution::NotFound);
}

#[test]
fn sphere_vertices_are_supported_without_assuming_two_incident_edges() {
    let backend = OcctBackend::new();
    let shape = backend.sphere_solid(5.0, T).unwrap().shape;
    let vertices = backend.vertex_descriptors(&shape, T).unwrap();
    assert_eq!(vertices.len(), 2);
    assert!(vertices.iter().all(|v| v.edge_use_count >= 1 && v.face_use_count >= 1));
}
