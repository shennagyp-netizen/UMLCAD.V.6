use umlcad_kernel_rust::functions::{
    geometry::{Arc, Circle, Geometry, Line, Point},
    relations::{evaluate_relation, RelationResidual},
    snapshot::{Endpoint, GeometryItem, Relation, RelationPoint, SemanticSnapshot},
    solver::scaled_damped_qr,
    spatial::point_distance,
    topology::build_topology,
};

fn item(id: &str, geometry: Geometry) -> GeometryItem {
    GeometryItem { id: id.into(), geometry, parameter_dependencies: vec![] }
}

fn snapshot(geometry: Vec<GeometryItem>, relations: Vec<(String, Relation)>) -> SemanticSnapshot {
    SemanticSnapshot { parameters: vec![], geometry, constraints: vec![], relations }
}

#[test]
fn symmetric_relation_uses_only_independent_equations() {
    let s = snapshot(
        vec![
            item("a", Geometry::Line(Line { start: Point { x: 0.0, y: 0.0 }, end: Point { x: 2.0, y: 0.0 } })),
            item("b", Geometry::Line(Line { start: Point { x: 4.0, y: 0.0 }, end: Point { x: 6.0, y: 0.0 } })),
            item("o", Geometry::Line(Line { start: Point { x: 3.0, y: 0.0 }, end: Point { x: 3.0, y: 1.0 } })),
        ],
        vec![
            ("sym".into(), Relation::Symmetric {
                first: RelationPoint::Endpoint { geometry_id: "a".into(), point: Endpoint::Start },
                second: RelationPoint::Endpoint { geometry_id: "b".into(), point: Endpoint::End },
                about: RelationPoint::Endpoint { geometry_id: "o".into(), point: Endpoint::Start },
            }),
        ],
    );
    let residual = evaluate_relation(&s, &s.relations[0].1).unwrap();
    assert_eq!(residual.residuals.len(), 2);
    assert_eq!(residual.scales.len(), 2);
}

#[test]
fn line_arc_detects_interior_circle_intersection() {
    let line = Geometry::Line(Line { start: Point { x: 5.0, y: -20.0 }, end: Point { x: 5.0, y: 20.0 } });
    let arc = Geometry::Arc(Arc { center: Point { x: 0.0, y: 0.0 }, radius: 10.0, start_angle: 0.0, end_angle: std::f64::consts::PI });
    assert!(point_distance(&line, &arc) <= 1.0e-9);
}

#[test]
fn arc_circle_detects_interior_tangency() {
    let arc = Geometry::Arc(Arc { center: Point { x: 0.0, y: 0.0 }, radius: 10.0, start_angle: 0.0, end_angle: std::f64::consts::PI });
    let circle = Geometry::Circle(Circle { center: Point { x: 0.0, y: 11.0 }, radius: 1.0 });
    assert!(point_distance(&arc, &circle) <= 1.0e-9);
}

#[test]
fn arc_arc_detects_intersection_away_from_endpoint_and_midpoint_samples() {
    let a = Geometry::Arc(Arc { center: Point { x: 0.0, y: 0.0 }, radius: 5.0, start_angle: 0.0, end_angle: std::f64::consts::FRAC_PI_2 });
    let b = Geometry::Arc(Arc { center: Point { x: 5.0, y: 0.0 }, radius: 5.0, start_angle: 2.0, end_angle: 4.0 });
    assert!(point_distance(&a, &b) <= 1.0e-8);
}

#[test]
fn multi_wrap_arc_uses_set_membership_and_preserves_winding_length() {
    let arc = Arc { center: Point { x: 0.0, y: 0.0 }, radius: 2.0, start_angle: 7.0 * std::f64::consts::PI, end_angle: 10.0 * std::f64::consts::PI };
    assert!(arc.validate().is_ok());
    assert!((arc.length() - 6.0 * std::f64::consts::PI).abs() <= 1.0e-9);
    assert!(arc.contains_point(Point { x: 0.0, y: 2.0 }));
}

#[test]
fn spatial_multi_wrap_membership_is_offset_invariant() {
    let arc = Geometry::Arc(Arc { center: Point { x: 0.0, y: 0.0 }, radius: 2.0, start_angle: 7.0 * std::f64::consts::PI, end_angle: 10.0 * std::f64::consts::PI });
    let line = Geometry::Line(Line { start: Point { x: 0.0, y: -3.0 }, end: Point { x: 0.0, y: 3.0 } });
    assert!(point_distance(&line, &arc) <= 1.0e-9);
}

#[test]
fn self_loop_contributes_two_to_topology_degree() {
    let s = snapshot(
        vec![
            item("loop", Geometry::Arc(Arc { center: Point { x: 0.0, y: 0.0 }, radius: 1.0, start_angle: 0.0, end_angle: 2.0 * std::f64::consts::PI })),
            item("tail", Geometry::Line(Line { start: Point { x: 1.0, y: 0.0 }, end: Point { x: 2.0, y: 0.0 } })),
        ],
        vec![],
    );
    let error = build_topology(&s).unwrap_err();
    assert!(error.contains("degree 3"), "unexpected topology error: {error}");
}

#[test]
fn zero_damping_solves_an_underdetermined_system_with_minimum_norm_delta() {
    let report = scaled_damped_qr(&[vec![1.0, 0.0]], &[1.0], 0.0, 1.0e-10).unwrap();
    assert!(report.delta.iter().all(|x| x.is_finite()));
    assert!((report.delta[0] + 1.0).abs() <= 1.0e-10);
    assert!(report.delta[1].abs() <= 1.0e-10);
}

#[test]
fn column_normalization_makes_rank_test_scale_invariant() {
    let small = scaled_damped_qr(&[vec![1.0e-12, 0.0], vec![0.0, 1.0e-12]], &[0.0, 0.0], 1.0e-12, 1.0e-10).unwrap();
    let large = scaled_damped_qr(&[vec![1.0e6, 0.0], vec![0.0, 1.0e6]], &[0.0, 0.0], 1.0e-12, 1.0e-10).unwrap();
    assert_eq!(small.rank, large.rank);
    assert_eq!(small.degrees_of_freedom, large.degrees_of_freedom);
}

fn _keep_relation_residual_type(_: &RelationResidual) {}
