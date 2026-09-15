use umlcad_kernel_rust::functions::nurbs_surface::{NurbsSurface2D, NurbsSurfaceError, Point3};

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3 { x, y, z }
}

#[test]
fn bilinear_degree_one_surface_matches_independent_plane_oracle() {
    let surface = NurbsSurface2D::new(
        1,
        1,
        vec![
            p(0.0, 0.0, 0.0), p(0.0, 1.0, 1.0),
            p(1.0, 0.0, 2.0), p(1.0, 1.0, 3.0),
        ],
        vec![1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
    );

    let q = surface.point_at(0.25, 0.75).unwrap();
    assert!((q.x - 0.25).abs() < 1e-12);
    assert!((q.y - 0.75).abs() < 1e-12);
    assert!((q.z - 1.25).abs() < 1e-12);
}

#[test]
fn rational_surface_weights_change_geometry_but_keep_parameter_domain() {
    let surface = NurbsSurface2D::new(
        1,
        1,
        vec![
            p(0.0, 0.0, 0.0), p(0.0, 1.0, 0.0),
            p(1.0, 0.0, 0.0), p(1.0, 1.0, 1.0),
        ],
        vec![1.0, 1.0, 1.0, 2.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
    );

    assert_eq!(surface.parameter_domain().unwrap(), (0.0, 1.0, 0.0, 1.0));
    let q = surface.point_at(0.5, 0.5).unwrap();
    assert!((q.x - 0.6).abs() < 1e-12);
    assert!((q.y - 0.6).abs() < 1e-12);
    assert!((q.z - 0.4).abs() < 1e-12);
}

#[test]
fn positive_weights_keep_surface_points_inside_the_control_point_box() {
    let surface = NurbsSurface2D::new(
        2,
        2,
        vec![
            p(0.0, 0.0, -1.0), p(0.0, 1.0, 2.0), p(0.0, 2.0, 0.0),
            p(1.0, 0.0, 3.0), p(1.0, 1.0, 5.0), p(1.0, 2.0, 1.0),
            p(2.0, 0.0, 0.0), p(2.0, 1.0, 4.0), p(2.0, 2.0, -2.0),
        ],
        vec![1.0; 9],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    );
    let bounds = surface.control_hull_bounds().unwrap();

    for (u, v) in [(0.0, 0.0), (0.2, 0.3), (0.5, 0.5), (0.8, 0.7), (1.0, 1.0)] {
        let q = surface.point_at(u, v).unwrap();
        assert!(q.x >= bounds.min.x - 1e-12 && q.x <= bounds.max.x + 1e-12);
        assert!(q.y >= bounds.min.y - 1e-12 && q.y <= bounds.max.y + 1e-12);
        assert!(q.z >= bounds.min.z - 1e-12 && q.z <= bounds.max.z + 1e-12);
    }
}

#[test]
fn translation_is_immutable_and_commutes_with_surface_evaluation() {
    let surface = NurbsSurface2D::new(
        1,
        1,
        vec![
            p(0.0, 0.0, 0.0), p(0.0, 1.0, 1.0),
            p(1.0, 0.0, 2.0), p(1.0, 1.0, 3.0),
        ],
        vec![1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
    );
    let moved = surface.translated(100.0, -20.0, 7.0).unwrap();
    let a = surface.point_at(0.3, 0.7).unwrap();
    let b = moved.point_at(0.3, 0.7).unwrap();

    assert!((b.x - (a.x + 100.0)).abs() < 1e-12);
    assert!((b.y - (a.y - 20.0)).abs() < 1e-12);
    assert!((b.z - (a.z + 7.0)).abs() < 1e-12);
    assert_eq!(surface.point_at(0.3, 0.7).unwrap(), a);
}

#[test]
fn validation_rejects_invalid_grid_weights_knots_and_parameters() {
    let points = vec![p(0.0, 0.0, 0.0), p(0.0, 1.0, 0.0), p(1.0, 0.0, 0.0), p(1.0, 1.0, 0.0)];
    let weights = vec![1.0; 4];
    let knots = vec![0.0, 0.0, 1.0, 1.0];

    assert_eq!(
        NurbsSurface2D::new(0, 1, points.clone(), weights.clone(), knots.clone(), knots.clone()).validate(),
        Err(NurbsSurfaceError::InvalidDegree)
    );
    assert_eq!(
        NurbsSurface2D::new(1, 1, points.clone(), vec![1.0, 1.0], knots.clone(), knots.clone()).validate(),
        Err(NurbsSurfaceError::InvalidWeightCount)
    );
    assert_eq!(
        NurbsSurface2D::new(1, 1, points.clone(), weights.clone(), vec![0.0, 1.0], knots.clone()).validate(),
        Err(NurbsSurfaceError::InvalidKnotCount)
    );
    assert_eq!(
        NurbsSurface2D::new(1, 1, points.clone(), weights.clone(), vec![0.0, 0.5, 0.25, 1.0], knots.clone()).validate(),
        Err(NurbsSurfaceError::KnotsMustBeNondecreasing)
    );
    assert!(
        NurbsSurface2D::new(1, 1, points.clone(), vec![1.0, -1.0, 1.0, 1.0], knots.clone(), knots.clone())
            .validate()
            .is_err()
    );
    let valid = NurbsSurface2D::new(1, 1, points, weights, knots.clone(), knots);
    assert_eq!(valid.point_at(-1e-12, 0.5), Err(NurbsSurfaceError::OutOfDomain));
    assert_eq!(valid.point_at(0.5, f64::NAN), Err(NurbsSurfaceError::NonFinite));
}
