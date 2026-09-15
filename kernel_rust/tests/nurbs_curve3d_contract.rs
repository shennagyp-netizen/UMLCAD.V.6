use umlcad_kernel_rust::functions::nurbs3d::{Nurbs3DError, NurbsCurve3D, Point3};

fn p(x: f64, y: f64, z: f64) -> Point3 { Point3 { x, y, z } }

#[test]
fn clamped_linear_nurbs_3d_matches_affine_oracle() {
    let curve = NurbsCurve3D::new(1, vec![p(0.0, 0.0, 0.0), p(10.0, 20.0, 30.0)], vec![1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]);
    assert_eq!(curve.parameter_domain().unwrap(), (0.0, 1.0));
    for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let q = curve.point_at(t).unwrap();
        assert!((q.x - 10.0 * t).abs() < 1e-12);
        assert!((q.y - 20.0 * t).abs() < 1e-12);
        assert!((q.z - 30.0 * t).abs() < 1e-12);
    }
}

#[test]
fn positive_weights_keep_rational_points_inside_control_point_box() {
    let curve = NurbsCurve3D::new(
        2,
        vec![p(-5.0, -10.0, 2.0), p(4.0, 20.0, 15.0), p(30.0, 3.0, -5.0)],
        vec![0.5, 2.0, 3.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    );
    let bounds = curve.control_hull_bounds().unwrap();
    for t in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0] {
        let q = curve.point_at(t).unwrap();
        assert!(q.x >= bounds.min.x - 1e-12 && q.x <= bounds.max.x + 1e-12);
        assert!(q.y >= bounds.min.y - 1e-12 && q.y <= bounds.max.y + 1e-12);
        assert!(q.z >= bounds.min.z - 1e-12 && q.z <= bounds.max.z + 1e-12);
    }
}

#[test]
fn translation_is_immutable_and_commutes_with_3d_rational_evaluation() {
    let curve = NurbsCurve3D::new(2, vec![p(0.0, 0.0, 0.0), p(1.0, 2.0, 3.0), p(4.0, 1.0, 5.0)], vec![1.0, 2.0, 1.0], vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    let moved = curve.translated(100.0, -50.0, 7.0).unwrap();
    let a = curve.point_at(0.4).unwrap();
    let b = moved.point_at(0.4).unwrap();
    assert_eq!(curve.point_at(0.4).unwrap(), a);
    assert!((b.x - (a.x + 100.0)).abs() < 1e-12);
    assert!((b.y - (a.y - 50.0)).abs() < 1e-12);
    assert!((b.z - (a.z + 7.0)).abs() < 1e-12);
}

#[test]
fn validation_rejects_nonfinite_and_invalid_projective_inputs() {
    let points = vec![p(0.0, 0.0, 0.0), p(1.0, 1.0, 1.0)];
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    assert_eq!(NurbsCurve3D::new(0, points.clone(), vec![1.0, 1.0], knots.clone()).validate(), Err(Nurbs3DError::InvalidDegree));
    assert_eq!(NurbsCurve3D::new(1, points.clone(), vec![1.0], knots.clone()).validate(), Err(Nurbs3DError::InvalidWeightCount));
    assert_eq!(NurbsCurve3D::new(1, points.clone(), vec![1.0, 0.0], knots.clone()).validate(), Err(Nurbs3DError::InvalidWeight));
    assert!(NurbsCurve3D::new(1, vec![p(f64::NAN, 0.0, 0.0), p(1.0, 1.0, 1.0)], vec![1.0, 1.0], knots.clone()).validate().is_err());
    assert!(NurbsCurve3D::new(1, points.clone(), vec![1.0, 1.0], vec![0.0, 0.5, 0.25, 1.0]).validate().is_err());
    assert_eq!(NurbsCurve3D::new(1, points, vec![1.0, 1.0], knots).point_at(f64::NAN), Err(Nurbs3DError::NonFinite));
}

#[test]
fn exact_endpoint_homogeneous_weighting_is_stable() {
    let curve = NurbsCurve3D::new(2, vec![p(3.0, 4.0, 5.0), p(9.0, 8.0, 7.0), p(10.0, 12.0, 14.0)], vec![1e-12, 3.0, 2.0], vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    assert_eq!(curve.point_at(0.0).unwrap(), p(3.0, 4.0, 5.0));
    assert_eq!(curve.point_at(1.0).unwrap(), p(10.0, 12.0, 14.0));
}
