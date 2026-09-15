use umlcad_kernel_rust::functions::bspline::{BSplineCurve2D, BSplineError, Point2};

fn p(x: f64, y: f64) -> Point2 {
    Point2 { x, y }
}

fn quadratic() -> BSplineCurve2D {
    BSplineCurve2D::new(
        2,
        vec![p(0.0, 0.0), p(1.0, 1.0), p(2.0, 0.0)],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    )
}

#[test]
fn quadratic_derivative_matches_independent_polynomial_oracle() {
    let curve = quadratic();

    let d0 = curve.derivative_at(0.0).unwrap();
    assert!((d0.x - 2.0).abs() < 1e-12);
    assert!((d0.y - 2.0).abs() < 1e-12);

    let d25 = curve.derivative_at(0.25).unwrap();
    assert!((d25.x - 2.0).abs() < 1e-12);
    assert!((d25.y - 1.0).abs() < 1e-12);

    let d50 = curve.derivative_at(0.5).unwrap();
    assert!((d50.x - 2.0).abs() < 1e-12);
    assert!(d50.y.abs() < 1e-12);

    let d1 = curve.derivative_at(1.0).unwrap();
    assert!((d1.x - 2.0).abs() < 1e-12);
    assert!((d1.y + 2.0).abs() < 1e-12);
}

#[test]
fn derivative_tangent_is_unit_length_and_matches_direction() {
    let curve = quadratic();
    let tangent = curve.tangent_at(0.25).unwrap();
    assert!((tangent.x - 2.0 / 5.0_f64.sqrt()).abs() < 1e-12);
    assert!((tangent.y - 1.0 / 5.0_f64.sqrt()).abs() < 1e-12);
    assert!((tangent.x.hypot(tangent.y) - 1.0).abs() < 1e-12);
}

#[test]
fn derivative_of_linear_bspline_is_explicitly_unsupported() {
    let curve = BSplineCurve2D::new(
        1,
        vec![p(0.0, 0.0), p(4.0, 2.0)],
        vec![0.0, 0.0, 1.0, 1.0],
    );
    assert_eq!(curve.derivative_at(0.5), Err(BSplineError::DerivativeDegreeUnsupported));
}

#[test]
fn derivative_preserves_translation_invariance_without_mutating_source() {
    let curve = quadratic();
    let moved = curve.translated(100.0, -50.0).unwrap();
    let original = curve.derivative_at(0.25).unwrap();
    let translated = moved.derivative_at(0.25).unwrap();

    assert_eq!(curve.derivative_at(0.25).unwrap(), original);
    assert!((translated.x - original.x).abs() < 1e-12);
    assert!((translated.y - original.y).abs() < 1e-12);
}

#[test]
fn derivative_domain_and_nonfinite_inputs_fail_closed() {
    let curve = quadratic();
    assert_eq!(curve.derivative_at(-1e-12), Err(BSplineError::OutOfDomain));
    assert_eq!(curve.derivative_at(1.0 + 1e-12), Err(BSplineError::OutOfDomain));
    assert_eq!(curve.derivative_at(f64::NAN), Err(BSplineError::NonFinite));
    assert_eq!(curve.derivative_at(f64::INFINITY), Err(BSplineError::NonFinite));
    assert_eq!(curve.tangent_at(f64::NAN), Err(BSplineError::NonFinite));
}
