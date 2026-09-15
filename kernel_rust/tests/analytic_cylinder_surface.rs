use std::f64::consts::PI;

use umlcad_kernel_rust::functions::surfaces::{CylinderSurface, Point3, SurfaceError};

#[test]
fn cylinder_surface_has_exact_area_parameterization_normal_and_distance() {
    let s = CylinderSurface::new(Point3 { x: 1.0, y: -2.0, z: 3.0 }, 5.0, 12.0);
    assert!(s.validate().is_ok());
    assert!((s.area() - 2.0 * PI * 5.0 * 12.0).abs() < 1e-12);

    let p = s.point_at(0.5, 0.5).unwrap();
    assert!((p.x - 1.0).abs() < 1e-12);
    assert!((p.y + 7.0).abs() < 1e-12);
    assert!((p.z - 3.0).abs() < 1e-12);

    let n = s.normal_at(0.5, 0.5).unwrap();
    assert!((n.norm() - 1.0).abs() < 1e-12);
    let radial = s.center().vector_to(p);
    assert!((radial.dot(n) - 5.0).abs() < 1e-12);
    assert!(n.z.abs() < 1e-12);

    assert!(s.distance_to_point(p).unwrap() < 1e-12);
    assert!((s.distance_to_point(Point3 { x: 1.0, y: -2.0, z: 9.0 }).unwrap() - 1.0).abs() < 1e-12);
    assert!((s.distance_to_point(Point3 { x: 1.0, y: -2.0, z: 14.0 }).unwrap() - 6.0).abs() < 1e-12);
}

#[test]
fn cylinder_surface_has_exact_axis_aligned_bounds() {
    let s = CylinderSurface::new(Point3 { x: 2.0, y: -3.0, z: 4.0 }, 5.0, 8.0);
    let b = s.bounding_box().unwrap();
    assert_eq!(b.min, Point3 { x: -3.0, y: -8.0, z: 0.0 });
    assert_eq!(b.max, Point3 { x: 7.0, y: 2.0, z: 8.0 });
}

#[test]
fn cylinder_surface_parameterization_is_periodic_in_azimuth_and_nonperiodic_in_height() {
    let s = CylinderSurface::new(Point3 { x: 0.0, y: 0.0, z: 0.0 }, 2.0, 10.0);
    assert_eq!(s.point_at(0.0, 0.25).unwrap(), s.point_at(1.0, 0.25).unwrap());
    assert_ne!(s.point_at(0.0, 0.0).unwrap(), s.point_at(0.0, 1.0).unwrap());
    assert!((s.point_at(0.25, 0.5).unwrap().z).abs() < 1e-12);
}

#[test]
fn cylinder_surface_rejects_invalid_inputs_and_distinguishes_nonfinite_parameters() {
    assert_eq!(
        CylinderSurface::new(Point3 { x: 0.0, y: 0.0, z: 0.0 }, 0.0, 1.0).validate(),
        Err(SurfaceError::InvalidRadius)
    );
    assert_eq!(
        CylinderSurface::new(Point3 { x: 0.0, y: 0.0, z: 0.0 }, 1.0, 0.0).validate(),
        Err(SurfaceError::InvalidExtent)
    );
    assert_eq!(
        CylinderSurface::new(Point3 { x: f64::NAN, y: 0.0, z: 0.0 }, 1.0, 1.0).validate(),
        Err(SurfaceError::NonFinite)
    );
    let s = CylinderSurface::new(Point3 { x: 0.0, y: 0.0, z: 0.0 }, 1.0, 1.0);
    assert_eq!(s.point_at(f64::NAN, 0.5), Err(SurfaceError::NonFinite));
    assert_eq!(s.point_at(0.5, f64::INFINITY), Err(SurfaceError::NonFinite));
    assert_eq!(s.point_at(-1e-6, 0.5), Err(SurfaceError::OutOfDomain));
    assert_eq!(s.point_at(0.5, 1.000001), Err(SurfaceError::OutOfDomain));
}

#[test]
fn cylinder_surface_translation_is_immutable_and_rejects_overflow() {
    let s = CylinderSurface::new(Point3 { x: 0.0, y: 0.0, z: 0.0 }, 3.0, 7.0);
    let moved = s.translated(4.0, -3.0, 2.0).unwrap();
    assert_eq!(moved.center(), Point3 { x: 4.0, y: -3.0, z: 2.0 });
    assert_eq!(moved.radius(), s.radius());
    assert_eq!(moved.height(), s.height());
    assert!((moved.area() - s.area()).abs() < 1e-12);
    assert_eq!(s.center(), Point3 { x: 0.0, y: 0.0, z: 0.0 });

    let overflowing = CylinderSurface::new(Point3 { x: f64::MAX, y: 0.0, z: 0.0 }, 1.0, 1.0);
    assert_eq!(
        overflowing.translated(f64::MAX, 0.0, 0.0),
        Err(SurfaceError::NonFinite)
    );
}
