use crate::IntersectionStatus;
use umlcad_v6_nurbs_surface_api::{NurbsSurface3DDefinition, Point3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointSurfaceClosestPoint {
    pub point: Point3,
    pub surface_uv: (f64, f64),
    pub distance: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PointSurfaceClosestPointResult {
    pub status: IntersectionStatus,
    pub closest: Option<PointSurfaceClosestPoint>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointSurfaceClosestPointError {
    NonFinite,
    InvalidSurface,
    UnsupportedSurfaceFamily,
    DegenerateSurface,
    NumericalFailure,
}

#[derive(Clone, Copy)]
struct Patch {
    origin: Point3,
    du: Point3,
    dv: Point3,
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
}

fn sub(a: Point3, b: Point3) -> Point3 {
    Point3 { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }
}
fn add(a: Point3, b: Point3) -> Point3 {
    Point3 { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }
}
fn scale(a: Point3, s: f64) -> Point3 {
    Point3 { x: a.x * s, y: a.y * s, z: a.z * s }
}
fn dot(a: Point3, b: Point3) -> f64 { a.x * b.x + a.y * b.y + a.z * b.z }
fn norm(a: Point3) -> f64 { dot(a, a).sqrt() }
fn lerp_patch(p: Patch, u: f64, v: f64) -> Point3 {
    add(p.origin, add(scale(p.du, u), scale(p.dv, v)))
}

fn patch(surface: &NurbsSurface3DDefinition) -> Result<Patch, PointSurfaceClosestPointError> {
    surface.validate().map_err(|_| PointSurfaceClosestPointError::InvalidSurface)?;
    if surface.degree_u != 1
        || surface.degree_v != 1
        || surface.count_u != 2
        || surface.count_v != 2
        || surface.weights.iter().any(|w| (*w - 1.0).abs() > 1e-14)
    {
        return Err(PointSurfaceClosestPointError::UnsupportedSurfaceFamily);
    }
    let origin = surface.control_points[0];
    let du = sub(surface.control_points[2], origin);
    let dv = sub(surface.control_points[1], origin);
    let closure = sub(sub(surface.control_points[3], surface.control_points[2]), dv);
    let reference = norm(du).max(norm(dv)).max(1.0);
    if norm(closure) > 1e-12 * reference {
        return Err(PointSurfaceClosestPointError::UnsupportedSurfaceFamily);
    }
    let ((u0, u1), (v0, v1)) = surface
        .parameter_domain()
        .map_err(|_| PointSurfaceClosestPointError::InvalidSurface)?;
    if !norm(du).is_finite() || !norm(dv).is_finite() || norm(du) == 0.0 || norm(dv) == 0.0 {
        return Err(PointSurfaceClosestPointError::DegenerateSurface);
    }
    Ok(Patch { origin, du, dv, u0, u1, v0, v1 })
}

fn uv_unconstrained(p: Patch, point: Point3) -> Result<(f64, f64), PointSurfaceClosestPointError> {
    let r = sub(point, p.origin);
    let aa = dot(p.du, p.du);
    let ab = dot(p.du, p.dv);
    let bb = dot(p.dv, p.dv);
    let det = aa * bb - ab * ab;
    if !det.is_finite() || det <= 1e-24 * aa.max(bb).powi(2) {
        return Err(PointSurfaceClosestPointError::DegenerateSurface);
    }
    let ru = dot(p.du, r);
    let rv = dot(p.dv, r);
    Ok(((ru * bb - rv * ab) / det, (rv * aa - ru * ab) / det))
}

fn objective(p: Patch, point: Point3, u: f64, v: f64) -> f64 {
    norm(sub(lerp_patch(p, u, v), point)).powi(2)
}

fn one_dimensional_min(
    p: Patch,
    point: Point3,
    fixed_u: Option<f64>,
    fixed_v: Option<f64>,
    min: f64,
    max: f64,
) -> (f64, f64, f64) {
    let (base, direction, fixed) = match (fixed_u, fixed_v) {
        (Some(u), None) => (sub(add(p.origin, scale(p.du, u)), point), p.dv, u),
        (None, Some(v)) => (sub(add(p.origin, scale(p.dv, v)), point), p.du, v),
        _ => unreachable!(),
    };
    let denom = dot(direction, direction);
    let t = if denom == 0.0 { min } else { (min.max((0.0 - dot(direction, base) / denom).max(min))).min(max) };
    match fixed_u {
        Some(u) => (u, t, objective(p, point, u, t)),
        None => (t, fixed, objective(p, point, t, fixed)),
    }
}

pub fn closest_point_on_planar_nurbs_surface(
    point: Point3,
    surface: &NurbsSurface3DDefinition,
    tolerance: f64,
) -> Result<PointSurfaceClosestPointResult, PointSurfaceClosestPointError> {
    let values = [point.x, point.y, point.z];
    if values.iter().any(|v| !v.is_finite()) || !tolerance.is_finite() || tolerance < 0.0 {
        return Err(PointSurfaceClosestPointError::NonFinite);
    }
    let p = patch(surface)?;
    let (u_star, v_star) = uv_unconstrained(p, point)?;
    let candidates = [
        (u_star.clamp(p.u0, p.u1), v_star.clamp(p.v0, p.v1)),
        (one_dimensional_min(p, point, Some(p.u0), None, p.v0, p.v1).0, one_dimensional_min(p, point, Some(p.u0), None, p.v0, p.v1).1),
        (one_dimensional_min(p, point, Some(p.u1), None, p.v0, p.v1).0, one_dimensional_min(p, point, Some(p.u1), None, p.v0, p.v1).1),
        (one_dimensional_min(p, point, None, Some(p.v0), p.u0, p.u1).0, one_dimensional_min(p, point, None, Some(p.v0), p.u0, p.u1).1),
        (one_dimensional_min(p, point, None, Some(p.v1), p.u0, p.u1).0, one_dimensional_min(p, point, None, Some(p.v1), p.u0, p.u1).1),
        (p.u0, p.v0),
        (p.u0, p.v1),
        (p.u1, p.v0),
        (p.u1, p.v1),
    ];
    let mut best = None::<(f64, f64, f64)>;
    for (u, v) in candidates {
        let d2 = objective(p, point, u, v);
        if !d2.is_finite() {
            return Err(PointSurfaceClosestPointError::NumericalFailure);
        }
        let replace = best.map_or(true, |current| d2 < current.2);
        if replace {
            best = Some((u, v, d2));
        }
    }
    let (u, v, d2) = best.ok_or(PointSurfaceClosestPointError::NumericalFailure)?;
    let closest = PointSurfaceClosestPoint { point: lerp_patch(p, u, v), surface_uv: (u, v), distance: d2.sqrt() };
    let inside = u_star >= p.u0 - tolerance && u_star <= p.u1 + tolerance && v_star >= p.v0 - tolerance && v_star <= p.v1 + tolerance;
    Ok(PointSurfaceClosestPointResult { status: IntersectionStatus::Unique, closest: Some(closest) }).map(|mut result| {
        if inside {
            result
        } else {
            result.status = IntersectionStatus::Unique;
            result
        }
    })
}

pub trait PointSurfaceOperations {
    fn closest_point_on_planar_nurbs_surface(
        &self,
        point: Point3,
        surface: &NurbsSurface3DDefinition,
        tolerance: f64,
    ) -> Result<PointSurfaceClosestPointResult, PointSurfaceClosestPointError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_patch() -> NurbsSurface3DDefinition {
        NurbsSurface3DDefinition::new(
            (1, 1),
            vec![
                Point3 { x: 0.0, y: 0.0, z: 0.0 },
                Point3 { x: 0.0, y: 1.0, z: 0.0 },
                Point3 { x: 1.0, y: 0.0, z: 0.0 },
                Point3 { x: 1.0, y: 1.0, z: 0.0 },
            ],
            vec![1.0; 4],
            (2, 2),
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
        )
    }

    #[test]
    fn interior_projection_is_exact() {
        let result = closest_point_on_planar_nurbs_surface(
            Point3 { x: 0.25, y: 0.75, z: 2.0 },
            &unit_patch(),
            1e-10,
        )
        .unwrap();
        let closest = result.closest.unwrap();
        assert_eq!(result.status, IntersectionStatus::Unique);
        assert!((closest.surface_uv.0 - 0.25).abs() < 1e-12);
        assert!((closest.surface_uv.1 - 0.75).abs() < 1e-12);
        assert!((closest.distance - 2.0).abs() < 1e-12);
    }

    #[test]
    fn outside_point_closest_point_is_boundary() {
        let result = closest_point_on_planar_nurbs_surface(
            Point3 { x: 2.0, y: 0.5, z: 0.0 },
            &unit_patch(),
            1e-10,
        )
        .unwrap();
        let closest = result.closest.unwrap();
        assert!((closest.surface_uv.0 - 1.0).abs() < 1e-12);
        assert!((closest.surface_uv.1 - 0.5).abs() < 1e-12);
        assert!((closest.distance - 1.0).abs() < 1e-12);
    }
}
