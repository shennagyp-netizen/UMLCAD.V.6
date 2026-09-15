use crate::IntersectionStatus;
use umlcad_v6_nurbs_surface_api::{NurbsSurface3DDefinition, Point3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceIntersectionEndpoint {
    pub point: Point3,
    pub first_uv: (f64, f64),
    pub second_uv: (f64, f64),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceIntersectionSegment {
    pub start: SurfaceIntersectionEndpoint,
    pub end: SurfaceIntersectionEndpoint,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceSurfaceIntersectionResult {
    pub status: IntersectionStatus,
    pub segments: Vec<SurfaceIntersectionSegment>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SurfaceSurfaceIntersectionError {
    NonFinite,
    InvalidSurface,
    UnsupportedSurfaceFamily,
    CoincidentOrUnderdetermined,
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

fn sub(a: Point3, b: Point3) -> Point3 { Point3 { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z } }
fn add(a: Point3, b: Point3) -> Point3 { Point3 { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z } }
fn scale(a: Point3, s: f64) -> Point3 { Point3 { x: a.x * s, y: a.y * s, z: a.z * s } }
fn dot(a: Point3, b: Point3) -> f64 { a.x * b.x + a.y * b.y + a.z * b.z }
fn cross(a: Point3, b: Point3) -> Point3 {
    Point3 { x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }
}
fn norm(a: Point3) -> f64 { dot(a, a).sqrt() }
fn point_at(p: Patch, su: f64, sv: f64) -> Point3 { add(p.origin, add(scale(p.du, su), scale(p.dv, sv))) }
fn parameter_from_normalized(p: Patch, su: f64, sv: f64) -> (f64, f64) {
    (p.u0 + su * (p.u1 - p.u0), p.v0 + sv * (p.v1 - p.v0))
}

fn patch(surface: &NurbsSurface3DDefinition) -> Result<Patch, SurfaceSurfaceIntersectionError> {
    surface.validate().map_err(|_| SurfaceSurfaceIntersectionError::InvalidSurface)?;
    if surface.degree_u != 1 || surface.degree_v != 1 || surface.count_u != 2 || surface.count_v != 2 || surface.weights.iter().any(|w| (*w - 1.0).abs() > 1e-14) {
        return Err(SurfaceSurfaceIntersectionError::UnsupportedSurfaceFamily);
    }
    let origin = surface.control_points[0];
    let du = sub(surface.control_points[2], origin);
    let dv = sub(surface.control_points[1], origin);
    let closure = sub(sub(surface.control_points[3], surface.control_points[2]), dv);
    let reference = norm(du).max(norm(dv)).max(1.0);
    if norm(closure) > 1e-12 * reference { return Err(SurfaceSurfaceIntersectionError::UnsupportedSurfaceFamily); }
    let ((u0, u1), (v0, v1)) = surface.parameter_domain().map_err(|_| SurfaceSurfaceIntersectionError::InvalidSurface)?;
    if norm(cross(du, dv)) <= 1e-14 * reference * reference { return Err(SurfaceSurfaceIntersectionError::UnsupportedSurfaceFamily); }
    Ok(Patch { origin, du, dv, u0, u1, v0, v1 })
}

fn normalized_uv_at(p: Patch, x: Point3) -> Result<(f64, f64), SurfaceSurfaceIntersectionError> {
    let r = sub(x, p.origin);
    let aa = dot(p.du, p.du);
    let ab = dot(p.du, p.dv);
    let bb = dot(p.dv, p.dv);
    let det = aa * bb - ab * ab;
    if !det.is_finite() || det.abs() <= 1e-14 * aa.max(bb).max(1.0) { return Err(SurfaceSurfaceIntersectionError::NumericalFailure); }
    let ru = dot(p.du, r);
    let rv = dot(p.dv, r);
    Ok(((ru * bb - rv * ab) / det, (rv * aa - ru * ab) / det))
}

fn uv_at(p: Patch, x: Point3) -> Result<(f64, f64), SurfaceSurfaceIntersectionError> {
    let (su, sv) = normalized_uv_at(p, x)?;
    Ok(parameter_from_normalized(p, su, sv))
}

fn constrain(base: f64, slope: f64, tol: f64, lo: &mut f64, hi: &mut f64) -> bool {
    if slope.abs() <= 1e-14 { return base >= -tol && base <= 1.0 + tol; }
    let t0 = (-base) / slope;
    let t1 = (1.0 - base) / slope;
    *lo = (*lo).max(t0.min(t1));
    *hi = (*hi).min(t0.max(t1));
    *lo <= *hi + tol
}

pub fn intersect_planar_nurbs_surfaces(first: &NurbsSurface3DDefinition, second: &NurbsSurface3DDefinition, tolerance: f64) -> Result<SurfaceSurfaceIntersectionResult, SurfaceSurfaceIntersectionError> {
    if !tolerance.is_finite() || tolerance < 0.0 { return Err(SurfaceSurfaceIntersectionError::NonFinite); }
    let a = patch(first)?;
    let b = patch(second)?;
    let n1 = cross(a.du, a.dv);
    let n2 = cross(b.du, b.dv);
    let dir = cross(n1, n2);
    let d2 = dot(dir, dir);
    let parallel_bound = 1e-24 * norm(n1).max(norm(n2)).max(1.0).powi(2);
    if !d2.is_finite() || d2 <= parallel_bound {
        let plane_offset = dot(n1, sub(b.origin, a.origin)).abs();
        let distance_tolerance = tolerance.max(1e-12 * norm(a.origin).max(norm(b.origin)).max(1.0)) * norm(n1);
        if !plane_offset.is_finite() { return Err(SurfaceSurfaceIntersectionError::NumericalFailure); }
        if plane_offset <= distance_tolerance {
            return Err(SurfaceSurfaceIntersectionError::CoincidentOrUnderdetermined);
        }
        return Ok(SurfaceSurfaceIntersectionResult { status: IntersectionStatus::NoIntersection, segments: Vec::new() });
    }
    let h1 = dot(n1, a.origin);
    let h2 = dot(n2, b.origin);
    let base = scale(cross(sub(scale(n2, h1), scale(n1, h2)), dir), 1.0 / d2);
    let direction = scale(dir, 1.0 / norm(dir));
    let sua = normalized_uv_at(a, base)?;
    let sua1 = normalized_uv_at(a, add(base, direction))?;
    let sub = normalized_uv_at(b, base)?;
    let sub1 = normalized_uv_at(b, add(base, direction))?;
    let mut lo = f64::NEG_INFINITY;
    let mut hi = f64::INFINITY;
    if !constrain(sua.0, sua1.0 - sua.0, tolerance, &mut lo, &mut hi) || !constrain(sua.1, sua1.1 - sua.1, tolerance, &mut lo, &mut hi) || !constrain(sub.0, sub1.0 - sub.0, tolerance, &mut lo, &mut hi) || !constrain(sub.1, sub1.1 - sub.1, tolerance, &mut lo, &mut hi) {
        return Ok(SurfaceSurfaceIntersectionResult { status: IntersectionStatus::NoIntersection, segments: Vec::new() });
    }
    if !lo.is_finite() || !hi.is_finite() { return Err(SurfaceSurfaceIntersectionError::NumericalFailure); }
    let su0 = sua.0 + (sua1.0 - sua.0) * lo;
    let sv0 = sua.1 + (sua1.1 - sua.1) * lo;
    let su1 = sua.0 + (sua1.0 - sua.0) * hi;
    let sv1 = sua.1 + (sua1.1 - sua.1) * hi;
    let p0 = point_at(a, su0, sv0);
    let p1 = point_at(a, su1, sv1);
    let e0 = SurfaceIntersectionEndpoint { point: p0, first_uv: uv_at(a, p0)?, second_uv: uv_at(b, p0)? };
    let e1 = SurfaceIntersectionEndpoint { point: p1, first_uv: uv_at(a, p1)?, second_uv: uv_at(b, p1)? };
    Ok(SurfaceSurfaceIntersectionResult { status: IntersectionStatus::Unique, segments: vec![SurfaceIntersectionSegment { start: e0, end: e1 }] })
}

pub trait SurfaceSurfaceOperations {
    fn intersect_planar_nurbs_surfaces(&self, first: &NurbsSurface3DDefinition, second: &NurbsSurface3DDefinition, tolerance: f64) -> Result<SurfaceSurfaceIntersectionResult, SurfaceSurfaceIntersectionError>;
}

#[cfg(test)]
mod tests {
    use super::*;
    fn horizontal() -> NurbsSurface3DDefinition {
        NurbsSurface3DDefinition::new((1, 1), vec![Point3 { x: 0.0, y: 0.0, z: 0.0 }, Point3 { x: 0.0, y: 1.0, z: 0.0 }, Point3 { x: 1.0, y: 0.0, z: 0.0 }, Point3 { x: 1.0, y: 1.0, z: 0.0 }], vec![1.0; 4], (2, 2), vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0])
    }
    fn vertical_non_unit_domain() -> NurbsSurface3DDefinition {
        NurbsSurface3DDefinition::new((1, 1), vec![Point3 { x: 0.5, y: 0.0, z: -1.0 }, Point3 { x: 0.5, y: 0.0, z: 1.0 }, Point3 { x: 0.5, y: 1.0, z: -1.0 }, Point3 { x: 0.5, y: 1.0, z: 1.0 }], vec![1.0; 4], (2, 2), vec![2.0, 2.0, 4.0, 4.0], vec![10.0, 10.0, 20.0, 20.0])
    }
    #[test]
    fn transverse_patches_have_bounded_intersection() {
        let r = intersect_planar_nurbs_surfaces(&horizontal(), &vertical_non_unit_domain(), 1e-10).unwrap();
        assert_eq!(r.status, IntersectionStatus::Unique);
        assert_eq!(r.segments.len(), 1);
        let s = r.segments[0];
        assert!((s.start.point.x - 0.5).abs() < 1e-10);
        assert!((s.end.point.x - 0.5).abs() < 1e-10);
        assert!((s.start.first_uv.0 - 0.5).abs() < 1e-10);
        assert!((s.start.second_uv.0 - 2.0).abs() < 1e-10);
    }
    #[test]
    fn parallel_disjoint_patches_are_no_intersection() {
        let mut q = horizontal();
        for p in &mut q.control_points { p.z = 1.0; }
        assert_eq!(intersect_planar_nurbs_surfaces(&horizontal(), &q, 1e-10).unwrap().status, IntersectionStatus::NoIntersection);
    }
    #[test]
    fn coincident_patches_are_explicitly_underdetermined() {
        assert_eq!(intersect_planar_nurbs_surfaces(&horizontal(), &horizontal(), 1e-10), Err(SurfaceSurfaceIntersectionError::CoincidentOrUnderdetermined));
    }
}
