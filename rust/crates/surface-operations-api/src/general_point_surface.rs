use crate::IntersectionStatus;
use umlcad_v6_nurbs_surface_api::{NurbsSurface3DDefinition, NurbsSurfaceEvaluationError, Point3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeneralPointSurfaceClosestPoint {
    pub point: Point3,
    pub surface_uv: (f64, f64),
    pub distance: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GeneralPointSurfaceClosestPointResult {
    pub status: IntersectionStatus,
    pub closest: GeneralPointSurfaceClosestPoint,
    pub stationary_candidates: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeneralPointSurfaceClosestPointError {
    NonFinite,
    InvalidSurface,
    NumericalFailure,
    InsufficientContinuity,
    NoConvergedCandidate,
}

fn sub(a: Point3, b: Point3) -> Point3 {
    Point3 { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }
}

fn dot(a: Point3, b: Point3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn norm(a: Point3) -> f64 {
    a.norm()
}

fn solve2(a00: f64, a01: f64, a11: f64, b0: f64, b1: f64) -> Option<(f64, f64)> {
    let det = a00 * a11 - a01 * a01;
    let scale = a00.abs().max(a01.abs()).max(a11.abs()).max(1.0);
    if !det.is_finite() || det.abs() <= 1e-14 * scale * scale { return None; }
    let du = (b0 * a11 - a01 * b1) / det;
    let dv = (a00 * b1 - b0 * a01) / det;
    if du.is_finite() && dv.is_finite() { Some((du, dv)) } else { None }
}

fn map_error(error: NurbsSurfaceEvaluationError) -> GeneralPointSurfaceClosestPointError {
    match error {
        NurbsSurfaceEvaluationError::InsufficientContinuity => GeneralPointSurfaceClosestPointError::InsufficientContinuity,
        NurbsSurfaceEvaluationError::OutOfDomain => GeneralPointSurfaceClosestPointError::NoConvergedCandidate,
        _ => GeneralPointSurfaceClosestPointError::NumericalFailure,
    }
}

fn kkt_stationary(g: f64, parameter: f64, lower: f64, upper: f64, tolerance: f64) -> bool {
    if (parameter - lower).abs() <= tolerance { g >= -tolerance }
    else if (parameter - upper).abs() <= tolerance { g <= tolerance }
    else { g.abs() <= tolerance }
}

fn add_candidate(
    candidates: &mut Vec<GeneralPointSurfaceClosestPoint>,
    candidate: GeneralPointSurfaceClosestPoint,
    scale: f64,
    residual_tol: f64,
    param_tol_u: f64,
    param_tol_v: f64,
) {
    let merge_tol = residual_tol.max(1e-9 * scale);
    if candidates.iter().all(|c| {
        norm(sub(c.point, candidate.point)) > merge_tol
            || (c.surface_uv.0 - candidate.surface_uv.0).abs() > param_tol_u * 10.0
            || (c.surface_uv.1 - candidate.surface_uv.1).abs() > param_tol_v * 10.0
    }) {
        candidates.push(candidate);
    }
}

fn boundary_candidate(
    point: Point3,
    surface: &NurbsSurface3DDefinition,
    fixed_u: Option<f64>,
    fixed_v: Option<f64>,
    seed: f64,
    lower: f64,
    upper: f64,
    residual_tol: f64,
    scale: f64,
) -> Result<Option<GeneralPointSurfaceClosestPoint>, GeneralPointSurfaceClosestPointError> {
    let mut x = seed.clamp(lower, upper);
    let parameter_tol = ((upper - lower).abs() * 1e-12).max(1e-13);
    for _ in 0..60 {
        let (u, v) = match (fixed_u, fixed_v) {
            (Some(u), None) => (u, x),
            (None, Some(v)) => (x, v),
            _ => return Ok(None),
        };
        let d = match surface.differential_at(u, v) {
            Ok(value) => value,
            Err(NurbsSurfaceEvaluationError::InsufficientContinuity) => {
                return Err(GeneralPointSurfaceClosestPointError::InsufficientContinuity)
            }
            Err(NurbsSurfaceEvaluationError::OutOfDomain) => return Ok(None),
            Err(_) => return Ok(None),
        };
        let r = sub(d.point, point);
        let tangent = if fixed_u.is_some() { d.dv } else { d.du };
        let second = if fixed_u.is_some() { d.dvv } else { d.duu };
        let g = dot(tangent, r);
        let h = dot(tangent, tangent) + dot(r, second);
        let g_tol = residual_tol.max(1e-12 * scale);
        if kkt_stationary(g, x, lower, upper, parameter_tol)
            || (g.abs() <= g_tol && x > lower + parameter_tol && x < upper - parameter_tol)
        {
            let distance = norm(r);
            if distance.is_finite() { return Ok(Some(GeneralPointSurfaceClosestPoint { point: d.point, surface_uv: (u, v), distance })); }
            return Ok(None);
        }
        if !h.is_finite() || h <= 1e-18 * scale.max(1.0) {
            break;
        }
        let mut next = x - g / h;
        if !next.is_finite() { break; }
        next = next.clamp(lower, upper);
        if (next - x).abs() <= parameter_tol {
            x = next;
            continue;
        }
        let current_f = 0.5 * dot(r, r);
        let (nu, nv) = match (fixed_u, fixed_v) {
            (Some(u), None) => (u, next),
            (None, Some(v)) => (next, v),
            _ => unreachable!(),
        };
        let nd = match surface.differential_at(nu, nv) {
            Ok(value) => value,
            Err(NurbsSurfaceEvaluationError::InsufficientContinuity) => return Err(GeneralPointSurfaceClosestPointError::InsufficientContinuity),
            Err(_) => break,
        };
        let nr = sub(nd.point, point);
        let next_f = 0.5 * dot(nr, nr);
        if next_f.is_finite() && next_f <= current_f {
            x = next;
        } else {
            x = 0.5 * (x + next);
        }
    }
    let (u, v) = match (fixed_u, fixed_v) {
        (Some(u), None) => (u, x),
        (None, Some(v)) => (x, v),
        _ => return Ok(None),
    };
    let d = surface.differential_at(u, v).map_err(map_error)?;
    let r = sub(d.point, point);
    let tangent = if fixed_u.is_some() { d.dv } else { d.du };
    let g = dot(tangent, r);
    let distance = norm(r);
    if distance.is_finite() && kkt_stationary(g, x, lower, upper, parameter_tol) {
        Ok(Some(GeneralPointSurfaceClosestPoint { point: d.point, surface_uv: (u, v), distance }))
    } else { Ok(None) }
}

pub fn closest_point_on_nurbs_surface(
    point: Point3,
    surface: &NurbsSurface3DDefinition,
    tolerance: f64,
) -> Result<GeneralPointSurfaceClosestPointResult, GeneralPointSurfaceClosestPointError> {
    if [point.x, point.y, point.z].iter().any(|v| !v.is_finite()) || !tolerance.is_finite() || tolerance < 0.0 {
        return Err(GeneralPointSurfaceClosestPointError::NonFinite);
    }
    surface.validate().map_err(|_| GeneralPointSurfaceClosestPointError::InvalidSurface)?;
    let ((u0, u1), (v0, v1)) = surface.parameter_domain().map_err(|_| GeneralPointSurfaceClosestPointError::InvalidSurface)?;
    let scale = surface.control_points.iter().map(|p| norm(*p)).fold(1.0_f64, f64::max).max(norm(point));
    let residual_tol = tolerance.max(1e-11 * scale);
    let param_tol_u = ((u1 - u0).abs() * 1e-12).max(1e-13);
    let param_tol_v = ((v1 - v0).abs() * 1e-12).max(1e-13);
    let mut candidates: Vec<GeneralPointSurfaceClosestPoint> = Vec::new();

    // Interior stationary points. Only accept an interior point when its full gradient is small.
    for iu in 0..7usize {
        for iv in 0..7usize {
            let mut u = u0 + (u1 - u0) * iu as f64 / 6.0;
            let mut v = v0 + (v1 - v0) * iv as f64 / 6.0;
            let mut converged = false;
            for _ in 0..60 {
                let d = match surface.differential_at(u, v) {
                    Ok(value) => value,
                    Err(NurbsSurfaceEvaluationError::InsufficientContinuity) => return Err(GeneralPointSurfaceClosestPointError::InsufficientContinuity),
                    Err(_) => break,
                };
                let r = sub(d.point, point);
                let g0 = dot(d.du, r);
                let g1 = dot(d.dv, r);
                if u > u0 + param_tol_u && u < u1 - param_tol_u && v > v0 + param_tol_v && v < v1 - param_tol_v && g0.hypot(g1) <= residual_tol.max(1e-12 * scale) {
                    converged = true;
                    break;
                }
                if u <= u0 + param_tol_u || u >= u1 - param_tol_u || v <= v0 + param_tol_v || v >= v1 - param_tol_v { break; }
                let h00 = dot(d.du, d.du) + dot(r, d.duu);
                let h01 = dot(d.du, d.dv) + dot(r, d.duv);
                let h11 = dot(d.dv, d.dv) + dot(r, d.dvv);
                let Some((du, dv)) = solve2(h00, h01, h11, -g0, -g1) else { break; };
                let mut accepted = false;
                let current_f = 0.5 * dot(r, r);
                let mut alpha = 1.0;
                for _ in 0..12 {
                    let next_u = (u + alpha * du).clamp(u0, u1);
                    let next_v = (v + alpha * dv).clamp(v0, v1);
                    if next_u <= u0 + param_tol_u || next_u >= u1 - param_tol_u || next_v <= v0 + param_tol_v || next_v >= v1 - param_tol_v { alpha *= 0.5; continue; }
                    let next = match surface.differential_at(next_u, next_v) {
                        Ok(value) => value,
                        Err(NurbsSurfaceEvaluationError::InsufficientContinuity) => return Err(GeneralPointSurfaceClosestPointError::InsufficientContinuity),
                        Err(_) => { alpha *= 0.5; continue; }
                    };
                    let next_r = sub(next.point, point);
                    let next_f = 0.5 * dot(next_r, next_r);
                    if next_f.is_finite() && next_f <= current_f { u = next_u; v = next_v; accepted = true; break; }
                    alpha *= 0.5;
                }
                if !accepted { break; }
            }
            if converged {
                let d = surface.differential_at(u, v).map_err(map_error)?;
                let r = sub(d.point, point);
                let candidate = GeneralPointSurfaceClosestPoint { point: d.point, surface_uv: (u, v), distance: norm(r) };
                if candidate.distance.is_finite() { add_candidate(&mut candidates, candidate, scale, residual_tol, param_tol_u, param_tol_v); }
            }
        }
    }

    // Constrained minima on each parametric edge. This is separate from the interior solve so an active bound
    // is handled by the correct one-dimensional KKT condition instead of pretending the two-dimensional gradient is zero.
    for seed_index in 0..9usize {
        let seed = if seed_index == 8 { 0.5 } else { seed_index as f64 / 8.0 };
        for fixed_u in [u0, u1] {
            if let Some(candidate) = boundary_candidate(point, surface, Some(fixed_u), None, v0 + seed * (v1 - v0), v0, v1, residual_tol, scale)? {
                add_candidate(&mut candidates, candidate, scale, residual_tol, param_tol_u, param_tol_v);
            }
        }
        for fixed_v in [v0, v1] {
            if let Some(candidate) = boundary_candidate(point, surface, None, Some(fixed_v), u0 + seed * (u1 - u0), u0, u1, residual_tol, scale)? {
                add_candidate(&mut candidates, candidate, scale, residual_tol, param_tol_u, param_tol_v);
            }
        }
    }

    // Corners are feasible constrained candidates regardless of first-derivative direction.
    for u in [u0, u1] {
        for v in [v0, v1] {
            let d = surface.differential_at(u, v).map_err(map_error)?;
            let distance = norm(sub(d.point, point));
            if distance.is_finite() { add_candidate(&mut candidates, GeneralPointSurfaceClosestPoint { point: d.point, surface_uv: (u, v), distance }, scale, residual_tol, param_tol_u, param_tol_v); }
        }
    }

    if candidates.is_empty() { return Err(GeneralPointSurfaceClosestPointError::NoConvergedCandidate); }
    candidates.sort_by(|a, b| a.distance.total_cmp(&b.distance).then(a.surface_uv.0.total_cmp(&b.surface_uv.0)).then(a.surface_uv.1.total_cmp(&b.surface_uv.1)));
    let best = candidates[0];
    let ambiguity_tol = residual_tol.max(1e-9 * scale);
    let near_ties = candidates.iter().skip(1).filter(|candidate| (candidate.distance - best.distance).abs() <= ambiguity_tol).count();
    Ok(GeneralPointSurfaceClosestPointResult { status: if near_ties == 0 { IntersectionStatus::Unique } else { IntersectionStatus::Ambiguous }, closest: best, stationary_candidates: candidates.len() })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn planar_bilinear() -> NurbsSurface3DDefinition {
        NurbsSurface3DDefinition::new((1, 1), vec![Point3 { x: 0.0, y: 0.0, z: 0.0 }, Point3 { x: 0.0, y: 1.0, z: 0.0 }, Point3 { x: 2.0, y: 0.0, z: 0.0 }, Point3 { x: 2.0, y: 1.0, z: 0.0 }], vec![1.0; 4], (2, 2), vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0])
    }
    #[test]
    fn interior_projection_on_general_surface_is_exact() {
        let r = closest_point_on_nurbs_surface(Point3 { x: 0.5, y: 0.25, z: 3.0 }, &planar_bilinear(), 1e-10).unwrap();
        assert_eq!(r.status, IntersectionStatus::Unique); assert!((r.closest.surface_uv.0 - 0.25).abs() < 1e-10); assert!((r.closest.surface_uv.1 - 0.25).abs() < 1e-10); assert!((r.closest.distance - 3.0).abs() < 1e-10);
    }
    #[test]
    fn arbitrary_parameter_domain_is_preserved() {
        let mut s = planar_bilinear(); s.knots_u = vec![2.0, 2.0, 4.0, 4.0]; s.knots_v = vec![10.0, 10.0, 20.0, 20.0];
        let r = closest_point_on_nurbs_surface(Point3 { x: 0.5, y: 0.25, z: 2.0 }, &s, 1e-10).unwrap();
        assert!((r.closest.surface_uv.0 - 2.5).abs() < 1e-10); assert!((r.closest.surface_uv.1 - 12.5).abs() < 1e-10);
    }
    #[test]
    fn boundary_projection_is_supported() {
        let r = closest_point_on_nurbs_surface(Point3 { x: 4.0, y: 0.5, z: 0.0 }, &planar_bilinear(), 1e-10).unwrap();
        assert!((r.closest.surface_uv.0 - 1.0).abs() < 1e-10); assert!((r.closest.surface_uv.1 - 0.5).abs() < 1e-10); assert!(r.closest.distance.abs() < 1e-10);
    }
    #[test]
    fn invalid_input_fails_closed() {
        let s = planar_bilinear();
        assert_eq!(closest_point_on_nurbs_surface(Point3 { x: f64::NAN, y: 0.0, z: 0.0 }, &s, 1e-10), Err(GeneralPointSurfaceClosestPointError::NonFinite));
    }
}
