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
    if !det.is_finite() || det.abs() <= 1e-14 * scale * scale {
        return None;
    }
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

pub fn closest_point_on_nurbs_surface(
    point: Point3,
    surface: &NurbsSurface3DDefinition,
    tolerance: f64,
) -> Result<GeneralPointSurfaceClosestPointResult, GeneralPointSurfaceClosestPointError> {
    if [point.x, point.y, point.z].iter().any(|v| !v.is_finite())
        || !tolerance.is_finite()
        || tolerance < 0.0
    {
        return Err(GeneralPointSurfaceClosestPointError::NonFinite);
    }
    surface.validate().map_err(|_| GeneralPointSurfaceClosestPointError::InvalidSurface)?;
    let ((u0, u1), (v0, v1)) = surface
        .parameter_domain()
        .map_err(|_| GeneralPointSurfaceClosestPointError::InvalidSurface)?;
    let scale = surface
        .control_points
        .iter()
        .map(|p| norm(*p))
        .fold(1.0_f64, f64::max)
        .max(norm(point));
    let residual_tol = tolerance.max(1e-11 * scale);
    let param_tol_u = ((u1 - u0).abs() * 1e-12).max(1e-13);
    let param_tol_v = ((v1 - v0).abs() * 1e-12).max(1e-13);

    let mut seeds = Vec::new();
    for iu in 0..7usize {
        for iv in 0..7usize {
            seeds.push((
                u0 + (u1 - u0) * iu as f64 / 6.0,
                v0 + (v1 - v0) * iv as f64 / 6.0,
            ));
        }
    }
    let mut candidates: Vec<GeneralPointSurfaceClosestPoint> = Vec::new();

    for (mut u, mut v) in seeds {
        let mut converged = false;
        for _ in 0..60 {
            let d = match surface.differential_at(u, v) {
                Ok(value) => value,
                Err(error) => {
                    if matches!(error, NurbsSurfaceEvaluationError::OutOfDomain) {
                        break;
                    }
                    if matches!(error, NurbsSurfaceEvaluationError::InsufficientContinuity) {
                        return Err(map_error(error));
                    }
                    break;
                }
            };
            let r = sub(d.point, point);
            let f = 0.5 * dot(r, r);
            if !f.is_finite() {
                break;
            }
            let g0 = dot(d.du, r);
            let g1 = dot(d.dv, r);
            let grad_norm = g0.hypot(g1);
            if grad_norm <= residual_tol.max(1e-12 * scale) && norm(r) <= residual_tol {
                converged = true;
                break;
            }

            let h00 = dot(d.du, d.du) + dot(r, d.duu);
            let h01 = dot(d.du, d.dv) + dot(r, d.duv);
            let h11 = dot(d.dv, d.dv) + dot(r, d.dvv);
            let Some((du, dv)) = solve2(h00, h01, h11, -g0, -g1) else {
                break;
            };

            let mut accepted = false;
            let mut alpha = 1.0;
            for _ in 0..12 {
                let next_u = (u + alpha * du).clamp(u0, u1);
                let next_v = (v + alpha * dv).clamp(v0, v1);
                let next = match surface.differential_at(next_u, next_v) {
                    Ok(value) => value,
                    Err(NurbsSurfaceEvaluationError::InsufficientContinuity) => {
                        return Err(GeneralPointSurfaceClosestPointError::InsufficientContinuity)
                    }
                    Err(_) => {
                        alpha *= 0.5;
                        continue;
                    }
                };
                let next_r = sub(next.point, point);
                let next_f = 0.5 * dot(next_r, next_r);
                if next_f.is_finite() && next_f <= f {
                    u = next_u;
                    v = next_v;
                    accepted = true;
                    break;
                }
                alpha *= 0.5;
            }
            if !accepted || (du.abs() <= param_tol_u && dv.abs() <= param_tol_v) {
                if accepted {
                    let d = surface.differential_at(u, v).map_err(map_error)?;
                    let r = sub(d.point, point);
                    if norm(r) <= residual_tol || dot(d.du, r).hypot(dot(d.dv, r)) <= residual_tol.max(1e-12 * scale) {
                        converged = true;
                    }
                }
                break;
            }
        }

        if converged {
            let d = surface.differential_at(u, v).map_err(map_error)?;
            let r = sub(d.point, point);
            let distance = norm(r);
            if !distance.is_finite() {
                return Err(GeneralPointSurfaceClosestPointError::NumericalFailure);
            }
            let candidate = GeneralPointSurfaceClosestPoint { point: d.point, surface_uv: (u, v), distance };
            let merge_tol = residual_tol.max(1e-9 * scale);
            if candidates.iter().all(|c| {
                norm(sub(c.point, candidate.point)) > merge_tol
                    || (c.surface_uv.0 - u).abs() > param_tol_u * 10.0
                    || (c.surface_uv.1 - v).abs() > param_tol_v * 10.0
            }) {
                candidates.push(candidate);
            }
        }
    }

    if candidates.is_empty() {
        return Err(GeneralPointSurfaceClosestPointError::NoConvergedCandidate);
    }
    candidates.sort_by(|a, b| {
        a.distance
            .total_cmp(&b.distance)
            .then(a.surface_uv.0.total_cmp(&b.surface_uv.0))
            .then(a.surface_uv.1.total_cmp(&b.surface_uv.1))
    });
    let best = candidates[0];
    let ambiguity_tol = residual_tol.max(1e-9 * scale);
    let near_ties = candidates
        .iter()
        .skip(1)
        .filter(|candidate| (candidate.distance - best.distance).abs() <= ambiguity_tol)
        .count();
    Ok(GeneralPointSurfaceClosestPointResult {
        status: if near_ties == 0 { IntersectionStatus::Unique } else { IntersectionStatus::Ambiguous },
        closest: best,
        stationary_candidates: candidates.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn planar_bilinear() -> NurbsSurface3DDefinition {
        NurbsSurface3DDefinition::new(
            (1, 1),
            vec![
                Point3 { x: 0.0, y: 0.0, z: 0.0 },
                Point3 { x: 0.0, y: 1.0, z: 0.0 },
                Point3 { x: 2.0, y: 0.0, z: 0.0 },
                Point3 { x: 2.0, y: 1.0, z: 0.0 },
            ],
            vec![1.0; 4],
            (2, 2),
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
        )
    }

    #[test]
    fn interior_projection_on_general_surface_is_exact() {
        let r = closest_point_on_nurbs_surface(
            Point3 { x: 0.5, y: 0.25, z: 3.0 },
            &planar_bilinear(),
            1e-10,
        )
        .unwrap();
        assert_eq!(r.status, IntersectionStatus::Unique);
        assert!((r.closest.surface_uv.0 - 0.25).abs() < 1e-10);
        assert!((r.closest.surface_uv.1 - 0.25).abs() < 1e-10);
        assert!((r.closest.distance - 3.0).abs() < 1e-10);
    }

    #[test]
    fn arbitrary_parameter_domain_is_preserved() {
        let mut s = planar_bilinear();
        s.knots_u = vec![2.0, 2.0, 4.0, 4.0];
        s.knots_v = vec![10.0, 10.0, 20.0, 20.0];
        let r = closest_point_on_nurbs_surface(
            Point3 { x: 0.5, y: 0.25, z: 2.0 },
            &s,
            1e-10,
        )
        .unwrap();
        assert!((r.closest.surface_uv.0 - 2.5).abs() < 1e-10);
        assert!((r.closest.surface_uv.1 - 12.5).abs() < 1e-10);
    }

    #[test]
    fn boundary_projection_is_supported() {
        let r = closest_point_on_nurbs_surface(
            Point3 { x: 4.0, y: 0.5, z: 0.0 },
            &planar_bilinear(),
            1e-10,
        )
        .unwrap();
        assert!((r.closest.surface_uv.0 - 1.0).abs() < 1e-10);
        assert!((r.closest.surface_uv.1 - 0.5).abs() < 1e-10);
        assert!(r.closest.distance.abs() < 1e-10);
    }

    #[test]
    fn invalid_input_fails_closed() {
        let s = planar_bilinear();
        assert_eq!(
            closest_point_on_nurbs_surface(
                Point3 { x: f64::NAN, y: 0.0, z: 0.0 },
                &s,
                1e-10,
            ),
            Err(GeneralPointSurfaceClosestPointError::NonFinite)
        );
    }
}
