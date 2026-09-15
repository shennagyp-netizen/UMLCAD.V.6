use crate::{
    IntersectionStatus, LineSegment3D, LineSurfaceIntersectionError,
    LineSurfaceIntersectionPoint, LineSurfaceIntersectionResult,
};
use umlcad_v6_nurbs_surface_api::{NurbsSurface3DDefinition, Point3};

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
fn cross(a: Point3, b: Point3) -> Point3 {
    Point3 { x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }
}
fn norm(a: Point3) -> f64 { dot(a, a).sqrt() }

pub(crate) fn classify_line_against_planar_patch(
    line: LineSegment3D,
    surface: &NurbsSurface3DDefinition,
    tolerance: f64,
) -> Option<Result<LineSurfaceIntersectionResult, LineSurfaceIntersectionError>> {
    if surface.degree_u != 1
        || surface.degree_v != 1
        || surface.count_u != 2
        || surface.count_v != 2
        || surface.weights.iter().any(|w| (*w - 1.0).abs() > 1e-14)
    {
        return None;
    }
    let ((u0, u1), (v0, v1)) = match surface.parameter_domain() {
        Ok(domain) => domain,
        Err(_) => return None,
    };
    let du = scale(
        sub(surface.control_points[2], surface.control_points[0]),
        1.0 / (u1 - u0),
    );
    let dv = scale(
        sub(surface.control_points[1], surface.control_points[0]),
        1.0 / (v1 - v0),
    );
    let normal = cross(du, dv);
    let normal_norm = norm(normal);
    if !normal_norm.is_finite() || normal_norm == 0.0 {
        return None;
    }
    let direction = sub(line.end, line.start);
    let denominator = dot(normal, direction);
    let plane_offset = dot(normal, sub(line.start, surface.control_points[0]));
    let scale_ref = normal_norm.max(1.0) * norm(direction).max(1.0);
    let eps = tolerance.max(1e-12) * scale_ref;
    if denominator.abs() <= eps {
        if plane_offset.abs() <= eps {
            return Some(Err(LineSurfaceIntersectionError::CoincidentOrUnderdetermined));
        }
        return Some(Ok(LineSurfaceIntersectionResult {
            status: IntersectionStatus::NoIntersection,
            points: Vec::new(),
        }));
    }
    let t = -plane_offset / denominator;
    if t < -tolerance || t > 1.0 + tolerance {
        return Some(Ok(LineSurfaceIntersectionResult {
            status: IntersectionStatus::NoIntersection,
            points: Vec::new(),
        }));
    }
    let t = t.clamp(0.0, 1.0);
    let point = add(line.start, scale(direction, t));
    let r = sub(point, surface.control_points[0]);
    let aa = dot(du, du);
    let ab = dot(du, dv);
    let bb = dot(dv, dv);
    let det = aa * bb - ab * ab;
    if !det.is_finite() || det <= 1e-24 * aa.max(bb).max(1.0).powi(2) {
        return Some(Err(LineSurfaceIntersectionError::NumericalFailure));
    }
    let ru = dot(du, r);
    let rv = dot(dv, r);
    let su = (ru * bb - rv * ab) / det;
    let sv = (rv * aa - ru * ab) / det;
    if su < -tolerance || su > 1.0 + tolerance || sv < -tolerance || sv > 1.0 + tolerance {
        return Some(Ok(LineSurfaceIntersectionResult {
            status: IntersectionStatus::NoIntersection,
            points: Vec::new(),
        }));
    }
    let u = (u0 + su * (u1 - u0)).clamp(u0, u1);
    let v = (v0 + sv * (v1 - v0)).clamp(v0, v1);
    Some(Ok(LineSurfaceIntersectionResult {
        status: IntersectionStatus::Unique,
        points: vec![LineSurfaceIntersectionPoint {
            line_parameter: t,
            u,
            v,
            point,
        }],
    }))
}
