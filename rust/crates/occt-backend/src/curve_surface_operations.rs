use umlcad_v6_nurbs_surface_api::NurbsSurface3DDefinition;
use umlcad_v6_surface_operations_api::{
    intersect_line_segment_nurbs_surface, CurveSurfaceOperations, IntersectionStatus,
    LineSegment3D, LineSurfaceIntersectionError, LineSurfaceIntersectionPoint,
    LineSurfaceIntersectionResult,
};

use crate::{OcctBackend, OCCT_OK};

unsafe extern "C" {
    fn umlcad_occt_line_surface_intersection(
        poles_xyz: *const f64,
        count_u: u32,
        count_v: u32,
        weights: *const f64,
        knots_u: *const f64,
        knot_count_u: u32,
        knots_v: *const f64,
        knot_count_v: u32,
        degree_u: u32,
        degree_v: u32,
        start_xyz: *const f64,
        end_xyz: *const f64,
        tolerance: f64,
        out_values: *mut f64,
        capacity: u32,
        out_count: *mut u32,
        out_tangent: *mut i32,
    ) -> i32;
}

impl CurveSurfaceOperations for OcctBackend {
    fn intersect_line_segment_nurbs_surface(
        &self,
        line: LineSegment3D,
        surface: &NurbsSurface3DDefinition,
        tolerance: f64,
    ) -> Result<LineSurfaceIntersectionResult, LineSurfaceIntersectionError> {
        line.validate()?;
        if !tolerance.is_finite() || tolerance < 0.0 {
            return Err(LineSurfaceIntersectionError::NonFinite);
        }
        surface
            .validate()
            .map_err(|_| LineSurfaceIntersectionError::InvalidSurface)?;

        let expected = intersect_line_segment_nurbs_surface(line, surface, tolerance)?;
        let mut poles_xyz = Vec::with_capacity(surface.control_points.len() * 3);
        for point in &surface.control_points {
            poles_xyz.extend([point.x, point.y, point.z]);
        }
        let start = [line.start.x, line.start.y, line.start.z];
        let end = [line.end.x, line.end.y, line.end.z];
        const CAPACITY: u32 = 64;
        let mut values = vec![0.0f64; CAPACITY as usize * 6];
        let mut count = 0u32;
        let mut tangent = 0i32;
        let status = unsafe {
            umlcad_occt_line_surface_intersection(
                poles_xyz.as_ptr(),
                surface.count_u as u32,
                surface.count_v as u32,
                surface.weights.as_ptr(),
                surface.knots_u.as_ptr(),
                surface.knots_u.len() as u32,
                surface.knots_v.as_ptr(),
                surface.knots_v.len() as u32,
                surface.degree_u as u32,
                surface.degree_v as u32,
                start.as_ptr(),
                end.as_ptr(),
                tolerance,
                values.as_mut_ptr(),
                CAPACITY,
                &mut count,
                &mut tangent,
            )
        };
        if status != OCCT_OK {
            return Err(LineSurfaceIntersectionError::NumericalFailure);
        }
        if tangent != 0 {
            return Err(LineSurfaceIntersectionError::TangentialContact);
        }
        if count > CAPACITY {
            return Err(LineSurfaceIntersectionError::NumericalFailure);
        }

        let direction = LineSegment3D { start: line.start, end: line.end };
        let dx = direction.end.x - direction.start.x;
        let dy = direction.end.y - direction.start.y;
        let dz = direction.end.z - direction.start.z;
        let denom = dx * dx + dy * dy + dz * dz;
        let native: Vec<LineSurfaceIntersectionPoint> = (0..count as usize)
            .map(|i| {
                let offset = 6 * i;
                let point = umlcad_v6_nurbs_surface_api::Point3 {
                    x: values[offset + 3],
                    y: values[offset + 4],
                    z: values[offset + 5],
                };
                let t = ((point.x - line.start.x) * dx
                    + (point.y - line.start.y) * dy
                    + (point.z - line.start.z) * dz)
                    / denom;
                LineSurfaceIntersectionPoint {
                    line_parameter: t,
                    u: values[offset],
                    v: values[offset + 1],
                    point,
                }
            })
            .collect();

        if native.len() != expected.points.len() {
            return Err(LineSurfaceIntersectionError::NumericalFailure);
        }
        let comparison_tol = tolerance.max(1e-9) * 20.0;
        for native_point in &native {
            if expected.points.iter().all(|expected_point| {
                (expected_point.point.x - native_point.point.x)
                    .hypot((expected_point.point.y - native_point.point.y)
                        .hypot(expected_point.point.z - native_point.point.z))
                    > comparison_tol
            }) {
                return Err(LineSurfaceIntersectionError::NumericalFailure);
            }
        }

        let status = match native.len() {
            0 => IntersectionStatus::NoIntersection,
            1 => IntersectionStatus::Unique,
            _ => IntersectionStatus::Ambiguous,
        };
        Ok(LineSurfaceIntersectionResult { status, points: native })
    }
}
