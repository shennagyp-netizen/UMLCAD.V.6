use crate::{
    classify_nurbs_surface_pair_relation, intersect_planar_nurbs_surfaces,
    IntersectionStatus, NurbsSurface3DDefinition, NurbsSurfacePairRelation,
    SurfaceSurfaceIntersectionError, SurfaceSurfaceIntersectionResult,
};

/// Intersects two NURBS surfaces with a conservative general-family front end.
///
/// The positive-weight NURBS control-net convex-hull property makes a separated
/// control-net AABB a valid certificate that the represented surfaces cannot
/// intersect. Only the already-established exact affine-planar patch family is
/// currently solved after that gate; all other potential contacts remain
/// explicitly unsupported rather than being converted into guessed geometry.
pub fn intersect_nurbs_surfaces(
    first: &NurbsSurface3DDefinition,
    second: &NurbsSurface3DDefinition,
    tolerance: f64,
) -> Result<SurfaceSurfaceIntersectionResult, SurfaceSurfaceIntersectionError> {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(SurfaceSurfaceIntersectionError::NonFinite);
    }
    let relation = classify_nurbs_surface_pair_relation(first, second, tolerance).map_err(|error| {
        match error {
            crate::NurbsSurfacePairRelationError::InvalidFirstSurface => {
                SurfaceSurfaceIntersectionError::InvalidSurface
            }
            crate::NurbsSurfacePairRelationError::InvalidSecondSurface => {
                SurfaceSurfaceIntersectionError::InvalidSurface
            }
            crate::NurbsSurfacePairRelationError::InvalidTolerance => {
                SurfaceSurfaceIntersectionError::NonFinite
            }
            crate::NurbsSurfacePairRelationError::NonFinite
            | crate::NurbsSurfacePairRelationError::NumericalFailure => {
                SurfaceSurfaceIntersectionError::NumericalFailure
            }
        }
    })?;

    match relation {
        NurbsSurfacePairRelation::DisjointCertified => Ok(SurfaceSurfaceIntersectionResult {
            status: IntersectionStatus::NoIntersection,
            segments: Vec::new(),
        }),
        NurbsSurfacePairRelation::PotentialContact => {
            intersect_planar_nurbs_surfaces(first, second, tolerance)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quadratic_patch(z: f64) -> NurbsSurface3DDefinition {
        let mut points = Vec::with_capacity(9);
        for i in 0..3 {
            for j in 0..3 {
                points.push(crate::Point3 {
                    x: i as f64 * 0.5,
                    y: j as f64 * 0.5,
                    z,
                });
            }
        }
        NurbsSurface3DDefinition::new(
            (2, 2),
            points,
            vec![1.0; 9],
            (3, 3),
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        )
    }

    #[test]
    fn separated_general_nurbs_surfaces_are_certified_disjoint() {
        let first = quadratic_patch(0.0);
        let second = quadratic_patch(10.0);
        let result = intersect_nurbs_surfaces(&first, &second, 1e-10).unwrap();
        assert_eq!(result.status, IntersectionStatus::NoIntersection);
        assert!(result.segments.is_empty());
    }

    #[test]
    fn potentially_contacting_non_planar_family_remains_explicitly_unsupported() {
        let mut second = quadratic_patch(0.0);
        second.control_points[4].z = 0.25;
        assert_eq!(
            intersect_nurbs_surfaces(&quadratic_patch(0.0), &second, 1e-10),
            Err(SurfaceSurfaceIntersectionError::UnsupportedSurfaceFamily)
        );
    }

    #[test]
    fn invalid_tolerance_fails_closed() {
        assert_eq!(
            intersect_nurbs_surfaces(&quadratic_patch(0.0), &quadratic_patch(1.0), -1.0),
            Err(SurfaceSurfaceIntersectionError::NonFinite)
        );
    }
}
