use super::{NativeShape, OcctBackend, OcctShape, OCCT_OK};
use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, GeometryEvidence, GeometryKind, GeometryResult, GeometryStatus, ToleranceContext};
use umlcad_v6_nurbs_surface_api::{NurbsSurface3DDefinition, NurbsSurfaceBackend};

unsafe extern "C" {
    fn umlcad_occt_nurbs_surface3d(
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
        face_tolerance: f64,
        out_shape: *mut *mut NativeShape,
    ) -> i32;
}

impl NurbsSurfaceBackend for OcctBackend {
    fn nurbs_surface3d(
        &self,
        definition: &NurbsSurface3DDefinition,
        tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        tolerance.validate()?;
        definition
            .validate()
            .map_err(|_| GeometryError::InvalidInput("invalid NURBS surface definition"))?;

        let mut poles_xyz = Vec::with_capacity(definition.control_points.len() * 3);
        for point in &definition.control_points {
            poles_xyz.extend([point.x, point.y, point.z]);
        }

        let mut raw = std::ptr::null_mut();
        let status = unsafe {
            umlcad_occt_nurbs_surface3d(
                poles_xyz.as_ptr(),
                definition.count_u as u32,
                definition.count_v as u32,
                definition.weights.as_ptr(),
                definition.knots_u.as_ptr(),
                definition.knots_u.len() as u32,
                definition.knots_v.as_ptr(),
                definition.knots_v.len() as u32,
                definition.degree_u as u32,
                definition.degree_v as u32,
                tolerance.modeling,
                &mut raw,
            )
        };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT NURBS surface construction failed"));
        }

        let raw = std::ptr::NonNull::new(raw)
            .ok_or(GeometryError::Unsupported("OCCT returned null"))?;
        Ok(GeometryResult {
            shape: OcctShape::from_raw(raw, GeometryKind::Surface),
            kind: GeometryKind::Surface,
            evidence: GeometryEvidence {
                status: GeometryStatus::Success,
                backend: self.backend_name(),
                tolerance,
                message: None,
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use umlcad_v6_geometry_api::GeometryBackend;
    use umlcad_v6_nurbs_surface_api::Point3;

    const TOLERANCE: ToleranceContext = ToleranceContext {
        modeling: 1e-9,
        validation: 1e-9,
    };

    fn bilinear() -> NurbsSurface3DDefinition {
        NurbsSurface3DDefinition::new(
            1,
            1,
            vec![
                Point3 { x: 0.0, y: 0.0, z: 0.0 },
                Point3 { x: 0.0, y: 1.0, z: 1.0 },
                Point3 { x: 1.0, y: 0.0, z: 1.0 },
                Point3 { x: 1.0, y: 1.0, z: 2.0 },
            ],
            vec![1.0; 4],
            2,
            2,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
        )
    }

    #[test]
    fn bilinear_surface_is_a_valid_surface() {
        let backend = OcctBackend::new();
        let result = backend.nurbs_surface3d(&bilinear(), TOLERANCE).unwrap();
        assert_eq!(result.kind, GeometryKind::Surface);
        assert!(backend.validate(&result.shape, TOLERANCE).unwrap().valid);
        let counts = backend.topology_counts(&result.shape, TOLERANCE).unwrap();
        assert_eq!(counts.faces, 1);
    }

    #[test]
    fn rational_surface_is_deterministic() {
        let backend = OcctBackend::new();
        let mut definition = bilinear();
        definition.weights[3] = 2.0;
        let first = backend.nurbs_surface3d(&definition, TOLERANCE).unwrap().shape;
        let second = backend.nurbs_surface3d(&definition, TOLERANCE).unwrap().shape;
        assert_eq!(backend.bounding_box(&first, TOLERANCE).unwrap(), backend.bounding_box(&second, TOLERANCE).unwrap());
        assert_eq!(backend.topology_counts(&first, TOLERANCE).unwrap(), backend.topology_counts(&second, TOLERANCE).unwrap());
    }

    #[test]
    fn invalid_surface_definition_fails_before_native_construction() {
        let backend = OcctBackend::new();
        let mut definition = bilinear();
        definition.weights[0] = 0.0;
        assert_eq!(
            backend.nurbs_surface3d(&definition, TOLERANCE),
            Err(GeometryError::InvalidInput("invalid NURBS surface definition"))
        );
    }
}
