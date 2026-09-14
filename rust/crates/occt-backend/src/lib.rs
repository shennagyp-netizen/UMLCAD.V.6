use std::ptr::NonNull;

use umlcad_v6_geometry_api::{
    BoundingBox, GeometryBackend, GeometryError, GeometryEvidence, GeometryKind, GeometryResult,
    GeometryStatus, ToleranceContext, TopologyCounts, ValidationResult,
};

#[repr(C)]
struct NativeShape {
    _private: [u8; 0],
}

unsafe extern "C" {
    fn umlcad_occt_box(width: f64, depth: f64, height: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_clone(input: *const NativeShape, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_translate(
        input: *const NativeShape,
        dx: f64,
        dy: f64,
        dz: f64,
        out_shape: *mut *mut NativeShape,
    ) -> i32;
    fn umlcad_occt_shape_rotate(
        input: *const NativeShape,
        axis_x: f64,
        axis_y: f64,
        axis_z: f64,
        angle_radians: f64,
        out_shape: *mut *mut NativeShape,
    ) -> i32;
    fn umlcad_occt_shape_bounding_box(input: *const NativeShape, out_bounds: *mut f64) -> i32;
    fn umlcad_occt_shape_topology_counts(input: *const NativeShape, out_counts: *mut u32) -> i32;
    fn umlcad_occt_shape_validate(input: *const NativeShape, valid: *mut i32, manifold: *mut i32) -> i32;
    fn umlcad_occt_shape_delete(shape: *mut NativeShape);
}

const OCCT_OK: i32 = 0;
const OCCT_INVALID_ARGUMENT: i32 = 1;
const OCCT_NULL_SHAPE: i32 = 2;
const OCCT_CONSTRUCTION_FAILED: i32 = 3;
const OCCT_TRANSFORM_FAILED: i32 = 4;
const OCCT_INTERNAL_ERROR: i32 = 5;

/// Conservative minimum edge length observed for the current Ubuntu/OCCT reference build.
/// This is a backend capability limit, not UMLCAD's semantic modeling tolerance.
const OCCT_REFERENCE_MIN_BOX_EDGE: f64 = 1e-6;

pub struct OcctShape {
    raw: NonNull<NativeShape>,
}

impl Clone for OcctShape {
    fn clone(&self) -> Self {
        let mut output = std::ptr::null_mut();
        let status = unsafe { umlcad_occt_shape_clone(self.raw.as_ptr(), &mut output) };
        assert_eq!(status, OCCT_OK, "OCCT shape clone failed with status {status}");
        let raw = NonNull::new(output).expect("OCCT clone returned a null shape");
        Self { raw }
    }
}

impl Drop for OcctShape {
    fn drop(&mut self) {
        unsafe { umlcad_occt_shape_delete(self.raw.as_ptr()) };
    }
}

pub struct OcctBackend;

impl OcctBackend {
    pub const fn new() -> Self {
        Self
    }

    fn validate_dimensions(
        width: f64,
        depth: f64,
        height: f64,
        modeling_tolerance: f64,
    ) -> Result<(), GeometryError> {
        if !width.is_finite() || !depth.is_finite() || !height.is_finite() {
            return Err(GeometryError::InvalidInput("box dimensions must be finite"));
        }
        if width <= 0.0 || depth <= 0.0 || height <= 0.0 {
            return Err(GeometryError::InvalidInput("box dimensions must be positive"));
        }

        let minimum_edge = OCCT_REFERENCE_MIN_BOX_EDGE.max(modeling_tolerance);
        if width <= minimum_edge || depth <= minimum_edge || height <= minimum_edge {
            return Err(GeometryError::InvalidInput(
                "box dimensions are below the reference backend resolution",
            ));
        }
        Ok(())
    }

    fn validate_translation(dx: f64, dy: f64, dz: f64) -> Result<(), GeometryError> {
        if !dx.is_finite() || !dy.is_finite() || !dz.is_finite() {
            return Err(GeometryError::InvalidInput("translation must be finite"));
        }
        Ok(())
    }

    fn validate_rotation(
        axis_x: f64,
        axis_y: f64,
        axis_z: f64,
        angle_radians: f64,
    ) -> Result<(), GeometryError> {
        if !axis_x.is_finite()
            || !axis_y.is_finite()
            || !axis_z.is_finite()
            || !angle_radians.is_finite()
        {
            return Err(GeometryError::InvalidInput("rotation must be finite"));
        }
        if axis_x == 0.0 && axis_y == 0.0 && axis_z == 0.0 {
            return Err(GeometryError::InvalidInput("rotation axis must be non-zero"));
        }
        Ok(())
    }

    fn map_status(status: i32, operation: &'static str) -> GeometryError {
        match status {
            OCCT_INVALID_ARGUMENT => GeometryError::InvalidInput(operation),
            OCCT_NULL_SHAPE => GeometryError::InvalidInput("OCCT shape is null"),
            OCCT_CONSTRUCTION_FAILED => GeometryError::Unsupported("OCCT construction failed"),
            OCCT_TRANSFORM_FAILED => GeometryError::Unsupported("OCCT transform failed"),
            OCCT_INTERNAL_ERROR => GeometryError::Unsupported("OCCT internal failure"),
            _ => GeometryError::Unsupported("unknown OCCT status"),
        }
    }

    fn evidence(
        &self,
        status: GeometryStatus,
        tolerance: ToleranceContext,
        message: Option<String>,
    ) -> GeometryEvidence {
        GeometryEvidence {
            status,
            backend: self.backend_name(),
            tolerance,
            message,
        }
    }
}

impl Default for OcctBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl GeometryBackend for OcctBackend {
    type Shape = OcctShape;

    fn backend_name(&self) -> &'static str {
        "occt"
    }

    fn box_solid(
        &self,
        width: f64,
        depth: f64,
        height: f64,
        tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        tolerance.validate()?;
        Self::validate_dimensions(width, depth, height, tolerance.modeling)?;

        let mut output = std::ptr::null_mut();
        let status = unsafe { umlcad_occt_box(width, depth, height, &mut output) };
        if status != OCCT_OK {
            return Err(Self::map_status(status, "OCCT box construction failed"));
        }

        let raw = NonNull::new(output)
            .ok_or(GeometryError::Unsupported("OCCT returned a null shape"))?;
        let shape = OcctShape { raw };

        Ok(GeometryResult {
            shape,
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, tolerance, None),
        })
    }

    fn translate(
        &self,
        shape: &Self::Shape,
        dx: f64,
        dy: f64,
        dz: f64,
        tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        tolerance.validate()?;
        Self::validate_translation(dx, dy, dz)?;

        let mut output = std::ptr::null_mut();
        let status = unsafe {
            umlcad_occt_shape_translate(shape.raw.as_ptr(), dx, dy, dz, &mut output)
        };
        if status != OCCT_OK {
            return Err(Self::map_status(status, "OCCT translation failed"));
        }

        let raw = NonNull::new(output)
            .ok_or(GeometryError::Unsupported("OCCT returned a null shape"))?;
        Ok(GeometryResult {
            shape: OcctShape { raw },
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, tolerance, None),
        })
    }

    fn rotate(
        &self,
        shape: &Self::Shape,
        axis_x: f64,
        axis_y: f64,
        axis_z: f64,
        angle_radians: f64,
        tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        tolerance.validate()?;
        Self::validate_rotation(axis_x, axis_y, axis_z, angle_radians)?;

        let mut output = std::ptr::null_mut();
        let status = unsafe {
            umlcad_occt_shape_rotate(
                shape.raw.as_ptr(),
                axis_x,
                axis_y,
                axis_z,
                angle_radians,
                &mut output,
            )
        };
        if status != OCCT_OK {
            return Err(Self::map_status(status, "OCCT rotation failed"));
        }

        let raw = NonNull::new(output)
            .ok_or(GeometryError::Unsupported("OCCT returned a null shape"))?;
        Ok(GeometryResult {
            shape: OcctShape { raw },
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, tolerance, None),
        })
    }

    fn bounding_box(
        &self,
        shape: &Self::Shape,
        tolerance: ToleranceContext,
    ) -> Result<BoundingBox, GeometryError> {
        tolerance.validate()?;

        let mut values = [0.0_f64; 6];
        let status = unsafe {
            umlcad_occt_shape_bounding_box(shape.raw.as_ptr(), values.as_mut_ptr())
        };
        if status != OCCT_OK {
            return Err(Self::map_status(status, "OCCT bounding-box measurement failed"));
        }

        let bounds = BoundingBox {
            min_x: values[0],
            min_y: values[1],
            min_z: values[2],
            max_x: values[3],
            max_y: values[4],
            max_z: values[5],
        };
        bounds.validate()?;
        Ok(bounds)
    }

    fn topology_counts(
        &self,
        shape: &Self::Shape,
        tolerance: ToleranceContext,
    ) -> Result<TopologyCounts, GeometryError> {
        tolerance.validate()?;

        let mut values = [0_u32; 5];
        let status = unsafe {
            umlcad_occt_shape_topology_counts(shape.raw.as_ptr(), values.as_mut_ptr())
        };
        if status != OCCT_OK {
            return Err(Self::map_status(status, "OCCT topology-count measurement failed"));
        }

        Ok(TopologyCounts {
            solids: values[0],
            shells: values[1],
            faces: values[2],
            edges: values[3],
            vertices: values[4],
        })
    }

    fn validate(
        &self,
        shape: &Self::Shape,
        tolerance: ToleranceContext,
    ) -> Result<ValidationResult, GeometryError> {
        tolerance.validate()?;

        let mut valid = 0;
        let mut manifold = 0;
        let status = unsafe {
            umlcad_occt_shape_validate(shape.raw.as_ptr(), &mut valid, &mut manifold)
        };
        if status != OCCT_OK {
            return Err(Self::map_status(status, "OCCT validation failed"));
        }

        Ok(ValidationResult {
            valid: valid != 0,
            manifold: manifold != 0,
            message: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: ToleranceContext = ToleranceContext {
        modeling: 1e-9,
        validation: 1e-9,
    };

    fn assert_close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() <= 1e-9);
    }

    #[test]
    fn backend_name_is_stable() {
        assert_eq!(OcctBackend::new().backend_name(), "occt");
    }

    #[test]
    fn valid_box_is_constructed_and_validated() {
        let backend = OcctBackend::new();
        let result = backend.box_solid(10.0, 20.0, 30.0, TOLERANCE).unwrap();
        assert_eq!(result.kind, GeometryKind::Solid);
        assert_eq!(result.evidence.status, GeometryStatus::Success);
        assert_eq!(result.evidence.tolerance, TOLERANCE);
        assert_eq!(backend.validate(&result.shape, TOLERANCE).unwrap(), ValidationResult {
            valid: true,
            manifold: true,
            message: None,
        });
    }

    #[test]
    fn topology_counts_match_occt_box() {
        let backend = OcctBackend::new();
        let shape = backend.box_solid(10.0, 20.0, 30.0, TOLERANCE).unwrap().shape;
        assert_eq!(backend.topology_counts(&shape, TOLERANCE).unwrap(), TopologyCounts {
            solids: 1,
            shells: 1,
            faces: 6,
            edges: 12,
            vertices: 8,
        });
    }

    #[test]
    fn bounding_box_matches_box_geometry() {
        let backend = OcctBackend::new();
        let shape = backend.box_solid(10.0, 20.0, 30.0, TOLERANCE).unwrap().shape;
        let bounds = backend.bounding_box(&shape, TOLERANCE).unwrap();
        assert_close(bounds.min_x, 0.0);
        assert_close(bounds.min_y, 0.0);
        assert_close(bounds.min_z, 0.0);
        assert_close(bounds.max_x, 10.0);
        assert_close(bounds.max_y, 20.0);
        assert_close(bounds.max_z, 30.0);
    }

    #[test]
    fn transformed_bounding_box_is_numerically_correct() {
        let backend = OcctBackend::new();
        let shape = backend.box_solid(10.0, 20.0, 30.0, TOLERANCE).unwrap().shape;
        let translated = backend
            .translate(&shape, 100.0, -200.0, 300.0, TOLERANCE)
            .unwrap()
            .shape;
        let translated_bounds = backend.bounding_box(&translated, TOLERANCE).unwrap();
        assert_close(translated_bounds.min_x, 100.0);
        assert_close(translated_bounds.min_y, -200.0);
        assert_close(translated_bounds.min_z, 300.0);
        assert_close(translated_bounds.max_x, 110.0);
        assert_close(translated_bounds.max_y, -180.0);
        assert_close(translated_bounds.max_z, 330.0);

        let rotated = backend
            .rotate(&shape, 0.0, 0.0, 1.0, std::f64::consts::FRAC_PI_2, TOLERANCE)
            .unwrap()
            .shape;
        let rotated_bounds = backend.bounding_box(&rotated, TOLERANCE).unwrap();
        assert_close(rotated_bounds.min_x, -20.0);
        assert_close(rotated_bounds.min_y, 0.0);
        assert_close(rotated_bounds.min_z, 0.0);
        assert_close(rotated_bounds.max_x, 0.0);
        assert_close(rotated_bounds.max_y, 10.0);
        assert_close(rotated_bounds.max_z, 30.0);
    }

    #[test]
    fn invalid_box_dimensions_are_rejected_before_ffi() {
        let backend = OcctBackend::new();
        for dimensions in [
            (0.0, 20.0, 30.0),
            (-1.0, 20.0, 30.0),
            (20.0, 0.0, 30.0),
            (20.0, -1.0, 30.0),
            (20.0, 30.0, 0.0),
            (20.0, 30.0, -1.0),
            (f64::NAN, 20.0, 30.0),
            (20.0, f64::INFINITY, 30.0),
            (20.0, 30.0, f64::NEG_INFINITY),
        ] {
            match backend.box_solid(dimensions.0, dimensions.1, dimensions.2, TOLERANCE) {
                Err(GeometryError::InvalidInput(_)) => {}
                Err(err) => panic!("unexpected geometry error: {err:?}"),
                Ok(_) => panic!("invalid dimensions unexpectedly succeeded"),
            }
        }
    }

    #[test]
    fn dimensions_at_or_below_reference_resolution_are_rejected_before_ffi() {
        let backend = OcctBackend::new();
        for edge in [1e-12, 5e-10, 1e-9, 1e-8, 1e-7, 1e-6] {
            match backend.box_solid(edge, edge * 2.0, edge * 3.0, TOLERANCE) {
                Err(GeometryError::InvalidInput(_)) => {}
                Err(err) => panic!("unexpected resolution-boundary error: {err:?}"),
                Ok(_) => panic!("dimension {edge:e} unexpectedly succeeded"),
            }
        }
    }

    #[test]
    fn invalid_tolerance_is_rejected() {
        let backend = OcctBackend::new();
        for tolerance in [
            ToleranceContext {
                modeling: -1e-9,
                validation: 1e-9,
            },
            ToleranceContext {
                modeling: f64::NAN,
                validation: 1e-9,
            },
            ToleranceContext {
                modeling: 1e-9,
                validation: f64::INFINITY,
            },
        ] {
            match backend.box_solid(10.0, 20.0, 30.0, tolerance) {
                Err(GeometryError::InvalidTolerance) => {}
                Err(err) => panic!("unexpected error for invalid tolerance: {err:?}"),
                Ok(_) => panic!("invalid tolerance unexpectedly succeeded"),
            }
        }
    }

    #[test]
    fn invalid_translation_is_rejected() {
        let backend = OcctBackend::new();
        let shape = backend.box_solid(10.0, 20.0, 30.0, TOLERANCE).unwrap().shape;
        for translation in [
            (f64::NAN, 0.0, 0.0),
            (0.0, f64::INFINITY, 0.0),
            (0.0, 0.0, f64::NEG_INFINITY),
        ] {
            match backend.translate(&shape, translation.0, translation.1, translation.2, TOLERANCE) {
                Err(GeometryError::InvalidInput(_)) => {}
                Err(err) => panic!("unexpected translation error: {err:?}"),
                Ok(_) => panic!("invalid translation unexpectedly succeeded"),
            }
        }
    }

    #[test]
    fn rotation_preserves_validation_and_evidence() {
        let backend = OcctBackend::new();
        let source = backend.box_solid(10.0, 20.0, 30.0, TOLERANCE).unwrap().shape;
        for angle in [0.0, std::f64::consts::FRAC_PI_2, std::f64::consts::PI] {
            let rotated = backend.rotate(&source, 0.0, 0.0, 1.0, angle, TOLERANCE).unwrap();
            assert_eq!(rotated.evidence.tolerance, TOLERANCE);
            assert_eq!(rotated.kind, GeometryKind::Solid);
            assert_eq!(backend.validate(&rotated.shape, TOLERANCE).unwrap(), ValidationResult {
                valid: true,
                manifold: true,
                message: None,
            });
        }
    }

    #[test]
    fn invalid_rotation_is_rejected_before_ffi() {
        let backend = OcctBackend::new();
        let shape = backend.box_solid(10.0, 20.0, 30.0, TOLERANCE).unwrap().shape;
        for input in [
            (0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, f64::NAN),
            (f64::INFINITY, 0.0, 1.0, 0.0),
        ] {
            match backend.rotate(&shape, input.0, input.1, input.2, input.3, TOLERANCE) {
                Err(GeometryError::InvalidInput(_)) => {}
                Err(err) => panic!("unexpected rotation error: {err:?}"),
                Ok(_) => panic!("invalid rotation unexpectedly succeeded"),
            }
        }
    }

    #[test]
    fn numeric_scale_survives_validation() {
        let backend = OcctBackend::new();
        for edge in [2e-6, 1e-5, 1e-3, 1e3, 1e6] {
            let result = backend
                .box_solid(edge, edge * 2.0, edge * 3.0, TOLERANCE)
                .unwrap_or_else(|error| panic!("failed supported scale {edge:e}: {error}"));
            let validation = backend
                .validate(&result.shape, TOLERANCE)
                .unwrap_or_else(|error| panic!("validation failed at supported scale {edge:e}: {error}"));
            assert_eq!(validation, ValidationResult {
                valid: true,
                manifold: true,
                message: None,
            });
        }
    }

    #[test]
    fn translation_returns_new_valid_shape_and_preserves_source() {
        let backend = OcctBackend::new();
        let source = backend.box_solid(10.0, 20.0, 30.0, TOLERANCE).unwrap().shape;
        let translated = backend
            .translate(&source, 1e6, -2e6, 3e6, TOLERANCE)
            .unwrap();

        assert_eq!(translated.evidence.tolerance, TOLERANCE);
        assert_eq!(backend.validate(&source, TOLERANCE).unwrap(), ValidationResult {
            valid: true,
            manifold: true,
            message: None,
        });
        assert_eq!(backend.validate(&translated.shape, TOLERANCE).unwrap(), ValidationResult {
            valid: true,
            manifold: true,
            message: None,
        });
    }

    #[test]
    fn deterministic_validation_is_stable() {
        let backend = OcctBackend::new();
        let mut expected = None;
        for _ in 0..32 {
            let shape = backend.box_solid(37.0, 11.0, 5.0, TOLERANCE).unwrap().shape;
            let current = backend.validate(&shape, TOLERANCE).unwrap();
            match &expected {
                Some(value) => assert_eq!(&current, value),
                None => expected = Some(current),
            }
        }
    }

    #[test]
    fn clone_has_independent_owner() {
        let backend = OcctBackend::new();
        let original = backend.box_solid(10.0, 20.0, 30.0, TOLERANCE).unwrap().shape;
        let clone = original.clone();
        drop(original);
        assert!(backend.validate(&clone, TOLERANCE).unwrap().valid);
    }

    #[test]
    fn repeated_lifetime_cycles_remain_valid() {
        let backend = OcctBackend::new();
        for _ in 0..512 {
            let source = backend.box_solid(10.0, 20.0, 30.0, TOLERANCE).unwrap().shape;
            let clone = source.clone();
            let translated = backend.translate(&clone, 123.0, -456.0, 789.0, TOLERANCE).unwrap();
            assert!(backend.validate(&source, TOLERANCE).unwrap().valid);
            assert!(backend.validate(&translated.shape, TOLERANCE).unwrap().valid);
        }
    }
}
