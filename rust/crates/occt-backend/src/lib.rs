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
    fn umlcad_occt_cylinder(radius: f64, height: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_sphere(radius: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_cone(base_radius: f64, top_radius: f64, height: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_extrude_polygon(
        points_xy: *const f64,
        point_count: u32,
        height: f64,
        out_shape: *mut *mut NativeShape,
    ) -> i32;
    fn umlcad_occt_fuse(left: *const NativeShape, right: *const NativeShape, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_common(left: *const NativeShape, right: *const NativeShape, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_cut(left: *const NativeShape, right: *const NativeShape, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_clone(input: *const NativeShape, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_translate(input: *const NativeShape, dx: f64, dy: f64, dz: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_rotate(input: *const NativeShape, axis_x: f64, axis_y: f64, axis_z: f64, angle_radians: f64, out_shape: *mut *mut NativeShape) -> i32;
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
const OCCT_REFERENCE_MIN_FEATURE: f64 = 1e-6;

pub struct OcctShape {
    raw: NonNull<NativeShape>,
}

impl Clone for OcctShape {
    fn clone(&self) -> Self {
        let mut output = std::ptr::null_mut();
        let status = unsafe { umlcad_occt_shape_clone(self.raw.as_ptr(), &mut output) };
        assert_eq!(status, OCCT_OK);
        Self {
            raw: NonNull::new(output).expect("OCCT clone returned null"),
        }
    }
}

impl Drop for OcctShape {
    fn drop(&mut self) {
        unsafe { umlcad_occt_shape_delete(self.raw.as_ptr()) }
    }
}

pub struct OcctBackend;

impl OcctBackend {
    pub const fn new() -> Self {
        Self
    }

    fn positive(v: f64, tolerance: f64, label: &'static str) -> Result<(), GeometryError> {
        if !v.is_finite() || v <= 0.0 {
            return Err(GeometryError::InvalidInput(label));
        }
        if v <= OCCT_REFERENCE_MIN_FEATURE.max(tolerance) {
            return Err(GeometryError::InvalidInput(
                "geometry feature is below the reference backend resolution",
            ));
        }
        Ok(())
    }

    fn box_dims(w: f64, d: f64, h: f64, tolerance: f64) -> Result<(), GeometryError> {
        Self::positive(w, tolerance, "box dimensions must be finite and positive")?;
        Self::positive(d, tolerance, "box dimensions must be finite and positive")?;
        Self::positive(h, tolerance, "box dimensions must be finite and positive")
    }

    fn cylinder(r: f64, h: f64, tolerance: f64) -> Result<(), GeometryError> {
        Self::positive(r, tolerance, "cylinder radius must be finite and positive")?;
        Self::positive(h, tolerance, "cylinder height must be finite and positive")
    }

    fn sphere(r: f64, tolerance: f64) -> Result<(), GeometryError> {
        Self::positive(r, tolerance, "sphere radius must be finite and positive")
    }

    fn cone(br: f64, tr: f64, h: f64, tolerance: f64) -> Result<(), GeometryError> {
        Self::positive(br, tolerance, "cone base radius must be finite and positive")?;
        Self::positive(tr, tolerance, "cone top radius must be finite and positive")?;
        Self::positive(h, tolerance, "cone height must be finite and positive")
    }

    fn polygon_extrusion(points: &[(f64, f64)], height: f64, tolerance: f64) -> Result<(), GeometryError> {
        if points.len() < 3 {
            return Err(GeometryError::InvalidInput("polygon extrusion requires at least three points"));
        }
        if points.iter().any(|(x, y)| !x.is_finite() || !y.is_finite()) {
            return Err(GeometryError::InvalidInput("polygon coordinates must be finite"));
        }
        Self::positive(height, tolerance, "extrusion height must be finite and positive")
    }

    fn finite_translation(x: f64, y: f64, z: f64) -> Result<(), GeometryError> {
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            Err(GeometryError::InvalidInput("translation must be finite"))
        } else {
            Ok(())
        }
    }

    fn rotation(ax: f64, ay: f64, az: f64, a: f64) -> Result<(), GeometryError> {
        if !ax.is_finite() || !ay.is_finite() || !az.is_finite() || !a.is_finite() {
            return Err(GeometryError::InvalidInput("rotation must be finite"));
        }
        if ax == 0.0 && ay == 0.0 && az == 0.0 {
            return Err(GeometryError::InvalidInput("rotation axis must be non-zero"));
        }
        Ok(())
    }

    fn status(s: i32, op: &'static str) -> GeometryError {
        match s {
            OCCT_INVALID_ARGUMENT => GeometryError::InvalidInput(op),
            OCCT_NULL_SHAPE => GeometryError::InvalidInput("OCCT shape is null"),
            OCCT_CONSTRUCTION_FAILED => GeometryError::Unsupported("OCCT construction failed"),
            OCCT_TRANSFORM_FAILED => GeometryError::Unsupported("OCCT transform failed"),
            OCCT_INTERNAL_ERROR => GeometryError::Unsupported("OCCT internal failure"),
            _ => GeometryError::Unsupported("unknown OCCT status"),
        }
    }

    fn evidence(&self, status: GeometryStatus, tolerance: ToleranceContext) -> GeometryEvidence {
        GeometryEvidence {
            status,
            backend: self.backend_name(),
            tolerance,
            message: None,
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

    fn box_solid(&self, w: f64, d: f64, h: f64, t: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        t.validate()?;
        Self::box_dims(w, d, h, t.modeling)?;
        let mut output = std::ptr::null_mut();
        let status = unsafe { umlcad_occt_box(w, d, h, &mut output) };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT box construction failed"));
        }
        let raw = NonNull::new(output).ok_or(GeometryError::Unsupported("OCCT returned null"))?;
        Ok(GeometryResult {
            shape: OcctShape { raw },
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, t),
        })
    }

    fn cylinder_solid(&self, r: f64, h: f64, t: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        t.validate()?;
        Self::cylinder(r, h, t.modeling)?;
        let mut output = std::ptr::null_mut();
        let status = unsafe { umlcad_occt_cylinder(r, h, &mut output) };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT cylinder construction failed"));
        }
        let raw = NonNull::new(output).ok_or(GeometryError::Unsupported("OCCT returned null"))?;
        Ok(GeometryResult {
            shape: OcctShape { raw },
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, t),
        })
    }

    fn sphere_solid(&self, r: f64, t: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        t.validate()?;
        Self::sphere(r, t.modeling)?;
        let mut output = std::ptr::null_mut();
        let status = unsafe { umlcad_occt_sphere(r, &mut output) };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT sphere construction failed"));
        }
        let raw = NonNull::new(output).ok_or(GeometryError::Unsupported("OCCT returned null"))?;
        Ok(GeometryResult {
            shape: OcctShape { raw },
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, t),
        })
    }

    fn cone_solid(&self, br: f64, tr: f64, h: f64, t: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        t.validate()?;
        Self::cone(br, tr, h, t.modeling)?;
        let mut output = std::ptr::null_mut();
        let status = unsafe { umlcad_occt_cone(br, tr, h, &mut output) };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT cone construction failed"));
        }
        let raw = NonNull::new(output).ok_or(GeometryError::Unsupported("OCCT returned null"))?;
        Ok(GeometryResult {
            shape: OcctShape { raw },
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, t),
        })
    }

    fn extrude_polygon(
        &self,
        points: &[(f64, f64)],
        height: f64,
        t: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        t.validate()?;
        Self::polygon_extrusion(points, height, t.modeling)?;
        let mut flat_points = Vec::with_capacity(points.len() * 2);
        for &(x, y) in points {
            flat_points.push(x);
            flat_points.push(y);
        }
        let mut output = std::ptr::null_mut();
        let status = unsafe {
            umlcad_occt_extrude_polygon(flat_points.as_ptr(), points.len() as u32, height, &mut output)
        };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT polygon extrusion construction failed"));
        }
        let raw = NonNull::new(output).ok_or(GeometryError::Unsupported("OCCT returned null"))?;
        Ok(GeometryResult {
            shape: OcctShape { raw },
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, t),
        })
    }

    fn fuse(&self, left: &Self::Shape, right: &Self::Shape, t: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        self.boolean(left, right, t, umlcad_occt_fuse)
    }

    fn common(&self, left: &Self::Shape, right: &Self::Shape, t: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        self.boolean(left, right, t, umlcad_occt_common)
    }

    fn cut(&self, left: &Self::Shape, right: &Self::Shape, t: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        self.boolean(left, right, t, umlcad_occt_cut)
    }

    fn translate(&self, sh: &Self::Shape, dx: f64, dy: f64, dz: f64, t: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        t.validate()?;
        Self::finite_translation(dx, dy, dz)?;
        let mut output = std::ptr::null_mut();
        let status = unsafe { umlcad_occt_shape_translate(sh.raw.as_ptr(), dx, dy, dz, &mut output) };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT translation failed"));
        }
        let raw = NonNull::new(output).ok_or(GeometryError::Unsupported("OCCT returned null"))?;
        Ok(GeometryResult {
            shape: OcctShape { raw },
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, t),
        })
    }

    fn rotate(&self, sh: &Self::Shape, ax: f64, ay: f64, az: f64, a: f64, t: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        t.validate()?;
        Self::rotation(ax, ay, az, a)?;
        let mut output = std::ptr::null_mut();
        let status = unsafe { umlcad_occt_shape_rotate(sh.raw.as_ptr(), ax, ay, az, a, &mut output) };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT rotation failed"));
        }
        let raw = NonNull::new(output).ok_or(GeometryError::Unsupported("OCCT returned null"))?;
        Ok(GeometryResult {
            shape: OcctShape { raw },
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, t),
        })
    }

    fn bounding_box(&self, sh: &Self::Shape, t: ToleranceContext) -> Result<BoundingBox, GeometryError> {
        t.validate()?;
        let mut values = [0.0; 6];
        let status = unsafe { umlcad_occt_shape_bounding_box(sh.raw.as_ptr(), values.as_mut_ptr()) };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT bounding-box measurement failed"));
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

    fn topology_counts(&self, sh: &Self::Shape, t: ToleranceContext) -> Result<TopologyCounts, GeometryError> {
        t.validate()?;
        let mut values = [0; 5];
        let status = unsafe { umlcad_occt_shape_topology_counts(sh.raw.as_ptr(), values.as_mut_ptr()) };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT topology-count measurement failed"));
        }
        Ok(TopologyCounts {
            solids: values[0],
            shells: values[1],
            faces: values[2],
            edges: values[3],
            vertices: values[4],
        })
    }

    fn validate(&self, sh: &Self::Shape, t: ToleranceContext) -> Result<ValidationResult, GeometryError> {
        t.validate()?;
        let (mut valid, mut manifold) = (0, 0);
        let status = unsafe { umlcad_occt_shape_validate(sh.raw.as_ptr(), &mut valid, &mut manifold) };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT validation failed"));
        }
        let valid_bool = valid != 0;
        let mut manifold_bool = manifold != 0;
        if valid_bool && !manifold_bool {
            let counts = self.topology_counts(sh, t)?;
            manifold_bool = counts.solids == 1 && counts.shells == 1 && counts.faces == 1 && counts.edges >= 1 && counts.vertices >= 1;
        }
        Ok(ValidationResult {
            valid: valid_bool,
            manifold: manifold_bool,
            message: None,
        })
    }
}

impl OcctBackend {
    fn boolean(
        &self,
        left: &OcctShape,
        right: &OcctShape,
        t: ToleranceContext,
        operation: unsafe extern "C" fn(*const NativeShape, *const NativeShape, *mut *mut NativeShape) -> i32,
    ) -> Result<GeometryResult<OcctShape>, GeometryError> {
        t.validate()?;
        let mut output = std::ptr::null_mut();
        let status = unsafe { operation(left.raw.as_ptr(), right.raw.as_ptr(), &mut output) };
        if status != OCCT_OK {
            return Err(Self::status(status, "OCCT Boolean operation failed"));
        }
        let raw = NonNull::new(output).ok_or(GeometryError::Unsupported("OCCT returned null"))?;
        Ok(GeometryResult {
            shape: OcctShape { raw },
            kind: GeometryKind::Solid,
            evidence: self.evidence(GeometryStatus::Success, t),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const T: ToleranceContext = ToleranceContext {
        modeling: 1e-9,
        validation: 1e-9,
    };

    #[test]
    fn boolean_family_is_valid() {
        let backend = OcctBackend::new();
        let a = backend.box_solid(10., 10., 10., T).unwrap().shape;
        let base = backend.box_solid(10., 10., 10., T).unwrap().shape;
        let c = backend.translate(&base, 5., 0., 0., T).unwrap().shape;
        for shape in [
            backend.fuse(&a, &c, T).unwrap().shape,
            backend.common(&a, &c, T).unwrap().shape,
            backend.cut(&a, &c, T).unwrap().shape,
        ] {
            assert!(backend.validate(&shape, T).unwrap().valid);
        }
    }

    #[test]
    fn fuse_is_deterministic() {
        let backend = OcctBackend::new();
        let a = backend.box_solid(10., 10., 10., T).unwrap().shape;
        let base = backend.box_solid(10., 10., 10., T).unwrap().shape;
        let b = backend.translate(&base, 5., 0., 0., T).unwrap().shape;
        let x = backend.fuse(&a, &b, T).unwrap().shape;
        let y = backend.fuse(&a, &b, T).unwrap().shape;
        assert_eq!(backend.bounding_box(&x, T).unwrap(), backend.bounding_box(&y, T).unwrap());
        assert_eq!(backend.topology_counts(&x, T).unwrap(), backend.topology_counts(&y, T).unwrap());
    }
}
