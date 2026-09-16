use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, GeometryResult, ToleranceContext};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    fn finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    fn sub(self, other: Self) -> Self {
        Self { x: self.x - other.x, y: self.y - other.y, z: self.z - other.z }
    }
    fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
    fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
    fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }
    fn scale(self, factor: f64) -> Self {
        Self { x: self.x * factor, y: self.y * factor, z: self.z * factor }
    }
    fn normalized(self) -> Result<Self, GeometryError> {
        let norm = self.norm();
        if !norm.is_finite() || norm == 0.0 {
            return Err(GeometryError::InvalidInput("sweep direction must be finite and non-zero"));
        }
        Ok(self.scale(1.0 / norm))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CircularProfile {
    pub center: Point3,
    pub normal: Point3,
    pub radius: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearPath {
    pub start: Point3,
    pub end: Point3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearCircularSweep {
    pub profile: CircularProfile,
    pub path: LinearPath,
}

impl LinearCircularSweep {
    pub fn validate(self, tolerance: ToleranceContext) -> Result<(), GeometryError> {
        tolerance.validate()?;
        if !self.profile.center.finite() || !self.profile.normal.finite() || !self.path.start.finite() || !self.path.end.finite() {
            return Err(GeometryError::InvalidInput("sweep contains non-finite geometry"));
        }
        if !self.profile.radius.is_finite() || self.profile.radius <= tolerance.modeling {
            return Err(GeometryError::InvalidInput("sweep profile radius must exceed modeling tolerance"));
        }
        let path = self.path.end.sub(self.path.start);
        if path.norm() <= tolerance.modeling {
            return Err(GeometryError::InvalidInput("sweep path must be non-degenerate"));
        }
        let path_unit = path.normalized()?;
        let normal_unit = self.profile.normal.normalized()?;
        if (1.0 - normal_unit.dot(path_unit).abs()) > tolerance.validation {
            return Err(GeometryError::InvalidInput("sweep profile normal must be parallel to the linear path"));
        }
        let center_delta = self.profile.center.sub(self.path.start).norm();
        if center_delta > tolerance.validation {
            return Err(GeometryError::InvalidInput("sweep profile center must coincide with path start"));
        }
        Ok(())
    }

    pub fn length(self, tolerance: ToleranceContext) -> Result<f64, GeometryError> {
        self.validate(tolerance)?;
        Ok(self.path.end.sub(self.path.start).norm())
    }

    pub fn volume(self, tolerance: ToleranceContext) -> Result<f64, GeometryError> {
        let length = self.length(tolerance)?;
        Ok(std::f64::consts::PI * self.profile.radius * self.profile.radius * length)
    }

    fn alignment_to_z(self, tolerance: ToleranceContext) -> Result<(Point3, f64), GeometryError> {
        self.validate(tolerance)?;
        let direction = self.path.end.sub(self.path.start).normalized()?;
        let z_axis = Point3 { x: 0.0, y: 0.0, z: 1.0 };
        let dot = direction.dot(z_axis).clamp(-1.0, 1.0);
        if dot >= 1.0 - tolerance.validation {
            return Ok((Point3 { x: 1.0, y: 0.0, z: 0.0 }, 0.0));
        }
        if dot <= -1.0 + tolerance.validation {
            return Ok((Point3 { x: 1.0, y: 0.0, z: 0.0 }, std::f64::consts::PI));
        }
        let axis = z_axis.cross(direction).normalized()?;
        Ok((axis, dot.acos()))
    }

    pub fn realize_with<B: GeometryBackend>(self, backend: &B, tolerance: ToleranceContext) -> Result<GeometryResult<B::Shape>, GeometryError> {
        self.validate(tolerance)?;
        let length = self.path.end.sub(self.path.start).norm();
        let base = backend.cylinder_solid(self.profile.radius, length, tolerance)?.shape;
        let (axis, angle) = self.alignment_to_z(tolerance)?;
        let rotated = backend.rotate(&base, axis.x, axis.y, axis.z, angle, tolerance)?.shape;
        backend.translate(&rotated, self.path.start.x, self.path.start.y, self.path.start.z, tolerance)
    }
}

pub trait SweepBackend: GeometryBackend {
    fn sweep_linear_circular(
        &self,
        definition: LinearCircularSweep,
        tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tolerance() -> ToleranceContext {
        ToleranceContext { modeling: 1e-9, validation: 1e-9 }
    }

    fn sweep() -> LinearCircularSweep {
        LinearCircularSweep {
            profile: CircularProfile {
                center: Point3 { x: 0.0, y: 0.0, z: 0.0 },
                normal: Point3 { x: 0.0, y: 0.0, z: 1.0 },
                radius: 2.0,
            },
            path: LinearPath {
                start: Point3 { x: 0.0, y: 0.0, z: 0.0 },
                end: Point3 { x: 0.0, y: 0.0, z: 10.0 },
            },
        }
    }

    #[test]
    fn linear_circular_sweep_has_exact_length_and_volume() {
        let value = sweep();
        assert_eq!(value.length(tolerance()).unwrap(), 10.0);
        assert!((value.volume(tolerance()).unwrap() - 40.0 * std::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn linear_circular_sweep_accepts_antiparallel_profile_normal() {
        let mut value = sweep();
        value.profile.normal = Point3 { x: 0.0, y: 0.0, z: -1.0 };
        assert!(value.validate(tolerance()).is_ok());
    }

    #[test]
    fn linear_sweep_alignment_handles_parallel_and_antiparallel_paths() {
        let mut value = sweep();
        let (axis, angle) = value.alignment_to_z(tolerance()).unwrap();
        assert_eq!(axis, Point3 { x: 1.0, y: 0.0, z: 0.0 });
        assert_eq!(angle, 0.0);

        value.path.end = Point3 { x: 0.0, y: 0.0, z: -10.0 };
        value.profile.normal = Point3 { x: 0.0, y: 0.0, z: -1.0 };
        let (axis, angle) = value.alignment_to_z(tolerance()).unwrap();
        assert_eq!(axis, Point3 { x: 1.0, y: 0.0, z: 0.0 });
        assert_eq!(angle, std::f64::consts::PI);
    }

    #[test]
    fn linear_sweep_alignment_is_finite_for_general_direction() {
        let mut value = sweep();
        value.path.end = Point3 { x: 3.0, y: 4.0, z: 12.0 };
        value.profile.normal = Point3 { x: 3.0, y: 4.0, z: 12.0 };
        let (axis, angle) = value.alignment_to_z(tolerance()).unwrap();
        assert!(axis.finite());
        assert!(angle.is_finite());
        assert!((axis.norm() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn sweep_rejects_misaligned_profile_normal() {
        let mut value = sweep();
        value.profile.normal = Point3 { x: 1.0, y: 0.0, z: 0.0 };
        assert_eq!(value.validate(tolerance()), Err(GeometryError::InvalidInput("sweep profile normal must be parallel to the linear path")));
    }

    #[test]
    fn sweep_rejects_misaligned_profile_center() {
        let mut value = sweep();
        value.profile.center = Point3 { x: 0.0, y: 0.0, z: 1e-3 };
        assert_eq!(value.validate(tolerance()), Err(GeometryError::InvalidInput("sweep profile center must coincide with path start")));
    }

    #[test]
    fn sweep_rejects_degenerate_path_and_radius() {
        let mut value = sweep();
        value.path.end = value.path.start;
        assert_eq!(value.validate(tolerance()), Err(GeometryError::InvalidInput("sweep path must be non-degenerate")));

        let mut value = sweep();
        value.profile.radius = 0.0;
        assert_eq!(value.validate(tolerance()), Err(GeometryError::InvalidInput("sweep profile radius must exceed modeling tolerance")));
    }

    #[test]
    fn sweep_rejects_non_finite_geometry() {
        let mut value = sweep();
        value.profile.radius = f64::NAN;
        assert_eq!(value.validate(tolerance()), Err(GeometryError::InvalidInput("sweep contains non-finite geometry")));
    }
}
