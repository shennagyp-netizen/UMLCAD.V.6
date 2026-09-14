use core::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryStatus {
    Success,
    InvalidInput,
    BackendFailure,
    TopologyInvalid,
    Ambiguous,
    Indeterminate,
    Unsupported,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryKind {
    Solid,
    Surface,
    Curve,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToleranceContext {
    pub modeling: f64,
    pub validation: f64,
}

impl ToleranceContext {
    pub fn validate(self) -> Result<(), GeometryError> {
        if !self.modeling.is_finite() || !self.validation.is_finite() {
            return Err(GeometryError::InvalidTolerance);
        }
        if self.modeling < 0.0 || self.validation < 0.0 {
            return Err(GeometryError::InvalidTolerance);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingBox {
    pub min_x: f64,
    pub min_y: f64,
    pub min_z: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub max_z: f64,
}

impl BoundingBox {
    pub fn validate(self) -> Result<(), GeometryError> {
        let values = [
            self.min_x,
            self.min_y,
            self.min_z,
            self.max_x,
            self.max_y,
            self.max_z,
        ];
        if values.iter().any(|value| !value.is_finite()) {
            return Err(GeometryError::InvalidInput("bounding box contains non-finite values"));
        }
        if self.min_x > self.max_x || self.min_y > self.max_y || self.min_z > self.max_z {
            return Err(GeometryError::InvalidInput("bounding box minimum exceeds maximum"));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TopologyCounts {
    pub solids: u32,
    pub shells: u32,
    pub faces: u32,
    pub edges: u32,
    pub vertices: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GeometryEvidence {
    pub status: GeometryStatus,
    pub backend: &'static str,
    pub tolerance: ToleranceContext,
    pub message: Option<String>,
}

#[derive(Debug, PartialEq)]
pub enum GeometryError {
    InvalidTolerance,
    InvalidInput(&'static str),
    Unsupported(&'static str),
}

impl fmt::Display for GeometryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidTolerance => write!(f, "invalid tolerance context"),
            Self::InvalidInput(v) => write!(f, "invalid input: {v}"),
            Self::Unsupported(v) => write!(f, "unsupported: {v}"),
        }
    }
}

pub trait GeometryBackend {
    type Shape: Clone;

    fn backend_name(&self) -> &'static str;

    fn box_solid(
        &self,
        width: f64,
        depth: f64,
        height: f64,
        tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError>;

    fn cylinder_solid(
        &self,
        radius: f64,
        height: f64,
        tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError>;

    fn translate(
        &self,
        shape: &Self::Shape,
        dx: f64,
        dy: f64,
        dz: f64,
        tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError>;

    fn rotate(
        &self,
        shape: &Self::Shape,
        axis_x: f64,
        axis_y: f64,
        axis_z: f64,
        angle_radians: f64,
        tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError>;

    fn bounding_box(
        &self,
        shape: &Self::Shape,
        tolerance: ToleranceContext,
    ) -> Result<BoundingBox, GeometryError>;

    fn topology_counts(
        &self,
        shape: &Self::Shape,
        tolerance: ToleranceContext,
    ) -> Result<TopologyCounts, GeometryError>;

    fn validate(
        &self,
        shape: &Self::Shape,
        tolerance: ToleranceContext,
    ) -> Result<ValidationResult, GeometryError>;
}

#[derive(Clone, Debug)]
pub struct GeometryResult<S> {
    pub shape: S,
    pub kind: GeometryKind,
    pub evidence: GeometryEvidence,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValidationResult {
    pub valid: bool,
    pub manifold: bool,
    pub message: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tolerance_must_be_finite_and_non_negative() {
        assert!(ToleranceContext { modeling: 0.0, validation: 1e-9 }.validate().is_ok());
        assert!(ToleranceContext { modeling: 1e-9, validation: 0.0 }.validate().is_ok());
        assert_eq!(
            ToleranceContext { modeling: -1.0, validation: 0.0 }.validate(),
            Err(GeometryError::InvalidTolerance)
        );
        assert_eq!(
            ToleranceContext { modeling: 0.0, validation: -1e-12 }.validate(),
            Err(GeometryError::InvalidTolerance)
        );
        assert_eq!(
            ToleranceContext { modeling: f64::NAN, validation: 0.0 }.validate(),
            Err(GeometryError::InvalidTolerance)
        );
        assert_eq!(
            ToleranceContext { modeling: 0.0, validation: f64::INFINITY }.validate(),
            Err(GeometryError::InvalidTolerance)
        );
    }

    #[test]
    fn bounding_box_requires_finite_ordered_extents() {
        assert!(BoundingBox {
            min_x: 0.0,
            min_y: 0.0,
            min_z: 0.0,
            max_x: 1.0,
            max_y: 2.0,
            max_z: 3.0,
        }
        .validate()
        .is_ok());

        assert_eq!(
            BoundingBox {
                min_x: f64::NAN,
                min_y: 0.0,
                min_z: 0.0,
                max_x: 1.0,
                max_y: 2.0,
                max_z: 3.0,
            }
            .validate(),
            Err(GeometryError::InvalidInput("bounding box contains non-finite values"))
        );
        assert_eq!(
            BoundingBox {
                min_x: 2.0,
                min_y: 0.0,
                min_z: 0.0,
                max_x: 1.0,
                max_y: 2.0,
                max_z: 3.0,
            }
            .validate(),
            Err(GeometryError::InvalidInput("bounding box minimum exceeds maximum"))
        );
    }

    #[test]
    fn topology_counts_are_explicit_and_unsigned() {
        let counts = TopologyCounts {
            solids: 1,
            shells: 1,
            faces: 6,
            edges: 12,
            vertices: 8,
        };
        assert_eq!(counts.solids, 1);
        assert_eq!(counts.vertices, 8);
    }
}
