use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, GeometryResult, ToleranceContext, ValidationResult};

/// OCCT integration boundary.
///
/// This crate deliberately does not expose OCCT C++ types to the rest of UMLCAD.
/// The actual FFI/build integration is introduced only after the backend-neutral
/// contract and red-team fixtures are established.
pub struct OcctBackend;

impl OcctBackend {
    pub const fn new() -> Self {
        Self
    }
}

impl Default for OcctBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl GeometryBackend for OcctBackend {
    type Shape = ();

    fn backend_name(&self) -> &'static str {
        "occt"
    }

    fn box_solid(
        &self,
        _width: f64,
        _depth: f64,
        _height: f64,
        _tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        Err(GeometryError::Unsupported("OCCT bridge not implemented yet"))
    }

    fn translate(
        &self,
        _shape: &Self::Shape,
        _dx: f64,
        _dy: f64,
        _dz: f64,
    ) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        Err(GeometryError::Unsupported("OCCT bridge not implemented yet"))
    }

    fn validate(
        &self,
        _shape: &Self::Shape,
        _tolerance: ToleranceContext,
    ) -> Result<ValidationResult, GeometryError> {
        Err(GeometryError::Unsupported("OCCT bridge not implemented yet"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use umlcad_v6_geometry_api::GeometryStatus;

    #[test]
    fn backend_name_is_stable() {
        assert_eq!(OcctBackend::new().backend_name(), "occt");
    }

    #[test]
    fn unimplemented_backend_is_explicitly_unsupported() {
        let result = OcctBackend::new().box_solid(
            10.0,
            20.0,
            30.0,
            ToleranceContext {
                modeling: 1e-9,
                validation: 1e-9,
            },
        );

        assert_eq!(
            result,
            Err(GeometryError::Unsupported("OCCT bridge not implemented yet"))
        );
        let _ = GeometryStatus::Unsupported;
    }
}
