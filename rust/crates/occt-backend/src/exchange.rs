use std::ffi::CString;
use std::path::Path;

use umlcad_v6_exchange_api::{ExchangeBackend, ExchangeDirection, ExchangeError, ExchangeEvidence, ExchangeFormat, ExchangeStatus};
use umlcad_v6_geometry_api::{GeometryEvidence, GeometryKind, GeometryResult, GeometryStatus, ToleranceContext};

use super::{NativeShape, OcctBackend, OcctShape, OCCT_CONSTRUCTION_FAILED, OCCT_INTERNAL_ERROR, OCCT_INVALID_ARGUMENT, OCCT_NULL_SHAPE, OCCT_OK};

unsafe extern "C" {
    fn umlcad_occt_shape_export_file(input: *const NativeShape, format: i32, path: *const std::ffi::c_char) -> i32;
    fn umlcad_occt_shape_import_file(format: i32, path: *const std::ffi::c_char, out_shape: *mut *mut NativeShape) -> i32;
}

fn format_code(format: ExchangeFormat) -> i32 {
    match format {
        ExchangeFormat::Step => 0,
        ExchangeFormat::Iges => 1,
    }
}

fn path_cstring(path: &Path) -> Result<CString, ExchangeError> {
    let value = path.to_str().ok_or(ExchangeError::InvalidPath)?;
    if value.is_empty() || value.contains('\0') {
        return Err(ExchangeError::InvalidPath);
    }
    CString::new(value).map_err(|_| ExchangeError::InvalidPath)
}

fn exchange_status(status: i32) -> ExchangeError {
    match status {
        OCCT_INVALID_ARGUMENT => ExchangeError::InvalidPath,
        OCCT_NULL_SHAPE => ExchangeError::EmptyResult,
        OCCT_CONSTRUCTION_FAILED | OCCT_INTERNAL_ERROR => ExchangeError::TranslationFailure,
        _ => ExchangeError::TranslationFailure,
    }
}

impl ExchangeBackend for OcctBackend {
    fn export_file(
        &self,
        shape: &Self::Shape,
        format: ExchangeFormat,
        path: &Path,
        tolerance: ToleranceContext,
    ) -> Result<ExchangeEvidence, ExchangeError> {
        tolerance
            .validate()
            .map_err(|_| ExchangeError::InvalidPath)?;
        let destination = path_cstring(path)?;
        let status = unsafe {
            umlcad_occt_shape_export_file(shape.raw.as_ptr(), format_code(format), destination.as_ptr())
        };
        if status != OCCT_OK {
            return Err(exchange_status(status));
        }
        let metadata = std::fs::metadata(path).map_err(|_| ExchangeError::IoFailure)?;
        if !metadata.is_file() {
            return Err(ExchangeError::IoFailure);
        }
        let bytes = metadata.len();
        if bytes == 0 {
            return Err(ExchangeError::EmptyResult);
        }
        Ok(ExchangeEvidence {
            format,
            direction: ExchangeDirection::Export,
            status: ExchangeStatus::Success,
            backend: self.backend_name(),
            bytes,
        })
    }

    fn import_file(
        &self,
        format: ExchangeFormat,
        path: &Path,
        tolerance: ToleranceContext,
    ) -> Result<GeometryResult<Self::Shape>, ExchangeError> {
        tolerance
            .validate()
            .map_err(|_| ExchangeError::InvalidPath)?;
        let metadata = std::fs::metadata(path).map_err(|_| ExchangeError::IoFailure)?;
        if !metadata.is_file() || metadata.len() == 0 {
            return Err(ExchangeError::EmptyResult);
        }
        let source = path_cstring(path)?;
        let mut raw = std::ptr::null_mut();
        let status = unsafe {
            umlcad_occt_shape_import_file(format_code(format), source.as_ptr(), &mut raw)
        };
        if status != OCCT_OK {
            return Err(exchange_status(status));
        }
        let raw = std::ptr::NonNull::new(raw).ok_or(ExchangeError::EmptyResult)?;
        let shape = OcctShape::from_raw(raw, GeometryKind::Solid);
        Ok(GeometryResult {
            shape,
            kind: GeometryKind::Solid,
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

    const TOLERANCE: ToleranceContext = ToleranceContext {
        modeling: 1e-9,
        validation: 1e-9,
    };

    #[test]
    fn unsupported_extension_is_rejected_before_backend_io() {
        assert_eq!(
            ExchangeFormat::from_extension(Path::new("part.obj")),
            Err(ExchangeError::UnsupportedFormat)
        );
    }

    #[test]
    fn exchange_path_with_embedded_nul_is_rejected() {
        let backend = OcctBackend::new();
        let shape = backend.box_solid(10.0, 10.0, 10.0, TOLERANCE).unwrap().shape;
        let error = backend.export_file(
            &shape,
            ExchangeFormat::Step,
            Path::new("bad\0.step"),
            TOLERANCE,
        );
        assert_eq!(error, Err(ExchangeError::InvalidPath));
    }
}
