use std::ptr::NonNull;
use umlcad_v6_geometry_api::{BoundingBox, EdgeDescriptor, FaceDescriptor, GeometryBackend, GeometryError, GeometryEvidence, GeometryKind, GeometryResult, GeometryStatus, ToleranceContext, TopologyCounts, ValidationResult, VertexDescriptor};
mod tessellation;
mod exchange;
mod nurbs_surface;
#[repr(C)] struct NativeShape{_private:[u8;0]}