use std::ptr::NonNull;

use crate::{NativeShape, OcctBackend, OcctShape};
use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, GeometryKind, GeometryResult, GeometryStatus, ToleranceContext};
use umlcad_v6_nurbs_surface_api::{NurbsSurface3DDefinition, NurbsSurfaceBackend, Point3 as NurbsPoint3};
use umlcad_v6_offset_api::{OffsetBackend, PlanarLineSegment3D, PlanarSurfacePatch3D};
use umlcad_v6_sweep_api::{LinearCircularSweep, SweepBackend};

unsafe extern "C" {
    fn umlcad_occt_sweep_linear_circular(start_x:f64,start_y:f64,start_z:f64,end_x:f64,end_y:f64,end_z:f64,radius:f64,normal_x:f64,normal_y:f64,normal_z:f64,out_shape:*mut *mut NativeShape)->i32;
}

impl OffsetBackend for OcctBackend {
    fn offset_planar_line(&self, definition: PlanarLineSegment3D, distance: f64, tolerance: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        let result=definition.offset(distance,tolerance)?;
        self.line_curve(result.start.x,result.start.y,result.start.z,result.end.x,result.end.y,result.end.z,tolerance)
    }
    fn offset_planar_surface(&self, definition: PlanarSurfacePatch3D, distance: f64, tolerance: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        let result=definition.offset(distance,tolerance)?;
        let u=result.u_dir.scale(result.width); let v=result.v_dir.scale(result.height); let p00=result.origin; let p01=result.origin.add_point(v); let p10=result.origin.add_point(u); let p11=p10.add_point(v);
        let surface=NurbsSurface3DDefinition::new((1,1),vec![NurbsPoint3{x:p00.x,y:p00.y,z:p00.z},NurbsPoint3{x:p01.x,y:p01.y,z:p01.z},NurbsPoint3{x:p10.x,y:p10.y,z:p10.z},NurbsPoint3{x:p11.x,y:p11.y,z:p11.z}],vec![1.0;4],(2,2),vec![0.0,0.0,1.0,1.0],vec![0.0,0.0,1.0,1.0]);
        self.nurbs_surface3d(&surface,tolerance)
    }
}

impl SweepBackend for OcctBackend {
    fn sweep_linear_circular(&self, definition: LinearCircularSweep, tolerance: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        definition.validate(tolerance)?; let mut raw=std::ptr::null_mut();
        let status=unsafe{umlcad_occt_sweep_linear_circular(definition.path.start.x,definition.path.start.y,definition.path.start.z,definition.path.end.x,definition.path.end.y,definition.path.end.z,definition.profile.radius,definition.profile.normal.x,definition.profile.normal.y,definition.profile.normal.z,&mut raw)};
        if status!=crate::OCCT_OK{return Err(Self::status(status,"OCCT linear circular sweep construction failed"));}
        let raw=NonNull::new(raw).ok_or(GeometryError::Unsupported("OCCT sweep returned null"))?;
        Ok(GeometryResult{shape:OcctShape::from_raw(raw,GeometryKind::Solid),kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,tolerance)})
    }
}

#[cfg(test)]
mod tests {
    use super::*; use umlcad_v6_geometry_api::{GeometryBackend,GeometryKind};
    const T:ToleranceContext=ToleranceContext{modeling:1e-9,validation:1e-9}; const OCCT_PLANAR_SURFACE_BOUND_TOLERANCE:f64=1e-6;
    #[test] fn planar_line_offset_realizes_as_curve_without_mutating_source(){let b=OcctBackend::new();let source=PlanarLineSegment3D{start:umlcad_v6_offset_api::Point3{x:0.,y:0.,z:0.},end:umlcad_v6_offset_api::Point3{x:10.,y:0.,z:0.},plane_normal:umlcad_v6_offset_api::Point3{x:0.,y:0.,z:1.}};let result=b.offset_planar_line(source,2.,T).unwrap();assert_eq!(result.kind,GeometryKind::Curve);assert_eq!(b.curve_length(&result.shape,T).unwrap(),10.);assert_eq!(source.start,umlcad_v6_offset_api::Point3{x:0.,y:0.,z:0.});}
    #[test] fn planar_surface_offset_realizes_as_one_surface_face(){let b=OcctBackend::new();let source=PlanarSurfacePatch3D{origin:umlcad_v6_offset_api::Point3{x:1.,y:2.,z:3.},u_dir:umlcad_v6_offset_api::Point3{x:1.,y:0.,z:0.},v_dir:umlcad_v6_offset_api::Point3{x:0.,y:1.,z:0.},width:5.,height:8.};let result=b.offset_planar_surface(source,4.,T).unwrap();let counts=b.topology_counts(&result.shape,T).unwrap();let bounds=b.bounding_box(&result.shape,T).unwrap();assert_eq!(result.kind,GeometryKind::Surface);assert_eq!(counts.faces,1);assert_eq!(counts.solids,0);assert!((bounds.min_x-1.).abs()<=OCCT_PLANAR_SURFACE_BOUND_TOLERANCE);assert!((bounds.max_x-6.).abs()<=OCCT_PLANAR_SURFACE_BOUND_TOLERANCE);assert!((bounds.min_y-2.).abs()<=OCCT_PLANAR_SURFACE_BOUND_TOLERANCE);assert!((bounds.max_y-10.).abs()<=OCCT_PLANAR_SURFACE_BOUND_TOLERANCE);assert!((bounds.min_z-7.).abs()<=OCCT_PLANAR_SURFACE_BOUND_TOLERANCE);assert!((bounds.max_z-7.).abs()<=OCCT_PLANAR_SURFACE_BOUND_TOLERANCE);}
    #[test] fn invalid_source_is_rejected_before_native_construction(){let b=OcctBackend::new();let source=PlanarLineSegment3D{start:umlcad_v6_offset_api::Point3{x:0.,y:0.,z:0.},end:umlcad_v6_offset_api::Point3{x:0.,y:0.,z:0.},plane_normal:umlcad_v6_offset_api::Point3{x:0.,y:0.,z:1.}};assert!(matches!(b.offset_planar_line(source,1.,T),Err(GeometryError::InvalidInput("planar line offset requires a non-degenerate segment"))));}
    #[test] fn linear_circular_sweep_realizes_as_one_valid_solid(){let b=OcctBackend::new();let definition=LinearCircularSweep{profile:umlcad_v6_sweep_api::CircularProfile{center:umlcad_v6_sweep_api::Point3{x:5.,y:-2.,z:3.},normal:umlcad_v6_sweep_api::Point3{x:0.,y:0.,z:1.},radius:2.},path:umlcad_v6_sweep_api::LinearPath{start:umlcad_v6_sweep_api::Point3{x:5.,y:-2.,z:3.},end:umlcad_v6_sweep_api::Point3{x:5.,y:-2.,z:13.}}};let result=b.sweep_linear_circular(definition,T).unwrap();assert_eq!(result.kind,GeometryKind::Solid);let validation=b.validate(&result.shape,T).unwrap();assert!(validation.valid);assert!(validation.manifold);let bounds=b.bounding_box(&result.shape,T).unwrap();assert!((bounds.min_x-3.).abs()<=1e-9);assert!((bounds.max_x-7.).abs()<=1e-9);assert!((bounds.min_y+4.).abs()<=1e-9);assert!((bounds.max_y-0.).abs()<=1e-9);assert!((bounds.min_z-3.).abs()<=1e-9);assert!((bounds.max_z-13.).abs()<=1e-9);assert!((std::f64::consts::PI*4.*10.-definition.volume(T).unwrap()).abs()<=1e-12);}
    #[test] fn linear_circular_sweep_supports_arbitrary_path_orientation(){let b=OcctBackend::new();let direction=(89.0f64).sqrt();let definition=LinearCircularSweep{profile:umlcad_v6_sweep_api::CircularProfile{center:umlcad_v6_sweep_api::Point3{x:1.,y:2.,z:3.},normal:umlcad_v6_sweep_api::Point3{x:5./direction,y:0.,z:8./direction},radius:1.},path:umlcad_v6_sweep_api::LinearPath{start:umlcad_v6_sweep_api::Point3{x:1.,y:2.,z:3.},end:umlcad_v6_sweep_api::Point3{x:6.,y:2.,z:11.}}};let result=b.sweep_linear_circular(definition,T).unwrap();let bounds=b.bounding_box(&result.shape,T).unwrap();assert!(bounds.min_x.is_finite()&&bounds.max_x.is_finite());assert!(bounds.min_y.is_finite()&&bounds.max_y.is_finite());assert!(bounds.min_z.is_finite()&&bounds.max_z.is_finite());assert!(b.validate(&result.shape,T).unwrap().valid);}
    #[test] fn linear_circular_sweep_is_deterministic(){let b=OcctBackend::new();let definition=LinearCircularSweep{profile:umlcad_v6_sweep_api::CircularProfile{center:umlcad_v6_sweep_api::Point3{x:0.,y:0.,z:0.},normal:umlcad_v6_sweep_api::Point3{x:3./13.,y:4./13.,z:12./13.},radius:1.5},path:umlcad_v6_sweep_api::LinearPath{start:umlcad_v6_sweep_api::Point3{x:0.,y:0.,z:0.},end:umlcad_v6_sweep_api::Point3{x:3.,y:4.,z:12.}}};let first=b.sweep_linear_circular(definition,T).unwrap();let second=b.sweep_linear_circular(definition,T).unwrap();assert_eq!(first.kind,second.kind);assert_eq!(b.topology_counts(&first.shape,T).unwrap(),b.topology_counts(&second.shape,T).unwrap());assert_eq!(b.bounding_box(&first.shape,T).unwrap(),b.bounding_box(&second.shape,T).unwrap());assert_eq!(first.evidence.status,GeometryStatus::Success);}
}
