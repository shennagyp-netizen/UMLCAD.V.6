use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, GeometryResult, ToleranceContext};
use umlcad_v6_offset_api::{OffsetBackend, PlanarLineSegment3D, PlanarSurfacePatch3D};
use umlcad_v6_nurbs_surface_api::{NurbsSurface3DDefinition, NurbsSurfaceBackend, Point3 as NurbsPoint3};
use crate::OcctBackend;

impl OffsetBackend for OcctBackend {
    fn offset_planar_line(&self, definition: PlanarLineSegment3D, distance: f64, tolerance: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        let result = definition.offset(distance, tolerance)?;
        self.line_curve(result.start.x, result.start.y, result.start.z, result.end.x, result.end.y, result.end.z, tolerance)
    }

    fn offset_planar_surface(&self, definition: PlanarSurfacePatch3D, distance: f64, tolerance: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError> {
        let result = definition.offset(distance, tolerance)?;
        let u = result.u_dir.scale(result.width);
        let v = result.v_dir.scale(result.height);
        let p00 = result.origin; let p01 = result.origin.add(v); let p10 = result.origin.add(u); let p11 = p10.add(v);
        let surface = NurbsSurface3DDefinition::new((1,1), vec![
            NurbsPoint3 { x:p00.x,y:p00.y,z:p00.z }, NurbsPoint3 { x:p01.x,y:p01.y,z:p01.z },
            NurbsPoint3 { x:p10.x,y:p10.y,z:p10.z }, NurbsPoint3 { x:p11.x,y:p11.y,z:p11.z },
        ], vec![1.0;4], (2,2), vec![0.,0.,1.,1.], vec![0.,0.,1.,1.]);
        self.nurbs_surface3d(&surface, tolerance)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use umlcad_v6_geometry_api::{GeometryBackend, GeometryKind};
    const T: ToleranceContext = ToleranceContext { modeling:1e-9, validation:1e-9 };

    #[test]
    fn planar_line_offset_realizes_as_curve_without_mutating_source() {
        let b=OcctBackend::new();
        let source=PlanarLineSegment3D { start:umlcad_v6_offset_api::Point3{x:0.,y:0.,z:0.}, end:umlcad_v6_offset_api::Point3{x:10.,y:0.,z:0.}, plane_normal:umlcad_v6_offset_api::Point3{x:0.,y:0.,z:1.} };
        let result=b.offset_planar_line(source,2.,T).unwrap();
        assert_eq!(result.kind,GeometryKind::Curve); assert_eq!(b.curve_length(&result.shape,T).unwrap(),10.); assert_eq!(source.start,umlcad_v6_offset_api::Point3{x:0.,y:0.,z:0.});
    }

    #[test]
    fn planar_surface_offset_realizes_as_one_surface_face() {
        let b=OcctBackend::new();
        let source=PlanarSurfacePatch3D { origin:umlcad_v6_offset_api::Point3{x:1.,y:2.,z:3.}, u_dir:umlcad_v6_offset_api::Point3{x:1.,y:0.,z:0.}, v_dir:umlcad_v6_offset_api::Point3{x:0.,y:1.,z:0.}, width:5., height:8. };
        let result=b.offset_planar_surface(source,4.,T).unwrap(); let counts=b.topology_counts(&result.shape,T).unwrap();
        assert_eq!(result.kind,GeometryKind::Surface); assert_eq!(counts.faces,1); assert_eq!(counts.solids,0); assert_eq!(b.bounding_box(&result.shape,T).unwrap().min_z,7.);
    }

    #[test]
    fn invalid_source_is_rejected_before_native_construction() {
        let b=OcctBackend::new();
        let source=PlanarLineSegment3D { start:umlcad_v6_offset_api::Point3{x:0.,y:0.,z:0.}, end:umlcad_v6_offset_api::Point3{x:0.,y:0.,z:0.}, plane_normal:umlcad_v6_offset_api::Point3{x:0.,y:0.,z:1.} };
        assert!(matches!(b.offset_planar_line(source,1.,T),Err(GeometryError::InvalidInput("planar line offset requires a non-degenerate segment"))));
    }
}
