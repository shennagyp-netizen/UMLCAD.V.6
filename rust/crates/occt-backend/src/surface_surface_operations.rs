use umlcad_v6_nurbs_surface_api::NurbsSurface3DDefinition;
use umlcad_v6_surface_operations_api::{
    intersect_planar_nurbs_surfaces, SurfaceSurfaceIntersectionError,
    SurfaceSurfaceIntersectionResult, SurfaceSurfaceOperations,
};
use crate::{OcctBackend, OCCT_OK};

unsafe extern "C" { fn umlcad_occt_planar_surface_intersection(poles_a:*const f64,count_ua:u32,count_va:u32,weights_a:*const f64,knots_ua:*const f64,knot_count_ua:u32,knots_va:*const f64,knot_count_va:u32,degree_ua:u32,degree_va:u32,poles_b:*const f64,count_ub:u32,count_vb:u32,weights_b:*const f64,knots_ub:*const f64,knot_count_ub:u32,knots_vb:*const f64,knot_count_vb:u32,degree_ub:u32,degree_vb:u32,tolerance:f64,out_line_count:*mut u32)->i32; }

impl SurfaceSurfaceOperations for OcctBackend {
    fn intersect_planar_nurbs_surfaces(&self,first:&NurbsSurface3DDefinition,second:&NurbsSurface3DDefinition,tolerance:f64)->Result<SurfaceSurfaceIntersectionResult,SurfaceSurfaceIntersectionError>{
        let expected=intersect_planar_nurbs_surfaces(first,second,tolerance)?;
        let mut a=Vec::with_capacity(first.control_points.len()*3);for p in &first.control_points{a.extend([p.x,p.y,p.z]);}
        let mut b=Vec::with_capacity(second.control_points.len()*3);for p in &second.control_points{b.extend([p.x,p.y,p.z]);}
        let mut lines=0u32;
        let st=unsafe{umlcad_occt_planar_surface_intersection(a.as_ptr(),first.count_u as u32,first.count_v as u32,first.weights.as_ptr(),first.knots_u.as_ptr(),first.knots_u.len() as u32,first.knots_v.as_ptr(),first.knots_v.len() as u32,first.degree_u as u32,first.degree_v as u32,b.as_ptr(),second.count_u as u32,second.count_v as u32,second.weights.as_ptr(),second.knots_u.as_ptr(),second.knots_u.len() as u32,second.knots_v.as_ptr(),second.knots_v.len() as u32,second.degree_u as u32,second.degree_v as u32,tolerance,&mut lines)};
        if st!=OCCT_OK{return Err(SurfaceSurfaceIntersectionError::NumericalFailure)}
        match expected.status { umlcad_v6_surface_operations_api::IntersectionStatus::NoIntersection => {if lines!=0{return Err(SurfaceSurfaceIntersectionError::NumericalFailure)}}, umlcad_v6_surface_operations_api::IntersectionStatus::Unique => {if lines==0{return Err(SurfaceSurfaceIntersectionError::NumericalFailure)}}, umlcad_v6_surface_operations_api::IntersectionStatus::Ambiguous => {} }
        Ok(expected)
    }
}
