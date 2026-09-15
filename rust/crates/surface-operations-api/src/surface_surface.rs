use crate::{IntersectionStatus, LineSurfaceIntersectionError};
use umlcad_v6_nurbs_surface_api::{NurbsSurface3DDefinition, Point3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceIntersectionEndpoint {
    pub point: Point3,
    pub first_uv: (f64, f64),
    pub second_uv: (f64, f64),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceIntersectionSegment {
    pub start: SurfaceIntersectionEndpoint,
    pub end: SurfaceIntersectionEndpoint,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceSurfaceIntersectionResult {
    pub status: IntersectionStatus,
    pub segments: Vec<SurfaceIntersectionSegment>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SurfaceSurfaceIntersectionError {
    NonFinite,
    InvalidSurface,
    UnsupportedSurfaceFamily,
    ParallelOrCoincident,
    NumericalFailure,
}

fn add(a: Point3, b: Point3) -> Point3 { Point3 { x: a.x+b.x, y: a.y+b.y, z: a.z+b.z } }
fn sub(a: Point3, b: Point3) -> Point3 { Point3 { x: a.x-b.x, y: a.y-b.y, z: a.z-b.z } }
fn scale(a: Point3, s: f64) -> Point3 { Point3 { x: a.x*s, y: a.y*s, z: a.z*s } }

fn affine_patch(surface: &NurbsSurface3DDefinition) -> Result<(Point3, Point3, Point3), SurfaceSurfaceIntersectionError> {
    if surface.degree_u != 1 || surface.degree_v != 1 || surface.count_u != 2 || surface.count_v != 2 || surface.weights.iter().any(|w| (*w - 1.0).abs() > 1e-14) {
        return Err(SurfaceSurfaceIntersectionError::UnsupportedSurfaceFamily);
    }
    let p00=surface.control_points[0]; let p01=surface.control_points[1]; let p10=surface.control_points[2]; let p11=surface.control_points[3];
    let origin=p00; let du=sub(p10,p00); let dv=sub(p01,p00);
    let closure= sub(sub(p11,p10), sub(p01,p00));
    if closure.norm() > 1e-12 * du.norm().max(dv.norm()).max(1.0) { return Err(SurfaceSurfaceIntersectionError::UnsupportedSurfaceFamily); }
    Ok((origin,du,dv))
}

fn plane_point_from_param(surface: &NurbsSurface3DDefinition, uv:(f64,f64))->Point3 {
    let p=surface.control_points[0];
    let du=sub(surface.control_points[2],p); let dv=sub(surface.control_points[1],p);
    add(p,add(scale(du,uv.0),scale(dv,uv.1)))
}

fn solve3(a:[[f64;3];3],b:Point3)->Option<[f64;3]> {
    let d=a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])-a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])+a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]);
    if !d.is_finite() || d.abs()<1e-14 { return None; }
    let mut c=a; c[0][0]=b.x;c[1][0]=b.y;c[2][0]=b.z; let x= c[0][0]*(c[1][1]*c[2][2]-c[1][2]*c[2][1])-c[0][1]*(c[1][0]*c[2][2]-c[1][2]*c[2][0])+c[0][2]*(c[1][0]*c[2][1]-c[1][1]*c[2][0]);
    c=a;c[0][1]=b.x;c[1][1]=b.y;c[2][1]=b.z; let y= c[0][0]*(c[1][1]*c[2][2]-c[1][2]*c[2][1])-c[0][1]*(c[1][0]*c[2][2]-c[1][2]*c[2][0])+c[0][2]*(c[1][0]*c[2][1]-c[1][1]*c[2][0]);
    c=a;c[0][2]=b.x;c[1][2]=b.y;c[2][2]=b.z; let z= c[0][0]*(c[1][1]*c[2][2]-c[1][2]*c[2][1])-c[0][1]*(c[1][0]*c[2][2]-c[1][2]*c[2][0])+c[0][2]*(c[1][0]*c[2][1]-c[1][1]*c[2][0]);
    Some([x/d,y/d,z/d])
}

pub fn intersect_planar_nurbs_surfaces(first:&NurbsSurface3DDefinition,second:&NurbsSurface3DDefinition,tolerance:f64)->Result<SurfaceSurfaceIntersectionResult,SurfaceSurfaceIntersectionError>{
    if !tolerance.is_finite()||tolerance<0.0{return Err(SurfaceSurfaceIntersectionError::NonFinite)}
    first.validate().map_err(|_|SurfaceSurfaceIntersectionError::InvalidSurface)?;second.validate().map_err(|_|SurfaceSurfaceIntersectionError::InvalidSurface)?;
    let (p,a,b)=affine_patch(first)?;let(q,c,d)=affine_patch(second)?;
    let n1=a.cross(b);let n2=c.cross(d);let cross=n1.cross(n2);let cm=cross.norm();
    if !cm.is_finite()||cm<=tolerance.max(1e-12){return Err(SurfaceSurfaceIntersectionError::ParallelOrCoincident)}
    let dir=scale(cross,1.0/(cm*cm));
    let rhs=sub(q,p);
    let coeff=solve3([[a.x,b.x,-cross.x],[a.y,b.y,-cross.y],[a.z,b.z,-cross.z]],rhs).ok_or(SurfaceSurfaceIntersectionError::NumericalFailure)?;
    let base=add(p,add(scale(a,coeff[0]),scale(b,coeff[1])));
    let direction=scale(cross,1.0/(cm));
    let mut lo=f64::NEG_INFINITY;let mut hi=f64::INFINITY;
    let domains=[first.parameter_domain().map_err(|_|SurfaceSurfaceIntersectionError::InvalidSurface)?,second.parameter_domain().map_err(|_|SurfaceSurfaceIntersectionError::InvalidSurface)?];
    let patches=[(p,a,b),(q,c,d)];
    for (patch,(u0v0,u1v1)) in patches.iter().zip(domains.iter()) {
        let (_,du,dv)=*patch;
        for (origin,vec,minv,maxv) in [(patch.0,du,u0v0.0,u1v1.0),(patch.0,dv,u0v0.1,u1v1.1)] {
            let k=vec.dot(direction);
            if k.abs()<=1e-14 { let residual=sub(base,origin); if residual.dot(vec).abs()>tolerance.max(1e-10){return Ok(SurfaceSurfaceIntersectionResult{status:IntersectionStatus::NoIntersection,segments:vec![]})}; continue; }
            let t0=minv;
            let t1=maxv;
            let current0=(t0 - 0.0);
            let current1=(t1 - 0.0);
            if current0 > current1 { continue; }
        }
    }
    let mut tmin=-1.0e6;let mut tmax=1.0e6;
    let axes=[(p,a,domains[0].0.0,domains[0].1.0),(p,b,domains[0].0.1,domains[0].1.1),(q,c,domains[1].0.0,domains[1].1.0),(q,d,domains[1].0.1,domains[1].1.1)];
    for (origin,vec,minv,maxv) in axes {
        let mut values=Vec::new();
        let k=vec.dot(direction);
        if k.abs()>1e-14 { values.push((minv*vec.norm().powi(2)-vec.dot(sub(base,origin)))/k); values.push((maxv*vec.norm().powi(2)-vec.dot(sub(base,origin)))/k); }
        if values.len()==2 { let l=values[0].min(values[1]);let h=values[0].max(values[1]);tmin=tmin.max(l);tmax=tmax.min(h); }
    }
    if tmax<tmin-tolerance { return Ok(SurfaceSurfaceIntersectionResult{status:IntersectionStatus::NoIntersection,segments:vec![]}); }
    let start=add(base,scale(direction,tmin));let end=add(base,scale(direction,tmax));
    let endpoint=|point:Point3|->Result<SurfaceIntersectionEndpoint,SurfaceSurfaceIntersectionError>{
        let rhs1=sub(point,p);let uv1=solve3([[a.x,b.x,0.0],[a.y,b.y,0.0],[a.z,b.z,0.0]],rhs1).map(|x|(x[0],x[1])).ok_or(SurfaceSurfaceIntersectionError::NumericalFailure)?;
        let rhs2=sub(point,q);let uv2=solve3([[c.x,d.x,0.0],[c.y,d.y,0.0],[c.z,d.z,0.0]],rhs2).map(|x|(x[0],x[1])).ok_or(SurfaceSurfaceIntersectionError::NumericalFailure)?;
        Ok(SurfaceIntersectionEndpoint{point,first_uv:uv1,second_uv:uv2})
    };
    let segment=SurfaceIntersectionSegment{start:endpoint(start)?,end:endpoint(end)?};
    Ok(SurfaceSurfaceIntersectionResult{status:IntersectionStatus::Unique,segments:vec![segment]})
}
