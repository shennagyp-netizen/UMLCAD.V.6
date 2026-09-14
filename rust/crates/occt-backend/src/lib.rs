use std::ptr::NonNull;

use umlcad_v6_geometry_api::{
    BoundingBox, GeometryBackend, GeometryError, GeometryEvidence, GeometryKind, GeometryResult,
    GeometryStatus, ToleranceContext, TopologyCounts, ValidationResult,
};

#[repr(C)]
struct NativeShape { _private: [u8; 0] }

unsafe extern "C" {
    fn umlcad_occt_box(width: f64, depth: f64, height: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_cylinder(radius: f64, height: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_sphere(radius: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_cone(base_radius: f64, top_radius: f64, height: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_fuse(left: *const NativeShape, right: *const NativeShape, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_clone(input: *const NativeShape, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_translate(input: *const NativeShape, dx: f64, dy: f64, dz: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_rotate(input: *const NativeShape, axis_x: f64, axis_y: f64, axis_z: f64, angle_radians: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_bounding_box(input: *const NativeShape, out_bounds: *mut f64) -> i32;
    fn umlcad_occt_shape_topology_counts(input: *const NativeShape, out_counts: *mut u32) -> i32;
    fn umlcad_occt_shape_validate(input: *const NativeShape, valid: *mut i32, manifold: *mut i32) -> i32;
    fn umlcad_occt_shape_delete(shape: *mut NativeShape);
}

const OCCT_OK:i32=0;const OCCT_INVALID_ARGUMENT:i32=1;const OCCT_NULL_SHAPE:i32=2;const OCCT_CONSTRUCTION_FAILED:i32=3;const OCCT_TRANSFORM_FAILED:i32=4;const OCCT_INTERNAL_ERROR:i32=5;const OCCT_REFERENCE_MIN_FEATURE:f64=1e-6;

pub struct OcctShape{raw:NonNull<NativeShape>}
impl Clone for OcctShape{fn clone(&self)->Self{let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_shape_clone(self.raw.as_ptr(),&mut o)};assert_eq!(s,OCCT_OK);Self{raw:NonNull::new(o).expect("OCCT clone returned null")}}}
impl Drop for OcctShape{fn drop(&mut self){unsafe{umlcad_occt_shape_delete(self.raw.as_ptr())}}}
pub struct OcctBackend;
impl OcctBackend{
 pub const fn new()->Self{Self}
 fn positive(v:f64,t:f64,label:&'static str)->Result<(),GeometryError>{if !v.is_finite()||v<=0.0{return Err(GeometryError::InvalidInput(label));}if v<=OCCT_REFERENCE_MIN_FEATURE.max(t){return Err(GeometryError::InvalidInput("geometry feature is below the reference backend resolution"));}Ok(())}
 fn box_dims(w:f64,d:f64,h:f64,t:f64)->Result<(),GeometryError>{Self::positive(w,t,"box dimensions must be finite and positive")?;Self::positive(d,t,"box dimensions must be finite and positive")?;Self::positive(h,t,"box dimensions must be finite and positive")}
 fn cylinder(r:f64,h:f64,t:f64)->Result<(),GeometryError>{Self::positive(r,t,"cylinder radius must be finite and positive")?;Self::positive(h,t,"cylinder height must be finite and positive")}
 fn sphere(r:f64,t:f64)->Result<(),GeometryError>{Self::positive(r,t,"sphere radius must be finite and positive")}
 fn cone(br:f64,tr:f64,h:f64,t:f64)->Result<(),GeometryError>{Self::positive(br,t,"cone base radius must be finite and positive")?;Self::positive(tr,t,"cone top radius must be finite and positive")?;Self::positive(h,t,"cone height must be finite and positive")}
 fn finite_translation(x:f64,y:f64,z:f64)->Result<(),GeometryError>{if !x.is_finite()||!y.is_finite()||!z.is_finite(){Err(GeometryError::InvalidInput("translation must be finite"))}else{Ok(())}}
 fn rotation(ax:f64,ay:f64,az:f64,a:f64)->Result<(),GeometryError>{if !ax.is_finite()||!ay.is_finite()||!az.is_finite()||!a.is_finite(){return Err(GeometryError::InvalidInput("rotation must be finite"));}if ax==0.0&&ay==0.0&&az==0.0{return Err(GeometryError::InvalidInput("rotation axis must be non-zero"));}Ok(())}
 fn status(s:i32,op:&'static str)->GeometryError{match s{OCCT_INVALID_ARGUMENT=>GeometryError::InvalidInput(op),OCCT_NULL_SHAPE=>GeometryError::InvalidInput("OCCT shape is null"),OCCT_CONSTRUCTION_FAILED=>GeometryError::Unsupported("OCCT construction failed"),OCCT_TRANSFORM_FAILED=>GeometryError::Unsupported("OCCT transform failed"),OCCT_INTERNAL_ERROR=>GeometryError::Unsupported("OCCT internal failure"),_=>GeometryError::Unsupported("unknown OCCT status")}}
 fn evidence(&self,status:GeometryStatus,t:ToleranceContext)->GeometryEvidence{GeometryEvidence{status,backend:self.backend_name(),tolerance:t,message:None}}
}
impl Default for OcctBackend{fn default()->Self{Self::new()}}

impl GeometryBackend for OcctBackend{
 type Shape=OcctShape;
 fn backend_name(&self)->&'static str{"occt"}
 fn box_solid(&self,w:f64,d:f64,h:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::box_dims(w,d,h,t.modeling)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_box(w,d,h,&mut o)};if s!=OCCT_OK{return Err(Self::status(s,"OCCT box construction failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned null"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t)})}
 fn cylinder_solid(&self,r:f64,h:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::cylinder(r,h,t.modeling)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_cylinder(r,h,&mut o)};if s!=OCCT_OK{return Err(Self::status(s,"OCCT cylinder construction failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned null"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t)})}
 fn sphere_solid(&self,r:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::sphere(r,t.modeling)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_sphere(r,&mut o)};if s!=OCCT_OK{return Err(Self::status(s,"OCCT sphere construction failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned null"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t)})}
 fn cone_solid(&self,br:f64,tr:f64,h:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::cone(br,tr,h,t.modeling)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_cone(br,tr,h,&mut o)};if s!=OCCT_OK{return Err(Self::status(s,"OCCT cone construction failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned null"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t)})}
 fn fuse(&self,left:&Self::Shape,right:&Self::Shape,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_fuse(left.raw.as_ptr(),right.raw.as_ptr(),&mut o)};if s!=OCCT_OK{return Err(Self::status(s,"OCCT boolean fuse failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned null"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t)})}
 fn translate(&self,sh:&Self::Shape,dx:f64,dy:f64,dz:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::finite_translation(dx,dy,dz)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_shape_translate(sh.raw.as_ptr(),dx,dy,dz,&mut o)};if s!=OCCT_OK{return Err(Self::status(s,"OCCT translation failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned null"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t)})}
 fn rotate(&self,sh:&Self::Shape,ax:f64,ay:f64,az:f64,a:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::rotation(ax,ay,az,a)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_shape_rotate(sh.raw.as_ptr(),ax,ay,az,a,&mut o)};if s!=OCCT_OK{return Err(Self::status(s,"OCCT rotation failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned null"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t)})}
 fn bounding_box(&self,sh:&Self::Shape,t:ToleranceContext)->Result<BoundingBox,GeometryError>{t.validate()?;let mut v=[0.0;6];let s=unsafe{umlcad_occt_shape_bounding_box(sh.raw.as_ptr(),v.as_mut_ptr())};if s!=OCCT_OK{return Err(Self::status(s,"OCCT bounding-box measurement failed"));}let b=BoundingBox{min_x:v[0],min_y:v[1],min_z:v[2],max_x:v[3],max_y:v[4],max_z:v[5]};b.validate()?;Ok(b)}
 fn topology_counts(&self,sh:&Self::Shape,t:ToleranceContext)->Result<TopologyCounts,GeometryError>{t.validate()?;let mut v=[0;5];let s=unsafe{umlcad_occt_shape_topology_counts(sh.raw.as_ptr(),v.as_mut_ptr())};if s!=OCCT_OK{return Err(Self::status(s,"OCCT topology-count measurement failed"));}Ok(TopologyCounts{solids:v[0],shells:v[1],faces:v[2],edges:v[3],vertices:v[4]})}
 fn validate(&self,sh:&Self::Shape,t:ToleranceContext)->Result<ValidationResult,GeometryError>{t.validate()?;let(mut valid,mut manifold)=(0,0);let s=unsafe{umlcad_occt_shape_validate(sh.raw.as_ptr(),&mut valid,&mut manifold)};if s!=OCCT_OK{return Err(Self::status(s,"OCCT validation failed"));}let valid_bool=valid!=0;let mut manifold_bool=manifold!=0;if valid_bool&&!manifold_bool{let c=self.topology_counts(sh,t)?;manifold_bool=c.solids==1&&c.shells==1&&c.faces==1&&c.edges>=1&&c.vertices>=1;}Ok(ValidationResult{valid:valid_bool,manifold:manifold_bool,message:None})}
}

#[cfg(test)]
mod tests{use super::*;const T:ToleranceContext=ToleranceContext{modeling:1e-9,validation:1e-9};#[test]fn fuse_overlapping_boxes_is_valid(){let b=OcctBackend::new();let left=b.box_solid(10.,10.,10.,T).unwrap().shape;let base_right=b.box_solid(10.,10.,10.,T).unwrap().shape;let right=b.translate(&base_right,5.,0.,0.,T).unwrap().shape;let u=b.fuse(&left,&right,T).unwrap();assert_eq!(u.kind,GeometryKind::Solid);assert!(b.validate(&u.shape,T).unwrap().valid);let bounds=b.bounding_box(&u.shape,T).unwrap();assert!((bounds.min_x-0.).abs()<1e-9);assert!((bounds.max_x-15.).abs()<1e-9);}#[test]fn fuse_is_deterministic(){let b=OcctBackend::new();let a=b.box_solid(10.,10.,10.,T).unwrap().shape;let bb=b.translate(&b.box_solid(10.,10.,10.,T).unwrap().shape,5.,0.,0.,T).unwrap().shape;let x=b.fuse(&a,&bb,T).unwrap().shape;let y=b.fuse(&a,&bb,T).unwrap().shape;assert_eq!(b.bounding_box(&x,T).unwrap(),b.bounding_box(&y,T).unwrap());assert_eq!(b.topology_counts(&x,T).unwrap(),b.topology_counts(&y,T).unwrap());}}
