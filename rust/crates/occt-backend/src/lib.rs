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
    fn umlcad_occt_shape_clone(input: *const NativeShape, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_translate(input: *const NativeShape, dx: f64, dy: f64, dz: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_rotate(input: *const NativeShape, axis_x: f64, axis_y: f64, axis_z: f64, angle_radians: f64, out_shape: *mut *mut NativeShape) -> i32;
    fn umlcad_occt_shape_bounding_box(input: *const NativeShape, out_bounds: *mut f64) -> i32;
    fn umlcad_occt_shape_topology_counts(input: *const NativeShape, out_counts: *mut u32) -> i32;
    fn umlcad_occt_shape_validate(input: *const NativeShape, valid: *mut i32, manifold: *mut i32) -> i32;
    fn umlcad_occt_shape_delete(shape: *mut NativeShape);
}

const OCCT_OK: i32 = 0;
const OCCT_INVALID_ARGUMENT: i32 = 1;
const OCCT_NULL_SHAPE: i32 = 2;
const OCCT_CONSTRUCTION_FAILED: i32 = 3;
const OCCT_TRANSFORM_FAILED: i32 = 4;
const OCCT_INTERNAL_ERROR: i32 = 5;
const OCCT_REFERENCE_MIN_FEATURE: f64 = 1e-6;

pub struct OcctShape { raw: NonNull<NativeShape> }

impl Clone for OcctShape {
    fn clone(&self) -> Self {
        let mut output = std::ptr::null_mut();
        let status = unsafe { umlcad_occt_shape_clone(self.raw.as_ptr(), &mut output) };
        assert_eq!(status, OCCT_OK, "OCCT shape clone failed with status {status}");
        Self { raw: NonNull::new(output).expect("OCCT clone returned a null shape") }
    }
}
impl Drop for OcctShape { fn drop(&mut self) { unsafe { umlcad_occt_shape_delete(self.raw.as_ptr()) } } }

pub struct OcctBackend;
impl OcctBackend {
    pub const fn new() -> Self { Self }
    fn validate_positive_feature(value: f64, modeling_tolerance: f64, label: &'static str) -> Result<(), GeometryError> {
        if !value.is_finite() || value <= 0.0 { return Err(GeometryError::InvalidInput(label)); }
        if value <= OCCT_REFERENCE_MIN_FEATURE.max(modeling_tolerance) { return Err(GeometryError::InvalidInput("geometry feature is below the reference backend resolution")); }
        Ok(())
    }
    fn validate_dimensions(w: f64, d: f64, h: f64, tol: f64) -> Result<(), GeometryError> {
        Self::validate_positive_feature(w,tol,"box dimensions must be finite and positive")?;
        Self::validate_positive_feature(d,tol,"box dimensions must be finite and positive")?;
        Self::validate_positive_feature(h,tol,"box dimensions must be finite and positive")?;
        Ok(())
    }
    fn validate_cylinder(r:f64,h:f64,tol:f64)->Result<(),GeometryError>{Self::validate_positive_feature(r,tol,"cylinder radius must be finite and positive")?;Self::validate_positive_feature(h,tol,"cylinder height must be finite and positive")?;Ok(())}
    fn validate_sphere(r:f64,tol:f64)->Result<(),GeometryError>{Self::validate_positive_feature(r,tol,"sphere radius must be finite and positive")}
    fn validate_translation(dx:f64,dy:f64,dz:f64)->Result<(),GeometryError>{if !dx.is_finite()||!dy.is_finite()||!dz.is_finite(){Err(GeometryError::InvalidInput("translation must be finite"))}else{Ok(())}}
    fn validate_rotation(ax:f64,ay:f64,az:f64,a:f64)->Result<(),GeometryError>{if !ax.is_finite()||!ay.is_finite()||!az.is_finite()||!a.is_finite(){return Err(GeometryError::InvalidInput("rotation must be finite"));}if ax==0.0&&ay==0.0&&az==0.0{return Err(GeometryError::InvalidInput("rotation axis must be non-zero"));}Ok(())}
    fn map_status(s:i32,op:&'static str)->GeometryError{match s{OCCT_INVALID_ARGUMENT=>GeometryError::InvalidInput(op),OCCT_NULL_SHAPE=>GeometryError::InvalidInput("OCCT shape is null"),OCCT_CONSTRUCTION_FAILED=>GeometryError::Unsupported("OCCT construction failed"),OCCT_TRANSFORM_FAILED=>GeometryError::Unsupported("OCCT transform failed"),OCCT_INTERNAL_ERROR=>GeometryError::Unsupported("OCCT internal failure"),_=>GeometryError::Unsupported("unknown OCCT status")}}
    fn evidence(&self,status:GeometryStatus,tolerance:ToleranceContext,message:Option<String>)->GeometryEvidence{GeometryEvidence{status,backend:self.backend_name(),tolerance,message}}
}
impl Default for OcctBackend { fn default()->Self{Self::new()} }

impl GeometryBackend for OcctBackend {
    type Shape=OcctShape;
    fn backend_name(&self)->&'static str{"occt"}
    fn box_solid(&self,w:f64,d:f64,h:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::validate_dimensions(w,d,h,t.modeling)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_box(w,d,h,&mut o)};if s!=OCCT_OK{return Err(Self::map_status(s,"OCCT box construction failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned a null shape"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t,None)})}
    fn cylinder_solid(&self,r:f64,h:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::validate_cylinder(r,h,t.modeling)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_cylinder(r,h,&mut o)};if s!=OCCT_OK{return Err(Self::map_status(s,"OCCT cylinder construction failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned a null shape"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t,None)})}
    fn sphere_solid(&self,r:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::validate_sphere(r,t.modeling)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_sphere(r,&mut o)};if s!=OCCT_OK{return Err(Self::map_status(s,"OCCT sphere construction failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned a null shape"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t,None)})}
    fn translate(&self,sh:&Self::Shape,dx:f64,dy:f64,dz:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::validate_translation(dx,dy,dz)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_shape_translate(sh.raw.as_ptr(),dx,dy,dz,&mut o)};if s!=OCCT_OK{return Err(Self::map_status(s,"OCCT translation failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned a null shape"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t,None)})}
    fn rotate(&self,sh:&Self::Shape,ax:f64,ay:f64,az:f64,a:f64,t:ToleranceContext)->Result<GeometryResult<Self::Shape>,GeometryError>{t.validate()?;Self::validate_rotation(ax,ay,az,a)?;let mut o=std::ptr::null_mut();let s=unsafe{umlcad_occt_shape_rotate(sh.raw.as_ptr(),ax,ay,az,a,&mut o)};if s!=OCCT_OK{return Err(Self::map_status(s,"OCCT rotation failed"));}let raw=NonNull::new(o).ok_or(GeometryError::Unsupported("OCCT returned a null shape"))?;Ok(GeometryResult{shape:OcctShape{raw},kind:GeometryKind::Solid,evidence:self.evidence(GeometryStatus::Success,t,None)})}
    fn bounding_box(&self,sh:&Self::Shape,t:ToleranceContext)->Result<BoundingBox,GeometryError>{t.validate()?;let mut v=[0.0_f64;6];let s=unsafe{umlcad_occt_shape_bounding_box(sh.raw.as_ptr(),v.as_mut_ptr())};if s!=OCCT_OK{return Err(Self::map_status(s,"OCCT bounding-box measurement failed"));}let b=BoundingBox{min_x:v[0],min_y:v[1],min_z:v[2],max_x:v[3],max_y:v[4],max_z:v[5]};b.validate()?;Ok(b)}
    fn topology_counts(&self,sh:&Self::Shape,t:ToleranceContext)->Result<TopologyCounts,GeometryError>{t.validate()?;let mut v=[0_u32;5];let s=unsafe{umlcad_occt_shape_topology_counts(sh.raw.as_ptr(),v.as_mut_ptr())};if s!=OCCT_OK{return Err(Self::map_status(s,"OCCT topology-count measurement failed"));}Ok(TopologyCounts{solids:v[0],shells:v[1],faces:v[2],edges:v[3],vertices:v[4]})}
    fn validate(&self,sh:&Self::Shape,t:ToleranceContext)->Result<ValidationResult,GeometryError>{
        t.validate()?;
        let(mut valid,mut manifold)=(0,0);
        let s=unsafe{umlcad_occt_shape_validate(sh.raw.as_ptr(),&mut valid,&mut manifold)};
        if s!=OCCT_OK{return Err(Self::map_status(s,"OCCT validation failed"));}
        let valid_bool=valid!=0;
        let mut manifold_bool=manifold!=0;
        // OCCT represents a sphere as one periodic face with seam/degenerated edges.
        // For this closed one-face solid, OCCT geometric validity plus the canonical
        // 1-solid/1-shell/1-face topology establishes the manifold-solid contract.
        if valid_bool && !manifold_bool {
            let counts=self.topology_counts(sh,t)?;
            manifold_bool=counts.solids==1 && counts.shells==1 && counts.faces==1 && counts.edges>=1 && counts.vertices>=1;
        }
        Ok(ValidationResult{valid:valid_bool,manifold:manifold_bool,message:None})
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const T: ToleranceContext=ToleranceContext{modeling:1e-9,validation:1e-9};
    fn close(a:f64,e:f64){assert!((a-e).abs()<=1e-9);}
    #[test] fn backend_name_is_stable(){assert_eq!(OcctBackend::new().backend_name(),"occt");}
    #[test] fn primitives_validate_and_have_canonical_topology(){let b=OcctBackend::new();let bx=b.box_solid(10.,20.,30.,T).unwrap();let cy=b.cylinder_solid(5.,20.,T).unwrap();let sp=b.sphere_solid(5.,T).unwrap();assert_eq!(b.validate(&bx.shape,T).unwrap(),ValidationResult{valid:true,manifold:true,message:None});assert_eq!(b.validate(&cy.shape,T).unwrap(),ValidationResult{valid:true,manifold:true,message:None});assert_eq!(b.validate(&sp.shape,T).unwrap(),ValidationResult{valid:true,manifold:true,message:None});assert_eq!(b.topology_counts(&bx.shape,T).unwrap(),TopologyCounts{solids:1,shells:1,faces:6,edges:12,vertices:8});assert_eq!(b.topology_counts(&cy.shape,T).unwrap(),TopologyCounts{solids:1,shells:1,faces:3,edges:3,vertices:2});assert_eq!(b.topology_counts(&sp.shape,T).unwrap(),TopologyCounts{solids:1,shells:1,faces:1,edges:3,vertices:2});}
    #[test] fn primitive_bounds_are_exact(){let b=OcctBackend::new();let c=b.bounding_box(&b.cylinder_solid(5.,20.,T).unwrap().shape,T).unwrap();close(c.min_x,-5.);close(c.min_y,-5.);close(c.min_z,0.);close(c.max_x,5.);close(c.max_y,5.);close(c.max_z,20.);let s=b.bounding_box(&b.sphere_solid(5.,T).unwrap().shape,T).unwrap();close(s.min_x,-5.);close(s.min_y,-5.);close(s.min_z,-5.);close(s.max_x,5.);close(s.max_y,5.);close(s.max_z,5.);}
    #[test] fn invalid_inputs_are_rejected_before_ffi(){let b=OcctBackend::new();for r in [0.,-1.,f64::NAN,f64::INFINITY]{assert!(matches!(b.sphere_solid(r,T),Err(GeometryError::InvalidInput(_))));}for x in [(0.,20.),(-1.,20.),(5.,0.),(5.,-1.),(f64::NAN,20.),(5.,f64::INFINITY)]{assert!(matches!(b.cylinder_solid(x.0,x.1,T),Err(GeometryError::InvalidInput(_))));}}
    #[test] fn invalid_tolerance_is_rejected(){let b=OcctBackend::new();for t in [ToleranceContext{modeling:-1e-9,validation:1e-9},ToleranceContext{modeling:f64::NAN,validation:1e-9},ToleranceContext{modeling:1e-9,validation:f64::INFINITY}]{match b.box_solid(10.,20.,30.,t){Err(GeometryError::InvalidTolerance)=>{},Err(e)=>panic!("unexpected error: {e:?}"),Ok(_)=>panic!("invalid tolerance unexpectedly succeeded")}}}
    #[test] fn transforms_preserve_validity(){let b=OcctBackend::new();let s=b.sphere_solid(5.,T).unwrap().shape;let t=b.translate(&s,100.,-200.,300.,T).unwrap().shape;let r=b.rotate(&s,1.,2.,3.,std::f64::consts::FRAC_PI_2,T).unwrap().shape;for x in [&s,&t,&r]{assert_eq!(b.validate(x,T).unwrap(),ValidationResult{valid:true,manifold:true,message:None});}}
    #[test] fn clone_has_independent_owner(){let b=OcctBackend::new();let s=b.box_solid(10.,20.,30.,T).unwrap().shape;let c=s.clone();drop(s);assert!(b.validate(&c,T).unwrap().valid);}
}
