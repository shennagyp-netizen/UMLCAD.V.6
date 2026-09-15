use umlcad_kernel_rust::functions::nurbs_surface::{NurbsSurface2D, NurbsSurfaceError, Point3};
use umlcad_kernel_rust::functions::nurbs_surface_differential::NurbsSurfaceDifferential;

fn p(x:f64,y:f64,z:f64)->Point3{Point3{x,y,z}}
fn plane()->NurbsSurface2D{NurbsSurface2D::new(1,1,vec![p(0.,0.,0.),p(0.,1.,1.),p(1.,0.,2.),p(1.,1.,3.)],vec![1.;4],vec![0.,0.,1.,1.],vec![0.,0.,1.,1.])}

#[test]fn bilinear_plane_derivatives_match_independent_oracle(){let s=plane();let du=s.derivative_u_at(.25,.75).unwrap();let dv=s.derivative_v_at(.25,.75).unwrap();assert!((du.x-1.).abs()<1e-12&&du.y.abs()<1e-12&&(du.z-2.).abs()<1e-12);assert!(dv.x.abs()<1e-12&&(dv.y-1.).abs()<1e-12&&(dv.z-1.).abs()<1e-12);}
#[test]fn planar_normal_is_unit_and_deterministic(){let s=plane();let n=s.normal_at(.3,.8).unwrap();let e=p(-2.,-1.,1.);let m=e.x.hypot(e.y.hypot(e.z));assert!((n.x-e.x/m).abs()<1e-12);assert!((n.y-e.y/m).abs()<1e-12);assert!((n.z-e.z/m).abs()<1e-12);}
#[test]fn translation_preserves_derivatives_and_normal(){let s=plane();let moved=s.translated(100.,-50.,7.).unwrap();assert_eq!(moved.derivative_u_at(.4,.6).unwrap(),s.derivative_u_at(.4,.6).unwrap());assert_eq!(moved.derivative_v_at(.4,.6).unwrap(),s.derivative_v_at(.4,.6).unwrap());assert_eq!(moved.normal_at(.4,.6).unwrap(),s.normal_at(.4,.6).unwrap());}
#[test]fn degenerate_surface_normal_is_rejected(){let s=NurbsSurface2D::new(1,1,vec![p(0.,0.,0.),p(0.,1.,0.),p(0.,0.,0.),p(0.,1.,0.)],vec![1.;4],vec![0.,0.,1.,1.],vec![0.,0.,1.,1.]);assert_eq!(s.normal_at(.5,.5),Err(NurbsSurfaceError::ZeroNormal));}
#[test]fn derivative_domain_is_fail_closed(){let s=plane();assert_eq!(s.derivative_u_at(-1e-12,.5),Err(NurbsSurfaceError::OutOfDomain));assert_eq!(s.derivative_v_at(.5,f64::NAN),Err(NurbsSurfaceError::NonFinite));}
