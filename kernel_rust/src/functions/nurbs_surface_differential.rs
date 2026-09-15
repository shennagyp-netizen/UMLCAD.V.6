use super::nurbs_surface::{NurbsSurface2D, NurbsSurfaceError, Point3};

pub trait NurbsSurfaceDifferential {
    fn derivative_u_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError>;
    fn derivative_v_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError>;
    fn normal_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError>;
}

impl NurbsSurfaceDifferential for NurbsSurface2D {
    fn derivative_u_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError> {
        derivative(self, u, v, true)
    }

    fn derivative_v_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError> {
        derivative(self, u, v, false)
    }

    fn normal_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError> {
        let du = self.derivative_u_at(u, v)?;
        let dv = self.derivative_v_at(u, v)?;
        let n = Point3 { x: du.y * dv.z - du.z * dv.y, y: du.z * dv.x - du.x * dv.z, z: du.x * dv.y - du.y * dv.x };
        let m = n.x.hypot(n.y.hypot(n.z));
        if !m.is_finite() { return Err(NurbsSurfaceError::Overflow); }
        if m == 0.0 { return Err(NurbsSurfaceError::ZeroNormal); }
        Ok(Point3 { x: n.x / m, y: n.y / m, z: n.z / m })
    }
}

fn derivative(s: &NurbsSurface2D, u: f64, v: f64, along_u: bool) -> Result<Point3, NurbsSurfaceError> {
    s.validate()?;
    if !u.is_finite() || !v.is_finite() { return Err(NurbsSurfaceError::NonFinite); }
    let (u0,u1,v0,v1)=s.parameter_domain()?;
    if u<u0||u>u1||v<v0||v>v1 { return Err(NurbsSurfaceError::OutOfDomain); }
    let eps=1e-7_f64.max((u1-u0).abs().max((v1-v0).abs())*1e-8);
    let q = if along_u {
        let lo=(u-eps).max(u0); let hi=(u+eps).min(u1);
        if hi==lo { return Err(NurbsSurfaceError::Degenerate); }
        let a=s.point_at(lo,v)?; let b=s.point_at(hi,v)?;
        Point3{x:(b.x-a.x)/(hi-lo),y:(b.y-a.y)/(hi-lo),z:(b.z-a.z)/(hi-lo)}
    } else {
        let lo=(v-eps).max(v0); let hi=(v+eps).min(v1);
        if hi==lo { return Err(NurbsSurfaceError::Degenerate); }
        let a=s.point_at(u,lo)?; let b=s.point_at(u,hi)?;
        Point3{x:(b.x-a.x)/(hi-lo),y:(b.y-a.y)/(hi-lo),z:(b.z-a.z)/(hi-lo)}
    };
    if !q.x.is_finite()||!q.y.is_finite()||!q.z.is_finite(){Err(NurbsSurfaceError::Overflow)}else{Ok(q)}
}
