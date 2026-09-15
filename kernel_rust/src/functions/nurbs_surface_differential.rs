use super::nurbs_surface::{NurbsSurface2D, NurbsSurfaceError, Point3};

pub trait NurbsSurfaceDifferential {
    fn derivative_u_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError>;
    fn derivative_v_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError>;
    fn normal_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError>;
}

impl NurbsSurfaceDifferential for NurbsSurface2D {
    fn derivative_u_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError> {
        let base = evaluate_homogeneous(self, u, v)?;
        let derivative = evaluate_homogeneous_derivative_u(self, u, v)?;
        quotient_derivative(base, derivative)
    }

    fn derivative_v_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError> {
        let base = evaluate_homogeneous(self, u, v)?;
        let derivative = evaluate_homogeneous_derivative_v(self, u, v)?;
        quotient_derivative(base, derivative)
    }

    fn normal_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError> {
        let du = self.derivative_u_at(u, v)?;
        let dv = self.derivative_v_at(u, v)?;
        let normal = Point3 {
            x: du.y * dv.z - du.z * dv.y,
            y: du.z * dv.x - du.x * dv.z,
            z: du.x * dv.y - du.y * dv.x,
        };
        let magnitude = normal.x.hypot(normal.y.hypot(normal.z));
        if !magnitude.is_finite() { return Err(NurbsSurfaceError::Overflow); }
        if magnitude == 0.0 { return Err(NurbsSurfaceError::ZeroNormal); }
        Ok(Point3 { x: normal.x / magnitude, y: normal.y / magnitude, z: normal.z / magnitude })
    }
}

#[derive(Clone, Copy)]
struct HomogeneousPoint { xw: f64, yw: f64, zw: f64, w: f64 }

impl HomogeneousPoint {
    fn lerp(self, other: Self, alpha: f64) -> Self {
        Self {
            xw: self.xw * (1.0 - alpha) + other.xw * alpha,
            yw: self.yw * (1.0 - alpha) + other.yw * alpha,
            zw: self.zw * (1.0 - alpha) + other.zw * alpha,
            w: self.w * (1.0 - alpha) + other.w * alpha,
        }
    }
}

fn validate_query(s: &NurbsSurface2D, u: f64, v: f64) -> Result<(), NurbsSurfaceError> {
    s.validate()?;
    if !u.is_finite() || !v.is_finite() { return Err(NurbsSurfaceError::NonFinite); }
    let (u0, u1, v0, v1) = s.parameter_domain()?;
    if u < u0 || u > u1 || v < v0 || v > v1 { return Err(NurbsSurfaceError::OutOfDomain); }
    Ok(())
}

fn control_count_u(s: &NurbsSurface2D) -> usize { s.knots_u.len() - s.degree_u - 1 }
fn control_count_v(s: &NurbsSurface2D) -> usize { s.knots_v.len() - s.degree_v - 1 }

fn span(t: f64, degree: usize, knots: &[f64], count: usize) -> usize {
    let last = count - 1;
    if t >= knots[count] { return last; }
    if t <= knots[degree] { return degree; }
    let (mut low, mut high) = (degree, count);
    let mut mid = (low + high) / 2;
    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] { high = mid; } else { low = mid; }
        mid = (low + high) / 2;
    }
    mid
}

fn de_boor(t: f64, span: usize, degree: usize, knots: &[f64], work: &mut [HomogeneousPoint]) -> HomogeneousPoint {
    if degree == 0 { return work[0]; }
    for level in 1..=degree {
        for j in (level..=degree).rev() {
            let i = span - degree + j;
            let denominator = knots[i + degree + 1 - level] - knots[i];
            let alpha = if denominator == 0.0 { 0.0 } else { (t - knots[i]) / denominator };
            work[j] = work[j - 1].lerp(work[j], alpha);
        }
    }
    work[degree]
}

fn evaluate_homogeneous(s: &NurbsSurface2D, u: f64, v: f64) -> Result<HomogeneousPoint, NurbsSurfaceError> {
    validate_query(s, u, v)?;
    let uc = control_count_u(s);
    let vc = control_count_v(s);
    let us = span(u, s.degree_u, &s.knots_u, uc);
    let mut rows = Vec::with_capacity(vc);
    for j in 0..vc {
        let mut work = Vec::with_capacity(s.degree_u + 1);
        for local in 0..=s.degree_u {
            let i = us - s.degree_u + local;
            let index = i * vc + j;
            let point = s.control_points[index];
            let weight = s.weights[index];
            work.push(HomogeneousPoint { xw: point.x * weight, yw: point.y * weight, zw: point.z * weight, w: weight });
        }
        rows.push(de_boor(u, us, s.degree_u, &s.knots_u, &mut work));
    }
    let vs = span(v, s.degree_v, &s.knots_v, vc);
    let mut work = Vec::with_capacity(s.degree_v + 1);
    for local in 0..=s.degree_v { work.push(rows[vs - s.degree_v + local]); }
    Ok(de_boor(v, vs, s.degree_v, &s.knots_v, &mut work))
}

fn evaluate_homogeneous_derivative_u(s: &NurbsSurface2D, u: f64, v: f64) -> Result<HomogeneousPoint, NurbsSurfaceError> {
    validate_query(s, u, v)?;
    if s.degree_u == 0 { return Err(NurbsSurfaceError::InvalidDegree); }
    let uc = control_count_u(s);
    let vc = control_count_v(s);
    let derivative_count = uc - 1;
    let derivative_knots = &s.knots_u[1..s.knots_u.len() - 1];
    let us = span(u, s.degree_u - 1, derivative_knots, derivative_count);
    let factor_degree = s.degree_u as f64;
    let mut rows = Vec::with_capacity(vc);
    for j in 0..vc {
        let mut work = Vec::with_capacity(s.degree_u);
        for local in 0..s.degree_u {
            let i = us - (s.degree_u - 1) + local;
            let a = i * vc + j;
            let b = (i + 1) * vc + j;
            let pa = s.control_points[a]; let pb = s.control_points[b];
            let wa = s.weights[a]; let wb = s.weights[b];
            let denominator = s.knots_u[i + s.degree_u + 1] - s.knots_u[i + 1];
            if denominator == 0.0 { return Err(NurbsSurfaceError::InvalidDomain); }
            let f = factor_degree / denominator;
            work.push(HomogeneousPoint {
                xw: (pb.x * wb - pa.x * wa) * f,
                yw: (pb.y * wb - pa.y * wa) * f,
                zw: (pb.z * wb - pa.z * wa) * f,
                w: (wb - wa) * f,
            });
        }
        rows.push(de_boor(u, us, s.degree_u - 1, derivative_knots, &mut work));
    }
    let vs = span(v, s.degree_v, &s.knots_v, vc);
    let mut work = Vec::with_capacity(s.degree_v + 1);
    for local in 0..=s.degree_v { work.push(rows[vs - s.degree_v + local]); }
    Ok(de_boor(v, vs, s.degree_v, &s.knots_v, &mut work))
}

fn evaluate_homogeneous_derivative_v(s: &NurbsSurface2D, u: f64, v: f64) -> Result<HomogeneousPoint, NurbsSurfaceError> {
    validate_query(s, u, v)?;
    if s.degree_v == 0 { return Err(NurbsSurfaceError::InvalidDegree); }
    let uc = control_count_u(s);
    let vc = control_count_v(s);
    let derivative_count = vc - 1;
    let derivative_knots = &s.knots_v[1..s.knots_v.len() - 1];
    let vs = span(v, s.degree_v - 1, derivative_knots, derivative_count);
    let us = span(u, s.degree_u, &s.knots_u, uc);
    let factor_degree = s.degree_v as f64;
    let mut columns = Vec::with_capacity(uc);
    for i in 0..uc {
        let mut work = Vec::with_capacity(s.degree_v);
        for local in 0..s.degree_v {
            let j = vs - (s.degree_v - 1) + local;
            let a = i * vc + j;
            let b = i * vc + j + 1;
            let pa = s.control_points[a]; let pb = s.control_points[b];
            let wa = s.weights[a]; let wb = s.weights[b];
            let denominator = s.knots_v[j + s.degree_v + 1] - s.knots_v[j + 1];
            if denominator == 0.0 { return Err(NurbsSurfaceError::InvalidDomain); }
            let f = factor_degree / denominator;
            work.push(HomogeneousPoint {
                xw: (pb.x * wb - pa.x * wa) * f,
                yw: (pb.y * wb - pa.y * wa) * f,
                zw: (pb.z * wb - pa.z * wa) * f,
                w: (wb - wa) * f,
            });
        }
        columns.push(de_boor(v, vs, s.degree_v - 1, derivative_knots, &mut work));
    }
    let mut work = Vec::with_capacity(s.degree_u + 1);
    for local in 0..=s.degree_u { work.push(columns[us - s.degree_u + local]); }
    Ok(de_boor(u, us, s.degree_u, &s.knots_u, &mut work))
}

fn quotient_derivative(base: HomogeneousPoint, derivative: HomogeneousPoint) -> Result<Point3, NurbsSurfaceError> {
    if [base.xw, base.yw, base.zw, base.w, derivative.xw, derivative.yw, derivative.zw, derivative.w].iter().any(|x| !x.is_finite()) {
        return Err(NurbsSurfaceError::Overflow);
    }
    if base.w <= 0.0 { return Err(NurbsSurfaceError::ZeroProjectiveWeight); }
    let w2 = base.w * base.w;
    if !w2.is_finite() || w2 == 0.0 { return Err(NurbsSurfaceError::Overflow); }
    let result = Point3 {
        x: (derivative.xw * base.w - base.xw * derivative.w) / w2,
        y: (derivative.yw * base.w - base.yw * derivative.w) / w2,
        z: (derivative.zw * base.w - base.zw * derivative.w) / w2,
    };
    if !result.x.is_finite() || !result.y.is_finite() || !result.z.is_finite() { return Err(NurbsSurfaceError::Overflow); }
    Ok(result)
}
