use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, GeometryResult, ToleranceContext};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    fn scale(self, s: f64) -> Self { Self { x: self.x * s, y: self.y * s, z: self.z * s } }
    fn add(self, other: Self) -> Self { Self { x: self.x + other.x, y: self.y + other.y, z: self.z + other.z } }
    pub fn dot(self, other: Self) -> f64 { self.x * other.x + self.y * other.y + self.z * other.z }
    pub fn cross(self, other: Self) -> Self {
        Self { x: self.y * other.z - self.z * other.y, y: self.z * other.x - self.x * other.z, z: self.x * other.y - self.y * other.x }
    }
    pub fn norm(self) -> f64 { self.x.hypot(self.y.hypot(self.z)) }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NurbsSurfaceDefinitionError {
    NonFinite,
    InvalidDegree,
    InvalidControlGrid,
    InvalidWeightCount,
    InvalidWeight,
    InvalidKnotCount,
    KnotsMustBeNondecreasing,
    NotClamped,
    InvalidDomain,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NurbsSurfaceEvaluationError {
    NonFinite,
    OutOfDomain,
    InsufficientContinuity,
    ZeroProjectiveWeight,
    Overflow,
    DegenerateNormal,
}

pub type ParameterDomain = ((f64, f64), (f64, f64));
pub type Degrees = (usize, usize);
pub type GridCounts = (usize, usize);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NurbsSurfaceDifferential {
    pub point: Point3,
    pub du: Point3,
    pub dv: Point3,
    pub duu: Point3,
    pub duv: Point3,
    pub dvv: Point3,
}

impl NurbsSurfaceDifferential {
    pub fn normal(&self) -> Result<Point3, NurbsSurfaceEvaluationError> {
        let cross = self.du.cross(self.dv);
        let magnitude = cross.norm();
        if !magnitude.is_finite() { return Err(NurbsSurfaceEvaluationError::Overflow); }
        if magnitude == 0.0 { return Err(NurbsSurfaceEvaluationError::DegenerateNormal); }
        Ok(cross.scale(1.0 / magnitude))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct NurbsSurface3DDefinition {
    pub degree_u: usize,
    pub degree_v: usize,
    pub control_points: Vec<Point3>,
    pub weights: Vec<f64>,
    pub count_u: usize,
    pub count_v: usize,
    pub knots_u: Vec<f64>,
    pub knots_v: Vec<f64>,
}

impl NurbsSurface3DDefinition {
    /// `degrees` is `(degree_u, degree_v)` and `counts` is `(count_u, count_v)`.
    pub fn new(degrees: Degrees, control_points: Vec<Point3>, weights: Vec<f64>, counts: GridCounts, knots_u: Vec<f64>, knots_v: Vec<f64>) -> Self {
        Self { degree_u: degrees.0, degree_v: degrees.1, control_points, weights, count_u: counts.0, count_v: counts.1, knots_u, knots_v }
    }

    pub fn validate(&self) -> Result<(), NurbsSurfaceDefinitionError> {
        if self.degree_u == 0 || self.degree_v == 0 { return Err(NurbsSurfaceDefinitionError::InvalidDegree); }
        if self.count_u < self.degree_u + 1 || self.count_v < self.degree_v + 1 { return Err(NurbsSurfaceDefinitionError::InvalidControlGrid); }
        let expected = self.count_u.checked_mul(self.count_v).ok_or(NurbsSurfaceDefinitionError::InvalidControlGrid)?;
        if self.control_points.len() != expected { return Err(NurbsSurfaceDefinitionError::InvalidControlGrid); }
        if self.weights.len() != expected { return Err(NurbsSurfaceDefinitionError::InvalidWeightCount); }
        if self.knots_u.len() != self.count_u + self.degree_u + 1 || self.knots_v.len() != self.count_v + self.degree_v + 1 { return Err(NurbsSurfaceDefinitionError::InvalidKnotCount); }
        if self.control_points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite()) || self.weights.iter().any(|w| !w.is_finite()) || self.knots_u.iter().any(|k| !k.is_finite()) || self.knots_v.iter().any(|k| !k.is_finite()) { return Err(NurbsSurfaceDefinitionError::NonFinite); }
        if self.weights.iter().any(|w| *w <= 0.0) { return Err(NurbsSurfaceDefinitionError::InvalidWeight); }
        if self.knots_u.windows(2).any(|p| p[1] < p[0]) || self.knots_v.windows(2).any(|p| p[1] < p[0]) { return Err(NurbsSurfaceDefinitionError::KnotsMustBeNondecreasing); }
        let u0 = self.knots_u[self.degree_u]; let u1 = self.knots_u[self.count_u]; let v0 = self.knots_v[self.degree_v]; let v1 = self.knots_v[self.count_v];
        if u1 <= u0 || v1 <= v0 { return Err(NurbsSurfaceDefinitionError::InvalidDomain); }
        if !self.knots_u[..=self.degree_u].iter().all(|k| *k == u0) || !self.knots_u[self.count_u..].iter().all(|k| *k == u1) || !self.knots_v[..=self.degree_v].iter().all(|k| *k == v0) || !self.knots_v[self.count_v..].iter().all(|k| *k == v1) { return Err(NurbsSurfaceDefinitionError::NotClamped); }
        Ok(())
    }

    pub fn parameter_domain(&self) -> Result<ParameterDomain, NurbsSurfaceDefinitionError> {
        self.validate()?;
        Ok(((self.knots_u[self.degree_u], self.knots_u[self.count_u]), (self.knots_v[self.degree_v], self.knots_v[self.count_v])))
    }

    pub fn differential_at(&self, u: f64, v: f64) -> Result<NurbsSurfaceDifferential, NurbsSurfaceEvaluationError> {
        self.validate().map_err(|_| NurbsSurfaceEvaluationError::Overflow)?;
        if !u.is_finite() || !v.is_finite() { return Err(NurbsSurfaceEvaluationError::NonFinite); }
        let ((u0, u1), (v0, v1)) = self.parameter_domain().map_err(|_| NurbsSurfaceEvaluationError::Overflow)?;
        if u < u0 || u > u1 || v < v0 || v > v1 { return Err(NurbsSurfaceEvaluationError::OutOfDomain); }
        require_second_order_continuity(u, self.degree_u, &self.knots_u, self.count_u)?;
        require_second_order_continuity(v, self.degree_v, &self.knots_v, self.count_v)?;

        let bu = basis_derivatives(u, self.degree_u, &self.knots_u, self.count_u);
        let bv = basis_derivatives(v, self.degree_v, &self.knots_v, self.count_v);
        let mut homogeneous = [[Homogeneous::zero(); 3]; 3];
        for i in 0..self.count_u {
            for j in 0..self.count_v {
                let index = i * self.count_v + j;
                let p = self.control_points[index]; let w = self.weights[index];
                let h = Homogeneous { xw: p.x * w, yw: p.y * w, zw: p.z * w, w };
                for ku in 0..=2 { for kv in 0..=2 { homogeneous[ku][kv] = homogeneous[ku][kv].add(h.scale(bu[i][ku] * bv[j][kv])); } }
            }
        }
        rationalize_differential(homogeneous)
    }
}

pub trait NurbsSurfaceBackend: GeometryBackend {
    fn nurbs_surface3d(&self, definition: &NurbsSurface3DDefinition, tolerance: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError>;
}

pub trait NurbsSurfaceDifferentialBackend: NurbsSurfaceBackend {
    fn nurbs_surface3d_differential_at(&self, shape: &Self::Shape, u: f64, v: f64, tolerance: ToleranceContext) -> Result<NurbsSurfaceDifferential, GeometryError>;
}

#[derive(Clone, Copy)]
struct Homogeneous { xw: f64, yw: f64, zw: f64, w: f64 }
impl Homogeneous {
    const fn zero() -> Self { Self { xw: 0.0, yw: 0.0, zw: 0.0, w: 0.0 } }
    fn add(self, other: Self) -> Self { Self { xw: self.xw + other.xw, yw: self.yw + other.yw, zw: self.zw + other.zw, w: self.w + other.w } }
    fn scale(self, s: f64) -> Self { Self { xw: self.xw * s, yw: self.yw * s, zw: self.zw * s, w: self.w * s } }
}

fn knot_continuity_order(parameter: f64, degree: usize, knots: &[f64], count: usize) -> usize {
    let multiplicity = knots.iter().filter(|k| **k == parameter).count();
    if parameter == knots[degree] || parameter == knots[count] { degree } else { degree.saturating_sub(multiplicity) }
}

fn require_second_order_continuity(parameter: f64, degree: usize, knots: &[f64], count: usize) -> Result<(), NurbsSurfaceEvaluationError> {
    if parameter == knots[degree] || parameter == knots[count] { return Ok(()); }
    if knot_continuity_order(parameter, degree, knots, count) < 2 { return Err(NurbsSurfaceEvaluationError::InsufficientContinuity); }
    Ok(())
}

fn basis_derivatives(t: f64, degree: usize, knots: &[f64], count: usize) -> Vec<[f64; 3]> {
    (0..count).map(|i| [basis_derivative_recursive(i, degree, t, knots, 0), basis_derivative_recursive(i, degree, t, knots, 1), basis_derivative_recursive(i, degree, t, knots, 2)]).collect()
}

fn basis_derivative_recursive(i: usize, degree: usize, t: f64, knots: &[f64], order: usize) -> f64 {
    if order > degree { return 0.0; }
    if degree == 0 {
        if order != 0 { return 0.0; }
        let last = knots.len() - 1;
        return if (knots[i] <= t && t < knots[i + 1]) || (t == knots[last] && i + 1 == last) { 1.0 } else { 0.0 };
    }
    if order == 0 {
        let left_den = knots[i + degree] - knots[i];
        let right_den = knots[i + degree + 1] - knots[i + 1];
        let left = if left_den == 0.0 { 0.0 } else { (t - knots[i]) / left_den * basis_derivative_recursive(i, degree - 1, t, knots, 0) };
        let right = if right_den == 0.0 { 0.0 } else { (knots[i + degree + 1] - t) / right_den * basis_derivative_recursive(i + 1, degree - 1, t, knots, 0) };
        return left + right;
    }
    let left_den = knots[i + degree] - knots[i];
    let right_den = knots[i + degree + 1] - knots[i + 1];
    let left = if left_den == 0.0 { 0.0 } else { degree as f64 / left_den * basis_derivative_recursive(i, degree - 1, t, knots, order - 1) };
    let right = if right_den == 0.0 { 0.0 } else { degree as f64 / right_den * basis_derivative_recursive(i + 1, degree - 1, t, knots, order - 1) };
    left - right
}

fn rationalize_differential(h: [[Homogeneous; 3]; 3]) -> Result<NurbsSurfaceDifferential, NurbsSurfaceEvaluationError> {
    if h.iter().flatten().any(|x| !x.xw.is_finite() || !x.yw.is_finite() || !x.zw.is_finite() || !x.w.is_finite()) { return Err(NurbsSurfaceEvaluationError::Overflow); }
    let w = h[0][0].w;
    if w <= 0.0 || !w.is_finite() { return Err(NurbsSurfaceEvaluationError::ZeroProjectiveWeight); }
    let point = Point3 { x: h[0][0].xw / w, y: h[0][0].yw / w, z: h[0][0].zw / w };
    let pu = quotient_first(h[1][0], h[0][0], point)?;
    let pv = quotient_first(h[0][1], h[0][0], point)?;
    let puu = quotient_second(h[2][0], h[1][0], h[0][0], pu, point);
    let puv = quotient_mixed(h[1][1], h[1][0], h[0][1], h[0][0], pu, pv, point);
    let pvv = quotient_second(h[0][2], h[0][1], h[0][0], pv, point);
    let result = NurbsSurfaceDifferential { point, du: pu?, dv: pv?, duu: puu?, duv: puv?, dvv: pvv? };
    if [result.point, result.du, result.dv, result.duu, result.duv, result.dvv].iter().flat_map(|p| [p.x, p.y, p.z]).any(|x| !x.is_finite()) { return Err(NurbsSurfaceEvaluationError::Overflow); }
    Ok(result)
}

fn quotient_first(d: Homogeneous, base: Homogeneous, point: Point3) -> Result<Point3, NurbsSurfaceEvaluationError> {
    if !base.w.is_finite() || base.w <= 0.0 { return Err(NurbsSurfaceEvaluationError::ZeroProjectiveWeight); }
    Ok(Point3 { x: (d.xw - point.x * d.w) / base.w, y: (d.yw - point.y * d.w) / base.w, z: (d.zw - point.z * d.w) / base.w })
}

fn quotient_second(d2: Homogeneous, d1: Homogeneous, base: Homogeneous, p1: Point3, p: Point3) -> Result<Point3, NurbsSurfaceEvaluationError> {
    if !base.w.is_finite() || base.w <= 0.0 { return Err(NurbsSurfaceEvaluationError::ZeroProjectiveWeight); }
    Ok(Point3 { x: (d2.xw - 2.0 * d1.w * p1.x - d2.w * p.x) / base.w, y: (d2.yw - 2.0 * d1.w * p1.y - d2.w * p.y) / base.w, z: (d2.zw - 2.0 * d1.w * p1.z - d2.w * p.z) / base.w })
}

fn quotient_mixed(d2: Homogeneous, du_h: Homogeneous, dv_h: Homogeneous, base: Homogeneous, du: Point3, dv: Point3, p: Point3) -> Result<Point3, NurbsSurfaceEvaluationError> {
    if !base.w.is_finite() || base.w <= 0.0 { return Err(NurbsSurfaceEvaluationError::ZeroProjectiveWeight); }
    Ok(Point3 { x: (d2.xw - du_h.w * dv.x - dv_h.w * du.x - d2.w * p.x) / base.w, y: (d2.yw - du_h.w * dv.y - dv_h.w * du.y - d2.w * p.y) / base.w, z: (d2.zw - du_h.w * dv.z - dv_h.w * du.z - d2.w * p.z) / base.w })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn bilinear() -> NurbsSurface3DDefinition {
        NurbsSurface3DDefinition::new((1, 1), vec![Point3 { x: 0.0, y: 0.0, z: 0.0 }, Point3 { x: 0.0, y: 1.0, z: 1.0 }, Point3 { x: 1.0, y: 0.0, z: 1.0 }, Point3 { x: 1.0, y: 1.0, z: 2.0 }], vec![1.0; 4], (2, 2), vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0])
    }
    #[test] fn bilinear_surface_is_valid() { assert_eq!(bilinear().parameter_domain().unwrap(), ((0.0, 1.0), (0.0, 1.0))); }
    #[test] fn bilinear_differential_is_exact() {
        let d = bilinear().differential_at(0.25, 0.75).unwrap();
        assert_eq!(d.point, Point3 { x: 0.25, y: 0.75, z: 1.0 });
        assert_eq!(d.du, Point3 { x: 1.0, y: 0.0, z: 1.0 });
        assert_eq!(d.dv, Point3 { x: 0.0, y: 1.0, z: 1.0 });
        assert_eq!(d.duu, Point3 { x: 0.0, y: 0.0, z: 0.0 });
        assert_eq!(d.duv, Point3 { x: 0.0, y: 0.0, z: 0.0 });
        assert_eq!(d.dvv, Point3 { x: 0.0, y: 0.0, z: 0.0 });
        let n = d.normal().unwrap();
        assert!((n.norm() - 1.0).abs() < 1e-14);
        assert!(n.dot(d.du).abs() < 1e-14 && n.dot(d.dv).abs() < 1e-14);
    }
    #[test] fn out_of_domain_is_rejected() { assert_eq!(bilinear().differential_at(-0.1, 0.5), Err(NurbsSurfaceEvaluationError::OutOfDomain)); }
    #[test] fn invalid_weight_is_rejected() { let mut s = bilinear(); s.weights[2] = 0.0; assert!(s.differential_at(0.5, 0.5).is_err()); }
    #[test] fn nondecreasing_knot_contract_is_enforced() { let mut s = bilinear(); s.knots_v[2] = -1.0; assert_eq!(s.validate(), Err(NurbsSurfaceDefinitionError::KnotsMustBeNondecreasing)); }
}
