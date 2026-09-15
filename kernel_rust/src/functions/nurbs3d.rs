#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    fn add(self, other: Self) -> Self {
        Self { x: self.x + other.x, y: self.y + other.y, z: self.z + other.z }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingBox3 {
    pub min: Point3,
    pub max: Point3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Nurbs3DError {
    NonFinite,
    InvalidDegree,
    InvalidControlPointCount,
    InvalidWeightCount,
    InvalidWeight,
    InvalidKnotCount,
    KnotsMustBeNondecreasing,
    NotClamped,
    InvalidDomain,
    OutOfDomain,
    Degenerate,
    ZeroProjectiveWeight,
    Overflow,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NurbsCurve3D {
    pub degree: usize,
    pub control_points: Vec<Point3>,
    pub weights: Vec<f64>,
    pub knots: Vec<f64>,
}

impl NurbsCurve3D {
    pub fn new(degree: usize, control_points: Vec<Point3>, weights: Vec<f64>, knots: Vec<f64>) -> Self {
        Self { degree, control_points, weights, knots }
    }

    pub fn validate(&self) -> Result<(), Nurbs3DError> {
        if self.degree == 0 { return Err(Nurbs3DError::InvalidDegree); }
        if self.control_points.len() < self.degree + 1 { return Err(Nurbs3DError::InvalidControlPointCount); }
        if self.weights.len() != self.control_points.len() { return Err(Nurbs3DError::InvalidWeightCount); }
        if self.knots.len() != self.control_points.len() + self.degree + 1 { return Err(Nurbs3DError::InvalidKnotCount); }
        if self.control_points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite()) { return Err(Nurbs3DError::NonFinite); }
        if self.weights.iter().any(|w| !w.is_finite()) { return Err(Nurbs3DError::NonFinite); }
        if self.weights.iter().any(|w| *w <= 0.0) { return Err(Nurbs3DError::InvalidWeight); }
        if self.knots.iter().any(|k| !k.is_finite()) { return Err(Nurbs3DError::NonFinite); }
        if self.knots.windows(2).any(|pair| pair[1] < pair[0]) { return Err(Nurbs3DError::KnotsMustBeNondecreasing); }
        let start = self.knots[self.degree];
        let end = self.knots[self.control_points.len()];
        if end <= start { return Err(Nurbs3DError::InvalidDomain); }
        if !self.knots[..=self.degree].iter().all(|k| *k == start)
            || !self.knots[self.control_points.len()..].iter().all(|k| *k == end)
        { return Err(Nurbs3DError::NotClamped); }
        if self.control_points.windows(2).all(|pair| pair[0] == pair[1]) { return Err(Nurbs3DError::Degenerate); }
        Ok(())
    }

    pub fn parameter_domain(&self) -> Result<(f64, f64), Nurbs3DError> {
        self.validate()?;
        Ok((self.knots[self.degree], self.knots[self.control_points.len()]))
    }

    pub fn point_at(&self, parameter: f64) -> Result<Point3, Nurbs3DError> {
        self.validate()?;
        if !parameter.is_finite() { return Err(Nurbs3DError::NonFinite); }
        let (start, end) = self.parameter_domain_unchecked();
        if parameter < start || parameter > end { return Err(Nurbs3DError::OutOfDomain); }
        let span = self.find_span(parameter);
        let mut work: Vec<HomogeneousPoint> = (span - self.degree..=span).map(|index| {
            let point = self.control_points[index];
            let weight = self.weights[index];
            HomogeneousPoint { xw: point.x * weight, yw: point.y * weight, zw: point.z * weight, w: weight }
        }).collect();
        for level in 1..=self.degree {
            for j in (level..=self.degree).rev() {
                let i = span - self.degree + j;
                let denominator = self.knots[i + self.degree + 1 - level] - self.knots[i];
                let alpha = if denominator == 0.0 { 0.0 } else { (parameter - self.knots[i]) / denominator };
                work[j] = work[j - 1].lerp(work[j], alpha);
            }
        }
        let result = work[self.degree];
        if !result.xw.is_finite() || !result.yw.is_finite() || !result.zw.is_finite() || !result.w.is_finite() { return Err(Nurbs3DError::Overflow); }
        if result.w <= 0.0 { return Err(Nurbs3DError::ZeroProjectiveWeight); }
        let point = Point3 { x: result.xw / result.w, y: result.yw / result.w, z: result.zw / result.w };
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() { return Err(Nurbs3DError::Overflow); }
        Ok(point)
    }

    pub fn control_hull_bounds(&self) -> Result<BoundingBox3, Nurbs3DError> {
        self.validate()?;
        let min = Point3 {
            x: self.control_points.iter().map(|p| p.x).fold(f64::INFINITY, f64::min),
            y: self.control_points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min),
            z: self.control_points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min),
        };
        let max = Point3 {
            x: self.control_points.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max),
            y: self.control_points.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max),
            z: self.control_points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max),
        };
        Ok(BoundingBox3 { min, max })
    }

    pub fn translated(&self, dx: f64, dy: f64, dz: f64) -> Result<Self, Nurbs3DError> {
        self.validate()?;
        if !dx.is_finite() || !dy.is_finite() || !dz.is_finite() { return Err(Nurbs3DError::NonFinite); }
        let shift = Point3 { x: dx, y: dy, z: dz };
        let control_points = self.control_points.iter().map(|p| p.add(shift)).collect::<Vec<_>>();
        if control_points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite()) { return Err(Nurbs3DError::Overflow); }
        Ok(Self::new(self.degree, control_points, self.weights.clone(), self.knots.clone()))
    }

    fn parameter_domain_unchecked(&self) -> (f64, f64) {
        (self.knots[self.degree], self.knots[self.control_points.len()])
    }

    fn find_span(&self, parameter: f64) -> usize {
        let n = self.control_points.len() - 1;
        if parameter >= self.knots[n + 1] { return n; }
        if parameter <= self.knots[self.degree] { return self.degree; }
        let mut low = self.degree;
        let mut high = n + 1;
        let mut mid = (low + high) / 2;
        while parameter < self.knots[mid] || parameter >= self.knots[mid + 1] {
            if parameter < self.knots[mid] { high = mid; } else { low = mid; }
            mid = (low + high) / 2;
        }
        mid
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn weighted_line_reproduces_3d_endpoints_and_midpoint() {
        let curve = NurbsCurve3D::new(
            1,
            vec![Point3 { x: 0.0, y: 0.0, z: 0.0 }, Point3 { x: 10.0, y: 20.0, z: 30.0 }],
            vec![1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
        );
        assert_eq!(curve.point_at(0.0).unwrap(), Point3 { x: 0.0, y: 0.0, z: 0.0 });
        assert_eq!(curve.point_at(1.0).unwrap(), Point3 { x: 10.0, y: 20.0, z: 30.0 });
        assert_eq!(curve.point_at(0.5).unwrap(), Point3 { x: 5.0, y: 10.0, z: 15.0 });
    }
}
