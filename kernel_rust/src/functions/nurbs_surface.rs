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

    fn scale(self, factor: f64) -> Self {
        Self { x: self.x * factor, y: self.y * factor, z: self.z * factor }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingBox3 {
    pub min: Point3,
    pub max: Point3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NurbsSurfaceError {
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
pub struct NurbsSurface2D {
    pub degree_u: usize,
    pub degree_v: usize,
    pub control_points: Vec<Point3>,
    pub weights: Vec<f64>,
    pub knots_u: Vec<f64>,
    pub knots_v: Vec<f64>,
}

impl NurbsSurface2D {
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        control_points: Vec<Point3>,
        weights: Vec<f64>,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
    ) -> Self {
        Self { degree_u, degree_v, control_points, weights, knots_u, knots_v }
    }

    pub fn validate(&self) -> Result<(), NurbsSurfaceError> {
        if self.degree_u == 0 || self.degree_v == 0 {
            return Err(NurbsSurfaceError::InvalidDegree);
        }
        let u_count = self.control_count_u();
        let v_count = self.control_count_v();
        if u_count < self.degree_u + 1 || v_count < self.degree_v + 1 {
            return Err(NurbsSurfaceError::InvalidControlPointCount);
        }
        if u_count * v_count != self.control_points.len() {
            return Err(NurbsSurfaceError::InvalidControlPointCount);
        }
        if self.weights.len() != self.control_points.len() {
            return Err(NurbsSurfaceError::InvalidWeightCount);
        }
        let expected_u_knots = u_count + self.degree_u + 1;
        let expected_v_knots = v_count + self.degree_v + 1;
        if self.knots_u.len() != expected_u_knots || self.knots_v.len() != expected_v_knots {
            return Err(NurbsSurfaceError::InvalidKnotCount);
        }
        if self.control_points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite()) {
            return Err(NurbsSurfaceError::NonFinite);
        }
        if self.weights.iter().any(|w| !w.is_finite()) {
            return Err(NurbsSurfaceError::NonFinite);
        }
        if self.weights.iter().any(|w| *w <= 0.0) {
            return Err(NurbsSurfaceError::InvalidWeight);
        }
        validate_knot_vector(&self.knots_u)?;
        validate_knot_vector(&self.knots_v)?;
        validate_clamped(&self.knots_u, self.degree_u, u_count)?;
        validate_clamped(&self.knots_v, self.degree_v, v_count)?;
        Ok(())
    }

    pub fn parameter_domain(&self) -> Result<(f64, f64, f64, f64), NurbsSurfaceError> {
        self.validate()?;
        Ok((
            self.knots_u[self.degree_u],
            self.knots_u[self.control_count_u()],
            self.knots_v[self.degree_v],
            self.knots_v[self.control_count_v()],
        ))
    }

    pub fn point_at(&self, u: f64, v: f64) -> Result<Point3, NurbsSurfaceError> {
        self.validate()?;
        if !u.is_finite() || !v.is_finite() {
            return Err(NurbsSurfaceError::NonFinite);
        }
        let (u0, u1, v0, v1) = self.parameter_domain_unchecked();
        if u < u0 || u > u1 || v < v0 || v > v1 {
            return Err(NurbsSurfaceError::OutOfDomain);
        }

        let u_count = self.control_count_u();
        let v_count = self.control_count_v();
        let u_span = find_span(u, self.degree_u, &self.knots_u, u_count);
        let mut row_reduced = Vec::with_capacity(v_count);

        for v_index in 0..v_count {
            let mut work = Vec::with_capacity(self.degree_u + 1);
            for local in 0..=self.degree_u {
                let u_index = u_span - self.degree_u + local;
                let index = u_index * v_count + v_index;
                let point = self.control_points[index];
                let weight = self.weights[index];
                work.push(HomogeneousPoint {
                    xw: point.x * weight,
                    yw: point.y * weight,
                    zw: point.z * weight,
                    w: weight,
                });
            }
            row_reduced.push(de_boor_homogeneous(u, u_span, self.degree_u, &self.knots_u, &mut work));
        }

        let v_span = find_span(v, self.degree_v, &self.knots_v, v_count);
        let mut work = Vec::with_capacity(self.degree_v + 1);
        for local in 0..=self.degree_v {
            work.push(row_reduced[v_span - self.degree_v + local]);
        }
        let result = de_boor_homogeneous(v, v_span, self.degree_v, &self.knots_v, &mut work);
        if !result.xw.is_finite() || !result.yw.is_finite() || !result.zw.is_finite() || !result.w.is_finite() {
            return Err(NurbsSurfaceError::Overflow);
        }
        if result.w <= 0.0 {
            return Err(NurbsSurfaceError::ZeroProjectiveWeight);
        }
        let point = Point3 { x: result.xw / result.w, y: result.yw / result.w, z: result.zw / result.w };
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            return Err(NurbsSurfaceError::Overflow);
        }
        Ok(point)
    }

    pub fn control_hull_bounds(&self) -> Result<BoundingBox3, NurbsSurfaceError> {
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

    pub fn translated(&self, dx: f64, dy: f64, dz: f64) -> Result<Self, NurbsSurfaceError> {
        self.validate()?;
        if !dx.is_finite() || !dy.is_finite() || !dz.is_finite() {
            return Err(NurbsSurfaceError::NonFinite);
        }
        let delta = Point3 { x: dx, y: dy, z: dz };
        let control_points = self.control_points.iter().map(|p| p.add(delta)).collect::<Vec<_>>();
        if control_points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite()) {
            return Err(NurbsSurfaceError::Overflow);
        }
        Ok(Self::new(
            self.degree_u,
            self.degree_v,
            control_points,
            self.weights.clone(),
            self.knots_u.clone(),
            self.knots_v.clone(),
        ))
    }

    fn control_count_u(&self) -> usize {
        // The grid shape is determined from the knot vector and degree.
        self.knots_u.len() - self.degree_u - 1
    }

    fn control_count_v(&self) -> usize {
        self.knots_v.len() - self.degree_v - 1
    }

    fn parameter_domain_unchecked(&self) -> (f64, f64, f64, f64) {
        (
            self.knots_u[self.degree_u],
            self.knots_u[self.control_count_u()],
            self.knots_v[self.degree_v],
            self.knots_v[self.control_count_v()],
        )
    }
}

#[derive(Clone, Copy)]
struct HomogeneousPoint {
    xw: f64,
    yw: f64,
    zw: f64,
    w: f64,
}

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

fn validate_knot_vector(knots: &[f64]) -> Result<(), NurbsSurfaceError> {
    if knots.iter().any(|k| !k.is_finite()) {
        return Err(NurbsSurfaceError::NonFinite);
    }
    if knots.len() < 2 || knots.windows(2).any(|pair| pair[1] < pair[0]) {
        return Err(NurbsSurfaceError::KnotsMustBeNondecreasing);
    }
    Ok(())
}

fn validate_clamped(knots: &[f64], degree: usize, control_count: usize) -> Result<(), NurbsSurfaceError> {
    if knots.len() <= control_count {
        return Err(NurbsSurfaceError::InvalidDomain);
    }
    let start = knots[degree];
    let end = knots[control_count];
    if end <= start {
        return Err(NurbsSurfaceError::InvalidDomain);
    }
    if !knots[..=degree].iter().all(|k| *k == start)
        || !knots[control_count..].iter().all(|k| *k == end)
    {
        return Err(NurbsSurfaceError::NotClamped);
    }
    Ok(())
}

fn find_span(parameter: f64, degree: usize, knots: &[f64], control_count: usize) -> usize {
    let n = control_count - 1;
    let end = knots[n + 1];
    if parameter >= end {
        return n;
    }
    if parameter <= knots[degree] {
        return degree;
    }
    let mut low = degree;
    let mut high = n + 1;
    let mut mid = (low + high) / 2;
    while parameter < knots[mid] || parameter >= knots[mid + 1] {
        if parameter < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

fn de_boor_homogeneous(
    parameter: f64,
    span: usize,
    degree: usize,
    knots: &[f64],
    work: &mut [HomogeneousPoint],
) -> HomogeneousPoint {
    for level in 1..=degree {
        for j in (level..=degree).rev() {
            let i = span - degree + j;
            let left = knots[i];
            let right = knots[i + degree + 1 - level];
            let denominator = right - left;
            let alpha = if denominator == 0.0 { 0.0 } else { (parameter - left) / denominator };
            work[j] = work[j - 1].lerp(work[j], alpha);
        }
    }
    work[degree]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bilinear_patch_matches_known_point() {
        let surface = NurbsSurface2D::new(
            1,
            1,
            vec![
                Point3 { x: 0.0, y: 0.0, z: 0.0 },
                Point3 { x: 0.0, y: 1.0, z: 1.0 },
                Point3 { x: 1.0, y: 0.0, z: 2.0 },
                Point3 { x: 1.0, y: 1.0, z: 3.0 },
            ],
            vec![1.0; 4],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
        );
        let q = surface.point_at(0.25, 0.75).unwrap();
        assert!((q.x - 0.25).abs() < 1e-12);
        assert!((q.y - 0.75).abs() < 1e-12);
        assert!((q.z - 1.25).abs() < 1e-12);
    }
}
