use umlcad_v6_geometry_api::{GeometryBackend, GeometryError, GeometryResult, ToleranceContext};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3 { pub x: f64, pub y: f64, pub z: f64 }

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

pub type ParameterDomain = ((f64, f64), (f64, f64));
pub type Degrees = (usize, usize);
pub type GridCounts = (usize, usize);

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
    pub fn new(
        degrees: Degrees,
        control_points: Vec<Point3>,
        weights: Vec<f64>,
        counts: GridCounts,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
    ) -> Self {
        Self {
            degree_u: degrees.0,
            degree_v: degrees.1,
            control_points,
            weights,
            count_u: counts.0,
            count_v: counts.1,
            knots_u,
            knots_v,
        }
    }

    pub fn validate(&self) -> Result<(), NurbsSurfaceDefinitionError> {
        if self.degree_u == 0 || self.degree_v == 0 {
            return Err(NurbsSurfaceDefinitionError::InvalidDegree);
        }
        if self.count_u < self.degree_u + 1 || self.count_v < self.degree_v + 1 {
            return Err(NurbsSurfaceDefinitionError::InvalidControlGrid);
        }
        let expected = self
            .count_u
            .checked_mul(self.count_v)
            .ok_or(NurbsSurfaceDefinitionError::InvalidControlGrid)?;
        if self.control_points.len() != expected {
            return Err(NurbsSurfaceDefinitionError::InvalidControlGrid);
        }
        if self.weights.len() != expected {
            return Err(NurbsSurfaceDefinitionError::InvalidWeightCount);
        }
        if self.knots_u.len() != self.count_u + self.degree_u + 1
            || self.knots_v.len() != self.count_v + self.degree_v + 1
        {
            return Err(NurbsSurfaceDefinitionError::InvalidKnotCount);
        }
        if self.control_points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite())
            || self.weights.iter().any(|w| !w.is_finite())
            || self.knots_u.iter().any(|k| !k.is_finite())
            || self.knots_v.iter().any(|k| !k.is_finite())
        {
            return Err(NurbsSurfaceDefinitionError::NonFinite);
        }
        if self.weights.iter().any(|w| *w <= 0.0) {
            return Err(NurbsSurfaceDefinitionError::InvalidWeight);
        }
        if self.knots_u.windows(2).any(|p| p[1] < p[0])
            || self.knots_v.windows(2).any(|p| p[1] < p[0])
        {
            return Err(NurbsSurfaceDefinitionError::KnotsMustBeNondecreasing);
        }
        let u0 = self.knots_u[self.degree_u];
        let u1 = self.knots_u[self.count_u];
        let v0 = self.knots_v[self.degree_v];
        let v1 = self.knots_v[self.count_v];
        if u1 <= u0 || v1 <= v0 {
            return Err(NurbsSurfaceDefinitionError::InvalidDomain);
        }
        if !self.knots_u[..=self.degree_u].iter().all(|k| *k == u0)
            || !self.knots_u[self.count_u..].iter().all(|k| *k == u1)
            || !self.knots_v[..=self.degree_v].iter().all(|k| *k == v0)
            || !self.knots_v[self.count_v..].iter().all(|k| *k == v1)
        {
            return Err(NurbsSurfaceDefinitionError::NotClamped);
        }
        Ok(())
    }

    pub fn parameter_domain(&self) -> Result<ParameterDomain, NurbsSurfaceDefinitionError> {
        self.validate()?;
        Ok((
            (self.knots_u[self.degree_u], self.knots_u[self.count_u]),
            (self.knots_v[self.degree_v], self.knots_v[self.count_v]),
        ))
    }
}

pub trait NurbsSurfaceBackend: GeometryBackend {
    fn nurbs_surface3d(&self, definition: &NurbsSurface3DDefinition, tolerance: ToleranceContext) -> Result<GeometryResult<Self::Shape>, GeometryError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bilinear() -> NurbsSurface3DDefinition {
        NurbsSurface3DDefinition::new(
            (1, 1),
            vec![
                Point3 { x: 0.0, y: 0.0, z: 0.0 }, Point3 { x: 0.0, y: 1.0, z: 1.0 },
                Point3 { x: 1.0, y: 0.0, z: 1.0 }, Point3 { x: 1.0, y: 1.0, z: 2.0 },
            ],
            vec![1.0; 4],
            (2, 2),
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
        )
    }

    #[test]
    fn bilinear_surface_is_valid() {
        assert_eq!(bilinear().parameter_domain().unwrap(), ((0.0, 1.0), (0.0, 1.0)));
    }

    #[test]
    fn invalid_weight_is_rejected() {
        let mut s = bilinear();
        s.weights[2] = 0.0;
        assert_eq!(s.validate(), Err(NurbsSurfaceDefinitionError::InvalidWeight));
    }

    #[test]
    fn nondecreasing_knot_contract_is_enforced() {
        let mut s = bilinear();
        s.knots_v[2] = -1.0;
        assert_eq!(s.validate(), Err(NurbsSurfaceDefinitionError::KnotsMustBeNondecreasing));
    }
}
