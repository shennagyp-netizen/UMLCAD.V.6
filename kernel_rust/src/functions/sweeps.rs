use thiserror::Error;

use super::surfaces::Point3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingBox3 {
    pub min: Point3,
    pub max: Point3,
}

#[derive(Error, Clone, Copy, Debug, PartialEq)]
pub enum SweepError {
    #[error("sweep contains non-finite values")]
    NonFinite,
    #[error("sweep radius must be positive")]
    InvalidRadius,
    #[error("sweep path is degenerate")]
    DegeneratePath,
    #[error("sweep path parameter is outside the unit domain")]
    OutOfDomain,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CircularSweep {
    pub path_start: Point3,
    pub path_end: Point3,
    pub radius: f64,
}

fn sub(a: Point3, b: Point3) -> Point3 {
    Point3 { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }
}

fn add(a: Point3, b: Point3) -> Point3 {
    Point3 { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }
}

fn scale(a: Point3, s: f64) -> Point3 {
    Point3 { x: a.x * s, y: a.y * s, z: a.z * s }
}

fn dot(a: Point3, b: Point3) -> f64 { a.x * b.x + a.y * b.y + a.z * b.z }

fn cross(a: Point3, b: Point3) -> Point3 {
    Point3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}

fn norm(a: Point3) -> f64 { dot(a, a).sqrt() }

fn normalized(a: Point3) -> Result<Point3, SweepError> {
    let n = norm(a);
    if !n.is_finite() { return Err(SweepError::NonFinite); }
    if n == 0.0 { return Err(SweepError::DegeneratePath); }
    Ok(scale(a, 1.0 / n))
}

fn finite_point(p: Point3) -> bool { p.x.is_finite() && p.y.is_finite() && p.z.is_finite() }

impl CircularSweep {
    pub fn new(path_start: Point3, path_end: Point3, radius: f64) -> Self {
        Self { path_start, path_end, radius }
    }

    fn axis_and_length(&self) -> Result<(Point3, f64), SweepError> {
        if !finite_point(self.path_start) || !finite_point(self.path_end) || !self.radius.is_finite() {
            return Err(SweepError::NonFinite);
        }
        if self.radius <= 0.0 { return Err(SweepError::InvalidRadius); }
        let delta = sub(self.path_end, self.path_start);
        let length = norm(delta);
        if !length.is_finite() { return Err(SweepError::NonFinite); }
        if length == 0.0 { return Err(SweepError::DegeneratePath); }
        Ok((scale(delta, 1.0 / length), length))
    }

    pub fn validate(&self) -> Result<(), SweepError> {
        let (axis, length) = self.axis_and_length()?;
        let values = [
            length,
            self.volume(),
            self.lateral_area(),
            self.total_surface_area(),
            axis.x,
            axis.y,
            axis.z,
        ];
        if values.iter().any(|v| !v.is_finite()) { return Err(SweepError::NonFinite); }
        self.bounding_box()?;
        Ok(())
    }

    pub fn length(&self) -> f64 {
        norm(sub(self.path_end, self.path_start))
    }

    pub fn volume(&self) -> f64 {
        std::f64::consts::PI * self.radius * self.radius * self.length()
    }

    pub fn lateral_area(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.radius * self.length()
    }

    pub fn total_surface_area(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.radius * (self.length() + self.radius)
    }

    pub fn bounding_box(&self) -> Result<BoundingBox3, SweepError> {
        let (axis, _) = self.axis_and_length()?;
        let ext = Point3 {
            x: self.radius * (1.0 - axis.x * axis.x).max(0.0).sqrt(),
            y: self.radius * (1.0 - axis.y * axis.y).max(0.0).sqrt(),
            z: self.radius * (1.0 - axis.z * axis.z).max(0.0).sqrt(),
        };
        let min = Point3 {
            x: self.path_start.x.min(self.path_end.x) - ext.x,
            y: self.path_start.y.min(self.path_end.y) - ext.y,
            z: self.path_start.z.min(self.path_end.z) - ext.z,
        };
        let max = Point3 {
            x: self.path_start.x.max(self.path_end.x) + ext.x,
            y: self.path_start.y.max(self.path_end.y) + ext.y,
            z: self.path_start.z.max(self.path_end.z) + ext.z,
        };
        if !finite_point(min) || !finite_point(max) { return Err(SweepError::NonFinite); }
        Ok(BoundingBox3 { min, max })
    }

    fn frame(&self) -> Result<(Point3, Point3, Point3), SweepError> {
        let (axis, _) = self.axis_and_length()?;
        let candidates = [
            Point3 { x: 1.0, y: 0.0, z: 0.0 },
            Point3 { x: 0.0, y: 1.0, z: 0.0 },
            Point3 { x: 0.0, y: 0.0, z: 1.0 },
        ];
        let reference = *candidates
            .iter()
            .min_by(|a, b| dot(axis, **a).abs().total_cmp(&dot(axis, **b).abs()))
            .ok_or(SweepError::DegeneratePath)?;
        let radial_u = normalized(cross(axis, reference))?;
        let radial_v = normalized(cross(axis, radial_u))?;
        Ok((axis, radial_u, radial_v))
    }

    pub fn surface_point_at(&self, path_parameter: f64, angle: f64) -> Result<Point3, SweepError> {
        if !path_parameter.is_finite() || !angle.is_finite() { return Err(SweepError::NonFinite); }
        if !(0.0..=1.0).contains(&path_parameter) { return Err(SweepError::OutOfDomain); }
        let (axis, u, v) = self.frame()?;
        let center = add(self.path_start, scale(axis, self.length() * path_parameter));
        let radial = add(scale(u, self.radius * angle.cos()), scale(v, self.radius * angle.sin()));
        let result = add(center, radial);
        if !finite_point(result) { return Err(SweepError::NonFinite); }
        Ok(result)
    }

    pub fn translated(&self, dx: f64, dy: f64, dz: f64) -> Result<Self, SweepError> {
        self.validate()?;
        if !dx.is_finite() || !dy.is_finite() || !dz.is_finite() { return Err(SweepError::NonFinite); }
        let start = Point3 { x: self.path_start.x + dx, y: self.path_start.y + dy, z: self.path_start.z + dz };
        let end = Point3 { x: self.path_end.x + dx, y: self.path_end.y + dy, z: self.path_end.z + dz };
        if !finite_point(start) || !finite_point(end) { return Err(SweepError::NonFinite); }
        Ok(Self { path_start: start, path_end: end, radius: self.radius })
    }
}
