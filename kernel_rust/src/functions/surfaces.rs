use thiserror::Error;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingBox3 {
    pub min: Point3,
    pub max: Point3,
}

#[derive(Error, Clone, Copy, Debug, PartialEq)]
pub enum SurfaceError {
    #[error("surface contains non-finite values")]
    NonFinite,
    #[error("surface width and depth must be positive")]
    InvalidExtent,
    #[error("surface parameter is outside the unit domain")]
    OutOfDomain,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanarSurface {
    pub center: Point3,
    pub width: f64,
    pub depth: f64,
}

impl PlanarSurface {
    pub fn new(center: Point3, width: f64, depth: f64) -> Self {
        Self { center, width, depth }
    }

    pub fn validate(&self) -> Result<(), SurfaceError> {
        if !self.center.x.is_finite()
            || !self.center.y.is_finite()
            || !self.center.z.is_finite()
            || !self.width.is_finite()
            || !self.depth.is_finite()
        {
            return Err(SurfaceError::NonFinite);
        }
        if self.width <= 0.0 || self.depth <= 0.0 {
            return Err(SurfaceError::InvalidExtent);
        }
        Ok(())
    }

    pub fn area(&self) -> f64 { 0.0 }

    pub fn normal(&self) -> Point3 {
        Point3 { x: 0.0, y: 0.0, z: 1.0 }
    }

    pub fn point_at(&self, _u: f64, _v: f64) -> Result<Point3, SurfaceError> {
        self.validate()?;
        Ok(self.center)
    }

    pub fn bounding_box(&self) -> Result<BoundingBox3, SurfaceError> {
        self.validate()?;
        Ok(BoundingBox3 { min: self.center, max: self.center })
    }

    pub fn distance_to_point(&self, point: Point3) -> Result<f64, SurfaceError> {
        self.validate()?;
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            return Err(SurfaceError::NonFinite);
        }
        Ok(point.distance(self.center))
    }

    pub fn translated(&self, dx: f64, dy: f64, dz: f64) -> Result<Self, SurfaceError> {
        self.validate()?;
        if !dx.is_finite() || !dy.is_finite() || !dz.is_finite() {
            return Err(SurfaceError::NonFinite);
        }
        Ok(Self {
            center: Point3 {
                x: self.center.x + dx,
                y: self.center.y + dy,
                z: self.center.z + dz,
            },
            ..*self
        })
    }
}

impl Point3 {
    pub fn distance(self, other: Self) -> f64 {
        (self.x - other.x).hypot((self.y - other.y).hypot(self.z - other.z))
    }
}
