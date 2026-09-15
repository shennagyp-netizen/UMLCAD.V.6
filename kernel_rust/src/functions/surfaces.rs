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
        Self {
            center,
            width,
            depth,
        }
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
        if !self.area().is_finite() {
            return Err(SurfaceError::NonFinite);
        }
        let half_width = self.width * 0.5;
        let half_depth = self.depth * 0.5;
        let bounds = [
            self.center.x - half_width,
            self.center.x + half_width,
            self.center.y - half_depth,
            self.center.y + half_depth,
        ];
        if bounds.iter().any(|value| !value.is_finite()) {
            return Err(SurfaceError::NonFinite);
        }
        Ok(())
    }

    pub fn area(&self) -> f64 {
        self.width * self.depth
    }

    pub fn normal(&self) -> Point3 {
        Point3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        }
    }

    pub fn point_at(&self, u: f64, v: f64) -> Result<Point3, SurfaceError> {
        self.validate()?;
        if !u.is_finite()
            || !v.is_finite()
            || !(0.0..=1.0).contains(&u)
            || !(0.0..=1.0).contains(&v)
        {
            return Err(SurfaceError::OutOfDomain);
        }
        Ok(Point3 {
            x: self.center.x + (u - 0.5) * self.width,
            y: self.center.y + (v - 0.5) * self.depth,
            z: self.center.z,
        })
    }

    pub fn bounding_box(&self) -> Result<BoundingBox3, SurfaceError> {
        self.validate()?;
        Ok(BoundingBox3 {
            min: Point3 {
                x: self.center.x - self.width * 0.5,
                y: self.center.y - self.depth * 0.5,
                z: self.center.z,
            },
            max: Point3 {
                x: self.center.x + self.width * 0.5,
                y: self.center.y + self.depth * 0.5,
                z: self.center.z,
            },
        })
    }

    pub fn distance_to_point(&self, point: Point3) -> Result<f64, SurfaceError> {
        self.validate()?;
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            return Err(SurfaceError::NonFinite);
        }
        let x_min = self.center.x - self.width * 0.5;
        let x_max = self.center.x + self.width * 0.5;
        let y_min = self.center.y - self.depth * 0.5;
        let y_max = self.center.y + self.depth * 0.5;
        let dx = if point.x < x_min {
            x_min - point.x
        } else if point.x > x_max {
            point.x - x_max
        } else {
            0.0
        };
        let dy = if point.y < y_min {
            y_min - point.y
        } else if point.y > y_max {
            point.y - y_max
        } else {
            0.0
        };
        Ok(dx.hypot(dy).hypot(point.z - self.center.z))
    }

    pub fn translated(&self, dx: f64, dy: f64, dz: f64) -> Result<Self, SurfaceError> {
        self.validate()?;
        if !dx.is_finite() || !dy.is_finite() || !dz.is_finite() {
            return Err(SurfaceError::NonFinite);
        }
        let center = Point3 {
            x: self.center.x + dx,
            y: self.center.y + dy,
            z: self.center.z + dz,
        };
        if !center.x.is_finite() || !center.y.is_finite() || !center.z.is_finite() {
            return Err(SurfaceError::NonFinite);
        }
        Ok(Self { center, ..*self })
    }
}

impl Point3 {
    pub fn distance(self, other: Self) -> f64 {
        (self.x - other.x).hypot((self.y - other.y).hypot(self.z - other.z))
    }
}
