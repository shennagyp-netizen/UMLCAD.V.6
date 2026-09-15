use super::surfaces::Point3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingBox3 {
    pub min: Point3,
    pub max: Point3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CircularSweep {
    pub path_start: Point3,
    pub path_end: Point3,
    pub radius: f64,
}

impl CircularSweep {
    pub fn new(path_start: Point3, path_end: Point3, radius: f64) -> Self {
        Self { path_start, path_end, radius }
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        Ok(())
    }

    pub fn length(&self) -> f64 { 0.0 }

    pub fn volume(&self) -> f64 { 0.0 }

    pub fn lateral_area(&self) -> f64 { 0.0 }

    pub fn total_surface_area(&self) -> f64 { 0.0 }

    pub fn bounding_box(&self) -> Result<BoundingBox3, &'static str> {
        Ok(BoundingBox3 {
            min: self.path_start,
            max: self.path_start,
        })
    }

    pub fn surface_point_at(&self, _path_parameter: f64, _angle: f64) -> Result<Point3, &'static str> {
        Ok(self.path_start)
    }

    pub fn translated(&self, dx: f64, dy: f64, dz: f64) -> Result<Self, &'static str> {
        Ok(Self {
            path_start: Point3 {
                x: self.path_start.x + dx,
                y: self.path_start.y + dy,
                z: self.path_start.z + dz,
            },
            path_end: Point3 {
                x: self.path_end.x + dx,
                y: self.path_end.y + dy,
                z: self.path_end.z + dz,
            },
            radius: self.radius,
        })
    }
}
