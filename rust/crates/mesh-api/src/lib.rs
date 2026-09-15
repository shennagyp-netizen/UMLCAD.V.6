#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vertex { pub x: f64, pub y: f64, pub z: f64 }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Triangle { pub a: u32, pub b: u32, pub c: u32 }

#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub triangles: Vec<Triangle>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TessellationOptions {
    pub linear_deflection: f64,
    pub angular_deflection_radians: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshError {
    NonFinite,
    InvalidDeflection,
    IndexOutOfBounds,
    DegenerateTriangle,
    NonManifoldConnectivity,
}

impl TessellationOptions {
    pub fn validate(&self) -> Result<(), MeshError> {
        if !self.linear_deflection.is_finite() || !self.angular_deflection_radians.is_finite() { return Err(MeshError::NonFinite); }
        if self.linear_deflection <= 0.0 || self.angular_deflection_radians <= 0.0 { return Err(MeshError::InvalidDeflection); }
        Ok(())
    }
}

impl Mesh {
    pub fn validate(&self) -> Result<(), MeshError> {
        if self.vertices.iter().any(|v| !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite()) { return Err(MeshError::NonFinite); }
        for t in &self.triangles {
            if t.a as usize >= self.vertices.len() || t.b as usize >= self.vertices.len() || t.c as usize >= self.vertices.len() { return Err(MeshError::IndexOutOfBounds); }
            if t.a == t.b || t.b == t.c || t.a == t.c { return Err(MeshError::DegenerateTriangle); }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn options_are_fail_closed() {
        assert!(TessellationOptions { linear_deflection: 0.1, angular_deflection_radians: 0.2 }.validate().is_ok());
        assert_eq!(TessellationOptions { linear_deflection: 0.0, angular_deflection_radians: 0.2 }.validate(), Err(MeshError::InvalidDeflection));
    }
    #[test]
    fn mesh_rejects_invalid_triangle_indices() {
        let mesh = Mesh { vertices: vec![Vertex { x:0.0,y:0.0,z:0.0 }], triangles: vec![Triangle { a:0,b:1,c:2 }] };
        assert_eq!(mesh.validate(), Err(MeshError::IndexOutOfBounds));
    }
}
