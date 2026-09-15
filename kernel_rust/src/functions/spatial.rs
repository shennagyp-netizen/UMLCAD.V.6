use super::geometry::{aabb_intersects, Arc, Circle, Geometry, Line, Point, EPSILON};

const PARAM_EPSILON: f64 = 1.0e-12;

#[derive(Clone, Debug, PartialEq)]
pub struct SpatialResult {
    pub first_geometry_id: String,
    pub second_geometry_id: String,
    pub distance: f64,
    pub intersects: bool,
    pub method: &'static str,
}

pub fn broad_phase_intersections(geometry: &[(String, Geometry)], tolerance: f64) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for i in 0..geometry.len() {
        for j in i + 1..geometry.len() {
            if aabb_intersects(geometry[i].1.aabb(), geometry[j].1.aabb(), tolerance) {
                out.push((geometry[i].0.clone(), geometry[j].0.clone()));
            }
        }
    }
    out
}

fn cross(a: Point, b: Point) -> f64 { a.x * b.y - a.y * b.x }

fn closest(l: Line, p: Point) -> Point {
    let d = l.end.sub(l.start);
    let den = d.dot(d);
    if den <= EPSILON { return l.start; }
    l.start.add(d.scale((p.sub(l.start).dot(d) / den).clamp(0.0, 1.0)))
}

fn segment_distance(a: Line, b: Line) -> f64 {
    let r = b.start.sub(a.start);
    let d = a.end.sub(a.start);
    let e = b.end.sub(b.start);
    let den = cross(d, e);
    if den.abs() > EPSILON {
        let t = cross(r, e) / den;
        let u = cross(r, d) / den;
        if (-PARAM_EPSILON..=1.0 + PARAM_EPSILON).contains(&t) && (-PARAM_EPSILON..=1.0 + PARAM_EPSILON).contains(&u) { return 0.0; }
    }
    [
        b.start.distance(closest(a, b.start)),
        b.end.distance(closest(a, b.end)),
        a.start.distance(closest(b, a.start)),
        a.end.distance(closest(b, a.end)),
    ].into_iter().fold(f64::INFINITY, f64::min)
}

fn circle_circle(a: Circle, b: Circle) -> f64 {
    let d = a.center.distance(b.center);
    if d <= EPSILON { return (a.radius - b.radius).abs(); }
    if d <= a.radius + b.radius + EPSILON && d + EPSILON >= (a.radius - b.radius).abs() { 0.0 } else { (d - a.radius - b.radius).max((a.radius - b.radius).abs() - d).abs() }
}

fn line_circle(l: Line, c: Circle) -> f64 {
    (c.center.distance(closest(l, c.center)) - c.radius).max(0.0)
}

fn circle_circle_intersections(a: Circle, b: Circle) -> Vec<Point> {
    let dx = b.center.x - a.center.x;
    let dy = b.center.y - a.center.y;
    let d = dx.hypot(dy);
    if d <= EPSILON || d > a.radius + b.radius + EPSILON || d + EPSILON < (a.radius - b.radius).abs() { return Vec::new(); }
    let aa = (a.radius * a.radius - b.radius * b.radius + d * d) / (2.0 * d);
    let h2 = a.radius * a.radius - aa * aa;
    if h2 < -EPSILON { return Vec::new(); }
    let h = h2.max(0.0).sqrt();
    let ux = dx / d;
    let uy = dy / d;
    let px = a.center.x + aa * ux;
    let py = a.center.y + aa * uy;
    let ox = -uy * h;
    let oy = ux * h;
    let p1 = Point { x: px + ox, y: py + oy };
    if h <= EPSILON { vec![p1] } else { vec![p1, Point { x: px - ox, y: py - oy }] }
}

fn line_circle_intersections(l: Line, c: Circle) -> Vec<Point> {
    let d = l.end.sub(l.start);
    let f = l.start.sub(c.center);
    let a = d.dot(d);
    if a <= EPSILON { return Vec::new(); }
    let b = 2.0 * f.dot(d);
    let cc = f.dot(f) - c.radius * c.radius;
    let mut disc = b * b - 4.0 * a * cc;
    let scale = (b * b).abs() + (4.0 * a * cc.abs()) + 1.0;
    if disc < 0.0 && disc >= -EPSILON * scale { disc = 0.0; }
    if disc < 0.0 { return Vec::new(); }
    let root = disc.sqrt();
    let mut out = Vec::new();
    for t in [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)] {
        if (-PARAM_EPSILON..=1.0 + PARAM_EPSILON).contains(&t) {
            let q = l.start.add(d.scale(t.clamp(0.0, 1.0)));
            if !out.iter().any(|p: &Point| p.distance(q) <= EPSILON) { out.push(q); }
        }
    }
    out
}

fn arc_point(a: Arc, t: f64) -> Point { Geometry::Arc(a).point_at(t) }
fn arc_distance(a: Arc, p: Point) -> f64 { Geometry::Arc(a).distance_to_point(p) }
fn point_on_arc(a: Arc, p: Point) -> bool { a.contains_point(p) }

fn line_arc(l: Line, a: Arc) -> f64 {
    let circle = Circle { center: a.center, radius: a.radius };
    if line_circle_intersections(l, circle).into_iter().any(|p| point_on_arc(a, p)) { return 0.0; }
    let q = closest(l, a.center);
    let radial = q.sub(a.center);
    let radial_norm = radial.norm();
    let radial_candidate = if radial_norm > EPSILON { Some(Point { x: a.center.x + radial.x * a.radius / radial_norm, y: a.center.y + radial.y * a.radius / radial_norm }) } else { None };
    let mut d = l.start.distance(arc_point(a, 0.0)).min(l.end.distance(arc_point(a, 1.0)));
    d = d.min(arc_distance(a, l.start)).min(arc_distance(a, l.end));
    if let Some(p) = radial_candidate { if point_on_arc(a, p) { d = d.min(l.point_distance(p)); } }
    d.min(line_circle(l, circle))
}

fn arc_circle(a: Arc, c: Circle) -> f64 {
    if circle_circle_intersections(Circle { center: a.center, radius: a.radius }, c).into_iter().any(|p| point_on_arc(a, p)) { return 0.0; }
    arc_distance(a, c.center).abs_sub(c.radius).abs()
}

fn arc_arc(a: Arc, b: Arc) -> f64 {
    let ca = Circle { center: a.center, radius: a.radius };
    let cb = Circle { center: b.center, radius: b.radius };
    if circle_circle_intersections(ca, cb).into_iter().any(|p| point_on_arc(a, p) && point_on_arc(b, p)) { return 0.0; }
    let mut d = f64::INFINITY;
    for p in [arc_point(a, 0.0), arc_point(a, 1.0), arc_point(b, 0.0), arc_point(b, 1.0)] {
        d = d.min(arc_distance(a, p)).min(arc_distance(b, p));
    }
    let pa = Geometry::Arc(a).distance_to_point(b.center);
    let pb = Geometry::Arc(b).distance_to_point(a.center);
    d.min((pa.powi(2) + pb.powi(2) - 2.0 * pa * pb).abs().sqrt())
}

trait PointLineDistance { fn point_distance(self, p: Point) -> f64; }
impl PointLineDistance for Line { fn point_distance(self, p: Point) -> f64 { p.distance(closest(self, p)) } }

fn pair_distance(a: &Geometry, b: &Geometry) -> f64 {
    match (a, b) {
        (Geometry::Line(x), Geometry::Line(y)) => segment_distance(*x, *y),
        (Geometry::Line(l), Geometry::Circle(c)) | (Geometry::Circle(c), Geometry::Line(l)) => line_circle(*l, *c),
        (Geometry::Line(l), Geometry::Arc(a)) | (Geometry::Arc(a), Geometry::Line(l)) => line_arc(*l, *a),
        (Geometry::Circle(a), Geometry::Circle(b)) => circle_circle(*a, *b),
        (Geometry::Circle(c), Geometry::Arc(a)) | (Geometry::Arc(a), Geometry::Circle(c)) => arc_circle(*a, *c),
        (Geometry::Arc(a), Geometry::Arc(b)) => arc_arc(*a, *b),
    }
}

pub fn point_distance(a: &Geometry, b: &Geometry) -> f64 { pair_distance(a, b) }

pub fn spatial_analysis(items: &[super::snapshot::GeometryItem], tolerance: f64) -> Vec<SpatialResult> {
    let mut out = Vec::new();
    for i in 0..items.len() {
        for j in i + 1..items.len() {
            let a = &items[i];
            let b = &items[j];
            if !aabb_intersects(a.geometry.aabb(), b.geometry.aabb(), tolerance) { continue; }
            let d = pair_distance(&a.geometry, &b.geometry);
            out.push(SpatialResult { first_geometry_id: a.id.clone(), second_geometry_id: b.id.clone(), distance: d, intersects: d <= tolerance, method: "analytic-2d" });
        }
    }
    out
}
