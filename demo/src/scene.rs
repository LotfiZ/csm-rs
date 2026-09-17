//! 2D scene geometry and ray casting for the demo.

/// A line segment between two world points.
#[derive(Clone, Copy, Debug)]
pub struct Segment {
    pub a: [f64; 2],
    pub b: [f64; 2],
}

impl Segment {
    pub fn new(a: [f64; 2], b: [f64; 2]) -> Self {
        Self { a, b }
    }
}

/// A named collection of segments.
pub struct Scene {
    pub name: &'static str,
    pub segments: Vec<Segment>,
    /// Suggested sensor path radius / extent for framing the view.
    pub extent: f64,
}

fn rectangle(min: [f64; 2], max: [f64; 2]) -> Vec<Segment> {
    vec![
        Segment::new([min[0], min[1]], [max[0], min[1]]),
        Segment::new([max[0], min[1]], [max[0], max[1]]),
        Segment::new([max[0], max[1]], [min[0], max[1]]),
        Segment::new([min[0], max[1]], [min[0], min[1]]),
    ]
}

impl Scene {
    /// An L-shaped room with an interior pillar: overlapping geometry changes
    /// as the sensor moves around the pillar.
    pub fn asymmetric_room() -> Self {
        let mut segments = rectangle([-6.0, -5.0], [6.0, 5.0]);
        segments.extend(rectangle([1.0, -1.0], [4.0, 2.0]));
        segments.extend(rectangle([-4.0, 2.5], [-1.0, 4.5]));
        Self {
            name: "asymmetric_room",
            segments,
            extent: 7.0,
        }
    }

    /// A long straight corridor with a short side pocket; translation along
    /// the corridor is weakly constrained, so yaw is ambiguous.
    pub fn ambiguous_corridor() -> Self {
        let mut segments = vec![
            Segment::new([-8.0, 2.0], [8.0, 2.0]),
            Segment::new([-8.0, -2.0], [8.0, -2.0]),
            Segment::new([-8.0, 2.0], [-8.0, -2.0]),
            Segment::new([8.0, 2.0], [8.0, -2.0]),
        ];
        segments.extend(rectangle([3.0, -2.0], [4.0, -0.5]));
        Self {
            name: "ambiguous_corridor",
            segments,
            extent: 8.0,
        }
    }

    /// A room split by a wall with a doorway, so part of the scan leaves the
    /// overlap as the sensor moves through.
    pub fn partial_overlap() -> Self {
        let mut segments = rectangle([-6.0, -4.0], [6.0, 4.0]);
        segments.push(Segment::new([0.0, -4.0], [0.0, -1.0]));
        segments.push(Segment::new([0.0, 1.0], [0.0, 4.0]));
        segments.extend(rectangle([4.5, 2.2], [5.6, 3.3]));
        Self {
            name: "partial_overlap",
            segments,
            extent: 7.0,
        }
    }

    pub fn by_name(name: &str) -> Self {
        match name {
            "ambiguous_corridor" => Self::ambiguous_corridor(),
            "partial_overlap" => Self::partial_overlap(),
            _ => Self::asymmetric_room(),
        }
    }
}

/// Cast a ray from `origin` in the unit direction `direction`; returns the
/// nearest intersection distance strictly ahead of the origin.
pub fn cast_ray(origin: [f64; 2], direction: [f64; 2], segments: &[Segment]) -> Option<f64> {
    let mut best = f64::INFINITY;
    for segment in segments {
        let edge = [segment.b[0] - segment.a[0], segment.b[1] - segment.a[1]];
        let denom = direction[0] * edge[1] - direction[1] * edge[0];
        if denom.abs() < 1e-12 {
            continue;
        }
        let diff = [segment.a[0] - origin[0], segment.a[1] - origin[1]];
        let t = (diff[0] * edge[1] - diff[1] * edge[0]) / denom;
        let u = (diff[0] * direction[1] - diff[1] * direction[0]) / denom;
        if t > 1e-9 && (0.0..=1.0).contains(&u) {
            best = best.min(t);
        }
    }
    best.is_finite().then_some(best)
}
