//! Box-to-box clearance between two CARLA bodies in the ground plane (roadmap 032 N2).
//!
//! The blocked rule must not hand a tracked NPC to kinematic next to the ego or another
//! physics body ("never fall back next to the ego"), and the collision stream alone cannot say
//! "next to": its events can arrive frames late (`rear_contact` run 3). So the rule also asks
//! the frame's world snapshot how far apart the boxes are. This is the oracle's measure
//! (`gap_geometry.signed_distance`, roadmap 024) ported as is: separating axes on the four
//! edge normals, the polygon distance when apart, minus the penetration depth when not.
//!
//! In the plane only, with no height gate -- unlike the oracle, which follows SSv2's
//! `CollisionCondition`. Deliberately conservative: an ego thrown up over an NPC (run 3: 4.6
//! m) is exactly when an NPC must not start ignoring physics under it.
//!
//! CARLA's frame throughout (left-handed, yaw in degrees). The formulas are the same in
//! either handedness as long as poses and offsets come from one frame.

/// A body's box in the ground plane: centre, heading, half sizes, m and degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarBox {
    pub cx: f64,
    pub cy: f64,
    pub yaw_deg: f64,
    pub half_length: f64,
    pub half_width: f64,
}

/// A body's box as CARLA describes it: `bounding_box()`'s centre offset and half extents,
/// in the actor's own frame. Kept per actor from its spawn.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyBox {
    pub offset_x: f64,
    pub offset_y: f64,
    pub half_length: f64,
    pub half_width: f64,
}

impl BodyBox {
    /// The box of an actor whose origin is at (`x`, `y`) heading `yaw_deg`.
    pub fn at(&self, x: f64, y: f64, yaw_deg: f64) -> PlanarBox {
        let (s, c) = yaw_deg.to_radians().sin_cos();
        PlanarBox {
            cx: x + c * self.offset_x - s * self.offset_y,
            cy: y + s * self.offset_x + c * self.offset_y,
            yaw_deg,
            half_length: self.half_length,
            half_width: self.half_width,
        }
    }
}

impl PlanarBox {
    fn axes(&self) -> [(f64, f64); 2] {
        let (s, c) = self.yaw_deg.to_radians().sin_cos();
        [(c, s), (-s, c)]
    }

    fn corners(&self) -> [(f64, f64); 4] {
        let [(ux, uy), (vx, vy)] = self.axes();
        let (hl, hw) = (self.half_length, self.half_width);
        [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)].map(|(sl, sw)| {
            (
                self.cx + sl * hl * ux + sw * hw * vx,
                self.cy + sl * hl * uy + sw * hw * vy,
            )
        })
    }
}

fn project(corners: &[(f64, f64); 4], ax: f64, ay: f64) -> (f64, f64) {
    corners
        .iter()
        .map(|(x, y)| x * ax + y * ay)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
            (lo.min(p), hi.max(p))
        })
}

fn point_segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let length2 = dx * dx + dy * dy;
    let t = if length2 == 0.0 {
        0.0
    } else {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / length2).clamp(0.0, 1.0)
    };
    (p.0 - (a.0 + t * dx)).hypot(p.1 - (a.1 + t * dy))
}

/// Distance between two disjoint convex polygons: the closest pair always has a vertex of
/// one on an edge of the other.
fn polygon_distance(pa: &[(f64, f64); 4], pb: &[(f64, f64); 4]) -> f64 {
    let mut best = f64::INFINITY;
    for (poly, other) in [(pa, pb), (pb, pa)] {
        for i in 0..4 {
            let (a, b) = (other[i], other[(i + 1) % 4]);
            for &p in poly {
                best = best.min(point_segment_distance(p, a, b));
            }
        }
    }
    best
}

/// Clearance between `a` and `b` (> 0), or minus their penetration depth (< 0); 0 touching.
pub fn signed_distance(a: &PlanarBox, b: &PlanarBox) -> f64 {
    let (ca, cb) = (a.corners(), b.corners());
    let mut depth = f64::INFINITY;
    for (ax, ay) in a.axes().into_iter().chain(b.axes()) {
        let (mina, maxa) = project(&ca, ax, ay);
        let (minb, maxb) = project(&cb, ax, ay);
        if maxa < minb || maxb < mina {
            return polygon_distance(&ca, &cb);
        }
        depth = depth.min(maxa - minb).min(maxb - mina);
    }
    -depth.max(0.0)
}

/// The smallest clearance from `subject` to any of `others`; `None` when there are none.
pub fn min_clearance(
    subject: &PlanarBox,
    others: impl IntoIterator<Item = PlanarBox>,
) -> Option<f64> {
    others
        .into_iter()
        .map(|o| signed_distance(subject, &o))
        .fold(None, |best: Option<f64>, d| {
            Some(best.map_or(d, |b| b.min(d)))
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    fn square(cx: f64, cy: f64, yaw_deg: f64) -> PlanarBox {
        PlanarBox {
            cx,
            cy,
            yaw_deg,
            half_length: 1.0,
            half_width: 1.0,
        }
    }

    /// The oracle's Model 3 (`test_gap_oracle.MODEL3`): 4.7 x 2.1, centred on the origin.
    const MODEL3: BodyBox = BodyBox {
        offset_x: 0.0,
        offset_y: 0.0,
        half_length: 2.35,
        half_width: 1.05,
    };

    #[test]
    fn apart_end_to_end_is_the_gap_between_the_faces() {
        // Two Model 3s nose to tail, origins 5.7 m apart: 1.0 m between bumpers.
        let a = MODEL3.at(0.0, 0.0, 0.0);
        let b = MODEL3.at(-5.7, 0.0, 0.0);
        assert!(close(signed_distance(&a, &b), 1.0));
        assert!(close(signed_distance(&b, &a), 1.0));
    }

    #[test]
    fn corner_to_corner_is_euclidean_not_the_separating_gap() {
        // Unit squares, corners at (1, 1) and (1.3, 1.4): 0.5 m apart, though the widest
        // separating gap on an axis is only 0.4.
        let d = signed_distance(&square(0.0, 0.0, 0.0), &square(2.3, 2.4, 0.0));
        assert!(close(d, 0.5), "{d}");
    }

    #[test]
    fn overlapping_boxes_report_minus_the_penetration_depth() {
        let d = signed_distance(&square(0.0, 0.0, 0.0), &square(1.8, 0.5, 0.0));
        assert!(close(d, -0.2), "{d}");
        assert!(close(
            signed_distance(&square(0.0, 0.0, 0.0), &square(2.0, 0.0, 0.0)),
            0.0
        ));
    }

    #[test]
    fn the_heading_turns_the_box_and_its_offset() {
        // A Model 3 whose box centre sits 1.5 m ahead of its origin, turned 90 degrees: the
        // box spans y -0.85..3.85 and x -1.05..1.05 (CARLA's frame, x forward turned to y).
        let car = BodyBox {
            offset_x: 1.5,
            ..MODEL3
        };
        let b = car.at(0.0, 0.0, 90.0);
        assert!(close(b.cx, 0.0) && close(b.cy, 1.5), "{b:?}");
        // A unit square centred at (0, 5.85): 1.0 m beyond the front face at y 3.85.
        assert!(close(signed_distance(&b, &square(0.0, 5.85, 0.0)), 1.0));
        // And at (3.05, 1.5): 1.0 m off the side face at x 1.05.
        assert!(close(signed_distance(&b, &square(3.05, 1.5, 0.0)), 1.0));
    }

    #[test]
    fn the_nearest_of_several_others_and_none_without_any() {
        let me = square(0.0, 0.0, 0.0);
        let others = [
            square(10.0, 0.0, 0.0),
            square(0.0, -2.7, 0.0),
            square(-30.0, 0.0, 45.0),
        ];
        let d = min_clearance(&me, others).expect("three others");
        assert!(close(d, 0.7), "{d}");
        assert_eq!(min_clearance(&me, []), None);
    }
}
