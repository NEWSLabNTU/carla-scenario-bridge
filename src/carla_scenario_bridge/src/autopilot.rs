//! Vehicles the simulator drives: SSv2 entities whose OpenSCENARIO controller is
//! `simulator_autopilot` (roadmap 017 step 6, docs/design/user-workflow.md "Background
//! vehicles").
//!
//! SSv2 runs no behavior for such an entity. csb spawns it with physics on, hands it to
//! CARLA's Traffic Manager (TM) once SSv2 starts its NPC logic, returns CARLA's pose in
//! `UpdateEntityStatus`, and turns the scenario's goals (`UpdateEntityGoal`) into a TM path.
//!
//! TM cannot route on its own between distant points: given a path, it chooses at each
//! junction the branch whose next point is closest, which takes the wrong turn whenever the
//! closest branch is not the one that leads to the goal (measured on Town01: one goal in
//! four missed with the goal alone as the path; dense points every 10 m also missed one).
//! So csb plans the lane route itself on CARLA's road topology ([`RoadGraph`]) and gives TM
//! one point just past every junction the route crosses, then the goal ([`RoadGraph::tm_path`]);
//! with that, every probed route arrived. TM leaves the path at its end and roams, so csb
//! also brings the vehicle to a stop at the goal ([`approach_speed`], [`ARRIVE_DISTANCE`]).
//!
//! Everything here is plain geometry and graph search, testable without CARLA; the CARLA
//! calls live in the coordinator.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// The behavior (OpenSCENARIO controller) name SSv2 sends in `SpawnVehicleEntityRequest`.
pub const BEHAVIOR: &str = "simulator_autopilot";

/// Exit points of one topology segment and entry points of the next lie within this of
/// each other (CARLA's topology repeats the same point; this allows for rounding).
const LINK_TOLERANCE_M: f64 = 0.5;

/// How far past a junction's exit the guiding path point is placed. Any distance on the
/// exit lane works (0.5 m and 5 m measured the same); a few metres keeps the point clear
/// of the junction's own overlapping connector lanes.
const EXIT_LEAD_M: f64 = 5.0;

/// The vehicle counts as arrived this close to the goal along its lane.
pub const ARRIVE_DISTANCE: f64 = 0.5;

/// At the goal, the vehicle is parked (released by TM, hand brake) once slower than this
/// [m/s]; until then TM is asked to stop it.
pub const PARK_SPEED: f64 = 0.1;

/// Deceleration csb plans the stop at the goal with [m/s^2]: comfortable, and inside every
/// performance bound a scenario is likely to declare.
pub const APPROACH_DECELERATION: f64 = 1.5;

/// Lowest speed asked of TM while still short of the goal [m/s]. Below about this TM's
/// controller stops the vehicle outright, short of the goal.
const APPROACH_MIN_SPEED: f64 = 1.0;

/// Traffic Manager's speed unit is km/h (LibCarla `SetDesiredSpeed`, despite carla-rust's
/// doc comment saying m/s); SSv2's is m/s.
pub fn mps_to_kmh(mps: f64) -> f32 {
    (mps * 3.6) as f32
}

/// Speed to ask of TM `remaining` metres before the goal, cruising at `cruise`: the speed
/// from which [`APPROACH_DECELERATION`] stops the vehicle at the goal, never below
/// [`APPROACH_MIN_SPEED`] nor above `cruise`.
pub fn approach_speed(remaining: f64, cruise: f64) -> f64 {
    let braking = (2.0 * APPROACH_DECELERATION * (remaining - ARRIVE_DISTANCE).max(0.0)).sqrt();
    braking.min(cruise).max(APPROACH_MIN_SPEED.min(cruise))
}

/// Distance before the goal at which the approach speed starts to matter at `speed`, with
/// a margin for TM's controller to follow.
pub fn approach_window(speed: f64) -> f64 {
    speed * speed / (2.0 * APPROACH_DECELERATION) + 10.0
}

/// A lane of a road section in CARLA's OpenDRIVE terms.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LaneKey {
    pub road: u32,
    pub section: u32,
    pub lane: i32,
}

pub type Point = [f64; 3];

/// One topology segment: a lane from its entry to its exit, sampled in driving order, in
/// CARLA's frame.
#[derive(Debug, Clone)]
pub struct Segment {
    pub key: LaneKey,
    pub is_junction: bool,
    pub points: Vec<Point>,
}

fn dist2d(a: &Point, b: &Point) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

fn polyline_length(points: &[Point]) -> f64 {
    points.windows(2).map(|w| dist2d(&w[0], &w[1])).sum()
}

/// The point `s` metres along `points`, clamped to its ends.
fn point_at(points: &[Point], s: f64) -> Point {
    let mut left = s.max(0.0);
    for w in points.windows(2) {
        let d = dist2d(&w[0], &w[1]);
        if left <= d && d > 0.0 {
            let t = left / d;
            return [
                w[0][0] + t * (w[1][0] - w[0][0]),
                w[0][1] + t * (w[1][1] - w[0][1]),
                w[0][2] + t * (w[1][2] - w[0][2]),
            ];
        }
        left -= d;
    }
    *points.last().expect("a segment has points")
}

/// Distance along `points` of the closest point to `p`, and how far `p` is from it.
fn project(points: &[Point], p: &Point) -> (f64, f64) {
    if points.len() == 1 {
        return (0.0, dist2d(&points[0], p));
    }
    let mut best = (0.0, f64::INFINITY);
    let mut along = 0.0;
    for w in points.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len2 = dx * dx + dy * dy;
        let t = if len2 > 0.0 {
            (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = [a[0] + t * dx, a[1] + t * dy, 0.0];
        let d = dist2d(&q, p);
        if d < best.1 {
            best = (along + t * len2.sqrt(), d);
        }
        along += len2.sqrt();
    }
    best
}

/// Where a point lies on the road graph: a segment and the distance along it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OnRoad {
    pub segment: usize,
    pub s: f64,
}

/// CARLA's lane topology as a graph of segments, for planning a vehicle's route.
#[derive(Debug, Default)]
pub struct RoadGraph {
    segments: Vec<Segment>,
    lengths: Vec<f64>,
    successors: Vec<Vec<usize>>,
    by_key: HashMap<LaneKey, Vec<usize>>,
}

#[derive(Debug, PartialEq)]
struct Frontier {
    cost: f64,
    segment: usize,
}

impl Eq for Frontier {}

impl Ord for Frontier {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap on cost.
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
            .then_with(|| other.segment.cmp(&self.segment))
    }
}

impl PartialOrd for Frontier {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl RoadGraph {
    /// Link every segment's exit to the entries that start where it ends.
    pub fn new(segments: Vec<Segment>) -> Self {
        let segments: Vec<Segment> = segments
            .into_iter()
            .filter(|s| !s.points.is_empty())
            .collect();
        let lengths = segments
            .iter()
            .map(|s| polyline_length(&s.points))
            .collect();
        let successors = segments
            .iter()
            .map(|from| {
                let exit = from.points.last().expect("non-empty");
                segments
                    .iter()
                    .enumerate()
                    .filter(|(_, to)| dist2d(exit, &to.points[0]) <= LINK_TOLERANCE_M)
                    .map(|(j, _)| j)
                    .collect()
            })
            .collect();
        let mut by_key: HashMap<LaneKey, Vec<usize>> = HashMap::new();
        for (i, s) in segments.iter().enumerate() {
            by_key.entry(s.key).or_default().push(i);
        }
        Self {
            segments,
            lengths,
            successors,
            by_key,
        }
    }

    pub fn len(&self) -> usize {
        self.segments.len()
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Place `p` on the lane `key` (as CARLA's `waypoint_at` reports it): the closest of the
    /// segments on that lane.
    pub fn locate(&self, key: LaneKey, p: &Point) -> Option<OnRoad> {
        self.by_key
            .get(&key)?
            .iter()
            .map(|&i| {
                let (s, d) = project(&self.segments[i].points, p);
                (i, s, d)
            })
            .min_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(Ordering::Equal))
            .map(|(segment, s, _)| OnRoad { segment, s })
    }

    /// The segments from `from` to `to` in driving order, shortest by length, or `None` if
    /// the lane topology does not connect them (a goal on another lane of the same road,
    /// or behind a one-way end).
    pub fn plan(&self, from: OnRoad, to: OnRoad) -> Option<Vec<usize>> {
        if from.segment == to.segment && to.s + 1e-6 >= from.s {
            return Some(vec![from.segment]);
        }
        let n = self.segments.len();
        let mut cost = vec![f64::INFINITY; n];
        let mut previous: Vec<Option<usize>> = vec![None; n];
        let mut heap = BinaryHeap::new();
        // Costs are distances to each segment's exit; leaving the start costs its remainder.
        let start_cost = self.lengths[from.segment] - from.s;
        for &next in &self.successors[from.segment] {
            let c = start_cost + self.lengths[next];
            if c < cost[next] {
                cost[next] = c;
                previous[next] = Some(from.segment);
                heap.push(Frontier {
                    cost: c,
                    segment: next,
                });
            }
        }
        while let Some(Frontier { cost: c, segment }) = heap.pop() {
            if c > cost[segment] {
                continue;
            }
            if segment == to.segment {
                break;
            }
            for &next in &self.successors[segment] {
                let nc = c + self.lengths[next];
                if nc < cost[next] {
                    cost[next] = nc;
                    previous[next] = Some(segment);
                    heap.push(Frontier {
                        cost: nc,
                        segment: next,
                    });
                }
            }
        }
        if !cost[to.segment].is_finite() {
            return None;
        }
        let mut route = vec![to.segment];
        let mut at = to.segment;
        // The start segment is reached when the walk hits it; a loop back onto the start
        // segment (goal behind on the same lane) ends at its second visit naturally.
        while let Some(p) = previous[at] {
            route.push(p);
            if p == from.segment {
                break;
            }
            at = p;
        }
        route.reverse();
        (route.first() == Some(&from.segment)).then_some(route)
    }

    /// The path TM is given for `route` ending at `goal`: a point [`EXIT_LEAD_M`] into every
    /// segment the route enters from a junction, then the goal.
    pub fn tm_path(&self, route: &[usize], goal: Point) -> Vec<Point> {
        let mut path: Vec<Point> = route
            .windows(2)
            .filter(|w| self.segments[w[0]].is_junction && !self.segments[w[1]].is_junction)
            .map(|w| {
                let seg = w[1];
                point_at(
                    &self.segments[seg].points,
                    EXIT_LEAD_M.min(self.lengths[seg] / 2.0),
                )
            })
            .collect();
        // A guiding point past the goal on the goal's own lane would pull TM beyond it.
        if let Some(last) = path.last() {
            if dist2d(last, &goal) < EXIT_LEAD_M {
                path.pop();
            }
        }
        path.push(goal);
        path
    }
}

/// A route being driven, and how far along it the vehicle has got.
#[derive(Debug, Clone, PartialEq)]
pub struct RouteProgress {
    /// Segments in driving order, possibly concatenated from several legs.
    pub route: Vec<usize>,
    /// Index into `route` of the furthest segment the vehicle has been seen on.
    pub reached: usize,
    /// The goal's distance along the last segment.
    pub goal_s: f64,
}

impl RouteProgress {
    pub fn new(route: Vec<usize>, goal_s: f64) -> Self {
        Self {
            route,
            reached: 0,
            goal_s,
        }
    }

    /// Note the vehicle's position; returns the distance left to the goal once it is on
    /// the route's last segment (negative when past it), `None` before that.
    ///
    /// Progress only moves forward, so a route that crosses its own last segment early
    /// (a loop) does not count as arriving the first time through.
    pub fn observe(&mut self, at: Option<OnRoad>) -> Option<f64> {
        let at = at?;
        if let Some(k) = self.route[self.reached..]
            .iter()
            .position(|&s| s == at.segment)
        {
            // Step at most two segments at a time (a short one can pass between two
            // frames), never further: a junction's connectors overlap, and the vehicle can
            // be matched for a frame to a segment the route only uses later.
            if k <= 2 {
                self.reached += k;
            }
        }
        let last = self.route.len() - 1;
        (self.reached == last && at.segment == self.route[last]).then_some(self.goal_s - at.s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(road: u32, lane: i32) -> LaneKey {
        LaneKey {
            road,
            section: 0,
            lane,
        }
    }

    fn straight(road: u32, from: (f64, f64), to: (f64, f64), junction: bool) -> Segment {
        let n = 10;
        let points = (0..=n)
            .map(|i| {
                let t = i as f64 / n as f64;
                [
                    from.0 + t * (to.0 - from.0),
                    from.1 + t * (to.1 - from.1),
                    0.0,
                ]
            })
            .collect();
        Segment {
            key: key(road, -1),
            is_junction: junction,
            points,
        }
    }

    /// A T: road 0 runs east to a junction at x=100, which branches left (north, road 10
    /// then road 2) and right (south, road 11 then road 3).
    fn tee() -> RoadGraph {
        RoadGraph::new(vec![
            straight(0, (0.0, 0.0), (100.0, 0.0), false),
            straight(10, (100.0, 0.0), (110.0, 10.0), true),
            straight(2, (110.0, 10.0), (110.0, 110.0), false),
            straight(11, (100.0, 0.0), (110.0, -10.0), true),
            straight(3, (110.0, -10.0), (110.0, -110.0), false),
        ])
    }

    #[test]
    fn segments_link_where_one_ends_and_the_next_begins() {
        let g = tee();
        assert_eq!(g.successors[0], vec![1, 3]);
        assert_eq!(g.successors[1], vec![2]);
        assert!(g.successors[2].is_empty());
    }

    #[test]
    fn a_point_is_located_on_its_lane_with_its_distance() {
        let g = tee();
        let at = g.locate(key(0, -1), &[30.0, 0.4, 0.0]).unwrap();
        assert_eq!(at.segment, 0);
        assert!((at.s - 30.0).abs() < 1e-9);
        assert_eq!(g.locate(key(99, -1), &[0.0, 0.0, 0.0]), None);
    }

    #[test]
    fn the_route_takes_the_branch_that_leads_to_the_goal() {
        let g = tee();
        let from = OnRoad {
            segment: 0,
            s: 10.0,
        };
        let south = OnRoad {
            segment: 4,
            s: 50.0,
        };
        assert_eq!(g.plan(from, south), Some(vec![0, 3, 4]));
        let north = OnRoad {
            segment: 2,
            s: 50.0,
        };
        assert_eq!(g.plan(from, north), Some(vec![0, 1, 2]));
    }

    #[test]
    fn a_goal_ahead_on_the_same_lane_needs_no_search() {
        let g = tee();
        let r = g.plan(
            OnRoad {
                segment: 0,
                s: 10.0,
            },
            OnRoad {
                segment: 0,
                s: 60.0,
            },
        );
        assert_eq!(r, Some(vec![0]));
    }

    #[test]
    fn an_unreachable_goal_has_no_route() {
        let g = tee();
        // Behind on the same dead-end lane, and from a branch back to the trunk.
        assert_eq!(
            g.plan(
                OnRoad {
                    segment: 0,
                    s: 60.0
                },
                OnRoad {
                    segment: 0,
                    s: 10.0
                }
            ),
            None
        );
        assert_eq!(
            g.plan(
                OnRoad {
                    segment: 2,
                    s: 10.0
                },
                OnRoad {
                    segment: 0,
                    s: 10.0
                }
            ),
            None
        );
    }

    #[test]
    fn tm_gets_a_point_past_each_junction_and_the_goal() {
        let g = tee();
        let goal = [110.0, -60.0, 0.0];
        let path = g.tm_path(&[0, 3, 4], goal);
        assert_eq!(path.len(), 2);
        assert!((path[0][0] - 110.0).abs() < 1e-9);
        assert!(
            (path[0][1] - -15.0).abs() < 1e-9,
            "5 m into road 3: {:?}",
            path[0]
        );
        assert_eq!(path[1], goal);
        // No junction: the goal alone.
        assert_eq!(g.tm_path(&[0], [50.0, 0.0, 0.0]), vec![[50.0, 0.0, 0.0]]);
    }

    #[test]
    fn a_guiding_point_beside_the_goal_is_dropped() {
        let g = tee();
        let goal = [110.0, -12.0, 0.0];
        assert_eq!(g.tm_path(&[0, 3, 4], goal), vec![goal]);
    }

    #[test]
    fn progress_reports_the_distance_left_only_on_the_last_segment() {
        let mut p = RouteProgress::new(vec![0, 3, 4], 50.0);
        assert_eq!(
            p.observe(Some(OnRoad {
                segment: 0,
                s: 90.0
            })),
            None
        );
        assert_eq!(p.observe(Some(OnRoad { segment: 3, s: 5.0 })), None);
        assert_eq!(
            p.observe(Some(OnRoad {
                segment: 4,
                s: 20.0
            })),
            Some(30.0)
        );
        assert_eq!(p.observe(None), None);
        assert_eq!(
            p.observe(Some(OnRoad {
                segment: 4,
                s: 51.0
            })),
            Some(-1.0)
        );
    }

    #[test]
    fn progress_does_not_jump_to_a_later_visit_of_a_segment() {
        // A loop: the last segment is also crossed first.
        let mut p = RouteProgress::new(vec![4, 0, 1, 2, 4], 50.0);
        assert_eq!(
            p.observe(Some(OnRoad {
                segment: 4,
                s: 60.0
            })),
            None
        );
        // A one-frame match onto a segment three ahead is ignored.
        assert_eq!(p.observe(Some(OnRoad { segment: 2, s: 1.0 })), None);
        assert_eq!(p.reached, 0);
        // Two ahead (a short segment passed between frames) is progress.
        assert_eq!(p.observe(Some(OnRoad { segment: 1, s: 1.0 })), None);
        assert_eq!(p.reached, 2);
    }

    #[test]
    fn the_approach_slows_to_stop_at_the_goal() {
        assert_eq!(approach_speed(1000.0, 8.0), 8.0);
        let v = approach_speed(12.5, 8.0);
        assert!((v - (2.0 * APPROACH_DECELERATION * 12.0).sqrt()).abs() < 1e-9);
        assert_eq!(approach_speed(0.6, 8.0), APPROACH_MIN_SPEED);
        assert_eq!(approach_speed(0.0, 0.5), 0.5);
        assert!(approach_window(8.0) > 8.0 * 8.0 / (2.0 * APPROACH_DECELERATION));
    }

    #[test]
    fn tm_speeds_are_in_kmh() {
        assert!((mps_to_kmh(10.0) - 36.0).abs() < 1e-4);
    }
}
