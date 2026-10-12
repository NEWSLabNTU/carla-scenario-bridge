//! Tracked NPC motion (roadmap 023): the per-frame command that keeps a physics-on NPC on the
//! pose SSv2 commands, or the snap that puts it back.
//!
//! Everything here is in the CARLA world frame (left-handed, yaw in degrees, positive
//! clockwise seen from above, so a positive yaw rate turns right like a positive steer).
//!
//! The body must be at the commanded pose `p*` *after* the tick, and a velocity set before
//! the tick acts during it, so the law aims at the start-of-step pose `s* = p* - v*·dt` (yaw
//! likewise). Aiming at `p*` itself leaves the body one frame ahead: 0.40 m at 8 m/s,
//! measured in the step-2 spike.

/// Snap when the in-plane error exceeds this, m.
pub const SNAP_DISTANCE: f64 = 0.5;
/// Snap when the yaw error exceeds this, degrees. Not applied to walkers, whose yaw CARLA
/// turns toward their direction of travel by itself.
pub const SNAP_YAW_DEG: f64 = 5.0;
/// Below this commanded speed no steer is fed forward (the curvature is undefined at rest).
const STEER_MIN_SPEED: f64 = 0.1;

/// Feedback gains, 1/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gains {
    pub position: f64,
    pub yaw: f64,
}

/// From the step-2 spike: settled p95 0.02-0.04 m and yaw p95 <= 1.0 deg on Town01.
pub const VEHICLE_GAINS: Gains = Gains {
    position: 5.0,
    yaw: 2.0,
};
/// From the step-2 spike: k_p 5-8 settles a walker to within 0.02 m.
pub const WALKER_GAINS: Gains = Gains {
    position: 6.0,
    yaw: 0.0,
};

/// What SSv2 commands for this frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Target {
    pub x: f64,
    pub y: f64,
    pub yaw_deg: f64,
    pub vx: f64,
    pub vy: f64,
    pub yaw_rate_deg: f64,
}

/// Where the body is now (before the tick).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Current {
    pub x: f64,
    pub y: f64,
    pub yaw_deg: f64,
}

/// A vehicle's steering geometry, for the steer feed-forward.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Steering {
    pub wheelbase: f64,
    pub max_steer_deg: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    /// Set this velocity and yaw rate; apply `steer` (-1..1) when given.
    Track {
        vx: f64,
        vy: f64,
        yaw_rate_deg: f64,
        steer: Option<f64>,
    },
    /// Teleport to the start-of-step pose and set the commanded velocity and yaw rate.
    Snap {
        x: f64,
        y: f64,
        yaw_deg: f64,
        vx: f64,
        vy: f64,
        yaw_rate_deg: f64,
    },
}

/// `deg` wrapped to (-180, 180].
pub fn wrap_deg(deg: f64) -> f64 {
    let w = (deg + 180.0).rem_euclid(360.0) - 180.0;
    if w == -180.0 {
        180.0
    } else {
        w
    }
}

/// The command for one frame. `steering` is `None` for walkers, which also never snap on yaw.
pub fn command(
    target: &Target,
    current: &Current,
    dt: f64,
    gains: Gains,
    steering: Option<Steering>,
) -> Command {
    let start_x = target.x - target.vx * dt;
    let start_y = target.y - target.vy * dt;
    let start_yaw = target.yaw_deg - target.yaw_rate_deg * dt;
    let ex = start_x - current.x;
    let ey = start_y - current.y;
    let eyaw = wrap_deg(start_yaw - current.yaw_deg);
    let checks_yaw = steering.is_some();
    if ex.hypot(ey) > SNAP_DISTANCE || (checks_yaw && eyaw.abs() > SNAP_YAW_DEG) {
        return Command::Snap {
            x: start_x,
            y: start_y,
            yaw_deg: wrap_deg(start_yaw),
            vx: target.vx,
            vy: target.vy,
            yaw_rate_deg: target.yaw_rate_deg,
        };
    }
    let steer = steering.map(|s| {
        let speed = target.vx.hypot(target.vy);
        if speed < STEER_MIN_SPEED || s.max_steer_deg <= 0.0 {
            return 0.0;
        }
        let curvature = target.yaw_rate_deg.to_radians() / speed;
        let angle = (s.wheelbase * curvature).atan().to_degrees();
        (angle / s.max_steer_deg).clamp(-1.0, 1.0)
    });
    Command::Track {
        vx: target.vx + gains.position * ex,
        vy: target.vy + gains.position * ey,
        yaw_rate_deg: target.yaw_rate_deg + gains.yaw * eyaw,
        steer,
    }
}

/// SSv2's twist for an entity, as the velocity and yaw rate of its CARLA body.
///
/// SSv2 gives the twist in the entity's own frame (ROS: x forward, y left), at the entity's
/// origin -- the rear axle for vehicles. The CARLA body's origin sits `offset` from it, so in
/// a turn the body moves at `v + ω × r`: 1 m/s more at 0.7 rad/s and a 1.5 m offset. The
/// result is in CARLA's world frame: y flipped, and a yaw rate in degrees per second,
/// positive clockwise from above.
///
/// `entity_yaw` is the entity's ROS yaw in radians; `offset` is in the entity's frame, m.
pub fn world_twist(
    entity_yaw: f64,
    vx_body: f64,
    vy_body: f64,
    yaw_rate: f64,
    offset_x: f64,
    offset_y: f64,
) -> (f64, f64, f64) {
    let (sin, cos) = entity_yaw.sin_cos();
    let vx = vx_body * cos - vy_body * sin;
    let vy = vx_body * sin + vy_body * cos;
    let rx = offset_x * cos - offset_y * sin;
    let ry = offset_x * sin + offset_y * cos;
    let (bx, by) = (vx - yaw_rate * ry, vy + yaw_rate * rx);
    (bx, -by, -yaw_rate.to_degrees())
}

/// The steering geometry from SSv2's declared axles (`positionX`, m from the entity origin)
/// and CARLA's front-wheel lock: `None` without a usable wheelbase or lock.
pub fn steering(front_axle_x: f64, rear_axle_x: f64, max_steer_deg: f64) -> Option<Steering> {
    let wheelbase = front_axle_x - rear_axle_x;
    (wheelbase.is_finite() && wheelbase > 0.1 && max_steer_deg > 0.0).then_some(Steering {
        wheelbase,
        max_steer_deg,
    })
}

/// One tracked entity's errors over an episode, measured before each step against the
/// start-of-step pose.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TrackingStats {
    errors: Vec<f32>,
    yaw_errors: Vec<f32>,
    snaps: u32,
}

/// A summary of [`TrackingStats`], for the log and the contact report.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackingSummary {
    pub frames: usize,
    pub error_p95: f64,
    pub error_max: f64,
    /// `None` for walkers, whose yaw CARLA turns itself.
    pub yaw_p95: Option<f64>,
    pub yaw_max: Option<f64>,
    pub snaps: u32,
}

impl TrackingStats {
    /// One frame: the in-plane error (m), the yaw error (degrees; `None` for a walker) and
    /// whether the frame snapped.
    pub fn record(&mut self, error: f64, yaw_error: Option<f64>, snapped: bool) {
        self.errors.push(error as f32);
        if let Some(y) = yaw_error {
            self.yaw_errors.push(y.abs() as f32);
        }
        self.snaps += u32::from(snapped);
    }

    pub fn summary(&self) -> TrackingSummary {
        let yaw = (!self.yaw_errors.is_empty()).then_some(&self.yaw_errors);
        TrackingSummary {
            frames: self.errors.len(),
            error_p95: p95(&self.errors),
            error_max: max(&self.errors),
            yaw_p95: yaw.map(|y| p95(y)),
            yaw_max: yaw.map(|y| max(y)),
            snaps: self.snaps,
        }
    }
}

/// Nearest-rank 95th percentile; 0 for no samples.
fn p95(values: &[f32]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut v = values.to_vec();
    v.sort_by(f32::total_cmp);
    let rank = ((0.95 * v.len() as f64).ceil() as usize).clamp(1, v.len());
    f64::from(v[rank - 1])
}

fn max(values: &[f32]) -> f64 {
    values.iter().copied().fold(0.0_f32, f32::max).into()
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.05;
    const MODEL3: Steering = Steering {
        wheelbase: 2.9,
        max_steer_deg: 70.0,
    };

    /// Westbound at 8 m/s along CARLA x, straight.
    fn cruising() -> Target {
        Target {
            x: 100.0,
            y: 50.0,
            yaw_deg: 180.0,
            vx: -8.0,
            vy: 0.0,
            yaw_rate_deg: 0.0,
        }
    }

    /// Exactly where the body should be before the tick: one step behind the target.
    fn on_track(t: &Target) -> Current {
        Current {
            x: t.x - t.vx * DT,
            y: t.y - t.vy * DT,
            yaw_deg: t.yaw_deg - t.yaw_rate_deg * DT,
        }
    }

    fn track(c: Command) -> (f64, f64, f64, Option<f64>) {
        match c {
            Command::Track {
                vx,
                vy,
                yaw_rate_deg,
                steer,
            } => (vx, vy, yaw_rate_deg, steer),
            other => panic!("expected Track, got {other:?}"),
        }
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn on_track_commands_the_target_velocity_unchanged() {
        let t = cruising();
        let (vx, vy, w, steer) = track(command(&t, &on_track(&t), DT, VEHICLE_GAINS, Some(MODEL3)));
        assert!(
            close(vx, -8.0) && close(vy, 0.0) && close(w, 0.0),
            "{vx} {vy} {w}"
        );
        assert_eq!(steer, Some(0.0));
    }

    #[test]
    fn aiming_at_the_target_itself_would_be_a_frame_ahead() {
        // The spike's first law: a body sitting on p* before the tick is 0.4 m (one frame at
        // 8 m/s) past where it should start, so the command slows it.
        let t = cruising();
        let at_target = Current {
            x: t.x,
            y: t.y,
            yaw_deg: t.yaw_deg,
        };
        let (vx, _, _, _) = track(command(&t, &at_target, DT, VEHICLE_GAINS, Some(MODEL3)));
        assert!(close(vx, -8.0 + VEHICLE_GAINS.position * 0.4), "{vx}");
    }

    #[test]
    fn a_lagging_body_is_sped_up_along_the_error() {
        let t = cruising();
        let mut c = on_track(&t);
        c.x += 0.2; // 0.2 m behind (westbound: behind is +x)
        let (vx, vy, _, _) = track(command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3)));
        assert!(close(vx, -8.0 - VEHICLE_GAINS.position * 0.2), "{vx}");
        assert!(close(vy, 0.0));
    }

    #[test]
    fn an_overshooting_body_is_slowed() {
        let t = cruising();
        let mut c = on_track(&t);
        c.x -= 0.2;
        let (vx, _, _, _) = track(command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3)));
        assert!(
            vx > -8.0 && close(vx, -8.0 + VEHICLE_GAINS.position * 0.2),
            "{vx}"
        );
    }

    #[test]
    fn a_yaw_error_is_corrected_across_the_wrap() {
        // Target 180 deg, body at -178 deg: 2 deg the short way, not 358.
        let t = cruising();
        let mut c = on_track(&t);
        c.yaw_deg = -178.0;
        let (_, _, w, _) = track(command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3)));
        assert!(close(w, -2.0 * VEHICLE_GAINS.yaw), "{w}");
    }

    #[test]
    fn snaps_beyond_the_distance_threshold_to_the_start_of_step_pose() {
        let t = cruising();
        let mut c = on_track(&t);
        c.y += 0.49;
        assert!(matches!(
            command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3)),
            Command::Track { .. }
        ));
        c.y += 0.02;
        match command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3)) {
            Command::Snap {
                x,
                y,
                yaw_deg,
                vx,
                vy,
                ..
            } => {
                assert!(close(x, 100.4) && close(y, 50.0) && close(yaw_deg, 180.0));
                assert!(close(vx, -8.0) && close(vy, 0.0));
            }
            other => panic!("expected Snap, got {other:?}"),
        }
    }

    #[test]
    fn snaps_beyond_the_yaw_threshold_but_a_walker_never_does() {
        let t = cruising();
        let mut c = on_track(&t);
        c.yaw_deg = 174.9;
        assert!(matches!(
            command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3)),
            Command::Snap { .. }
        ));
        let walker = Target {
            vx: -1.4,
            ..cruising()
        };
        let mut c = on_track(&walker);
        c.yaw_deg = 90.0;
        let (_, _, _, steer) = track(command(&walker, &c, DT, WALKER_GAINS, None));
        assert_eq!(steer, None);
    }

    #[test]
    fn a_stopped_target_holds_still_without_dividing_by_zero() {
        let t = Target {
            vx: 0.0,
            vy: 0.0,
            yaw_rate_deg: 0.0,
            ..cruising()
        };
        let (vx, vy, w, steer) = track(command(&t, &on_track(&t), DT, VEHICLE_GAINS, Some(MODEL3)));
        assert!(close(vx, 0.0) && close(vy, 0.0) && close(w, 0.0));
        assert_eq!(steer, Some(0.0));
    }

    #[test]
    fn steer_follows_the_curvature_and_saturates() {
        // A right turn (positive yaw rate) of radius 10 m at 5 m/s: 28.6 deg/s.
        let rate = (5.0_f64 / 10.0).to_degrees();
        let t = Target {
            vx: -5.0,
            yaw_rate_deg: rate,
            ..cruising()
        };
        let (_, _, _, steer) = track(command(&t, &on_track(&t), DT, VEHICLE_GAINS, Some(MODEL3)));
        let expected = (2.9_f64 / 10.0).atan().to_degrees() / 70.0;
        assert!(close(steer.unwrap(), expected), "{steer:?}");
        // A left turn steers left.
        let left = Target {
            yaw_rate_deg: -rate,
            ..t
        };
        let (_, _, _, steer) = track(command(
            &left,
            &on_track(&left),
            DT,
            VEHICLE_GAINS,
            Some(MODEL3),
        ));
        assert!(close(steer.unwrap(), -expected));
        // Turning on the spot at walking pace: clamped to full lock.
        let tight = Target {
            vx: -0.5,
            yaw_rate_deg: 90.0,
            ..cruising()
        };
        let (_, _, _, steer) = track(command(
            &tight,
            &on_track(&tight),
            DT,
            VEHICLE_GAINS,
            Some(MODEL3),
        ));
        assert_eq!(steer, Some(1.0));
    }

    #[test]
    fn wrap_stays_in_the_half_open_range() {
        assert!(close(wrap_deg(190.0), -170.0));
        assert!(close(wrap_deg(-190.0), 170.0));
        assert!(close(wrap_deg(180.0), 180.0));
        assert!(close(wrap_deg(-180.0), 180.0));
        assert!(close(wrap_deg(540.0), 180.0));
    }

    #[test]
    fn a_straight_twist_turns_with_the_heading_and_flips_y() {
        // Heading +90 deg in ROS (north, +y), 5 m/s forward: ROS (0, 5) -> CARLA (0, -5).
        let (vx, vy, w) = world_twist(std::f64::consts::FRAC_PI_2, 5.0, 0.0, 0.0, 1.5, 0.0);
        assert!(
            vx.abs() < 1e-9 && close(vy, -5.0) && close(w, 0.0),
            "{vx} {vy} {w}"
        );
    }

    #[test]
    fn in_a_turn_the_body_moves_faster_than_the_rear_axle() {
        // Heading 0, 5 m/s, turning left at 0.7 rad/s, body centre 1.5 m ahead: the centre
        // moves 0.7 * 1.5 = 1.05 m/s sideways (ROS +y, CARLA -y); CARLA's yaw rate is -0.7
        // rad/s in degrees.
        let (vx, vy, w) = world_twist(0.0, 5.0, 0.0, 0.7, 1.5, 0.0);
        assert!(close(vx, 5.0) && close(vy, -1.05), "{vx} {vy}");
        assert!(close(w, -0.7_f64.to_degrees()), "{w}");
    }

    #[test]
    fn steering_needs_a_wheelbase_and_a_lock() {
        assert_eq!(
            steering(3.1, 0.0, 70.0),
            Some(Steering {
                wheelbase: 3.1,
                max_steer_deg: 70.0
            })
        );
        assert_eq!(steering(0.0, 0.0, 70.0), None);
        assert_eq!(steering(3.1, 0.0, 0.0), None);
        assert_eq!(steering(f64::NAN, 0.0, 70.0), None);
    }

    #[test]
    fn tracking_stats_summarise_error_yaw_and_snaps() {
        let mut s = TrackingStats::default();
        for i in 1..=100 {
            s.record(f64::from(i) / 100.0, Some(-f64::from(i) / 10.0), i == 100);
        }
        let sum = s.summary();
        // Stored as f32: compare at f32 precision.
        let near = |a: f64, b: f64| (a - b).abs() < 1e-5;
        assert_eq!((sum.frames, sum.snaps), (100, 1));
        assert!(
            near(sum.error_p95, 0.95) && near(sum.error_max, 1.0),
            "{sum:?}"
        );
        assert!(near(sum.yaw_p95.unwrap(), 9.5) && near(sum.yaw_max.unwrap(), 10.0));
        let mut walker = TrackingStats::default();
        walker.record(0.1, None, false);
        assert_eq!(walker.summary().yaw_p95, None);
        assert_eq!(TrackingStats::default().summary().error_p95, 0.0);
    }
}
