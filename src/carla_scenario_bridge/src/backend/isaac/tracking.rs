//! Tracked NPCs on Isaac (roadmap 044): the CARLA backend's law (`npc_tracking`, roadmaps 023
//! and 032) applied to the world server's dynamic bodies.
//!
//! Everything here is in Isaac's world frame, which is ROS's (right-handed, yaw positive
//! counter-clockwise). `npc_tracking` is written in CARLA's frame, but its arithmetic only
//! needs poses, velocities and yaw rates from one frame, so it is used as is. Two things
//! differ from CARLA: no steer (a box has no tyres to undo a set yaw rate; the yaw rate is
//! set directly, as `set_target_angular_velocity` is), and a snap's height is SSv2's (the
//! world server has no ground query; the body falls the last centimetres).

use crate::npc_tracking::{self, Command, Steering, TrackingStats};

/// The steering geometry handed to `npc_tracking::command` for a vehicle that declares no
/// axles: only its presence matters here (it turns the yaw snap on), its steer is not used.
pub const DEFAULT_STEERING: Steering = Steering {
    wheelbase: 2.9,
    max_steer_deg: 70.0,
};

/// One tracked NPC's state across frames (the CARLA backend's `TrackedNpc`).
#[derive(Debug, Clone, Default)]
pub struct Tracked {
    /// Declared `(maxAcceleration, maxDeceleration)`, m/s2.
    pub performance: Option<(f64, f64)>,
    pub steering: Option<Steering>,
    /// SSv2's heading last frame, degrees (the yaw rate comes from the poses).
    pub last_yaw_deg: Option<f64>,
    /// The planar velocity commanded last frame, m/s.
    pub last_command: Option<(f64, f64)>,
    pub blocked: bool,
    pub held: bool,
    /// Not tracked yet (`npc_tracking::first_frame_change`).
    pub first: bool,
    /// Handed to kinematic after a blocked snap: posed by SSv2 from then on.
    pub kinematic: bool,
    /// The body's velocity after the last step (world frame, at its origin), from the server.
    pub velocity: [f64; 3],
    pub stats: TrackingStats,
    /// The frame of its last contact with the ego.
    pub last_ego_contact: Option<u64>,
}

impl Tracked {
    pub fn new(performance: Option<(f64, f64)>, steering: Option<Steering>) -> Self {
        Self {
            performance,
            steering: Some(steering.unwrap_or(DEFAULT_STEERING)),
            first: true,
            ..Self::default()
        }
    }

    /// Whether its last ego contact is within `npc_tracking::CONTACT_HOLD_FRAMES` of `frame`.
    pub fn in_contact(&self, frame: u64) -> bool {
        self.last_ego_contact
            .is_some_and(|f| frame.saturating_sub(f) <= npc_tracking::CONTACT_HOLD_FRAMES)
    }
}

/// SSv2's commanded pose for the frame, world frame: position, roll, pitch, yaw (degrees).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetPose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub yaw_deg: f64,
}

/// The body's pose before the step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyPose {
    pub x: f64,
    pub y: f64,
    pub yaw_deg: f64,
}

/// What the world server is asked to do with the body this step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Drive {
    /// Set this velocity (world frame, at the origin, m/s) and yaw rate (rad/s).
    Twist { linear: [f64; 3], yaw_rate: f64 },
    /// Put it at `pose` first (upright, SSv2's roll and pitch), then the twist.
    Teleport {
        pose: TargetPose,
        linear: [f64; 3],
        yaw_rate: f64,
    },
    /// Hand it to kinematic for good, at SSv2's pose (the blocked fallback).
    Kinematic,
}

/// One frame's inputs besides the NPC's own state.
#[derive(Debug, Clone, Copy)]
pub struct Frame {
    pub target: TargetPose,
    /// SSv2's twist in the entity frame (x forward, y left), m/s, and its yaw rate, rad/s;
    /// `None` while SSv2 holds the NPC still (before `npc_logic_started`).
    pub twist: Option<(f64, f64, f64)>,
    pub body: BodyPose,
    /// Box-to-box clearance to the nearest other body, m; `None` with none.
    pub clearance: Option<f64>,
    /// The SSv2 frame being commanded (for the contact hold).
    pub frame: u64,
    pub dt: f64,
}

/// Errors of the frame, for the stats and the log.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Outcome {
    pub drive: Drive,
    pub error: f64,
    pub yaw_error: f64,
    pub snapped: bool,
    pub held: bool,
}

/// One frame of a tracked NPC: the CARLA backend's `track_npc` without CARLA.
pub fn track(npc: &mut Tracked, f: &Frame) -> Outcome {
    let t = f.target;
    let yaw_rate_deg = match f.twist {
        Some((_, _, reported)) => {
            npc_tracking::pose_yaw_rate(npc.last_yaw_deg, t.yaw_deg, f.dt, reported.to_degrees())
        }
        None => 0.0,
    };
    npc.last_yaw_deg = Some(t.yaw_deg);
    let (vx_body, vy_body) = f.twist.map_or((0.0, 0.0), |(x, y, _)| (x, y));
    let (s, c) = t.yaw_deg.to_radians().sin_cos();
    let target = npc_tracking::Target {
        x: t.x,
        y: t.y,
        yaw_deg: t.yaw_deg,
        vx: vx_body * c - vy_body * s,
        vy: vx_body * s + vy_body * c,
        yaw_rate_deg,
    };
    let current = npc_tracking::Current {
        x: f.body.x,
        y: f.body.y,
        yaw_deg: f.body.yaw_deg,
    };
    let gains = npc_tracking::VEHICLE_GAINS;
    let in_contact = npc.in_contact(f.frame);
    let mut hold = in_contact;
    let mut command = npc_tracking::command(&target, &current, f.dt, gains, npc.steering, !hold);
    if !hold && matches!(command, Command::Snap { .. }) && npc_tracking::held(false, f.clearance) {
        hold = true;
        command = npc_tracking::command(&target, &current, f.dt, gains, npc.steering, false);
    }
    let body_planar = (npc.velocity[0], npc.velocity[1]);
    let blocked = npc_tracking::blocked(
        npc.blocked,
        npc.last_command,
        (target.vx, target.vy),
        body_planar,
        hold,
    );
    npc.blocked = blocked;
    npc.held = hold;
    let action = npc_tracking::act(command, blocked);
    let error = (t.x - target.vx * f.dt - f.body.x).hypot(t.y - target.vy * f.dt - f.body.y);
    let yaw_error = npc_tracking::wrap_deg(t.yaw_deg - yaw_rate_deg * f.dt - f.body.yaw_deg);
    let snapped = matches!(action, npc_tracking::Action::Command(Command::Snap { .. }));
    npc.stats.record(error, Some(yaw_error), snapped);
    let drive = match action {
        npc_tracking::Action::Fallback => {
            npc.kinematic = true;
            npc.stats.record_fallback();
            Drive::Kinematic
        }
        npc_tracking::Action::Command(Command::Track {
            vx,
            vy,
            yaw_rate_deg,
            ..
        }) => {
            let (vx, vy) = npc_tracking::bounded_command(
                (vx, vy),
                body_planar,
                target.yaw_rate_deg,
                f.dt,
                npc_tracking::first_frame_change(
                    npc.first,
                    f.twist.is_some(),
                    npc_tracking::max_change(npc.performance, f.dt),
                ),
            );
            npc.last_command = Some((vx, vy));
            Drive::Twist {
                linear: [vx, vy, npc.velocity[2]],
                yaw_rate: yaw_rate_deg.to_radians(),
            }
        }
        npc_tracking::Action::Command(Command::Snap {
            x,
            y,
            yaw_deg,
            vx,
            vy,
            yaw_rate_deg,
        }) => {
            npc.last_command = Some((vx, vy));
            Drive::Teleport {
                pose: TargetPose {
                    x,
                    y,
                    z: t.z,
                    roll_deg: t.roll_deg,
                    pitch_deg: t.pitch_deg,
                    yaw_deg,
                },
                linear: [vx, vy, 0.0],
                yaw_rate: yaw_rate_deg.to_radians(),
            }
        }
    };
    npc.first = false;
    Outcome {
        drive,
        error,
        yaw_error,
        snapped,
        held: hold,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.05;

    fn npc() -> Tracked {
        Tracked::new(Some((5.0, 8.0)), None)
    }

    /// Westbound (yaw 180) at 8 m/s, the body exactly a step behind the target.
    fn frame(target_x: f64, body_x: f64, frame: u64) -> Frame {
        Frame {
            target: TargetPose {
                x: target_x,
                y: -130.0,
                z: 0.0,
                roll_deg: 0.0,
                pitch_deg: 0.0,
                yaw_deg: 180.0,
            },
            twist: Some((8.0, 0.0, 0.0)),
            body: BodyPose {
                x: body_x,
                y: -130.0,
                yaw_deg: 180.0,
            },
            clearance: None,
            frame,
            dt: DT,
        }
    }

    fn twist(o: &Outcome) -> ([f64; 3], f64) {
        match o.drive {
            Drive::Twist { linear, yaw_rate } => (linear, yaw_rate),
            other => panic!("expected a twist, got {other:?}"),
        }
    }

    #[test]
    fn the_first_frame_of_a_moving_npc_starts_at_speed() {
        // SSv2 adds it moving; the body is at rest: the first command is not force-bounded.
        let mut n = npc();
        let o = track(&mut n, &frame(100.0, 100.4, 1));
        let (v, w) = twist(&o);
        assert!((v[0] + 8.0).abs() < 1e-9 && v[1].abs() < 1e-9, "{v:?}");
        assert_eq!(w, 0.0);
        assert!(!n.first);
    }

    #[test]
    fn later_frames_are_force_bounded_from_the_body_velocity() {
        let mut n = npc();
        track(&mut n, &frame(100.0, 100.4, 1));
        n.velocity = [0.0, 0.0, 0.0]; // held back: the body did not move
        let o = track(&mut n, &frame(99.6, 100.4, 2));
        let (v, _) = twist(&o);
        // (8 + 2) m/s2 x 0.05 s
        assert!((v[0] + 0.5).abs() < 1e-9, "{v:?}");
    }

    #[test]
    fn a_body_on_track_gets_the_target_velocity_in_the_world_frame() {
        let mut n = npc();
        n.first = false;
        n.velocity = [-8.0, 0.0, -0.1];
        let o = track(&mut n, &frame(100.0, 100.4, 5));
        let (v, _) = twist(&o);
        assert!((v[0] + 8.0).abs() < 1e-9 && v[1].abs() < 1e-9, "{v:?}");
        // Gravity keeps z: the body's vertical velocity is passed through.
        assert_eq!(v[2], -0.1);
        assert!(o.error < 1e-9 && !o.snapped);
    }

    #[test]
    fn far_off_snaps_at_ssv2_height_and_attitude() {
        let mut n = npc();
        n.first = false;
        n.velocity = [-8.0, 0.0, 0.0];
        let mut f = frame(100.0, 101.4, 5);
        f.target.z = 0.3;
        let o = track(&mut n, &f);
        match o.drive {
            Drive::Teleport { pose, linear, .. } => {
                assert!((pose.x - 100.4).abs() < 1e-9 && pose.z == 0.3, "{pose:?}");
                assert!(
                    (linear[0] + 8.0).abs() < 1e-9 && linear[1].abs() < 1e-9,
                    "{linear:?}"
                );
                assert_eq!(linear[2], 0.0);
            }
            other => panic!("expected a teleport, got {other:?}"),
        }
        assert!(o.snapped);
    }

    #[test]
    fn a_body_touching_the_ego_is_never_snapped() {
        let mut n = npc();
        n.first = false;
        n.velocity = [-8.0, 0.0, 0.0];
        n.last_ego_contact = Some(3);
        let o = track(&mut n, &frame(100.0, 101.4, 5));
        assert!(matches!(o.drive, Drive::Twist { .. }) && o.held);
    }

    #[test]
    fn a_body_next_to_another_is_held_not_snapped() {
        let mut n = npc();
        n.first = false;
        n.velocity = [-8.0, 0.0, 0.0];
        let mut f = frame(100.0, 101.4, 5);
        f.clearance = Some(0.2);
        let o = track(&mut n, &f);
        assert!(matches!(o.drive, Drive::Twist { .. }) && o.held);
    }

    #[test]
    fn a_blocked_body_that_would_snap_falls_back_to_kinematic() {
        let mut n = npc();
        n.first = false;
        n.last_command = Some((-8.0, 0.0));
        n.velocity = [0.5, 0.0, 0.0]; // thrown back by what it hit
        let o = track(&mut n, &frame(100.0, 101.4, 5));
        assert_eq!(o.drive, Drive::Kinematic);
        assert!(n.kinematic);
        assert_eq!(n.stats.summary().fallback, 1);
    }

    #[test]
    fn a_held_npc_gets_no_motion() {
        let mut n = npc();
        let mut f = frame(100.0, 100.0, 1);
        f.twist = None;
        let o = track(&mut n, &f);
        let (v, w) = twist(&o);
        assert_eq!((v[0], v[1], w), (0.0, 0.0, 0.0));
    }

    #[test]
    fn the_yaw_rate_comes_from_ssv2_poses_counter_clockwise() {
        let mut n = npc();
        n.first = false;
        n.velocity = [-8.0, 0.0, 0.0];
        let mut f = frame(100.0, 100.4, 5);
        f.target.yaw_deg = 179.0;
        f.body.yaw_deg = 178.0;
        n.last_yaw_deg = Some(178.0);
        let o = track(&mut n, &f);
        let (_, w) = twist(&o);
        // 1 deg in 0.05 s = 20 deg/s left, plus the gain on what is left of the error.
        assert!(w > 20f64.to_radians() - 1e-9, "{w}");
    }
}
