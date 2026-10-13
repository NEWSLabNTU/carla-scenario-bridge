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
/// Added to a vehicle's declared acceleration bound for the position correction, m/s2.
pub const FORCE_MARGIN: f64 = 2.0;
/// Acceleration bound for a vehicle that declares no `Performance`, m/s2.
pub const DEFAULT_MAX_ACCEL: f64 = 10.0;
/// Frames after the ego's last contact with a tracked NPC during which it is not snapped
/// (one second at 20 Hz): SSv2's pose for it may run through the ego.
pub const CONTACT_HOLD_FRAMES: u64 = 20;
/// How long a tracked frame waits for the client's cached world snapshot to catch up with
/// the last tick before using it anyway (roadmap 023, "a stale snapshot fails the frame").
/// A few tens of ms: under 1 ms is the normal case (LibCarla's `Tick` waits for its own
/// frame), and a frame is 50 ms at 20 Hz, of which CARLA's tick already takes ~10.
pub const SNAPSHOT_FRESH_WAIT: std::time::Duration = std::time::Duration::from_millis(30);
/// Pause between re-reads of a stale snapshot. A re-read is a local copy, not an RPC.
pub const SNAPSHOT_REREAD_PAUSE: std::time::Duration = std::time::Duration::from_millis(1);
/// Consecutive frames on which *every* tracked NPC may be missing from the snapshot before
/// the frame fails: 5 frames (0.25 s at 20 Hz) is a lag; longer is a real loss.
pub const MISS_LIMIT_FRAMES: u32 = 5;

/// Feedback gains, 1/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gains {
    pub position: f64,
    pub yaw: f64,
}

/// Position 5/s from 023's step-2 spike (settled p95 0.02-0.04 m on Town01's straights).
/// Yaw 4/s since roadmap 032 step 4, when the yaw correction began to be steered: a body turns
/// at ~0.98 of its steer's kinematic rate, and the yaw error a corner leaves is that shortfall
/// over the gain -- in the pure model at 0.95, 8 m/s on 6-20 m radii, 1.1-0.3 deg at 4/s
/// against 2.0 on 6 m at 023's 2/s (budget 2 deg). On the bench: yaw p95 1.55 -> 0.66 deg.
pub const VEHICLE_GAINS: Gains = Gains {
    position: 5.0,
    yaw: 4.0,
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

/// Blocked rule (roadmap 032, gap A): a body moving along the velocity it was commanded last
/// frame at less than this fraction of it -- or against it -- is held by something. Chosen
/// from the 024 bench: the 19 free NPCs' worst frame keeps 0.94 of the command (corners at
/// 8 m/s), `npc_05` meeting the lamp pole at frame 25 keeps -0.27 (-8.0 -> +2.17 m/s).
pub const BLOCKED_FRACTION: f64 = 0.5;
/// Below this commanded speed the blocked rule is off, m/s: a body starting or stopping
/// moves at a few tenths of a m/s that no fraction judges reliably. Under a walker's pace
/// (1.4 m/s), above the force bound's 0.5 m/s per frame.
pub const BLOCKED_MIN_SPEED: f64 = 1.0;

/// A tracked body whose CARLA box is closer than this to the ego's or any other physics
/// body's (box to box, the frame's world snapshot) is held as if in contact (roadmap 032
/// N2): not snapped, never handed to kinematic, m. The collision stream alone cannot be
/// trusted with "next to": in `rear_contact` run 3 the ego contact arrived 4 frames late and
/// `npc_rear` fell back with its box 0.163 m from the ego's, then drove through it. 0.5 m is
/// three times that, and one frame of motion at a 10 m/s closing speed. It does not reach
/// free traffic: over every step-7 trace (bench x3, starters, gap scenarios), no NPC that was
/// not touching the ego came within 0.5 m of another body, while at 0.75 m the bench's own
/// NPCs start to (16-20 frames per run, passing in the corners); `npc_05` falls back 38.6 m
/// from the nearest body.
pub const PROXIMITY_HOLD_M: f64 = 0.5;

/// Whether a tracked body is held this frame -- not snapped, not blocked -- by a contact
/// with the ego (`in_contact`, 023) or by another body's box within
/// [`PROXIMITY_HOLD_M`] (`clearance`, `None` with no other body to measure against).
pub fn held(in_contact: bool, clearance: Option<f64>) -> bool {
    in_contact || clearance.is_some_and(|d| d < PROXIMITY_HOLD_M)
}

/// Whether a tracked body is blocked this frame (roadmap 032, gap A).
///
/// It *becomes* blocked when its velocity along the velocity it was commanded last frame
/// (`last_command`, set before the tick it has just had) is below [`BLOCKED_FRACTION`] of
/// it, or reversed, while that command is above [`BLOCKED_MIN_SPEED`]. Against last frame's
/// command, not SSv2's: a body accelerating at the force bound (a start, a speed step) moves
/// at what it was given and is not blocked.
///
/// It *stays* blocked (`was_blocked`) until it makes the same progress along SSv2's
/// velocity (`target`), or SSv2 slows below the minimum: once held, the force-bounded
/// command follows the body (bench `npc_05`, frame 26: commanded +1.7 m/s, away from the
/// pole, and moving at it), so last frame's command alone would clear it a frame before the
/// snap that throws it.
///
/// Never while the body is in contact with the ego: 023's contact hold covers those.
pub fn blocked(
    was_blocked: bool,
    last_command: Option<(f64, f64)>,
    target: (f64, f64),
    body: (f64, f64),
    in_contact: bool,
) -> bool {
    if in_contact {
        return false;
    }
    if last_command.is_some_and(|c| short_of(body, c)) {
        return true;
    }
    was_blocked && short_of(body, target)
}

/// Whether `body` moves along `reference` at less than [`BLOCKED_FRACTION`] of it, with
/// `reference` faster than [`BLOCKED_MIN_SPEED`].
fn short_of(body: (f64, f64), reference: (f64, f64)) -> bool {
    let speed = reference.0.hypot(reference.1);
    if !speed.is_finite() || speed <= BLOCKED_MIN_SPEED {
        return false;
    }
    let along = (body.0 * reference.0 + body.1 * reference.1) / speed;
    along < BLOCKED_FRACTION * speed
}

/// What a tracked frame does with its body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Action {
    /// Track or snap, as `command` decided.
    Command(Command),
    /// Hand the body to kinematic for the rest of the episode: physics off, teleported to
    /// SSv2's pose every frame. A blocked body is never snapped into what blocks it.
    Fallback,
}

/// The frame's action: a snap of a [`blocked`] body falls back to kinematic; anything else
/// is the command as is (a blocked body under the snap thresholds keeps tracking).
pub fn act(command: Command, blocked: bool) -> Action {
    match command {
        Command::Snap { .. } if blocked => Action::Fallback,
        c => Action::Command(c),
    }
}

/// Where a snap puts the body besides x, y and yaw (roadmap 032, gap A), CARLA degrees and m.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SnapAttitude {
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub z: f64,
}

/// A snap's roll, pitch and z: SSv2's roll and pitch, not the body's (a tumbling body is put
/// back upright), and the box bottom on the ground under the target -- `lift` (origin above
/// the box bottom) over `ground_z`, or over SSv2's `commanded_z` when there is no ground.
pub fn snap_attitude(
    target_roll_deg: f64,
    target_pitch_deg: f64,
    ground_z: Option<f64>,
    commanded_z: f64,
    lift: f64,
) -> SnapAttitude {
    SnapAttitude {
        roll_deg: target_roll_deg,
        pitch_deg: target_pitch_deg,
        z: ground_z.unwrap_or(commanded_z) + lift,
    }
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
/// With `snap_allowed` false (the body is in contact) the command is always `Track`.
pub fn command(
    target: &Target,
    current: &Current,
    dt: f64,
    gains: Gains,
    steering: Option<Steering>,
    snap_allowed: bool,
) -> Command {
    let start_x = target.x - target.vx * dt;
    let start_y = target.y - target.vy * dt;
    let start_yaw = target.yaw_deg - target.yaw_rate_deg * dt;
    let ex = start_x - current.x;
    let ey = start_y - current.y;
    let eyaw = wrap_deg(start_yaw - current.yaw_deg);
    let checks_yaw = steering.is_some();
    if snap_allowed && (ex.hypot(ey) > SNAP_DISTANCE || (checks_yaw && eyaw.abs() > SNAP_YAW_DEG)) {
        return Command::Snap {
            x: start_x,
            y: start_y,
            yaw_deg: wrap_deg(start_yaw),
            vx: target.vx,
            vy: target.vy,
            yaw_rate_deg: target.yaw_rate_deg,
        };
    }
    let yaw_rate_deg = target.yaw_rate_deg + gains.yaw * eyaw;
    // Steered from the commanded yaw rate, correction included (roadmap 032, gap B): the
    // tyres undo a set angular velocity within the tick -- `npc_11` on a straight held a
    // 2.4 deg error for 60 frames, commanded -4.7 deg/s and turning 0.0 -- so the steer is
    // what turns the body, and steering SSv2's rate alone left every yaw error in place.
    let steer = steering.map(|s| {
        let speed = target.vx.hypot(target.vy);
        if speed < STEER_MIN_SPEED || s.max_steer_deg <= 0.0 {
            return 0.0;
        }
        let curvature = yaw_rate_deg.to_radians() / speed;
        let angle = (s.wheelbase * curvature).atan().to_degrees();
        (angle / s.max_steer_deg).clamp(-1.0, 1.0)
    });
    Command::Track {
        vx: target.vx + gains.position * ex,
        vy: target.vy + gains.position * ey,
        yaw_rate_deg,
        steer,
    }
}

/// `command` bounded to a finite force: its change from the body's current velocity is at
/// most `max_change` (m/s), in any direction. `set_target_velocity` otherwise resets the
/// velocity outright -- an unbounded force, which a body held back by a contact turns into
/// an unbounded shove. Free-running, the body already moves at last frame's command and the
/// change is small.
pub fn limit_change(command: (f64, f64), body: (f64, f64), max_change: f64) -> (f64, f64) {
    let (dx, dy) = (command.0 - body.0, command.1 - body.1);
    let change = dx.hypot(dy);
    if change <= max_change || change == 0.0 {
        return command;
    }
    let k = max_change.max(0.0) / change;
    (body.0 + dx * k, body.1 + dy * k)
}

/// `v` turned through one frame of the target's yaw rate (CARLA frame: positive turns x
/// toward y, as the yaw does). A body following a turn needs its velocity turned by `ω·dt`
/// every frame -- the centripetal change, `v·ω·dt`: 0.46 m/s a frame at 8 m/s on a 7 m
/// radius -- which is the turn itself, not a correction.
pub fn turned(v: (f64, f64), yaw_rate_deg: f64, dt: f64) -> (f64, f64) {
    let (sin, cos) = (yaw_rate_deg.to_radians() * dt).sin_cos();
    (v.0 * cos - v.1 * sin, v.0 * sin + v.1 * cos)
}

/// The curvature feed-forward of the force bound (roadmap 032, gap B): [`limit_change`]
/// measured from the body's velocity carried through the target's turn ([`turned`]), not from
/// the body's velocity itself. As built in 023 the bound charged the turn to the force: on
/// the bench's 6-7 m corners at 8 m/s the centripetal 9-11 m/s2 (12-18 at SSv2's entries)
/// used all of the 10 m/s2 bound, nothing was left to correct with, and the body drifted to
/// the outside of every corner, 0.24-0.58 m. Turning never adds speed, so a body held back
/// by a contact still pushes with no more than `max_change`. Straight, it is `limit_change`.
pub fn bounded_command(
    command: (f64, f64),
    body: (f64, f64),
    yaw_rate_deg: f64,
    dt: f64,
    max_change: f64,
) -> (f64, f64) {
    limit_change(command, turned(body, yaw_rate_deg, dt), max_change)
}

/// The velocity change allowed per frame for a vehicle declaring `max_acceleration` and
/// `max_deceleration` (m/s2; `None` when it declares no `Performance`).
pub fn max_change(declared: Option<(f64, f64)>, dt: f64) -> f64 {
    let bound = declared
        .map(|(a, d)| a.abs().max(d.abs()))
        .filter(|b| b.is_finite() && *b > 0.0)
        .unwrap_or(DEFAULT_MAX_ACCEL - FORCE_MARGIN);
    (bound + FORCE_MARGIN) * dt
}

/// The velocity change allowed on a tracked frame, m/s (roadmap 032, gap D): `bound`
/// ([`max_change`]), except on a body's first tracked frame when SSv2 already moves it
/// (`moving`: its twist is passed, i.e. not held before `npc_logic_started`). SSv2 adds an
/// entity moving -- UC-ACC's `Npc1` at ~5.5 m/s -- but its spawn request carries no speed
/// (`initial_speed` is always 0 in `traffic_simulator`'s `API::spawn`); unbounded, that first
/// command sets the body's velocity outright, so it starts at speed instead of accelerating
/// from rest at the force bound and falling 0.57 m behind. A held NPC starts at rest.
pub fn first_frame_change(first: bool, moving: bool, bound: f64) -> f64 {
    if first && moving {
        f64::INFINITY
    } else {
        bound
    }
}

/// Below this commanded speed a walker stands still, m/s (roadmap 032, gap C). A standing
/// SSv2 pedestrian's poses step by its position precision -- 0.004 and 0.008 m/s alternating,
/// a command of at most 0.007 m/s (`town01_pedestrian`, 024 step 7, 203 standing frames) --
/// and CARLA turns a walker toward any direction it is given, at 18 deg per frame, however
/// slow. Seven times that noise; left uncorrected under it, a walker's error is at most
/// `floor / WALKER_GAINS.position` = 0.008 m, under a twentieth of G3's 0.2 m.
pub const WALKER_SPEED_FLOOR: f64 = 0.05;

/// A walker's `WalkerControl` for a commanded velocity: `(direction x, direction y, speed)`,
/// CARLA frame. Below [`WALKER_SPEED_FLOOR`] it stands -- speed 0, facing SSv2's heading
/// (`heading_deg`, CARLA yaw) -- instead of turning toward a normalised noise vector.
pub fn walker_control(vx: f64, vy: f64, heading_deg: f64) -> (f64, f64, f64) {
    let speed = vx.hypot(vy);
    if speed.is_finite() && speed >= WALKER_SPEED_FLOOR {
        (vx / speed, vy / speed, speed)
    } else {
        let (sin, cos) = heading_deg.to_radians().sin_cos();
        (cos, sin, 0.0)
    }
}

/// Above this a heading change between frames is a teleport, not a turn, deg/s.
pub const MAX_POSE_YAW_RATE_DEG: f64 = 180.0;

/// The yaw rate SSv2's poses imply, deg/s: the heading change since the previous frame over
/// `dt`. SSv2's NPC twist reports none (0 through a corner), so its own poses are the source.
/// No previous heading, or a change faster than [`MAX_POSE_YAW_RATE_DEG`] (a teleport):
/// `reported`.
pub fn pose_yaw_rate(previous_deg: Option<f64>, now_deg: f64, dt: f64, reported: f64) -> f64 {
    match previous_deg {
        Some(prev) if dt > 0.0 => {
            let rate = wrap_deg(now_deg - prev) / dt;
            if rate.abs() <= MAX_POSE_YAW_RATE_DEG {
                rate
            } else {
                reported
            }
        }
        _ => reported,
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
    missed: u32,
    fallback: u32,
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
    /// Frames skipped because the NPC was not in the world snapshot: no command sent.
    pub missed: u32,
    /// Frames driven kinematic after a blocked body fell back (roadmap 032), the hand-over
    /// frame included; 0 for an NPC tracked to the end.
    pub fallback: u32,
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

    /// One frame skipped: the NPC was not in the world snapshot.
    pub fn record_missed(&mut self) {
        self.missed += 1;
    }

    /// One frame driven kinematic after a fallback (roadmap 032).
    pub fn record_fallback(&mut self) {
        self.fallback += 1;
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
            missed: self.missed,
            fallback: self.fallback,
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

/// Whether a world snapshot of `snapshot_frame` is at least as new as the last tick
/// (`None`: nothing ticked yet in this world, so any snapshot will do).
pub fn snapshot_is_fresh(snapshot_frame: u64, last_tick: Option<u64>) -> bool {
    last_tick.is_none_or(|tick| snapshot_frame >= tick)
}

/// A snapshot from [`read_fresh`] and how it was obtained.
#[derive(Debug)]
pub struct FreshRead<S> {
    pub snapshot: S,
    /// Its frame number.
    pub frame: u64,
    /// Reads it took; 1 when the first was fresh.
    pub reads: u32,
    /// Whether it is as new as the last tick; `false` once `bound` ran out.
    pub fresh: bool,
}

/// Read a snapshot, and re-read it while it is older than `last_tick`, for at most `bound`
/// (pausing `pause` between reads). A read error ends it at once. Past the bound the last
/// read is returned as is, `fresh: false`.
pub fn read_fresh<S, E>(
    last_tick: Option<u64>,
    bound: std::time::Duration,
    pause: std::time::Duration,
    mut read: impl FnMut() -> Result<S, E>,
    frame_of: impl Fn(&S) -> u64,
) -> Result<FreshRead<S>, E> {
    let started = std::time::Instant::now();
    let mut reads = 0;
    loop {
        let snapshot = read()?;
        reads += 1;
        let frame = frame_of(&snapshot);
        let fresh = snapshot_is_fresh(frame, last_tick);
        if fresh || started.elapsed() >= bound {
            return Ok(FreshRead {
                snapshot,
                frame,
                reads,
                fresh,
            });
        }
        std::thread::sleep(pause);
    }
}

/// The miss rule for tracked NPCs not in the world snapshot (roadmap 023): a missing NPC is
/// skipped for the frame, warned about once per episode; only every tracked NPC missing
/// for more than [`MISS_LIMIT_FRAMES`] consecutive frames fails the frame.
#[derive(Debug, Default)]
pub struct MissWatch {
    warned: std::collections::HashSet<String>,
    /// Consecutive frames on which every tracked NPC looked up was missing.
    all_missed_run: u32,
}

impl MissWatch {
    /// `name` was missing this frame. Returns whether to warn (its first miss this episode).
    pub fn missed(&mut self, name: &str) -> bool {
        self.warned.insert(name.to_string())
    }

    /// The end of a frame in which `looked_up` tracked NPCs were looked up and `missed` of
    /// them were not in the snapshot. Returns whether the frame fails.
    pub fn end_frame(&mut self, looked_up: usize, missed: usize) -> bool {
        if looked_up == 0 {
            return false;
        }
        if missed >= looked_up {
            self.all_missed_run += 1;
            self.all_missed_run > MISS_LIMIT_FRAMES
        } else {
            self.all_missed_run = 0;
            false
        }
    }

    /// A new episode: warn afresh, forget the run.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
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
        let (vx, vy, w, steer) = track(command(
            &t,
            &on_track(&t),
            DT,
            VEHICLE_GAINS,
            Some(MODEL3),
            true,
        ));
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
        let (vx, _, _, _) = track(command(
            &t,
            &at_target,
            DT,
            VEHICLE_GAINS,
            Some(MODEL3),
            true,
        ));
        assert!(close(vx, -8.0 + VEHICLE_GAINS.position * 0.4), "{vx}");
    }

    #[test]
    fn a_lagging_body_is_sped_up_along_the_error() {
        let t = cruising();
        let mut c = on_track(&t);
        c.x += 0.2; // 0.2 m behind (westbound: behind is +x)
        let (vx, vy, _, _) = track(command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), true));
        assert!(close(vx, -8.0 - VEHICLE_GAINS.position * 0.2), "{vx}");
        assert!(close(vy, 0.0));
    }

    #[test]
    fn an_overshooting_body_is_slowed() {
        let t = cruising();
        let mut c = on_track(&t);
        c.x -= 0.2;
        let (vx, _, _, _) = track(command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), true));
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
        let (_, _, w, _) = track(command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), true));
        assert!(close(w, -2.0 * VEHICLE_GAINS.yaw), "{w}");
    }

    #[test]
    fn snaps_beyond_the_distance_threshold_to_the_start_of_step_pose() {
        let t = cruising();
        let mut c = on_track(&t);
        c.y += 0.49;
        assert!(matches!(
            command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), true),
            Command::Track { .. }
        ));
        c.y += 0.02;
        match command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), true) {
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
            command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), true),
            Command::Snap { .. }
        ));
        let walker = Target {
            vx: -1.4,
            ..cruising()
        };
        let mut c = on_track(&walker);
        c.yaw_deg = 90.0;
        let (_, _, _, steer) = track(command(&walker, &c, DT, WALKER_GAINS, None, true));
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
        let (vx, vy, w, steer) = track(command(
            &t,
            &on_track(&t),
            DT,
            VEHICLE_GAINS,
            Some(MODEL3),
            true,
        ));
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
        let (_, _, _, steer) = track(command(
            &t,
            &on_track(&t),
            DT,
            VEHICLE_GAINS,
            Some(MODEL3),
            true,
        ));
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
            true,
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
            true,
        ));
        assert_eq!(steer, Some(1.0));
    }

    #[test]
    fn no_snap_while_in_contact() {
        // Held 0.8 m back by the ego: a snap would teleport the body into it.
        let t = cruising();
        let mut c = on_track(&t);
        c.x += 0.8;
        assert!(matches!(
            command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), true),
            Command::Snap { .. }
        ));
        let (vx, _, _, _) = track(command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), false));
        assert!(close(vx, -8.0 - VEHICLE_GAINS.position * 0.8), "{vx}");
    }

    #[test]
    fn the_velocity_change_is_capped_by_the_declared_bounds() {
        // Declared 5 / 8 m/s2: 10 m/s2 with the margin, 0.5 m/s in a 0.05 s frame.
        let cap = max_change(Some((5.0, 8.0)), DT);
        assert!(close(cap, 0.5), "{cap}");
        assert!(close(max_change(None, DT), DEFAULT_MAX_ACCEL * DT));
        // Held to 4 m/s by a contact, commanded 8 m/s: it may push to 4.5, no more.
        let v = limit_change((-8.0, 0.0), (-4.0, 0.0), cap);
        assert!(close(v.0, -4.5) && close(v.1, 0.0), "{v:?}");
        // Free-running at last frame's command: unchanged.
        assert_eq!(limit_change((-8.1, 0.2), (-8.0, 0.0), cap), (-8.1, 0.2));
    }

    #[test]
    fn the_yaw_rate_comes_from_the_poses_unless_they_jump() {
        // 2 deg a frame at 20 Hz: 40 deg/s, whatever the twist said.
        assert!(close(pose_yaw_rate(Some(10.0), 12.0, DT, 0.0), 40.0));
        // Across the wrap.
        assert!(close(pose_yaw_rate(Some(179.0), -179.0, DT, 0.0), 40.0));
        // First frame, or a teleport (90 deg in one frame): the reported rate.
        assert!(close(pose_yaw_rate(None, 12.0, DT, 5.0), 5.0));
        assert!(close(pose_yaw_rate(Some(0.0), 90.0, DT, 5.0), 5.0));
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

    const WEST: (f64, f64) = (-8.0, 0.0);

    #[test]
    fn npc_05_at_the_lamp_pole_is_blocked() {
        // 024 bench, frame 25: commanded -8.0 m/s westward, the body bounced to +2.17.
        assert!(blocked(false, Some(WEST), WEST, (2.17, 0.0), false));
        // Stopped dead, or sliding sideways along the obstacle: no progress, blocked.
        assert!(blocked(false, Some(WEST), WEST, (0.0, 0.0), false));
        assert!(blocked(false, Some(WEST), WEST, (-0.5, 3.0), false));
    }

    #[test]
    fn a_held_body_stays_blocked_until_it_makes_progress() {
        // Frame 26: the force-bounded command has followed the body (+2.17 + 0.5 cap toward
        // the target = +1.67) and the body moves at it -- still held by the pole.
        assert!(blocked(true, Some((1.67, 0.0)), WEST, (1.6, 0.0), false));
        // The same frame without the latch: not blocked against its own command.
        assert!(!blocked(false, Some((1.67, 0.0)), WEST, (1.6, 0.0), false));
        // Free again and back above the fraction of SSv2's speed: cleared.
        assert!(!blocked(true, Some((-4.5, 0.0)), WEST, (-4.4, 0.0), false));
        // SSv2 stopped it: nothing to be blocked from.
        assert!(!blocked(
            true,
            Some((0.5, 0.0)),
            (0.0, 0.0),
            (0.4, 0.0),
            false
        ));
    }

    #[test]
    fn a_free_body_is_not_blocked() {
        // The bench's worst free frame: 0.94 of the command, in a corner.
        assert!(!blocked(
            false,
            Some((-8.39, 0.0)),
            WEST,
            (-7.89, 0.3),
            false
        ));
        // Starting from rest at the force bound, SSv2 already at 8 m/s: it moves at what it
        // was given.
        assert!(!blocked(
            false,
            Some((-0.5, 0.0)),
            WEST,
            (-0.49, 0.0),
            false
        ));
        assert!(!blocked(false, Some((-3.0, 0.0)), WEST, (-2.9, 0.0), false));
        // Below the minimum commanded speed the rule is off.
        assert!(!blocked(false, Some((-0.9, 0.0)), WEST, (0.0, 0.0), false));
        // No command yet (first frame).
        assert!(!blocked(false, None, WEST, (0.0, 0.0), false));
        // In contact with the ego: the contact hold's case, not this rule's -- latched or not.
        assert!(!blocked(false, Some(WEST), WEST, (2.17, 0.0), true));
        assert!(!blocked(true, Some(WEST), WEST, (2.17, 0.0), true));
        // Either side of the fraction.
        let edge = -8.0 * BLOCKED_FRACTION;
        assert!(!blocked(false, Some(WEST), WEST, (edge - 0.01, 0.0), false));
        assert!(blocked(false, Some(WEST), WEST, (edge + 0.01, 0.0), false));
    }

    #[test]
    fn a_blocked_body_is_never_snapped_into_place() {
        let t = cruising();
        let mut c = on_track(&t);
        c.x -= 0.9; // npc_05's 0.9 m error, held back by the pole
        let snap = command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), true);
        assert!(matches!(snap, Command::Snap { .. }));
        assert_eq!(act(snap, true), Action::Fallback);
        // Unblocked, the same snap is a snap.
        assert_eq!(act(snap, false), Action::Command(snap));
        // Blocked but within the snap thresholds: it keeps tracking.
        let track = command(&t, &on_track(&t), DT, VEHICLE_GAINS, Some(MODEL3), true);
        assert_eq!(act(track, true), Action::Command(track));
    }

    #[test]
    fn a_body_next_to_another_is_held_as_in_contact() {
        // rear_contact run 3, frame 269: npc_rear's box 0.163 m from the ego's, the ego
        // contact not yet drained (4 frames late).
        assert!(held(false, Some(0.163)));
        assert!(held(false, Some(-0.2)), "overlapping boxes");
        assert!(held(true, None), "an ego contact holds as before");
        assert!(held(true, Some(40.0)));
        // npc_05 at the lamp pole: the nearest body 38.6 m away. Free traffic: 0.5 m and up.
        assert!(!held(false, Some(38.6)));
        assert!(!held(false, Some(PROXIMITY_HOLD_M)));
        assert!(!held(false, None), "nothing else in the world");
    }

    #[test]
    fn a_blocked_body_next_to_the_ego_never_falls_back() {
        // The run-3 frame: blocked (moving -1.56 m/s against SSv2's westward command, the
        // latch set) and far enough off to snap -- but 0.163 m from the ego, contact unseen.
        let t = cruising();
        let mut c = on_track(&t);
        c.x -= 0.9;
        let hold = held(false, Some(0.163));
        let is_blocked = blocked(true, Some(WEST), WEST, (1.56, 0.0), hold);
        let action = act(
            command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), !hold),
            is_blocked,
        );
        assert!(
            matches!(action, Action::Command(Command::Track { .. })),
            "held: tracked under the force bound, neither snapped nor kinematic: {action:?}"
        );
        // The same body 38.6 m from anything (npc_05 at its pole): falls back as before.
        let hold = held(false, Some(38.6));
        let is_blocked = blocked(true, Some(WEST), WEST, (1.56, 0.0), hold);
        assert_eq!(
            act(
                command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), !hold),
                is_blocked
            ),
            Action::Fallback
        );
    }

    #[test]
    fn an_unblocked_snap_is_upright_on_the_ground() {
        // The body tumbling (roll -75, pitch -30, 12 m up); SSv2 says level road.
        let a = snap_attitude(0.0, 0.0, Some(0.02), 0.0, 0.008);
        assert_eq!(a.roll_deg, 0.0);
        assert_eq!(a.pitch_deg, 0.0);
        assert!(close(a.z, 0.028), "{a:?}");
        // SSv2's own attitude on a slope is kept.
        let a = snap_attitude(1.5, -4.0, Some(10.0), 0.0, 0.0);
        assert!(close(a.roll_deg, 1.5) && close(a.pitch_deg, -4.0) && close(a.z, 10.0));
        // No ground: SSv2's z.
        assert!(close(snap_attitude(0.0, 0.0, None, 0.3, 0.9).z, 1.2));
    }

    #[test]
    fn fallback_frames_are_counted() {
        let mut s = TrackingStats::default();
        s.record(0.9, Some(0.0), false);
        s.record_fallback();
        s.record_fallback();
        let sum = s.summary();
        assert_eq!((sum.frames, sum.snaps, sum.fallback), (1, 0, 2));
        assert_eq!(TrackingStats::default().summary().fallback, 0);
    }

    #[test]
    fn a_standing_walker_faces_ssv2s_heading_and_does_not_move() {
        // town01_pedestrian frames 135-156: SSv2's position noise, 0.0038-0.0075 m/s in any
        // direction, used to be normalised into the walker's direction.
        for (vx, vy) in [(0.0, 0.0), (0.0038, 0.0), (-0.005, 0.0049), (0.0, -0.0075)] {
            let (dx, dy, speed) = walker_control(vx, vy, 90.0);
            assert_eq!(speed, 0.0);
            assert!(close(dx, 0.0) && close(dy, 1.0), "({dx}, {dy})");
        }
        let (dx, dy, _) = walker_control(0.01, 0.0, -161.6);
        assert!(
            close(dx, (-161.6f64).to_radians().cos()) && close(dy, (-161.6f64).to_radians().sin())
        );
        // Not a number: stand.
        assert_eq!(walker_control(f64::NAN, 0.0, 0.0), (1.0, 0.0, 0.0));
    }

    #[test]
    fn a_walking_walker_goes_where_it_is_commanded() {
        let (dx, dy, speed) = walker_control(0.0, 1.4, 0.0);
        assert!(close(dx, 0.0) && close(dy, 1.0) && close(speed, 1.4));
        // At the floor it walks; just under it, it stands.
        assert!(close(
            walker_control(WALKER_SPEED_FLOOR, 0.0, 90.0).2,
            WALKER_SPEED_FLOOR
        ));
        assert_eq!(walker_control(WALKER_SPEED_FLOOR - 1e-4, 0.0, 90.0).2, 0.0);
        // Left uncorrected under the floor, the error stays far inside G3's 0.2 m.
        const { assert!(WALKER_SPEED_FLOOR / WALKER_GAINS.position < 0.01) };
    }

    /// Share of the steer's kinematic yaw rate a tracked body turns at: the bench's corners
    /// turn at 0.98 of it (024/032 traces, `npc_09` at 65 deg/s); 0.95 leaves a margin.
    const UNDERSTEER: f64 = 0.95;

    /// SSv2's target on a 90 deg right turn of radius `r` at `speed`, CARLA frame: 1 s
    /// eastbound along +x, the arc, then southbound (+y) out of it. `(target, s)` at time `t`.
    fn corner_target(t: f64, speed: f64, r: f64) -> Target {
        let lead = speed; // 1 s of straight
        let arc = r * std::f64::consts::FRAC_PI_2;
        let s = speed * t;
        let rate = (speed / r).to_degrees();
        if s < lead {
            Target {
                x: s,
                y: 0.0,
                yaw_deg: 0.0,
                vx: speed,
                vy: 0.0,
                yaw_rate_deg: 0.0,
            }
        } else if s < lead + arc {
            let phi = (s - lead) / r;
            Target {
                x: lead + r * phi.sin(),
                y: r * (1.0 - phi.cos()),
                yaw_deg: phi.to_degrees(),
                vx: speed * phi.cos(),
                vy: speed * phi.sin(),
                yaw_rate_deg: rate,
            }
        } else {
            Target {
                x: lead + r,
                y: r + (s - lead - arc),
                yaw_deg: 90.0,
                vx: 0.0,
                vy: speed,
                yaw_rate_deg: 0.0,
            }
        }
    }

    /// Track [`corner_target`] through the pure model of a tracked body the bench's traces
    /// show (roadmap 032 step 4): the body keeps the velocity it is set over the tick (within
    /// 0.03 m/s in corners), and turns only as its steer turns it -- a set angular velocity is
    /// undone by the tyres within the tick (`npc_11`, 2.4 deg off on a straight: commanded
    /// -4.7 deg/s, turned 0.0) -- at [`UNDERSTEER`] of the kinematic bicycle's rate. The yaw
    /// rate comes from SSv2's poses, as `track_npc` takes it. Returns (max xy error m, max yaw
    /// error deg, snaps), measured after each tick against SSv2's pose, from the arc on.
    fn run_corner(speed: f64, r: f64) -> (f64, f64, u32) {
        let bound = max_change(Some((5.0, 8.0)), DT); // the bench's declared Performance
        let first = corner_target(0.0, speed, r);
        let (mut x, mut y, mut yaw) = (first.x, first.y, first.yaw_deg);
        let mut v = (first.vx, first.vy);
        let mut previous_yaw = first.yaw_deg;
        let (mut worst_xy, mut worst_yaw, mut snaps) = (0.0_f64, 0.0_f64, 0);
        for k in 1..=100 {
            let mut t = corner_target(f64::from(k) * DT, speed, r);
            t.yaw_rate_deg = pose_yaw_rate(Some(previous_yaw), t.yaw_deg, DT, 0.0);
            previous_yaw = t.yaw_deg;
            let current = Current { x, y, yaw_deg: yaw };
            let (vx, vy, steer) = match command(&t, &current, DT, VEHICLE_GAINS, Some(MODEL3), true)
            {
                Command::Track { vx, vy, steer, .. } => (vx, vy, steer.unwrap()),
                Command::Snap {
                    x: sx,
                    y: sy,
                    yaw_deg,
                    vx,
                    vy,
                    ..
                } => {
                    snaps += 1;
                    (x, y, yaw) = (sx, sy, yaw_deg);
                    (vx, vy, 0.0)
                }
            };
            v = bounded_command((vx, vy), v, t.yaw_rate_deg, DT, bound);
            let wheel = (steer * MODEL3.max_steer_deg).to_radians();
            let turn = UNDERSTEER * v.0.hypot(v.1) * wheel.tan() / MODEL3.wheelbase;
            x += v.0 * DT;
            y += v.1 * DT;
            yaw += turn.to_degrees() * DT;
            if f64::from(k) * DT >= 1.0 {
                worst_xy = worst_xy.max((x - t.x).hypot(y - t.y));
                worst_yaw = worst_yaw.max(wrap_deg(yaw - t.yaw_deg).abs());
            }
        }
        (worst_xy, worst_yaw, snaps)
    }

    #[test]
    fn a_constant_radius_corner_is_tracked_within_budget_at_8_mps() {
        // The bench's tight corners: 8 m/s on 6-7 m, 9.1-10.7 m/s2 of centripetal
        // acceleration against the 10 m/s2 force bound (SSv2's entries reach 12-18). As built
        // in 023 the bound spent itself turning the velocity and the body drifted to the
        // outside (this model, 6 m: 0.22 m and growing; the bench: 0.5 m and a snap), and the
        // yaw, steered from SSv2's rate alone, fell behind for good (4.6-4.8 deg here).
        for r in [6.0, 7.0, 10.0, 20.0] {
            let (xy, yaw, snaps) = run_corner(8.0, r);
            assert_eq!(snaps, 0, "r {r}: snapped");
            assert!(xy <= 0.2, "r {r}: xy {xy:.3} m");
            assert!(yaw <= 2.0, "r {r}: yaw {yaw:.2} deg");
        }
    }

    #[test]
    fn the_turn_is_not_charged_to_the_force_bound() {
        // On a 7 m turn at 8 m/s the velocity turns 0.057 rad a frame: 0.46 m/s of change.
        let rate = (8.0_f64 / 7.0).to_degrees();
        let body = (8.0, 0.0);
        let turned_body = turned(body, rate, DT);
        let a = (8.0_f64 / 7.0) * DT;
        assert!(close(turned_body.0, 8.0 * a.cos()) && close(turned_body.1, 8.0 * a.sin()));
        // A command on the turned velocity passes unbounded ...
        assert_eq!(
            bounded_command(turned_body, body, rate, DT, 0.1),
            turned_body
        );
        // ... and a correction on top of it is bounded from there.
        let push = (turned_body.0, turned_body.1 + 1.0);
        let v = bounded_command(push, body, rate, DT, 0.1);
        assert!(
            close(v.0, turned_body.0) && close(v.1, turned_body.1 + 0.1),
            "{v:?}"
        );
        // Straight, it is limit_change exactly: a body held at 4 m/s may push to 4.5.
        assert_eq!(
            bounded_command((-8.0, 0.0), (-4.0, 0.0), 0.0, DT, 0.5),
            limit_change((-8.0, 0.0), (-4.0, 0.0), 0.5)
        );
        // Turning never adds speed to a held body.
        let held = bounded_command((0.0, 8.0), (2.0, 0.0), rate, DT, 0.5);
        assert!(held.0.hypot(held.1) <= 2.5 + 1e-9, "{held:?}");
    }

    #[test]
    fn a_yaw_error_is_steered_out() {
        // 2 deg left of SSv2's heading on a straight: the tyres undo a set angular velocity,
        // so the correction is steered, at the same rate as the yaw-rate command.
        let t = cruising();
        let mut c = on_track(&t);
        c.yaw_deg = 178.0;
        let (_, _, w, steer) = track(command(&t, &c, DT, VEHICLE_GAINS, Some(MODEL3), true));
        assert!(close(w, 2.0 * VEHICLE_GAINS.yaw), "{w}");
        let expected = (2.9 * w.to_radians() / 8.0).atan().to_degrees() / 70.0;
        assert!(
            steer.unwrap() > 0.0 && close(steer.unwrap(), expected),
            "{steer:?}"
        );
    }

    #[test]
    fn an_npc_added_moving_starts_at_speed() {
        let bound = max_change(None, DT);
        // UC-ACC Npc1: added at 5.5 m/s, first tracked frame. The command is not bounded,
        // so the body takes it outright instead of 0.49 m/s.
        let change = first_frame_change(true, true, bound);
        assert_eq!(limit_change((5.5, 0.0), (0.0, 0.0), change), (5.5, 0.0));
        // Afterwards the force bound holds again.
        let change = first_frame_change(false, true, bound);
        assert!(close(limit_change((5.5, 0.0), (0.0, 0.0), change).0, bound));
        // Still held by SSv2 (npc_logic_started false): it starts at rest, bounded.
        assert_eq!(first_frame_change(true, false, bound), bound);
    }

    #[test]
    fn a_snapshot_older_than_the_last_tick_is_stale() {
        assert!(snapshot_is_fresh(10, None));
        assert!(snapshot_is_fresh(10, Some(10)));
        assert!(snapshot_is_fresh(11, Some(10)));
        assert!(!snapshot_is_fresh(9, Some(10)));
    }

    #[test]
    fn a_stale_view_is_re_read_until_it_catches_up() {
        let frames = std::cell::Cell::new(7_u64);
        let read = || -> Result<u64, ()> {
            let f = frames.get();
            frames.set(f + 1);
            Ok(f)
        };
        let r = read_fresh(
            Some(9),
            std::time::Duration::from_secs(1),
            std::time::Duration::ZERO,
            read,
            |f| *f,
        )
        .unwrap();
        assert_eq!((r.frame, r.reads, r.fresh), (9, 3, true));
    }

    #[test]
    fn a_view_that_never_catches_up_is_used_after_the_bound() {
        let bound = std::time::Duration::from_millis(5);
        let started = std::time::Instant::now();
        let r = read_fresh(
            Some(9),
            bound,
            std::time::Duration::from_millis(1),
            || Ok::<_, ()>(3_u64),
            |f| *f,
        )
        .unwrap();
        assert!(!r.fresh && r.frame == 3 && r.reads > 1, "{r:?}");
        assert!(started.elapsed() >= bound);
        // A fresh first read never waits, and an error ends the wait at once.
        let r = read_fresh(Some(9), bound, bound, || Ok::<_, ()>(9_u64), |f| *f).unwrap();
        assert_eq!((r.reads, r.fresh), (1, true));
        assert!(read_fresh(Some(9), bound, bound, || Err::<u64, _>("gone"), |f| *f).is_err());
    }

    #[test]
    fn a_missing_npc_is_skipped_and_counted_not_an_error() {
        let mut w = MissWatch::default();
        let mut stats = TrackingStats::default();
        // Two NPCs, one missing, many frames: never an error, warned once, counted each time.
        let mut warnings = 0;
        for _ in 0..(MISS_LIMIT_FRAMES * 3) {
            warnings += u32::from(w.missed("npc_a"));
            stats.record_missed();
            assert!(!w.end_frame(2, 1));
        }
        assert_eq!(warnings, 1);
        let sum = stats.summary();
        assert_eq!((sum.frames, sum.missed), (0, MISS_LIMIT_FRAMES * 3));
        // Every NPC missing, but only for the limit: still a lag.
        for _ in 0..MISS_LIMIT_FRAMES {
            assert!(!w.end_frame(2, 2));
        }
        // A frame with nothing tracked neither fails nor ends the run.
        assert!(!w.end_frame(0, 0));
        // A new episode warns afresh.
        w.reset();
        assert!(w.missed("npc_a"));
    }

    #[test]
    fn every_npc_missing_beyond_the_limit_fails_the_frame() {
        let mut w = MissWatch::default();
        for _ in 0..MISS_LIMIT_FRAMES {
            assert!(!w.end_frame(3, 3));
        }
        assert!(w.end_frame(3, 3));
        assert!(w.end_frame(3, 3));
        // One NPC back ends the run.
        assert!(!w.end_frame(3, 2));
        assert!(!w.end_frame(3, 3));
    }
}
