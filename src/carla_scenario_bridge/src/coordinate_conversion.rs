/// Coordinate conversion between ROS (right-handed) and CARLA (left-handed) frames.
///
/// ROS:   X = Forward, Y = Left,  Z = Up, Radians
/// CARLA: X = Forward, Y = Right, Z = Up, Degrees
///
/// Conversion: Y-axis flip on positions, roll/yaw sign flip + degree/radian on rotations.
/// Identical to autoware_carla_bridge's coordinate_conversion.rs.
use std::f64::consts::PI;

use crate::proto::geometry_msgs::{Point, Pose, Quaternion};

/// Convert a ROS position to CARLA position (Y-flip).
pub fn ros_to_carla_position(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    (x, -y, z)
}

/// Convert a CARLA position to ROS position (Y-flip).
pub fn carla_to_ros_position(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    (x, -y, z)
}

/// Convert ROS euler angles (radians) to CARLA euler angles (degrees).
/// Roll and yaw are sign-flipped; pitch is not.
pub fn ros_to_carla_rotation(roll: f64, pitch: f64, yaw: f64) -> (f64, f64, f64) {
    (-roll * 180.0 / PI, pitch * 180.0 / PI, -yaw * 180.0 / PI)
}

/// Convert CARLA euler angles (degrees) to ROS euler angles (radians).
/// Roll and yaw are sign-flipped; pitch is not.
pub fn carla_to_ros_rotation(roll: f64, pitch: f64, yaw: f64) -> (f64, f64, f64) {
    (-roll * PI / 180.0, pitch * PI / 180.0, -yaw * PI / 180.0)
}

/// Convert quaternion to euler angles (roll, pitch, yaw) in radians. ZYX convention.
pub fn quaternion_to_euler(qx: f64, qy: f64, qz: f64, qw: f64) -> (f64, f64, f64) {
    let sinr_cosp = 2.0 * (qw * qx + qy * qz);
    let cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    let roll = sinr_cosp.atan2(cosr_cosp);

    let sinp = 2.0 * (qw * qy - qz * qx);
    let pitch = if sinp.abs() >= 1.0 {
        sinp.signum() * PI / 2.0
    } else {
        sinp.asin()
    };

    let siny_cosp = 2.0 * (qw * qz + qx * qy);
    let cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    let yaw = siny_cosp.atan2(cosy_cosp);

    (roll, pitch, yaw)
}

/// Convert euler angles (roll, pitch, yaw) in radians to quaternion (x, y, z, w). ZYX convention.
pub fn euler_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> (f64, f64, f64, f64) {
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();

    let w = cr * cp * cy + sr * sp * sy;
    let x = sr * cp * cy - cr * sp * sy;
    let y = cr * sp * cy + sr * cp * sy;
    let z = cr * cp * sy - sr * sp * cy;

    (x, y, z, w)
}

/// Where the CARLA actor origin sits relative to the SSv2 entity origin, in the entity's
/// own frame (ROS body axes: x forward, y left). Metres.
///
/// SSv2 poses an entity at the origin its scenario declares, and the scenario states where
/// the bounding box sits relative to that origin as `BoundingBox.center`. Vehicles declare
/// `Center x="1.5"`: the origin is the rear axle, the body centre 1.5 m ahead of it. A CARLA
/// vehicle's origin is its body centre, so the two differ by exactly that centre.
///
/// Only x and y are carried. `center.z` is half the height -- CARLA's origin is not at the
/// box's vertical centre either, and spawn height is resolved against the ground anyway.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct OriginOffset {
    pub x: f64,
    pub y: f64,
}

/// Move an SSv2 entity-origin position to where the CARLA actor origin must go.
///
/// ROS frame, before the Y-flip: `x' = x + cx·cos(yaw) − cy·sin(yaw)`,
/// `y' = y + cx·sin(yaw) + cy·cos(yaw)`. `yaw` is the ROS yaw in radians; z is unchanged.
pub fn entity_origin_to_actor_origin(x: f64, y: f64, yaw: f64, offset: OriginOffset) -> (f64, f64) {
    let (sin, cos) = yaw.sin_cos();
    (
        x + offset.x * cos - offset.y * sin,
        y + offset.x * sin + offset.y * cos,
    )
}

/// The inverse of [`entity_origin_to_actor_origin`]: from a CARLA actor origin (converted
/// to the ROS frame) back to the SSv2 entity origin. Same yaw, offset subtracted.
pub fn actor_origin_to_entity_origin(x: f64, y: f64, yaw: f64, offset: OriginOffset) -> (f64, f64) {
    let (sin, cos) = yaw.sin_cos();
    (
        x - offset.x * cos + offset.y * sin,
        y - offset.x * sin - offset.y * cos,
    )
}

/// Convert a protobuf Pose (ROS frame) to a CARLA Transform (location + rotation).
/// Returns (x, y, z, roll_deg, pitch_deg, yaw_deg) in CARLA frame.
pub fn ros_pose_to_carla(pose: &Pose) -> (f64, f64, f64, f64, f64, f64) {
    let pos = pose.position.as_ref().unwrap();
    let ori = pose.orientation.as_ref().unwrap();

    let (cx, cy, cz) = ros_to_carla_position(pos.x, pos.y, pos.z);
    let (roll, pitch, yaw) = quaternion_to_euler(ori.x, ori.y, ori.z, ori.w);
    let (cr, cp, cyaw) = ros_to_carla_rotation(roll, pitch, yaw);

    (cx, cy, cz, cr, cp, cyaw)
}

/// Convert a CARLA transform to a protobuf Pose (ROS frame).
pub fn carla_to_ros_pose(
    x: f32,
    y: f32,
    z: f32,
    roll_deg: f32,
    pitch_deg: f32,
    yaw_deg: f32,
) -> Pose {
    let (rx, ry, rz) = carla_to_ros_position(x as f64, y as f64, z as f64);
    let (ros_roll, ros_pitch, ros_yaw) =
        carla_to_ros_rotation(roll_deg as f64, pitch_deg as f64, yaw_deg as f64);
    let (qx, qy, qz, qw) = euler_to_quaternion(ros_roll, ros_pitch, ros_yaw);

    Pose {
        position: Some(Point {
            x: rx,
            y: ry,
            z: rz,
        }),
        orientation: Some(Quaternion {
            x: qx,
            y: qy,
            z: qz,
            w: qw,
        }),
    }
}

/// Convert CARLA linear velocity (f32 Vector3D) to ROS frame (Y-flip only).
pub fn carla_to_ros_velocity(vx: f32, vy: f32, vz: f32) -> (f64, f64, f64) {
    (vx as f64, -(vy as f64), vz as f64)
}

/// Convert CARLA angular velocity to ROS frame.
/// Roll and yaw are sign-flipped; pitch is not. Units are rad/s in both frames.
pub fn carla_to_ros_angular_velocity(wx: f32, wy: f32, wz: f32) -> (f64, f64, f64) {
    (-(wx as f64), wy as f64, -(wz as f64))
}

/// Convert CARLA linear acceleration (f32 Vector3D) to ROS frame (Y-flip only).
pub fn carla_to_ros_acceleration(ax: f32, ay: f32, az: f32) -> (f64, f64, f64) {
    (ax as f64, -(ay as f64), az as f64)
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;

    #[test]
    fn round_trip_position() {
        let (cx, cy, cz) = ros_to_carla_position(10.0, 5.0, 0.0);
        assert_eq!(cx, 10.0);
        assert_eq!(cy, -5.0);
        assert_eq!(cz, 0.0);
        let (rx, ry, rz) = carla_to_ros_position(cx, cy, cz);
        assert!((rx - 10.0).abs() < 1e-10);
        assert!((ry - 5.0).abs() < 1e-10);
        assert!((rz - 0.0).abs() < 1e-10);
    }

    #[test]
    fn round_trip_rotation() {
        let (cr, cp, cy) = ros_to_carla_rotation(0.1, 0.2, 0.3);
        let (rr, rp, ry) = carla_to_ros_rotation(cr, cp, cy);
        assert!((rr - 0.1).abs() < 1e-10);
        assert!((rp - 0.2).abs() < 1e-10);
        assert!((ry - 0.3).abs() < 1e-10);
    }

    #[test]
    fn round_trip_quaternion_euler() {
        let (qx, qy, qz, qw) = euler_to_quaternion(0.1, 0.2, 0.3);
        let (r, p, y) = quaternion_to_euler(qx, qy, qz, qw);
        assert!((r - 0.1).abs() < 1e-10);
        assert!((p - 0.2).abs() < 1e-10);
        assert!((y - 0.3).abs() < 1e-10);
    }

    const EPS: f64 = 1e-9;

    fn assert_close(actual: (f64, f64), expected: (f64, f64)) {
        assert!(
            (actual.0 - expected.0).abs() < EPS && (actual.1 - expected.1).abs() < EPS,
            "expected {expected:?}, got {actual:?}"
        );
    }

    const REAR_AXLE: OriginOffset = OriginOffset { x: 1.5, y: 0.0 };
    const SKEWED: OriginOffset = OriginOffset { x: 1.5, y: 0.3 };

    #[test]
    fn the_actor_origin_sits_ahead_of_the_rear_axle_along_the_heading() {
        // yaw 0: ahead is +x.
        assert_close(
            entity_origin_to_actor_origin(10.0, 5.0, 0.0, REAR_AXLE),
            (11.5, 5.0),
        );
        // yaw 90 deg: ahead is +y (ROS: left).
        assert_close(
            entity_origin_to_actor_origin(10.0, 5.0, FRAC_PI_2, REAR_AXLE),
            (10.0, 6.5),
        );
        // yaw 180 deg: ahead is -x.
        assert_close(
            entity_origin_to_actor_origin(10.0, 5.0, PI, REAR_AXLE),
            (8.5, 5.0),
        );
        // yaw -90 deg: ahead is -y.
        assert_close(
            entity_origin_to_actor_origin(10.0, 5.0, -FRAC_PI_2, REAR_AXLE),
            (10.0, 3.5),
        );
    }

    #[test]
    fn a_lateral_offset_rotates_with_the_heading() {
        // center.y is to the entity's left (ROS body frame).
        assert_close(
            entity_origin_to_actor_origin(10.0, 5.0, 0.0, SKEWED),
            (11.5, 5.3),
        );
        assert_close(
            entity_origin_to_actor_origin(10.0, 5.0, FRAC_PI_2, SKEWED),
            (9.7, 6.5),
        );
        assert_close(
            entity_origin_to_actor_origin(10.0, 5.0, PI, SKEWED),
            (8.5, 4.7),
        );
        assert_close(
            entity_origin_to_actor_origin(10.0, 5.0, -FRAC_PI_2, SKEWED),
            (10.3, 3.5),
        );
    }

    #[test]
    fn the_readback_undoes_the_placement() {
        for yaw in [0.0, FRAC_PI_2, PI, -FRAC_PI_2, 0.7, -2.3] {
            for offset in [REAR_AXLE, SKEWED] {
                let (ax, ay) = entity_origin_to_actor_origin(320.0, -129.8, yaw, offset);
                assert_close(
                    actor_origin_to_entity_origin(ax, ay, yaw, offset),
                    (320.0, -129.8),
                );
            }
        }
    }

    #[test]
    fn the_readback_at_yaw_180_moves_back_along_the_heading() {
        // The probe's ego: actor at x=318.5 heading -x, rear axle 1.5 m behind it.
        assert_close(
            actor_origin_to_entity_origin(318.5, -129.8, PI, REAR_AXLE),
            (320.0, -129.8),
        );
    }

    #[test]
    fn a_zero_offset_is_the_identity() {
        let zero = OriginOffset::default();
        for yaw in [0.0, FRAC_PI_2, PI, -FRAC_PI_2, 1.1] {
            assert_close(
                entity_origin_to_actor_origin(3.0, -4.0, yaw, zero),
                (3.0, -4.0),
            );
            assert_close(
                actor_origin_to_entity_origin(3.0, -4.0, yaw, zero),
                (3.0, -4.0),
            );
        }
    }
}
