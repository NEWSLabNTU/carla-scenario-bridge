//! Tell the sensor bridges to release their sensors before we despawn a vehicle.
//!
//! # Why
//!
//! This bridge owns the lifetime of every entity a scenario spawns, and despawns them
//! with `destroy_with_children` -- which is required, because CARLA leaves a destroyed
//! vehicle's sensors alive and an orphaned IMU segfaults the server on its next tick.
//!
//! But those sensors belong to somebody else. `acb_bridge` attaches them and calls
//! `Listen()`, so it owns their stream sessions, and destroying a sensor that is still
//! listening leaves that client retrying a dead stream forever. CARLA 0.9.16 answers those
//! retries at ~48,000 `Invalid session: no stream available with id N` a second -- 2.6 MB/s
//! of log -- which starves the simulation and eventually segfaults the server.
//!
//! Neither side can fix this alone. The listener cannot unsubscribe after the fact
//! (`Sensor::stop()` on a destroyed actor fails), and we cannot stop a stream we never
//! subscribed to (`ServerSideSensor::Stop()` returns early unless the *calling client*
//! is the listener). So we say so first, and give them a moment.
//!
//! See `src/autoware_carla_bridge/docs/issues/015-*`.
//!
//! # Protocol
//!
//! | direction | socket | frame |
//! |---|---|---|
//! | here -> bridges | PUB / SUB | `release <actor_id> <role_name>` |
//! | bridges -> here | PULL / PUSH | `released <actor_id>` |
//!
//! Best effort, and deliberately so: a scenario must not stall because a sensor bridge is
//! absent or slow. We wait for one ack, or for a short timeout, and then despawn either
//! way. Missing the ack costs a burst of server log; refusing to despawn would cost the
//! run.

use std::time::{Duration, Instant};

use color_eyre::eyre::{Result, WrapErr};

/// A vehicle with no attached sensors still gets a notice -- we cannot know what is
/// attached without asking the server, and the notice is cheaper than the query.
pub struct SensorReleaseNotifier {
    publisher: zmq::Socket,
    acks: zmq::Socket,
    timeout: Duration,
}

impl SensorReleaseNotifier {
    /// Bind the release channel.
    ///
    /// `notify_endpoint` is ours to PUB on; `ack_endpoint` is ours to PULL on.
    pub fn bind(notify_endpoint: &str, ack_endpoint: &str, timeout: Duration) -> Result<Self> {
        let ctx = zmq::Context::new();

        let publisher = ctx.socket(zmq::PUB).wrap_err("create release PUB socket")?;
        publisher
            .bind(notify_endpoint)
            .wrap_err_with(|| format!("bind release PUB socket to {notify_endpoint}"))?;

        let acks = ctx
            .socket(zmq::PULL)
            .wrap_err("create release PULL socket")?;
        acks.bind(ack_endpoint)
            .wrap_err_with(|| format!("bind release PULL socket to {ack_endpoint}"))?;

        tracing::info!(
            "Sensor release channel: notifying on {notify_endpoint}, acks on {ack_endpoint} \
             (timeout {} ms)",
            timeout.as_millis()
        );

        Ok(Self {
            publisher,
            acks,
            timeout,
        })
    }

    /// Ask whoever is listening to release their sensors on `actor_id`, and wait briefly.
    ///
    /// Returns `true` if a bridge acknowledged. `false` means nobody answered in time,
    /// which is the normal case for an entity with no sensor bridge attached to it.
    pub fn release(&self, actor_id: u32, role_name: &str) -> bool {
        let name = if role_name.is_empty() { "-" } else { role_name };
        if let Err(e) = self
            .publisher
            .send(format!("release {actor_id} {name}").as_str(), 0)
        {
            tracing::debug!("Sensor release channel: could not send notice: {e}");
            return false;
        }

        // Drain acks until one matches, or the deadline passes. Acks for other actors can
        // be in the queue when several entities are torn down together.
        let expected = format!("released {actor_id}");
        let deadline = Instant::now() + self.timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                tracing::debug!(
                    "Sensor release channel: no acknowledgement for actor {actor_id} \
                     within {} ms; despawning anyway",
                    self.timeout.as_millis()
                );
                return false;
            }
            let _ = self.acks.set_rcvtimeo(remaining.as_millis() as i32);
            match self.acks.recv_string(0) {
                Ok(Ok(frame)) if frame.trim() == expected => {
                    tracing::debug!("Sensor release channel: actor {actor_id} released");
                    return true;
                }
                // Someone else's ack, or a malformed frame: keep waiting.
                Ok(_) => continue,
                Err(_) => continue,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Binding twice on the same endpoint must fail rather than silently share it -- two
    /// notifiers would each wait for acks the other consumed.
    #[test]
    fn a_second_bind_on_the_same_endpoint_fails() {
        let notify = "tcp://127.0.0.1:5586";
        let ack = "tcp://127.0.0.1:5587";
        let first = SensorReleaseNotifier::bind(notify, ack, Duration::from_millis(10));
        assert!(first.is_ok());
        let second = SensorReleaseNotifier::bind(notify, ack, Duration::from_millis(10));
        assert!(second.is_err());
    }

    /// With nobody listening the notice still goes out and the wait ends at the timeout,
    /// so a scenario without any sensor bridge is delayed by the timeout and no more.
    #[test]
    fn release_without_a_listener_times_out_and_returns_false() {
        let notifier = SensorReleaseNotifier::bind(
            "tcp://127.0.0.1:5588",
            "tcp://127.0.0.1:5589",
            Duration::from_millis(50),
        )
        .unwrap();
        let start = Instant::now();
        assert!(!notifier.release(1234, "hero"));
        let waited = start.elapsed();
        assert!(
            waited >= Duration::from_millis(40),
            "returned too early: {waited:?}"
        );
        assert!(
            waited < Duration::from_millis(500),
            "waited too long: {waited:?}"
        );
    }
}
