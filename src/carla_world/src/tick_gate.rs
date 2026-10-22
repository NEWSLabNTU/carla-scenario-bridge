//! The tick gate: csb waits, before each `world.tick()`, until a follower that asked for it
//! has finished the frame csb ticked last.
//!
//! csb is the only process that ticks CARLA (docs/design/time-and-ticking.md). A process
//! that follows csb's frames instead of ticking (2026-vehicle-consensus' junction harness)
//! acts a frame late, or skips frames, whenever one of its rounds takes longer than csb's
//! frame. With the gate the follower still never ticks, but csb holds the next tick until the
//! follower says it is done with the current frame, so the follower sees every frame.
//!
//! Protocol, UDP datagrams to the gate's address (`CSB_TICK_GATE`, or `tick_gate` in
//! `bridge_config.yaml`):
//!
//! - `HELLO` -- a follower joins: from now on every tick waits for it;
//! - `DONE <frame>` -- the follower finished CARLA frame `<frame>`;
//! - `BYE` -- the follower leaves: ticks no longer wait.
//!
//! The wait is bounded (`tick_gate_timeout_ms`): a follower that does not answer in time does
//! not stop the simulation, the tick goes ahead and the miss is counted. A follower silent
//! for [`SILENT_LEAVES`] is taken as gone, so a crashed follower costs at most a few
//! bounded waits. The wait runs in csb's request path before the tick, never inside a tick
//! or a sensor callback.

use std::net::UdpSocket;
use std::time::{Duration, Instant};

/// A follower silent this long has left (its process died without a `BYE`).
pub const SILENT_LEAVES: Duration = Duration::from_secs(5);

/// One datagram of the protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateMsg {
    Hello,
    Done(u64),
    Bye,
}

impl GateMsg {
    pub fn parse(text: &str) -> Option<Self> {
        let mut f = text.split_whitespace();
        match f.next()? {
            "HELLO" => Some(Self::Hello),
            "BYE" => Some(Self::Bye),
            "DONE" => f.next()?.parse().ok().map(Self::Done),
            _ => None,
        }
    }
}

/// Whether a follower is attached and the last frame it finished.
#[derive(Debug, Clone, Default)]
pub struct GateState {
    pub active: bool,
    pub done: Option<u64>,
    pub last_heard: Option<Instant>,
}

impl GateState {
    pub fn apply(&mut self, msg: GateMsg, now: Instant) {
        self.last_heard = Some(now);
        match msg {
            GateMsg::Hello => {
                self.active = true;
                self.done = None;
            }
            GateMsg::Done(f) => {
                self.active = true;
                self.done = Some(self.done.map_or(f, |d| d.max(f)));
            }
            GateMsg::Bye => {
                self.active = false;
                self.done = None;
            }
        }
    }

    /// Whether the tick after `frame` must still wait.
    pub fn holds(&self, frame: u64, now: Instant) -> bool {
        if !self.active {
            return false;
        }
        if self
            .last_heard
            .is_some_and(|t| now.duration_since(t) > SILENT_LEAVES)
        {
            return false;
        }
        self.done.is_none_or(|d| d < frame)
    }
}

/// The gate's socket and state.
pub struct TickGate {
    sock: UdpSocket,
    state: GateState,
    timeout: Duration,
    /// Ticks that went ahead because the follower did not finish in time.
    pub missed: u64,
    /// Ticks that waited, and how long in all.
    pub waits: u64,
    pub waited: Duration,
}

impl TickGate {
    /// Bind the gate's address (`host:port`).
    pub fn bind(addr: &str, timeout: Duration) -> std::io::Result<Self> {
        let sock = UdpSocket::bind(addr)?;
        sock.set_nonblocking(true)?;
        Ok(Self {
            sock,
            state: GateState::default(),
            timeout,
            missed: 0,
            waits: 0,
            waited: Duration::ZERO,
        })
    }

    fn drain(&mut self) {
        let mut buf = [0u8; 256];
        while let Ok(n) = self.sock.recv(&mut buf) {
            if let Some(m) = GateMsg::parse(&String::from_utf8_lossy(&buf[..n])) {
                if m == GateMsg::Hello && !self.state.active {
                    tracing::info!("tick gate: a follower joined; every tick waits for it");
                }
                if m == GateMsg::Bye && self.state.active {
                    tracing::info!(
                        "tick gate: the follower left ({} waits, {:.1} s waited, {} missed)",
                        self.waits,
                        self.waited.as_secs_f64(),
                        self.missed
                    );
                }
                self.state.apply(m, Instant::now());
            }
        }
    }

    /// Before the tick that follows `frame` (the frame csb ticked last): wait until the
    /// follower finished it, at most the gate's timeout.
    pub fn wait_for(&mut self, frame: u64) {
        self.drain();
        let start = Instant::now();
        if !self.state.holds(frame, start) {
            return;
        }
        self.waits += 1;
        // Blocking reads with a short timeout, so a datagram ends the wait at once.
        let _ = self.sock.set_nonblocking(false);
        let _ = self.sock.set_read_timeout(Some(Duration::from_millis(2)));
        while self.state.holds(frame, Instant::now()) {
            if start.elapsed() >= self.timeout {
                self.missed += 1;
                if self.missed.is_power_of_two() {
                    tracing::warn!(
                        "tick gate: the follower did not finish frame {frame} within {} ms; \
                         ticking on ({} miss(es) so far)",
                        self.timeout.as_millis(),
                        self.missed
                    );
                }
                break;
            }
            let mut buf = [0u8; 256];
            if let Ok(n) = self.sock.recv(&mut buf) {
                if let Some(m) = GateMsg::parse(&String::from_utf8_lossy(&buf[..n])) {
                    self.state.apply(m, Instant::now());
                }
            }
        }
        let _ = self.sock.set_nonblocking(true);
        self.waited += start.elapsed();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn messages_parse() {
        assert_eq!(GateMsg::parse("HELLO"), Some(GateMsg::Hello));
        assert_eq!(GateMsg::parse("DONE 42"), Some(GateMsg::Done(42)));
        assert_eq!(GateMsg::parse("BYE\n"), Some(GateMsg::Bye));
        assert_eq!(GateMsg::parse("DONE x"), None);
        assert_eq!(GateMsg::parse("NOPE"), None);
    }

    #[test]
    fn a_tick_waits_only_for_an_attached_follower_behind_the_frame() {
        let t0 = Instant::now();
        let mut s = GateState::default();
        assert!(!s.holds(10, t0), "no follower: no wait");
        s.apply(GateMsg::Hello, t0);
        assert!(s.holds(10, t0), "joined, nothing finished yet");
        s.apply(GateMsg::Done(9), t0);
        assert!(s.holds(10, t0));
        s.apply(GateMsg::Done(10), t0);
        assert!(!s.holds(10, t0));
        s.apply(GateMsg::Done(8), t0);
        assert_eq!(
            s.done,
            Some(10),
            "a late datagram never moves the frame back"
        );
        s.apply(GateMsg::Bye, t0);
        assert!(!s.holds(11, t0));
    }

    #[test]
    fn a_silent_follower_is_taken_as_gone() {
        let t0 = Instant::now();
        let mut s = GateState::default();
        s.apply(GateMsg::Done(3), t0);
        assert!(s.holds(4, t0));
        assert!(!s.holds(4, t0 + SILENT_LEAVES + Duration::from_millis(1)));
    }

    #[test]
    fn the_gate_waits_until_the_frame_is_done_and_not_longer() {
        let mut g = TickGate::bind("127.0.0.1:0", Duration::from_millis(500)).unwrap();
        let addr = g.sock.local_addr().unwrap();
        let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        peer.send_to(b"HELLO", addr).unwrap();
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            peer.send_to(b"DONE 7", addr).unwrap();
        });
        let start = Instant::now();
        g.wait_for(7);
        let took = start.elapsed();
        t.join().unwrap();
        assert!(
            took >= Duration::from_millis(40) && took < Duration::from_millis(400),
            "{took:?}"
        );
        assert_eq!(g.missed, 0);
        // Nothing for frame 8: the wait ends at the timeout and is counted.
        g.wait_for(8);
        assert_eq!(g.missed, 1);
    }
}
