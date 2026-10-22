//! The world protocol's client end (roadmap 046): one REQ socket to a world server, every
//! request stamped with this process's [`ClientId`].
//!
//! Relaxed and correlated, so a request after a timeout is possible and a late reply to the
//! old one is dropped. The bridge's `RemoteBackend` drives a server through it; the
//! `csb_world_check` tool probes and checks one.

use std::time::Duration;

use prost::Message;

use crate::proto::csb_world as wire;

/// The protocol version this build speaks (`proto/csb_world.proto`).
pub const PROTOCOL_VERSION: u32 = 4;

/// Why a round trip failed.
#[derive(Debug)]
pub enum CallError {
    /// No reply within the bound: the server is hung or gone.
    Timeout(Duration),
    /// The server answered with an error (`fatal`: it is ending).
    Refused { error: String, fatal: bool },
    /// ZMQ or decoding failed.
    Io(String),
}

impl std::fmt::Display for CallError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout(t) => write!(f, "no answer within {} s", t.as_secs_f64()),
            Self::Refused { error, .. } => f.write_str(error),
            Self::Io(e) => f.write_str(e),
        }
    }
}

/// A process start time, clock ticks since boot (`/proc/<pid>/stat` field 22), or `None`.
pub fn start_time(pid: u32) -> Option<u64> {
    let stat = std::fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
    // The command name (field 2) may hold spaces and parentheses: split after its last ')'.
    let rest = &stat[stat.rfind(')')? + 2..];
    rest.split_whitespace().nth(19)?.parse().ok()
}

/// This host's name.
pub fn hostname() -> String {
    let mut buf = [0u8; 256];
    // SAFETY: gethostname writes at most `buf.len()` bytes into `buf`.
    let rc = unsafe { libc::gethostname(buf.as_mut_ptr().cast(), buf.len()) };
    if rc != 0 {
        return String::new();
    }
    let end = buf.iter().position(|b| *b == 0).unwrap_or(buf.len());
    String::from_utf8_lossy(&buf[..end]).into_owned()
}

/// This process as the protocol's `ClientId`.
pub fn client_id() -> wire::ClientId {
    let pid = std::process::id();
    wire::ClientId {
        host: hostname(),
        pid,
        start_time: start_time(pid).unwrap_or(0),
    }
}

/// Whether `client` provably no longer runs: on this host, and its pid is free or now
/// belongs to a process started at another time. A client on another host is never provably
/// gone (its lease decides).
pub fn is_gone(client: &wire::ClientId) -> bool {
    if client.host.is_empty() || client.host != hostname() {
        return false;
    }
    match start_time(client.pid) {
        None => true,
        Some(t) => client.start_time != 0 && t != client.start_time,
    }
}

/// `host:pid`, for messages.
pub fn describe(client: &wire::ClientId) -> String {
    if client.pid == 0 {
        "(unnamed client)".into()
    } else {
        format!("{}:{}", client.host, client.pid)
    }
}

pub struct Client {
    ctx: zmq::Context,
    endpoint: String,
    socket: zmq::Socket,
    id: wire::ClientId,
}

impl Client {
    pub fn connect(endpoint: &str) -> eyre::Result<Self> {
        let ctx = zmq::Context::new();
        let socket = Self::socket(&ctx, endpoint)?;
        Ok(Self {
            ctx,
            endpoint: endpoint.to_string(),
            socket,
            id: client_id(),
        })
    }

    /// A client that names itself `id` (the conformance check plays a second bridge).
    pub fn connect_as(endpoint: &str, id: wire::ClientId) -> eyre::Result<Self> {
        let mut c = Self::connect(endpoint)?;
        c.id = id;
        Ok(c)
    }

    fn socket(ctx: &zmq::Context, endpoint: &str) -> eyre::Result<zmq::Socket> {
        let socket = ctx.socket(zmq::REQ)?;
        socket.set_req_relaxed(true)?;
        socket.set_req_correlate(true)?;
        socket.set_linger(0)?;
        socket.connect(endpoint)?;
        Ok(socket)
    }

    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }

    pub fn id(&self) -> &wire::ClientId {
        &self.id
    }

    /// One round trip. An `ok: false` reply is [`CallError::Refused`].
    pub fn call(
        &mut self,
        request: wire::request::Request,
        timeout: Duration,
    ) -> Result<wire::Reply, CallError> {
        let bytes = wire::Request {
            request: Some(request),
            client: Some(self.id.clone()),
        }
        .encode_to_vec();
        self.socket
            .set_rcvtimeo(timeout.as_millis().min(i32::MAX as u128) as i32)
            .map_err(|e| CallError::Io(e.to_string()))?;
        self.socket
            .send(bytes, 0)
            .map_err(|e| CallError::Io(format!("send: {e}")))?;
        let reply = match self.socket.recv_bytes(0) {
            Ok(b) => b,
            Err(zmq::Error::EAGAIN) => return Err(CallError::Timeout(timeout)),
            Err(e) => return Err(CallError::Io(format!("recv: {e}"))),
        };
        let reply = wire::Reply::decode(reply.as_slice())
            .map_err(|e| CallError::Io(format!("undecodable reply: {e}")))?;
        if reply.ok {
            Ok(reply)
        } else {
            Err(CallError::Refused {
                error: reply.error,
                fatal: reply.fatal,
            })
        }
    }

    /// A fresh socket, after a timeout left the old one waiting.
    pub fn reset(&mut self) {
        match Self::socket(&self.ctx, &self.endpoint) {
            Ok(s) => self.socket = s,
            Err(e) => tracing::warn!("World server socket not rebuilt: {e}"),
        }
    }

    /// `Hello` in this build's version: the server's description, or why not. A `probe`
    /// takes nothing (no ownership, no lease).
    pub fn hello(&mut self, probe: bool, timeout: Duration) -> Result<wire::HelloReply, CallError> {
        let reply = self.call(
            wire::request::Request::Hello(wire::Hello {
                protocol_version: PROTOCOL_VERSION,
                probe,
            }),
            timeout,
        )?;
        match reply.reply {
            Some(wire::reply::Reply::Hello(h)) if h.protocol_version == PROTOCOL_VERSION => Ok(h),
            Some(wire::reply::Reply::Hello(h)) => Err(CallError::Refused {
                error: format!(
                    "the world server speaks protocol {}, this bridge {PROTOCOL_VERSION}",
                    h.protocol_version
                ),
                fatal: false,
            }),
            other => Err(CallError::Io(format!("hello answered with {other:?}"))),
        }
    }
}

/// A capability's short name (`SIGNALS`), for messages.
pub fn capability_name(c: wire::Capability) -> &'static str {
    c.as_str_name().trim_start_matches("CAPABILITY_")
}

/// The capabilities a `HelloReply` declares.
pub fn capabilities(hello: &wire::HelloReply) -> std::collections::BTreeSet<i32> {
    hello.capabilities.iter().copied().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn this_process_is_not_gone_and_a_reused_pid_is() {
        let me = client_id();
        assert!(me.start_time > 0);
        assert!(!is_gone(&me));
        let reused = wire::ClientId {
            start_time: me.start_time + 1,
            ..me.clone()
        };
        assert!(is_gone(&reused));
        let free = wire::ClientId {
            pid: (1 << 22) + 7,
            ..me.clone()
        };
        assert!(is_gone(&free));
        let elsewhere = wire::ClientId {
            host: "elsewhere".into(),
            ..free
        };
        assert!(!is_gone(&elsewhere));
    }

    #[test]
    fn capabilities_read_without_their_prefix() {
        assert_eq!(
            capability_name(wire::Capability::TracksTargets),
            "TRACKS_TARGETS"
        );
    }
}
