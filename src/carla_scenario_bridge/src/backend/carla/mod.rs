//! CARLA as csb's simulator: [`CarlaBackend`], through carla-rust (roadmap 035).
//!
//! Everything that knows CARLA lives under this module -- its left-handed frame, actor ids,
//! blueprints, towns, synchronous mode and the Traffic Manager, the lights and their table
//! for acb, the collision sensors, and the gap trace's CARLA half from its snapshots. The rest of csb
//! sees it through [`crate::backend::SimBackend`].

mod backend;
pub mod collision_monitor;
pub mod resolved_signals;
pub mod traffic_light_mapper;
pub mod version;
mod weather;

use carla::client::Client;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

pub use backend::CarlaBackend;

use crate::config::BridgeConfig;

/// Connect to the configured CARLA, retrying every 5 s until it answers with this build's
/// version and a loaded map, and build the backend on it. `None` when `shutdown` was set
/// first.
pub fn connect(
    config: BridgeConfig,
    config_dir: PathBuf,
    shutdown: &AtomicBool,
) -> eyre::Result<Option<CarlaBackend>> {
    let host = config.carla.host.clone();
    let port = config.carla.port;
    let Some(client) = connect_to_carla(&host, port, shutdown) else {
        return Ok(None);
    };
    let world = client.world()?;
    tracing::info!("CARLA world acquired");
    // The backend keeps the client so it can rebuild the world after a CARLA outage.
    Ok(Some(CarlaBackend::new(
        client, world, host, port, config_dir, config,
    )))
}

fn connect_to_carla(host: &str, port: u16, shutdown: &AtomicBool) -> Option<Client> {
    loop {
        if shutdown.load(Ordering::SeqCst) {
            return None;
        }

        match Client::connect(host, port, None) {
            Ok(mut client) => {
                if let Err(e) = client.set_timeout(Duration::from_secs(30)) {
                    tracing::warn!("Failed to set timeout: {e}, retrying in 5s...");
                    std::thread::sleep(Duration::from_secs(5));
                    continue;
                }
                // A server of another CARLA release is refused like an unreachable one.
                match version::verify(&client) {
                    Ok(server) => tracing::info!(
                        "CARLA server version {server} matches this build ({})",
                        version::BUILT_FOR
                    ),
                    Err(e) => {
                        tracing::error!("Refusing CARLA at {host}:{port}: {e}; retrying in 5s...");
                        std::thread::sleep(Duration::from_secs(5));
                        continue;
                    }
                }
                match client.world() {
                    // A starting CARLA serves RPC before its first map is loaded (empty map
                    // name); anything sent to it then can wedge it. Wait for the map.
                    Ok(world) if world.map().map(|m| m.name()).unwrap_or_default().is_empty() => {
                        tracing::warn!("CARLA is still loading its first map, retrying in 5s...");
                    }
                    Ok(_) => {
                        tracing::info!("Connected to CARLA at {host}:{port}");
                        return Some(client);
                    }
                    Err(e) => {
                        tracing::warn!("CARLA not ready: {e}, retrying in 5s...");
                    }
                }
            }
            Err(e) => {
                tracing::warn!("CARLA connection failed: {e}, retrying in 5s...");
            }
        }

        std::thread::sleep(Duration::from_secs(5));
    }
}
