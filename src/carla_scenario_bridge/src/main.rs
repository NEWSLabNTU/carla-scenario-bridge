mod coordinator;
mod entity_manager;
mod proto;
mod traffic_light_mapper;
mod zmq_server;

use color_eyre::eyre::Result;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();

    let shutdown = Arc::new(AtomicBool::new(false));
    {
        let shutdown = shutdown.clone();
        ctrlc::set_handler(move || {
            tracing::info!("Ctrl-C received, shutting down...");
            shutdown.store(true, Ordering::SeqCst);
        })?;
    }

    // TODO(Phase 1): Parse config (CARLA port, SSv2 port)
    // TODO(Phase 1): Connect to CARLA with retry loop
    // TODO(Phase 1): Start ZMQ server and process requests

    tracing::info!("carla-scenario-bridge starting");
    tracing::info!("Waiting for implementation...");

    while !shutdown.load(Ordering::SeqCst) {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    tracing::info!("Shutdown complete");
    Ok(())
}
