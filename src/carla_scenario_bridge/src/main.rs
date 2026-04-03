mod coordinator;
mod coordinate_conversion;
mod entity_manager;
mod proto;
mod traffic_light_mapper;
mod zmq_server;

use carla::client::Client;
use color_eyre::eyre::Result;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();

    // Config from environment
    let carla_host =
        std::env::var("CARLA_HOST").unwrap_or_else(|_| "localhost".to_string());
    let carla_port: u16 = std::env::var("CARLA_PORT")
        .unwrap_or_else(|_| "2000".to_string())
        .parse()
        .unwrap_or(2000);
    let ssv2_port: u16 = std::env::var("SSV2_PORT")
        .unwrap_or_else(|_| "5555".to_string())
        .parse()
        .unwrap_or(5555);

    tracing::info!("carla-scenario-bridge starting");
    tracing::info!("  CARLA:  {carla_host}:{carla_port}");
    tracing::info!("  SSv2:   tcp://*:{ssv2_port}");

    let shutdown = Arc::new(AtomicBool::new(false));
    {
        let shutdown = shutdown.clone();
        ctrlc::set_handler(move || {
            tracing::info!("Ctrl-C received, shutting down...");
            shutdown.store(true, Ordering::SeqCst);
        })?;
    }

    // Connect to CARLA with infinite retry
    let client = match connect_to_carla(&carla_host, carla_port, &shutdown) {
        Some(c) => c,
        None => {
            tracing::info!("Shutdown before CARLA connection");
            return Ok(());
        }
    };

    let world = client.world()?;
    tracing::info!("CARLA world acquired");

    // Create coordinator and ZMQ server
    let coord = coordinator::Coordinator::new(world);
    let zmq_ctx = zmq::Context::new();
    let mut server = zmq_server::ZmqServer::new(&zmq_ctx, ssv2_port, coord)?;

    // Run server loop
    server.run(shutdown);

    // Cleanup: restore CARLA async mode
    server.cleanup();

    tracing::info!("Shutdown complete");
    Ok(())
}

fn connect_to_carla(
    host: &str,
    port: u16,
    shutdown: &AtomicBool,
) -> Option<Client> {
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
                match client.world() {
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
