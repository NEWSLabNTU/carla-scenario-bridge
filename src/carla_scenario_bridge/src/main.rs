mod config;
mod coordinate_conversion;
mod coordinator;
mod entity_manager;
mod lanelet_map;
mod map_resolver;
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
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    // Per-map config (signal mappings) and the bridge config live in the same directory.
    let config_dir = std::env::var("CSB_CONFIG_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::path::PathBuf::from("config"));

    // File first, then environment overrides -- launch files set CARLA_HOST and friends per
    // run, and a checked-in config must not override what a launch explicitly asked for.
    let mut config = config::BridgeConfig::load_or_default(&config_dir.join("bridge_config.yaml"))?;
    config.apply_env_overrides();

    let carla_host = config.carla.host.clone();
    let carla_port = config.carla.port;
    let ssv2_port = config.ssv2.port;

    tracing::info!("carla-scenario-bridge starting");
    tracing::info!("  CARLA:  {carla_host}:{carla_port}");
    tracing::info!("  SSv2:   tcp://*:{ssv2_port}");
    tracing::info!("  Config: {}", config_dir.display());
    tracing::info!("  Ego role_name: {}", config.ego.role_name);
    if config.background_avs.is_empty() {
        tracing::info!("  Background AVs: none (single-ego run)");
    } else {
        tracing::info!("  Background AVs: {}", config.background_avs.len());
        for av in &config.background_avs {
            tracing::info!("    - {} (domain {:?})", av.role_name, av.ros_domain_id);
        }
    }

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

    // The coordinator keeps the client so it can rebuild the world after a CARLA outage.
    let coord = coordinator::Coordinator::new(
        client,
        world,
        carla_host.clone(),
        carla_port,
        config_dir,
        config,
    );
    let zmq_ctx = zmq::Context::new();
    let mut server = zmq_server::ZmqServer::new(&zmq_ctx, ssv2_port, coord)?;

    // Run server loop
    server.run(shutdown);

    // Cleanup: restore CARLA async mode
    server.cleanup();

    tracing::info!("Shutdown complete");
    Ok(())
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
