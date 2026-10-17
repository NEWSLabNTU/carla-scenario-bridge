mod actor_census;
mod agent_link;
mod autopilot;
mod backend;
mod blueprint_boxes;
mod body_clearance;
mod budgets;
mod clock_store;
mod config;
mod contact_report;
mod coordinate_conversion;
mod coordinator;
mod entity_manager;
mod episode_clock;
mod epoch_marker;
mod frame_stats;
mod gap_trace;
mod lanelet_map;
mod map_resolver;
mod npc_tracking;
mod proto;
mod ros_args;
mod sensor_release;
mod spawn_ladder;
mod tick_watchdog;
mod weather;
mod world_services;
mod zmq_server;

use color_eyre::eyre::Result;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use backend::SimBackend;

/// Exit status after a hung tick (`EX_TEMPFAIL`): not a clean exit, so the supervisor
/// restarts the bridge.
const TICK_HUNG_EXIT: i32 = 75;

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    // ROS parameters from `ros2 run ... --ros-args -p` or a launch file's params file.
    let params = ros_args::RosParams::from_args(std::env::args().skip(1))?;

    // Which config file: the `config_file` parameter, then CSB_CONFIG_DIR, then the installed
    // package's, then ./config (a source checkout). Per-map files (traffic_lights_<town>.yaml)
    // are looked up beside it.
    let config_file = params
        .path("config_file")
        .or_else(|| {
            std::env::var("CSB_CONFIG_DIR")
                .ok()
                .map(|d| std::path::PathBuf::from(d).join("bridge_config.yaml"))
        })
        .or_else(ros_args::installed_config)
        .unwrap_or_else(|| std::path::PathBuf::from("config/bridge_config.yaml"));
    let config_dir = config_file
        .parent()
        .map(std::path::Path::to_path_buf)
        .unwrap_or_default();

    // File, then environment, then ROS parameters: each later source is a more specific
    // statement of what this run wants.
    let mut config = config::BridgeConfig::load_or_default(&config_file)?;
    config.apply_env_overrides();
    config.apply_ros_params(&params)?;

    let carla_host = config.carla.host.clone();
    let carla_port = config.carla.port;
    let ssv2_port = config.ssv2.port;

    tracing::info!("carla-scenario-bridge starting");
    tracing::info!("  Backend: {}", config.backend.name());
    tracing::info!("  CARLA:  {carla_host}:{carla_port}");
    tracing::info!("  SSv2:   tcp://*:{ssv2_port}");
    tracing::info!("  Config: {}", config_file.display());
    tracing::info!("  Ego role_name: {}", config.ego.role_name);
    tracing::info!(
        "  Agent relay (controller `agent`): {}",
        config.agent_relay()
    );
    tracing::info!(
        "  Weather: {}",
        if config.weather.is_empty() {
            "CARLA's (not set)"
        } else {
            config.weather.as_str()
        }
    );

    let shutdown = Arc::new(AtomicBool::new(false));
    {
        let shutdown = shutdown.clone();
        ctrlc::set_handler(move || {
            tracing::info!("Ctrl-C received, shutting down...");
            shutdown.store(true, Ordering::SeqCst);
        })?;
    }

    let mut backend: Box<dyn SimBackend> = match config.backend {
        config::Backend::Carla => {
            // Connect to CARLA with infinite retry.
            let generate = generate_signal_table_arg();
            let Some(mut carla) = backend::carla::connect(config, config_dir, &shutdown)? else {
                tracing::info!("Shutdown before CARLA connection");
                return Ok(());
            };
            // Generator mode: write a map dir's CARLA traffic light table, then leave CARLA
            // as found.
            if let Some(map_dir) = generate {
                let result = carla.generate_signal_table(&map_dir);
                carla.shutdown();
                return result;
            }
            Box::new(carla)
        }
        config::Backend::Isaac => {
            if generate_signal_table_arg().is_some() {
                eyre::bail!("--generate-signal-table is CARLA's: run it with backend carla");
            }
            tracing::info!("  Isaac world server: {}", config.isaac.endpoint);
            let Some(isaac) = backend::isaac::connect(config, &shutdown)? else {
                tracing::info!("Shutdown before the Isaac world server answered");
                return Ok(());
            };
            Box::new(isaac)
        }
    };
    backend.before_serving();
    let coord = coordinator::Coordinator::new(backend);

    // World configuration services. Served from their own thread, applied on this one.
    // Without them the bridge still serves SSv2, so a ROS failure is loud but not fatal.
    let world_commands = match world_services::start(shutdown.clone()) {
        Ok(rx) => Some(rx),
        Err(e) => {
            tracing::error!("{e:#}; /carla/* services are unavailable");
            None
        }
    };

    let zmq_ctx = zmq::Context::new();
    let mut server = zmq_server::ZmqServer::new(&zmq_ctx, ssv2_port, coord, world_commands)?;

    // Run server loop
    server.run(shutdown);

    // A tick that never returned (roadmap 032 N5): SSv2 has its error; end the process without
    // touching CARLA again -- the tick thread may still be inside LibCarla -- so the
    // supervisor (play_launch respawn, `just run`'s loop) starts a fresh bridge. Its first
    // Initialize reaps what this one left.
    if let Some(reason) = server.wedged() {
        // The socket first, so the replies the drain queued leave: the context terminates
        // with its last handle and waits out the socket's linger.
        server.close_flushed();
        drop(zmq_ctx);
        server.abandon_after_hang();
        tracing::error!("Exiting with status {TICK_HUNG_EXIT}: {reason}");
        use std::io::Write;
        let _ = std::io::stdout().flush();
        let _ = std::io::stderr().flush();
        // `_exit`, not `exit`: no atexit handlers or C++ static destructors, which could
        // wait on the Traffic Manager mutex the hung thread holds (034).
        // SAFETY: ends the process at once; nothing after it runs.
        unsafe { libc::_exit(TICK_HUNG_EXIT) };
    }

    // The idle check found the CARLA connection dead: the server went away or was replaced.
    // This client must not live on -- its stream subscription would reattach to the next
    // server and stall it (Coordinator::probe_connection_idle) -- and there is nothing to
    // restore on a server that is gone. Exit for a fresh bridge, as after a hung tick.
    if let Some(reason) = server.server_lost() {
        server.close_flushed();
        drop(zmq_ctx);
        tracing::error!("Exiting with status {TICK_HUNG_EXIT}: {reason}");
        use std::io::Write;
        let _ = std::io::stdout().flush();
        let _ = std::io::stderr().flush();
        // SAFETY: ends the process at once; nothing after it runs. `_exit` skips the C++
        // destructors, which would make RPCs on the dead connection.
        unsafe { libc::_exit(TICK_HUNG_EXIT) };
    }

    // Cleanup: restore CARLA async mode
    server.cleanup();

    tracing::info!("Shutdown complete");
    Ok(())
}

/// `--generate-signal-table <map dir>` (before any `--ros-args`).
fn generate_signal_table_arg() -> Option<std::path::PathBuf> {
    let mut args = std::env::args().skip(1).take_while(|a| a != "--ros-args");
    while let Some(a) = args.next() {
        if a == "--generate-signal-table" {
            return args.next().map(std::path::PathBuf::from);
        }
    }
    None
}
