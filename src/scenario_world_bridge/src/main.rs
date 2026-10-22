//! The scenario bridge (roadmaps 017, 035, 046): SSv2's `simulation_interface` on one side,
//! one world server over the world protocol on the other. Which simulator that is, the
//! bridge does not know: whichever world server answers at `world.endpoint` (CARLA's,
//! Isaac Sim's) declares what it offers in its `Hello`.

mod backend;
mod coordinator;
mod frame_stats;
mod world_services;
mod zmq_server;

// The simulator-free core, under the paths the bridge's modules name.
pub(crate) use swb_core::{
    agent_link, autopilot, body_clearance, budgets, config, contact_report, coordinate_conversion,
    entity_manager, gap_trace, map_resolver, npc_tracking, proto, ros_args, weather,
};

use color_eyre::eyre::Result;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use backend::SimBackend;

/// Exit status after a hung tick (`EX_TEMPFAIL`): not a clean exit, so the supervisor
/// restarts the bridge.
const TICK_HUNG_EXIT: i32 = 75;

fn main() -> Result<()> {
    color_eyre::install()?;
    // Ends with its supervisor: a play_launch killed with SIGKILL takes the bridge along.
    swb_core::world_server::die_with_parent();
    // SAFETY: getppid has no preconditions.
    let supervisor = unsafe { libc::getppid() };
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    // ROS parameters from `ros2 run ... --ros-args -p` or a launch file's params file.
    let params = ros_args::RosParams::from_args(std::env::args().skip(1))?;

    // Which config file: the `config_file` parameter, then the installed package's, then
    // ./config (a source checkout). The node declares the one chosen as its read-only `config_file` parameter.
    let config_file = ros_args::config_file(&params)?;

    // File, then environment, then ROS parameters: each later source is a more specific
    // statement of what this run wants.
    let mut config = config::BridgeConfig::load_for(&config_file, config::ConfigOwner::Bridge)?;
    config.apply_env_overrides();
    config.apply_ros_params(&params)?;

    let ssv2_port = config.ssv2.port;

    tracing::info!("scenario bridge starting");
    tracing::info!("  World server: {}", config.world.endpoint);
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
            "the simulator's (not set)"
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

    // The world server first: the SSv2 port is bound only once a world server has taken
    // this bridge as its owner, so a scenario never reaches a bridge without a world.
    let Some(remote) = backend::remote::connect(config, &shutdown)? else {
        tracing::info!("Shutdown before a world server answered");
        return Ok(());
    };
    let mut backend: Box<dyn SimBackend> = Box::new(remote);
    backend.before_serving();
    let coord = coordinator::Coordinator::new(backend);

    // World configuration services. Served from their own thread, applied on this one.
    // Without them the bridge still serves SSv2, so a ROS failure is loud but not fatal.
    let world_commands = match world_services::start(shutdown.clone(), &config_file) {
        Ok(rx) => Some(rx),
        Err(e) => {
            tracing::error!("{e:#}; weather services are unavailable");
            None
        }
    };

    let zmq_ctx = zmq::Context::new();
    let mut server = zmq_server::ZmqServer::new(&zmq_ctx, ssv2_port, coord, world_commands)?;

    // Run server loop
    server.run(shutdown);

    // A world request that never returned, or a world server that cannot go on (roadmap 032
    // N5, 034): SSv2 has its error; end the process without another word to the world server,
    // so the supervisor (play_launch respawn) starts a fresh bridge, which takes the world over
    // from this one (the owner lease).
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
        // `_exit`, not `exit`: nothing more runs (034).
        // SAFETY: ends the process at once; nothing after it runs.
        unsafe { libc::_exit(TICK_HUNG_EXIT) };
    }

    // The idle check found the world server gone for good: exit for a fresh bridge, which
    // waits for one to answer, as after a hung request.
    if let Some(reason) = server.server_lost() {
        server.close_flushed();
        drop(zmq_ctx);
        tracing::error!("Exiting with status {TICK_HUNG_EXIT}: {reason}");
        use std::io::Write;
        let _ = std::io::stdout().flush();
        let _ = std::io::stderr().flush();
        // SAFETY: ends the process at once; nothing after it runs.
        unsafe { libc::_exit(TICK_HUNG_EXIT) };
    }

    // The world server outlives the bridge: release it -- unless the supervisor is gone, which
    // takes the world server down too (or the owner lease frees it): nothing to wait for then.
    // SAFETY: getppid has no preconditions.
    if unsafe { libc::getppid() } == supervisor {
        server.cleanup();
    } else {
        tracing::info!("The supervisor is gone: ending without releasing the world server");
    }

    tracing::info!("Shutdown complete");
    Ok(())
}

#[cfg(test)]
mod tests {
    /// The bridge knows no simulator (roadmap 046): nothing in its dependency graph is a
    /// simulator's client library. Read from the workspace's Cargo.lock, which lists every
    /// package's dependencies.
    #[test]
    fn no_simulator_crate_is_in_the_bridges_dependency_graph() {
        let lock =
            std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../Cargo.lock"))
                .expect("read Cargo.lock");
        let mut deps: std::collections::HashMap<String, Vec<String>> = Default::default();
        for block in lock.split("[[package]]").skip(1) {
            let name = block
                .lines()
                .find_map(|l| l.strip_prefix("name = \""))
                .map(|n| n.trim_end_matches('"').to_string())
                .expect("a package name");
            let list = block
                .split("dependencies = [")
                .nth(1)
                .map(|d| d.split(']').next().unwrap_or_default())
                .unwrap_or_default()
                .split(',')
                .filter_map(|d| d.trim().trim_matches('"').split(' ').next())
                .filter(|d| !d.is_empty())
                .map(str::to_string)
                .collect();
            deps.entry(name).or_insert(list);
        }
        let mut seen = std::collections::BTreeSet::new();
        let mut todo = vec!["scenario_world_bridge".to_string()];
        while let Some(p) = todo.pop() {
            if seen.insert(p.clone()) {
                todo.extend(deps.get(&p).cloned().unwrap_or_default());
            }
        }
        assert!(seen.len() > 20, "found only {seen:?}");
        let simulators: Vec<&String> = seen
            .iter()
            .filter(|p| p.starts_with("carla") && p.as_str() != "scenario_world_bridge")
            .collect();
        assert!(
            simulators.is_empty(),
            "the bridge depends on a simulator: {simulators:?}"
        );
    }
}
