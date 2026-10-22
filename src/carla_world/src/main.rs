//! The CARLA world server (roadmap 046): CARLA behind the world protocol, for the scenario
//! bridge.
//!
//!     carla_world [--generate-signal-table <map dir>] --ros-args -p port:=5591 \
//!         -p carla_host:=localhost -p carla_port:=2000 -p ssv2_port:=5555 \
//!         [-p config_file:=<carla_world.yaml>]
//!
//! One process per CARLA server, and the only one that ticks it: it holds an exclusive lock
//! on `/tmp/swb-locks/carla-<host>-<port>.lock` for its whole life, and a second one is
//! refused (exit 1, after 30 s so a supervisor's respawn does not spin). It binds its world
//! port before anything else, so a second server on the port is refused too. It answers the protocol at once; until CARLA is up its
//! `Hello` says not ready, and it keeps trying. A CARLA call that never returns, or a CARLA
//! that went away (the idle check), ends it with exit 75 for its supervisor to restart, with
//! no further word to CARLA (roadmap 034; CLAUDE.md "A CARLA client must not outlive its
//! server"). It ends with its parent, too (`PR_SET_PDEATHSIG`).
//!
//! `--generate-signal-table <map dir>`: write the map's CARLA traffic light table, then leave
//! CARLA as found.

mod actor_census;
mod backend;
mod blueprint_boxes;
mod clock_store;
mod epoch_marker;
mod sensor_release;
mod spawn_ladder;
mod tick_gate;
mod tick_watchdog;
mod world;

pub(crate) use csb_core::{
    agent_link, autopilot, body_clearance, budgets, config, contact_report, coordinate_conversion,
    entity_manager, episode_clock, gap_trace, lanelet_map, map_resolver, npc_tracking, proto,
    ros_args, weather,
};

use std::io::Write;
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use color_eyre::eyre::{self, Result};
use csb_core::world_server::{self, End, Shell, World};

/// Exit status when a CARLA call hung or CARLA went away (`EX_TEMPFAIL`): restarted.
const RESTART_EXIT: i32 = 75;
/// Exit status when this server must not run (its port or CARLA's lock is held).
const REFUSED_EXIT: i32 = 1;
/// How long a refused server waits before it exits: play_launch respawns a node on any exit,
/// so a refusal must not turn into a loop that floods the log every respawn_delay.
const REFUSED_PAUSE: std::time::Duration = std::time::Duration::from_secs(30);

/// Refuse to run: say why, wait out [`REFUSED_PAUSE`], exit 1.
fn refuse(why: &eyre::Report) -> ! {
    tracing::error!("{why:#}; exiting in {} s", REFUSED_PAUSE.as_secs());
    std::thread::sleep(REFUSED_PAUSE);
    std::process::exit(REFUSED_EXIT);
}

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();
    world_server::die_with_parent();

    let params = ros_args::RosParams::from_args(std::env::args().skip(1))?;
    let config_file = ros_args::config_file_for(&params, "carla_world", "carla_world.yaml");
    let config_dir = config_file
        .parent()
        .map(Path::to_path_buf)
        .unwrap_or_default();
    let mut config = config::BridgeConfig::load_for(&config_file, config::ConfigOwner::CarlaWorld)?;
    config.apply_env_overrides();
    config.apply_ros_params(&params)?;
    // The bridge writes the contact reports and the gap trace from what this server sends.
    config.contact_report_dir = Some(contact_report::OFF.to_string());
    config.gap_trace_dir = String::new();
    let port: u16 = match params.get("port").filter(|v| !v.is_empty()) {
        Some(p) => p
            .parse()
            .map_err(|e| eyre::eyre!("parameter port:={p}: {e}"))?,
        None => 5591,
    };

    let shutdown = Arc::new(AtomicBool::new(false));
    {
        let shutdown = shutdown.clone();
        ctrlc::set_handler(move || {
            tracing::info!("signal received, stopping");
            shutdown.store(true, Ordering::SeqCst);
        })?;
    }

    // One ticking client per CARLA server, held by the kernel for this process's life.
    let _lock = match carla_lock(&config.carla.host, config.carla.port) {
        Ok(lock) => lock,
        Err(e) => refuse(&e),
    };

    if let Some(map_dir) = generate_signal_table_arg() {
        let Some(mut carla) = backend::carla::connect(config, config_dir, &shutdown)? else {
            return Ok(());
        };
        let result = carla.generate_signal_table(&map_dir);
        carla.shutdown();
        return result;
    }

    let ctx = zmq::Context::new();
    let endpoint = format!("tcp://127.0.0.1:{port}");
    let socket = match world_server::bind(&ctx, &endpoint) {
        Ok(s) => s,
        Err(e) => refuse(&e),
    };
    tracing::info!(
        "CARLA world server on {endpoint} (pid {}), CARLA {}:{}, config {}",
        std::process::id(),
        config.carla.host,
        config.carla.port,
        config_file.display()
    );

    let mut world = world::CarlaWorld::new(config, config_dir);
    world.try_connect();
    let mut shell = Shell::new(world);
    let end = world_server::serve(&socket, &mut shell, &shutdown, budgets::IDLE_PROBE_AFTER)?;
    match end {
        End::Stopped => {
            tracing::info!("Stopping: undoing what this server did to CARLA");
            shell.world.shutdown();
            drop(socket);
            Ok(())
        }
        End::Fatal(reason) | End::Lost(reason) => {
            // The replies are out (the shell answered before ending). Nothing more to CARLA:
            // a call may still be inside LibCarla, or the server is gone.
            if let Some(backend) = shell.world.backend_mut() {
                backend.abandon_after_hang();
            }
            drop(socket);
            drop(ctx);
            tracing::error!("Exiting with status {RESTART_EXIT}: {reason}");
            let _ = std::io::stdout().flush();
            let _ = std::io::stderr().flush();
            // SAFETY: ends the process at once; nothing after it runs. `_exit` skips the C++
            // destructors, which could wait on the Traffic Manager mutex a hung thread holds
            // or make RPCs on a dead connection (034).
            unsafe { libc::_exit(RESTART_EXIT) };
        }
    }
}

/// The lock that makes this the only world server ticking `host:port`: an exclusive `flock`
/// on `/tmp/swb-locks/carla-<host>-<port>.lock`, released by the kernel when the process
/// ends however it ends. The holder's pid is written in it, for the refusal's message.
fn carla_lock(host: &str, port: u16) -> Result<std::fs::File> {
    let dir = PathBuf::from("/tmp/swb-locks");
    if !dir.exists() {
        std::fs::create_dir_all(&dir)?;
        // Every user's world servers share it, as they share the CARLA ports.
        use std::os::unix::fs::PermissionsExt;
        let _ = std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o1777));
    }
    let path = dir.join(format!("carla-{host}-{port}.lock"));
    let mut file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(&path)
        .map_err(|e| eyre::eyre!("open {}: {e}", path.display()))?;
    // SAFETY: flock on a file descriptor this function owns.
    let rc = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
    if rc != 0 {
        let holder = std::fs::read_to_string(&path).unwrap_or_default();
        eyre::bail!(
            "another world server already drives CARLA at {host}:{port} (pid {}, lock {}): one \
             client ticks a CARLA server; stop it, or give this one another CARLA",
            holder.trim(),
            path.display()
        );
    }
    file.set_len(0)?;
    write!(file, "{}", std::process::id())?;
    Ok(file)
}

/// `--generate-signal-table <map dir>` (before any `--ros-args`).
fn generate_signal_table_arg() -> Option<PathBuf> {
    let mut args = std::env::args().skip(1).take_while(|a| a != "--ros-args");
    while let Some(a) = args.next() {
        if a == "--generate-signal-table" {
            return args.next().map(PathBuf::from);
        }
    }
    None
}
