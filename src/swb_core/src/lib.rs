//! The scenario bridge's simulator-free core (roadmap 046).
//!
//! What the bridge and every world server share: SSv2's `simulation_interface` and the world
//! protocol as Rust types ([`proto`]), the [`sim::SimBackend`] seam and its types, and the
//! policy that knows no simulator -- the entity table, the episode clock, NPC tracking, the
//! gap trace, contact reports, the agent link, weather presets, the configuration. Nothing
//! here depends on a simulator's client library: the dependency graph is the boundary.

pub mod agent_link;
pub mod autopilot;
pub mod body_clearance;
pub mod budgets;
pub mod config;
pub mod contact_report;
pub mod coordinate_conversion;
pub mod entity_manager;
pub mod episode_clock;
pub mod fake;
pub mod gap_trace;
pub mod lanelet_map;
pub mod map_resolver;
pub mod npc_tracking;
pub mod proto;
pub mod ros_args;
pub mod sim;
pub mod weather;
pub mod world_client;
pub mod world_server;
