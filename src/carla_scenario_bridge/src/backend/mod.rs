//! The bridge's simulator seam (roadmaps 035, 046): the [`SimBackend`] trait and its types
//! (`csb_core::sim`), and the one implementation, [`remote::RemoteBackend`], which drives a
//! world server over the world protocol. The coordinator's tests run on `csb_core::fake`.

pub use csb_core::sim::*;

#[cfg(test)]
pub use csb_core::fake;

pub mod remote;
