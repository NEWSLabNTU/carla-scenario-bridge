//! The bridge's simulator seam (roadmaps 035, 046): the [`SimBackend`] trait and its types
//! (`swb_core::sim`), and the one implementation, [`remote::RemoteBackend`], which drives a
//! world server over the world protocol. The coordinator's tests run on `swb_core::fake`.

pub use swb_core::sim::*;

#[cfg(test)]
pub use swb_core::fake;

pub mod remote;
