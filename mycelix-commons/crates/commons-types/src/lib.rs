// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Mycelix Commons Shared Types & Utilities
//!
//! Common functionality for all domain zomes in the Commons cluster:
//! - Batch query operations (solving N+1 query patterns)
//! - Anchor utilities for consistent indexing
//! - Bridge types for cross-domain communication
//! - Geographic types shared across domains
//! - Mission-neutral maritime evidence carried by existing bridge events
//! - Local replay/gap/fork classification for maritime store-forward streams
//! - Recipe-free regenerative component provenance carried by maritime evidence
//! - Recipe-free regenerative genome lineage provenance carried by maritime evidence
//! - Recipe-free regenerative viability provenance carried by maritime evidence
//! - Recipe-free regenerative epoch-handoff provenance carried by maritime evidence
//! - Recovery-aware regenerative lineage-experiment provenance carried by maritime evidence
//! - Controlled regenerative viability-frontier provenance carried by maritime evidence
//! - Successor lineage-depth provenance carried by maritime evidence

pub mod anchors;
pub mod batch;
pub mod bridge_types;
pub mod geo;
pub mod maritime;
pub mod maritime_stream;
pub mod regenerative_component;
pub mod regenerative_epoch_handoff;
pub mod regenerative_genome;
pub mod regenerative_lineage_experiment;
pub mod regenerative_successor_depth;
pub mod regenerative_viability;
pub mod regenerative_viability_frontier;

pub use anchors::*;
pub use batch::*;
pub use bridge_types::*;
pub use geo::*;
pub use maritime::*;
pub use maritime_stream::*;
pub use regenerative_component::*;
pub use regenerative_epoch_handoff::*;
pub use regenerative_genome::*;
pub use regenerative_lineage_experiment::*;
pub use regenerative_successor_depth::*;
pub use regenerative_viability::*;
pub use regenerative_viability_frontier::*;
