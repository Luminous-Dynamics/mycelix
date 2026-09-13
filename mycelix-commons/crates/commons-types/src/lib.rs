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
//! - Multi-generation regenerative lineage-depth provenance carried by maritime evidence
//! - Descendant reproduction-policy frontier provenance carried by maritime evidence
//! - Decomposed descendant generation-policy metrics carried by maritime evidence
//! - Policy-normalized regenerative lineage provenance carried by maritime evidence
//! - Reproduction-policy sensitivity-surface provenance carried by maritime evidence
//! - Immutable policy-surface revision/supersession provenance carried by maritime evidence
//! - Intergenerational regenerative support-basis provenance carried by maritime evidence
//! - Role-support closure continuity provenance carried by maritime evidence
//! - Disturbance-conditioned regenerative recovery-coordinate provenance
//! - Local recovery-reserve replay/gap/fork classification
//! - Explicit evidence-bearing recovery-reserve fork resolution
//! - Governance/threshold-signature authority provenance for fork resolution
//! - Composed governance-bound fork application receipts

pub mod anchors;
pub mod batch;
pub mod bridge_types;
pub mod geo;
pub mod maritime;
pub mod maritime_stream;
pub mod regenerative_component;
pub mod regenerative_epoch_handoff;
pub mod regenerative_generation_policy_metrics;
pub mod regenerative_genome;
pub mod regenerative_lineage_experiment;
pub mod regenerative_multigeneration_depth;
pub mod regenerative_policy_normalized_lineage;
pub mod regenerative_policy_sensitivity_surface;
pub mod regenerative_policy_surface_revision;
pub mod regenerative_recovery_coordinate;
pub mod regenerative_recovery_fork_resolution;
pub mod regenerative_recovery_governance_application;
pub mod regenerative_recovery_governance_authority;
pub mod regenerative_recovery_reserve_stream;
pub mod regenerative_reproduction_policy_frontier;
pub mod regenerative_successor_depth;
pub mod regenerative_support_basis;
pub mod regenerative_support_closure_continuity;
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
pub use regenerative_generation_policy_metrics::*;
pub use regenerative_genome::*;
pub use regenerative_lineage_experiment::*;
pub use regenerative_multigeneration_depth::*;
pub use regenerative_policy_normalized_lineage::*;
pub use regenerative_policy_sensitivity_surface::*;
pub use regenerative_policy_surface_revision::*;
pub use regenerative_recovery_coordinate::*;
pub use regenerative_recovery_fork_resolution::*;
pub use regenerative_recovery_governance_application::*;
pub use regenerative_recovery_governance_authority::*;
pub use regenerative_recovery_reserve_stream::*;
pub use regenerative_reproduction_policy_frontier::*;
pub use regenerative_successor_depth::*;
pub use regenerative_support_basis::*;
pub use regenerative_support_closure_continuity::*;
pub use regenerative_viability::*;
pub use regenerative_viability_frontier::*;
