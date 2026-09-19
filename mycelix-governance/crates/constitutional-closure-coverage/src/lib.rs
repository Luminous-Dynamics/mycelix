// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fault-aware closure coverage for constitutional lifecycle decisions.
//!
//! Temporal provenance answers what evidence/closure history exists. This layer
//! answers the narrower lifecycle question: is that history currently sufficient
//! authority to terminalize a revocation?

use constitutional_temporal_provenance::{
    CrossOrderRelation, EvidenceClosure, TemporalEvidenceState, TemporalIntegrityFault,
};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum PreRevocationCoverage {
    /// Revocation is effective at sequence 1, so there is no positive earlier
    /// effective sequence in which admissible finality could exist.
    EmptyPreRevocationInterval {
        revocation_effective_seq: u64,
        finality_domain_id: String,
        policy_version: String,
    },

    /// No currently authoritative closure covers every positive effective
    /// sequence before the revocation.
    Open {
        revocation_effective_seq: u64,
        required_closed_through_effective_seq: u64,
        finality_domain_id: String,
        policy_version: String,
    },

    /// A currently authoritative closure covers the complete admissible
    /// pre-revocation interval. The exact closure object is returned so callers
    /// cannot terminalize from a copied integer watermark alone.
    Closed {
        revocation_effective_seq: u64,
        required_closed_through_effective_seq: u64,
        finality_domain_id: String,
        policy_version: String,
        observation_domain_id: String,
        cross_order_relation: CrossOrderRelation,
        closure: EvidenceClosure,
    },

    /// Closure/finality assumptions for this domain are contradictory. Historical
    /// closure records remain evidence but provide no lifecycle authority.
    IntegrityFault {
        revocation_effective_seq: u64,
        finality_domain_id: String,
        policy_version: String,
        fault: TemporalIntegrityFault,
        latest_closure_id: Option<String>,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ClosureCoverageError {
    ZeroRevocationEffectiveSequence,
}

/// Assess whether the effective-order interval before a revocation is closed for
/// lifecycle purposes.
///
/// Ordering rule:
/// - `revocation_effective_seq` and closure watermark are in the same finality
///   effective-order domain and may be compared;
/// - verifier observation order is never used to decide legal precedence here;
/// - an active temporal integrity fault dominates all historical closure data.
pub fn assess_pre_revocation_coverage(
    state: &TemporalEvidenceState,
    revocation_effective_seq: u64,
) -> Result<PreRevocationCoverage, ClosureCoverageError> {
    if revocation_effective_seq == 0 {
        return Err(ClosureCoverageError::ZeroRevocationEffectiveSequence);
    }

    if let Some(fault) = state.integrity_fault.clone() {
        return Ok(PreRevocationCoverage::IntegrityFault {
            revocation_effective_seq,
            finality_domain_id: state.policy.domain_id.clone(),
            policy_version: state.policy.policy_version.clone(),
            fault,
            latest_closure_id: state.latest_closure_id.clone(),
        });
    }

    if revocation_effective_seq == 1 {
        return Ok(PreRevocationCoverage::EmptyPreRevocationInterval {
            revocation_effective_seq,
            finality_domain_id: state.policy.domain_id.clone(),
            policy_version: state.policy.policy_version.clone(),
        });
    }

    let required_closed_through_effective_seq = revocation_effective_seq - 1;
    if let Some(closure) = state.latest_closure() {
        if closure.closed_through_effective_seq >= required_closed_through_effective_seq {
            return Ok(PreRevocationCoverage::Closed {
                revocation_effective_seq,
                required_closed_through_effective_seq,
                finality_domain_id: state.policy.domain_id.clone(),
                policy_version: state.policy.policy_version.clone(),
                observation_domain_id: state.observation_order.domain_id.clone(),
                cross_order_relation: state.observation_order.relation,
                closure: closure.clone(),
            });
        }
    }

    Ok(PreRevocationCoverage::Open {
        revocation_effective_seq,
        required_closed_through_effective_seq,
        finality_domain_id: state.policy.domain_id.clone(),
        policy_version: state.policy.policy_version.clone(),
    })
}

/// Compatibility helper for callers that only need a yes/no answer.
///
/// Lifecycle code should prefer `assess_pre_revocation_coverage()` so it retains
/// exact closure/fault provenance. `IntegrityFault` is always fail-closed.
pub fn permits_terminal_revocation(coverage: &PreRevocationCoverage) -> bool {
    matches!(
        coverage,
        PreRevocationCoverage::EmptyPreRevocationInterval { .. }
            | PreRevocationCoverage::Closed { .. }
    )
}
