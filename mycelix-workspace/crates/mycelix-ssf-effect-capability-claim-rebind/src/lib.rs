// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact convergence of direct and historically reconciled durable SSF
//! effect-capability claims.
//!
//! Once a durable claim exists, the claim record is the cross-restart
//! execution-attempt identity. This crate deliberately does not reconstruct the
//! transient capability after restart. It consumes the direct live capability
//! on the fast path, or exact-matches an archived capability audit binding on
//! the historical path, and produces a non-executable claimed-lineage token.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_durable_effect_capability_claim::{
    DurableEffectCapabilityClaimFrontierV1, DurablyClaimedEffectCapabilityV1,
    EffectCapabilityClaimAttemptManifestV1, EffectCapabilityClaimDispositionV1,
    EffectCapabilityClaimReceiptCommitment, EffectCapabilityClaimRecordCommitment,
    EffectCapabilityClaimStoreDescriptorV1, ExactTransientEffectCapabilityV1,
};
use mycelix_ssf_historical_effect_capability_claim::{
    HistoricalEffectCapabilityClaimDispositionV1, HistoricalEffectCapabilityClaimV1,
};
use mycelix_ssf_transient_effect_capability::TransientEffectCapabilityBindingV1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ClaimedEffectLineageEvidenceKindV1 {
    DirectClaim,
    HistoricalReconciliation,
}

/// Copyable audit binding of one exact durable claimed-effect lineage.
///
/// There is deliberately no public constructor from this binding. The only
/// constructors consume either the direct durable-claim typestate or a
/// historical `Claimed` proof plus the exact archived capability binding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReboundClaimedEffectLineageBindingV1 {
    pub kind: ClaimedEffectLineageEvidenceKindV1,
    pub capability: TransientEffectCapabilityBindingV1,
    pub manifest: EffectCapabilityClaimAttemptManifestV1,
    pub claim_record: EffectCapabilityClaimRecordCommitment,
    pub post_frontier: DurableEffectCapabilityClaimFrontierV1,
    pub verification_descriptor: EffectCapabilityClaimStoreDescriptorV1,
    pub evidence_receipt: EffectCapabilityClaimReceiptCommitment,
    pub evidence_valid_until: u64,
    pub claim_eligibility_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectCapabilityClaimRebindError {
    CapabilityBindingMismatch,
    SourceDispositionNotClaimed,
    ClaimRecordMismatch,
    ClaimFrontierMismatch,
    HistoricalReceiptMissing,
    HistoricalClaimCeilingMissing,
}

/// Claimed execution-attempt lineage, still one boundary short of any provider
/// resolution or actuator invocation.
pub struct ReboundClaimedEffectLineageV1 {
    binding: ReboundClaimedEffectLineageBindingV1,
}

impl ReboundClaimedEffectLineageV1 {
    pub const fn binding(&self) -> ReboundClaimedEffectLineageBindingV1 {
        self.binding
    }

    pub const fn claim_record(&self) -> EffectCapabilityClaimRecordCommitment {
        self.binding.claim_record
    }

    pub const fn capability_binding(&self) -> TransientEffectCapabilityBindingV1 {
        self.binding.capability
    }

    pub const fn claim_eligibility_valid_until(&self) -> u64 {
        self.binding.claim_eligibility_valid_until
    }

    pub const fn durable_claim_lineage_established(&self) -> bool {
        true
    }

    pub const fn reconstructs_transient_capability(&self) -> bool {
        false
    }

    pub const fn requires_fresh_pre_invocation_revalidation(&self) -> bool {
        true
    }

    pub const fn provider_handle_resolved(&self) -> bool {
        false
    }

    pub const fn actuator_invocation_performed(&self) -> bool {
        false
    }

    pub const fn eligible_for_actuator_without_revalidation(&self) -> bool {
        false
    }
}

fn exact_claim_ceiling(
    evidence_valid_until: u64,
    capability_valid_until: u64,
    claim_time_valid_until: u64,
    inherited_claim_ceiling: u64,
) -> u64 {
    evidence_valid_until
        .min(capability_valid_until)
        .min(claim_time_valid_until)
        .min(inherited_claim_ceiling)
}

/// Consume the live direct claim. The old transient capability is deliberately
/// not returned or retained: after durable claim, the claim record is the
/// execution-attempt lineage.
pub fn rebind_direct_claim<S, C, TQ>(
    claimed: DurablyClaimedEffectCapabilityV1<S, C, TQ>,
) -> Result<ReboundClaimedEffectLineageV1, EffectCapabilityClaimRebindError>
where
    C: ExactTransientEffectCapabilityV1,
{
    let capability = claimed.capability().exact_binding();
    let manifest = claimed.manifest();
    if manifest.subject().capability != capability {
        return Err(EffectCapabilityClaimRebindError::CapabilityBindingMismatch);
    }

    let receipt = claimed.receipt();
    let (claim_record, post_frontier) = match receipt.disposition {
        EffectCapabilityClaimDispositionV1::Claimed {
            claim_record,
            post_frontier,
        } => (claim_record, post_frontier),
        _ => return Err(EffectCapabilityClaimRebindError::SourceDispositionNotClaimed),
    };

    if claim_record != claimed.claim_record() {
        return Err(EffectCapabilityClaimRebindError::ClaimRecordMismatch);
    }
    if post_frontier != claimed.post_frontier() {
        return Err(EffectCapabilityClaimRebindError::ClaimFrontierMismatch);
    }

    let claim_eligibility_valid_until = exact_claim_ceiling(
        receipt.valid_until,
        capability.valid_until,
        manifest.subject().claim_time_valid_until,
        claimed.claim_eligibility_valid_until(),
    );

    // `claimed` is consumed and dropped here. No transient capability object is
    // returned from this transition.
    Ok(ReboundClaimedEffectLineageV1 {
        binding: ReboundClaimedEffectLineageBindingV1 {
            kind: ClaimedEffectLineageEvidenceKindV1::DirectClaim,
            capability,
            manifest,
            claim_record,
            post_frontier,
            verification_descriptor: receipt.store,
            evidence_receipt: receipt.receipt_commitment,
            evidence_valid_until: receipt.valid_until,
            claim_eligibility_valid_until,
        },
    })
}

/// Rebind a historically proven durable claim to the exact archived capability
/// audit binding. This does not reconstruct `TransientEffectCapabilityV1`.
pub fn rebind_historical_claim<S>(
    capability: TransientEffectCapabilityBindingV1,
    historical: HistoricalEffectCapabilityClaimV1<S>,
) -> Result<ReboundClaimedEffectLineageV1, EffectCapabilityClaimRebindError> {
    let manifest = historical.manifest();
    if manifest.subject().capability != capability {
        return Err(EffectCapabilityClaimRebindError::CapabilityBindingMismatch);
    }

    let (claim_record, post_frontier) = match historical.disposition() {
        HistoricalEffectCapabilityClaimDispositionV1::Claimed {
            claim_record,
            post_frontier,
        } => (claim_record, post_frontier),
        _ => return Err(EffectCapabilityClaimRebindError::SourceDispositionNotClaimed),
    };

    let receipt = historical
        .receipt()
        .copied()
        .ok_or(EffectCapabilityClaimRebindError::HistoricalReceiptMissing)?;
    let inherited_claim_ceiling = historical
        .claim_eligibility_valid_until()
        .ok_or(EffectCapabilityClaimRebindError::HistoricalClaimCeilingMissing)?;

    let claim_eligibility_valid_until = exact_claim_ceiling(
        receipt.valid_until,
        capability.valid_until,
        manifest.subject().claim_time_valid_until,
        inherited_claim_ceiling,
    );

    Ok(ReboundClaimedEffectLineageV1 {
        binding: ReboundClaimedEffectLineageBindingV1 {
            kind: ClaimedEffectLineageEvidenceKindV1::HistoricalReconciliation,
            capability,
            manifest,
            claim_record,
            post_frontier,
            verification_descriptor: historical.expected_reconciler().descriptor(),
            evidence_receipt: receipt.receipt_commitment,
            evidence_valid_until: receipt.valid_until,
            claim_eligibility_valid_until,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direct_and_historical_paths_remain_distinct_audit_domains() {
        assert_ne!(
            ClaimedEffectLineageEvidenceKindV1::DirectClaim,
            ClaimedEffectLineageEvidenceKindV1::HistoricalReconciliation
        );
    }

    #[test]
    fn fresh_historical_evidence_cannot_extend_old_claim_authority() {
        assert_eq!(exact_claim_ceiling(180, 120, 100, 90), 90);
    }

    #[test]
    fn capability_lifetime_can_be_the_oldest_boundary() {
        assert_eq!(exact_claim_ceiling(180, 80, 100, 90), 80);
    }
}
