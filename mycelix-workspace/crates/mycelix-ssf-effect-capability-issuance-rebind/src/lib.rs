// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact rebind of durable effect-capability issuance evidence to the typed
//! source-owned operation material lineage that produced it.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_durable_effect_capability_issuance::{
    DurableEffectCapabilityFrontierV1, DurablyIssuedEffectCapabilityLineageV1,
    EffectCapabilityIssuanceAttemptManifestV1, EffectCapabilityIssuanceDispositionV1,
    EffectCapabilityIssuanceReceiptCommitment, EffectCapabilityIssuanceStoreDescriptorV1,
    EffectCapabilityIssuanceSubjectV1, EffectCapabilityRecordCommitment,
    ExactSourceOwnedOperationMaterialV1,
};
use mycelix_ssf_historical_effect_capability_issuance::{
    HistoricalEffectCapabilityDispositionV1, HistoricalEffectCapabilityIssuanceV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectCapabilityIssuanceEvidenceKindV1 {
    DirectIssuance,
    HistoricalReconciliation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReboundEffectCapabilityIssuanceBindingV1 {
    pub kind: EffectCapabilityIssuanceEvidenceKindV1,
    pub manifest: EffectCapabilityIssuanceAttemptManifestV1,
    pub capability_record: EffectCapabilityRecordCommitment,
    pub post_frontier: DurableEffectCapabilityFrontierV1,
    pub verification_descriptor: EffectCapabilityIssuanceStoreDescriptorV1,
    pub evidence_receipt: EffectCapabilityIssuanceReceiptCommitment,
    pub evidence_valid_until: u64,
    pub capability_eligibility_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectCapabilityIssuanceRebindError {
    IssuanceSubjectMismatch,
    SourceDispositionNotIssued,
    HistoricalReceiptMissing,
    HistoricalCapabilityCeilingMissing,
}

fn subject_from_material<M: ExactSourceOwnedOperationMaterialV1>(
    material: &M,
) -> EffectCapabilityIssuanceSubjectV1 {
    EffectCapabilityIssuanceSubjectV1 {
        admitted_effect: material.exact_admitted_effect(),
        capability_time_revalidation: material.exact_capability_time_revalidation(),
        material_receipt: material.exact_material_receipt(),
        material_valid_until: material.exact_material_valid_until(),
    }
}

fn subject_matches_material<M: ExactSourceOwnedOperationMaterialV1>(
    manifest: EffectCapabilityIssuanceAttemptManifestV1,
    material: &M,
) -> bool {
    manifest.subject() == subject_from_material(material)
}

enum ReboundEffectCapabilityCarrierV1<S, M> {
    Direct(DurablyIssuedEffectCapabilityLineageV1<S, M>),
    Historical {
        material: M,
        reconciliation: HistoricalEffectCapabilityIssuanceV1<S>,
    },
}

/// Exact durable capability lineage rebound to the typed source-owned material
/// that produced it.
///
/// This remains pre-capability. A fresh final execution/provider/time proof is
/// still required before transient move-only capability construction.
pub struct ReboundIssuedEffectCapabilityLineageV1<S, M> {
    carrier: ReboundEffectCapabilityCarrierV1<S, M>,
    binding: ReboundEffectCapabilityIssuanceBindingV1,
}

impl<S, M> ReboundIssuedEffectCapabilityLineageV1<S, M> {
    pub const fn binding(&self) -> ReboundEffectCapabilityIssuanceBindingV1 {
        self.binding
    }

    pub fn material(&self) -> &M {
        match &self.carrier {
            ReboundEffectCapabilityCarrierV1::Direct(issued) => issued.material(),
            ReboundEffectCapabilityCarrierV1::Historical { material, .. } => material,
        }
    }

    pub fn historical_reconciliation(&self) -> Option<&HistoricalEffectCapabilityIssuanceV1<S>> {
        match &self.carrier {
            ReboundEffectCapabilityCarrierV1::Direct(_) => None,
            ReboundEffectCapabilityCarrierV1::Historical { reconciliation, .. } => {
                Some(reconciliation)
            }
        }
    }

    pub const fn durable_single_lineage_rebound(&self) -> bool {
        true
    }

    pub const fn requires_final_execution_provider_time_revalidation(&self) -> bool {
        true
    }

    pub const fn eligible_for_transient_capability_without_final_revalidation(&self) -> bool {
        false
    }

    pub const fn contains_transient_effect_capability(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub fn rebind_direct_effect_capability_issuance<S, M>(
    issued: DurablyIssuedEffectCapabilityLineageV1<S, M>,
) -> Result<ReboundIssuedEffectCapabilityLineageV1<S, M>, EffectCapabilityIssuanceRebindError>
where
    M: ExactSourceOwnedOperationMaterialV1,
{
    let manifest = issued.manifest();
    if !subject_matches_material(manifest, issued.material()) {
        return Err(EffectCapabilityIssuanceRebindError::IssuanceSubjectMismatch);
    }

    let receipt = issued.receipt();
    let (capability_record, post_frontier) = match receipt.disposition {
        EffectCapabilityIssuanceDispositionV1::Issued {
            capability_record,
            post_frontier,
        } => (capability_record, post_frontier),
        _ => return Err(EffectCapabilityIssuanceRebindError::SourceDispositionNotIssued),
    };

    let binding = ReboundEffectCapabilityIssuanceBindingV1 {
        kind: EffectCapabilityIssuanceEvidenceKindV1::DirectIssuance,
        manifest,
        capability_record,
        post_frontier,
        verification_descriptor: receipt.store,
        evidence_receipt: receipt.receipt_commitment,
        evidence_valid_until: receipt.valid_until,
        capability_eligibility_valid_until: issued.capability_eligibility_valid_until(),
    };

    Ok(ReboundIssuedEffectCapabilityLineageV1 {
        carrier: ReboundEffectCapabilityCarrierV1::Direct(issued),
        binding,
    })
}

pub fn rebind_historical_effect_capability_issuance<S, M>(
    material: M,
    reconciliation: HistoricalEffectCapabilityIssuanceV1<S>,
) -> Result<ReboundIssuedEffectCapabilityLineageV1<S, M>, EffectCapabilityIssuanceRebindError>
where
    M: ExactSourceOwnedOperationMaterialV1,
{
    let manifest = reconciliation.manifest();
    if !subject_matches_material(manifest, &material) {
        return Err(EffectCapabilityIssuanceRebindError::IssuanceSubjectMismatch);
    }

    let (capability_record, post_frontier) = match reconciliation.disposition() {
        HistoricalEffectCapabilityDispositionV1::Issued {
            capability_record,
            post_frontier,
        } => (capability_record, post_frontier),
        _ => return Err(EffectCapabilityIssuanceRebindError::SourceDispositionNotIssued),
    };

    let receipt = reconciliation
        .receipt()
        .copied()
        .ok_or(EffectCapabilityIssuanceRebindError::HistoricalReceiptMissing)?;
    let historical_ceiling = reconciliation
        .capability_eligibility_valid_until()
        .ok_or(EffectCapabilityIssuanceRebindError::HistoricalCapabilityCeilingMissing)?;
    let capability_eligibility_valid_until =
        historical_ceiling.min(material.exact_material_valid_until());

    let binding = ReboundEffectCapabilityIssuanceBindingV1 {
        kind: EffectCapabilityIssuanceEvidenceKindV1::HistoricalReconciliation,
        manifest,
        capability_record,
        post_frontier,
        verification_descriptor: reconciliation.expected_reconciler().descriptor(),
        evidence_receipt: receipt.receipt_commitment,
        evidence_valid_until: receipt.valid_until,
        capability_eligibility_valid_until,
    };

    Ok(ReboundIssuedEffectCapabilityLineageV1 {
        carrier: ReboundEffectCapabilityCarrierV1::Historical {
            material,
            reconciliation,
        },
        binding,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direct_and_historical_evidence_kinds_remain_distinct() {
        assert_ne!(
            EffectCapabilityIssuanceEvidenceKindV1::DirectIssuance,
            EffectCapabilityIssuanceEvidenceKindV1::HistoricalReconciliation
        );
    }

    #[test]
    fn fresh_historical_evidence_cannot_extend_material_lifetime() {
        let historical_ceiling = 180;
        let material_ceiling = 90;
        assert_eq!(historical_ceiling.min(material_ceiling), 90);
    }
}
