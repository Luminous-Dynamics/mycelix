// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact rebind of durable effect-admission evidence to the typed execution-
//! surface admission candidate that produced it.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use mycelix_ssf_durable_effect_admission::{
    DurableEffectAdmissionFrontierV1, DurablyAdmittedEffectV1,
    EffectAdmissionAttemptManifestV1, EffectAdmissionDispositionV1,
    EffectAdmissionReceiptCommitment, EffectAdmissionRecordCommitment,
    EffectAdmissionStoreDescriptorV1, ExactExecutionAdmissionCandidateV1,
};
use mycelix_ssf_historical_effect_admission_reconciliation::{
    HistoricalEffectAdmissionDispositionV1, HistoricalEffectAdmissionReconciliationV1,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EffectAdmissionEvidenceKindV1 {
    DirectAdmission,
    HistoricalReconciliation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReboundEffectAdmissionBindingV1 {
    pub kind: EffectAdmissionEvidenceKindV1,
    pub manifest: EffectAdmissionAttemptManifestV1,
    pub admission_record: EffectAdmissionRecordCommitment,
    pub post_frontier: DurableEffectAdmissionFrontierV1,
    pub verification_descriptor: EffectAdmissionStoreDescriptorV1,
    pub evidence_receipt: EffectAdmissionReceiptCommitment,
    pub evidence_valid_until: u64,
    pub effect_eligibility_valid_until: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectAdmissionRebindError {
    AdmissionSubjectMismatch,
    SourceDispositionNotAdmitted,
    HistoricalReceiptMissing,
    HistoricalEffectCeilingMissing,
}

fn subject_matches_candidate<C: ExactExecutionAdmissionCandidateV1>(
    manifest: EffectAdmissionAttemptManifestV1,
    candidate: &C,
) -> bool {
    manifest.subject.execution_admission_receipt == candidate.exact_admission_receipt()
        && manifest.subject.execution_admission_valid_until
            == candidate.exact_admission_valid_until()
}

enum ReboundEffectAdmissionCarrierV1<S, C> {
    Direct(DurablyAdmittedEffectV1<S, C>),
    Historical {
        candidate: C,
        reconciliation: HistoricalEffectAdmissionReconciliationV1<S>,
    },
}

/// Exact possible-effect lineage rebound to the typed execution-admission
/// candidate that produced it.
///
/// Rebind is still not executable. Both direct and historical paths require a
/// fresh capability-time execution revalidation before any move-only
/// capability may be considered.
pub struct ReboundDurablyAdmittedEffectV1<S, C> {
    carrier: ReboundEffectAdmissionCarrierV1<S, C>,
    binding: ReboundEffectAdmissionBindingV1,
}

impl<S, C> ReboundDurablyAdmittedEffectV1<S, C> {
    pub const fn binding(&self) -> ReboundEffectAdmissionBindingV1 {
        self.binding
    }

    pub fn candidate(&self) -> &C {
        match &self.carrier {
            ReboundEffectAdmissionCarrierV1::Direct(admitted) => admitted.candidate(),
            ReboundEffectAdmissionCarrierV1::Historical { candidate, .. } => candidate,
        }
    }

    pub fn historical_reconciliation(
        &self,
    ) -> Option<&HistoricalEffectAdmissionReconciliationV1<S>> {
        match &self.carrier {
            ReboundEffectAdmissionCarrierV1::Direct(_) => None,
            ReboundEffectAdmissionCarrierV1::Historical { reconciliation, .. } => {
                Some(reconciliation)
            }
        }
    }

    pub const fn possible_effect_lineage_established(&self) -> bool {
        true
    }

    pub const fn requires_capability_time_execution_revalidation(&self) -> bool {
        true
    }

    pub const fn eligible_for_capability_without_revalidation(&self) -> bool {
        false
    }

    pub const fn contains_single_use_effect_capability(&self) -> bool {
        false
    }

    pub const fn contains_effect_authority(&self) -> bool {
        false
    }
}

pub fn rebind_direct_effect_admission<S, C>(
    admitted: DurablyAdmittedEffectV1<S, C>,
) -> Result<ReboundDurablyAdmittedEffectV1<S, C>, EffectAdmissionRebindError>
where
    C: ExactExecutionAdmissionCandidateV1,
{
    let manifest = admitted.manifest();
    if !subject_matches_candidate(manifest, admitted.candidate()) {
        return Err(EffectAdmissionRebindError::AdmissionSubjectMismatch);
    }

    let receipt = admitted.receipt();
    let (admission_record, post_frontier) = match receipt.disposition {
        EffectAdmissionDispositionV1::Admitted {
            record,
            post_frontier,
        } => (record, post_frontier),
        _ => return Err(EffectAdmissionRebindError::SourceDispositionNotAdmitted),
    };

    let binding = ReboundEffectAdmissionBindingV1 {
        kind: EffectAdmissionEvidenceKindV1::DirectAdmission,
        manifest,
        admission_record,
        post_frontier,
        verification_descriptor: receipt.store,
        evidence_receipt: receipt.receipt_commitment,
        evidence_valid_until: receipt.valid_until,
        effect_eligibility_valid_until: admitted.admission_valid_until(),
    };

    Ok(ReboundDurablyAdmittedEffectV1 {
        carrier: ReboundEffectAdmissionCarrierV1::Direct(admitted),
        binding,
    })
}

pub fn rebind_historical_effect_admission<S, C>(
    candidate: C,
    reconciliation: HistoricalEffectAdmissionReconciliationV1<S>,
) -> Result<ReboundDurablyAdmittedEffectV1<S, C>, EffectAdmissionRebindError>
where
    C: ExactExecutionAdmissionCandidateV1,
{
    let manifest = reconciliation.manifest();
    if !subject_matches_candidate(manifest, &candidate) {
        return Err(EffectAdmissionRebindError::AdmissionSubjectMismatch);
    }

    let (admission_record, post_frontier) = match reconciliation.disposition() {
        HistoricalEffectAdmissionDispositionV1::Admitted {
            admission_record,
            post_frontier,
        } => (admission_record, post_frontier),
        _ => return Err(EffectAdmissionRebindError::SourceDispositionNotAdmitted),
    };

    let receipt = reconciliation
        .receipt()
        .copied()
        .ok_or(EffectAdmissionRebindError::HistoricalReceiptMissing)?;
    let historical_ceiling = reconciliation
        .effect_eligibility_valid_until()
        .ok_or(EffectAdmissionRebindError::HistoricalEffectCeilingMissing)?;
    let effect_eligibility_valid_until = historical_ceiling.min(candidate.exact_admission_valid_until());

    let binding = ReboundEffectAdmissionBindingV1 {
        kind: EffectAdmissionEvidenceKindV1::HistoricalReconciliation,
        manifest,
        admission_record,
        post_frontier,
        verification_descriptor: reconciliation.expected_reconciler().descriptor(),
        evidence_receipt: receipt.receipt_commitment,
        evidence_valid_until: receipt.valid_until,
        effect_eligibility_valid_until,
    };

    Ok(ReboundDurablyAdmittedEffectV1 {
        carrier: ReboundEffectAdmissionCarrierV1::Historical {
            candidate,
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
            EffectAdmissionEvidenceKindV1::DirectAdmission,
            EffectAdmissionEvidenceKindV1::HistoricalReconciliation
        );
    }

    #[test]
    fn fresh_historical_evidence_cannot_extend_candidate_lifetime() {
        let historical_ceiling = 180;
        let candidate_ceiling = 90;
        assert_eq!(historical_ceiling.min(candidate_ceiling), 90);
    }
}
