use std::collections::{BTreeMap, BTreeSet};

use mycelix_finance_sync_capability::RailCapabilityProfileV1;
use mycelix_finance_sync_graph::{BoundedText, Commitment32};

use crate::canonical::{derive_bundle_commitment, derive_item_commitment};
use crate::{
    CapabilityEvidenceDimensionV1, ContradictedClaimSubjectV1, EvidenceBundleDispositionV1,
    EvidenceError, RailCapabilityEvidenceBundleInputV1, RailCapabilityEvidenceBundleV1,
    StructuralEvidenceClaimV1, StructuralEvidenceRecordV1, MAX_CONFLICT_IDENTITIES,
    MAX_CONTRADICTED_SUBJECTS, MAX_EVIDENCE_ITEMS, MAX_EVIDENCE_ITEMS_PER_DIMENSION,
};

pub fn build_rail_capability_evidence_bundle_v1(
    static_profile: &RailCapabilityProfileV1,
    input: RailCapabilityEvidenceBundleInputV1,
) -> Result<RailCapabilityEvidenceBundleV1, EvidenceError> {
    if input.static_profile_commitment != static_profile.profile_commitment() {
        return Err(EvidenceError::StaticProfileCommitmentMismatch);
    }
    if input.items.len() > MAX_EVIDENCE_ITEMS {
        return Err(EvidenceError::TooManyEvidenceItems);
    }

    let mut counts = BTreeMap::<CapabilityEvidenceDimensionV1, usize>::new();
    let mut identity_frontier = BTreeMap::<BoundedText, Commitment32>::new();
    let mut identity_conflicts = BTreeSet::<BoundedText>::new();
    let mut claim_polarity = BTreeMap::<(CapabilityEvidenceDimensionV1, Commitment32), u8>::new();
    let mut records = BTreeMap::<Commitment32, StructuralEvidenceRecordV1>::new();

    for item in input.items {
        let count = counts.entry(item.dimension).or_default();
        *count = count
            .checked_add(1)
            .ok_or(EvidenceError::TooManyEvidenceItemsForDimension)?;
        if *count > MAX_EVIDENCE_ITEMS_PER_DIMENSION {
            return Err(EvidenceError::TooManyEvidenceItemsForDimension);
        }

        let item_commitment = derive_item_commitment(static_profile.profile_commitment(), &item)?;

        let exact_duplicate = match identity_frontier.get(&item.evidence_id).copied() {
            Some(prior) if prior == item_commitment => true,
            Some(_) => {
                identity_conflicts.insert(item.evidence_id.clone());
                if identity_conflicts.len() > MAX_CONFLICT_IDENTITIES {
                    return Err(EvidenceError::TooManyConflictIdentities);
                }
                false
            }
            None => {
                identity_frontier.insert(item.evidence_id.clone(), item_commitment);
                false
            }
        };
        if exact_duplicate {
            continue;
        }

        let polarity = match item.claim {
            StructuralEvidenceClaimV1::SupportsDeclaredStaticSemantics => 0b01,
            StructuralEvidenceClaimV1::ContradictsDeclaredStaticSemantics => 0b10,
            StructuralEvidenceClaimV1::ObservesRelatedFact | StructuralEvidenceClaimV1::Indeterminate => 0,
        };
        if polarity != 0 {
            *claim_polarity
                .entry((item.dimension, item.claim_subject_commitment))
                .or_default() |= polarity;
        }

        records.entry(item_commitment).or_insert_with(|| StructuralEvidenceRecordV1 {
            dimension: item.dimension,
            evidence_id: item.evidence_id,
            item_commitment,
            claim: item.claim,
            claim_subject_commitment: item.claim_subject_commitment,
        });
    }

    let contradicted_claim_subjects: Vec<_> = claim_polarity
        .into_iter()
        .filter_map(|((dimension, claim_subject_commitment), polarity)| {
            (polarity == 0b11).then_some(ContradictedClaimSubjectV1 {
                dimension,
                claim_subject_commitment,
            })
        })
        .collect();
    if contradicted_claim_subjects.len() > MAX_CONTRADICTED_SUBJECTS {
        return Err(EvidenceError::TooManyContradictedSubjects);
    }

    let disposition = match (
        identity_conflicts.is_empty(),
        contradicted_claim_subjects.is_empty(),
    ) {
        (true, true) => EvidenceBundleDispositionV1::NoDetectedConflict,
        (false, true) => EvidenceBundleDispositionV1::IdentityConflicted,
        (true, false) => EvidenceBundleDispositionV1::SemanticallyContradicted,
        (false, false) => {
            EvidenceBundleDispositionV1::IdentityConflictedAndSemanticallyContradicted
        }
    };

    let mut bundle = RailCapabilityEvidenceBundleV1 {
        static_profile_commitment: static_profile.profile_commitment(),
        evidence_profile: input.evidence_profile,
        evidence_context_commitment: input.evidence_context_commitment,
        evidence_records: records.into_values().collect(),
        conflicted_evidence_ids: identity_conflicts.into_iter().collect(),
        contradicted_claim_subjects,
        disposition,
        bundle_commitment: Commitment32::from_bytes([0_u8; 32]),
    };
    bundle.bundle_commitment = derive_bundle_commitment(&bundle)?;
    Ok(bundle)
}
