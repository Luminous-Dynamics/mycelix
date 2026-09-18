use std::collections::BTreeSet;

use mycelix_business_core::ReferenceId;

use crate::{FinalityProfile, FinalityProfileError, ReversalModel};

/// Named construction input for the registered v1 finality profile.
///
/// This type is an API misuse-resistance boundary, not a new canonical format.
/// It is never serialized by the settlement commitment code and it does not
/// accept a caller-supplied profile digest. Conversion validates the same v1
/// shape rules and derives the digest through the existing `FinalityProfile`
/// constructor.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinalityProfileDraftV1 {
    pub id: ReferenceId,
    pub revision: u64,
    pub rail: ReferenceId,
    pub network: ReferenceId,
    pub required_evidence_kinds: BTreeSet<ReferenceId>,
    pub min_distinct_sources: u16,
    pub max_observation_age_ms: u64,
    pub reversal_model: ReversalModel,
}

impl FinalityProfile {
    /// Construct the registered v1 profile from explicit named inputs.
    ///
    /// Equivalent values produce the exact same canonical profile bytes and
    /// digest as the legacy positional v1 constructor.
    pub fn try_from_draft(draft: FinalityProfileDraftV1) -> Result<Self, FinalityProfileError> {
        draft.try_into()
    }
}

impl TryFrom<FinalityProfileDraftV1> for FinalityProfile {
    type Error = FinalityProfileError;

    fn try_from(draft: FinalityProfileDraftV1) -> Result<Self, Self::Error> {
        Self::new(
            draft.id,
            draft.revision,
            draft.rail,
            draft.network,
            draft.required_evidence_kinds,
            draft.min_distinct_sources,
            draft.max_observation_age_ms,
            draft.reversal_model,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{canonical_finality_profile_bytes, finality_profile_commitment};

    fn reference(value: &str) -> ReferenceId {
        ReferenceId::new(value).expect("static reference")
    }

    fn draft() -> FinalityProfileDraftV1 {
        FinalityProfileDraftV1 {
            id: reference("finality:bank:v1"),
            revision: 1,
            rail: reference("rail:bank"),
            network: reference("network:test-bank"),
            required_evidence_kinds: BTreeSet::from([reference("evidence:provider-settlement")]),
            min_distinct_sources: 1,
            max_observation_age_ms: 60_000,
            reversal_model: ReversalModel::MayReverse,
        }
    }

    #[test]
    fn named_draft_is_byte_identical_to_legacy_v1_constructor() {
        let input = draft();
        let legacy = FinalityProfile::new(
            input.id.clone(),
            input.revision,
            input.rail.clone(),
            input.network.clone(),
            input.required_evidence_kinds.clone(),
            input.min_distinct_sources,
            input.max_observation_age_ms,
            input.reversal_model,
        )
        .expect("legacy v1 profile");
        let named = FinalityProfile::try_from_draft(input).expect("named v1 profile");

        assert_eq!(
            canonical_finality_profile_bytes(&named).expect("named canonical bytes"),
            canonical_finality_profile_bytes(&legacy).expect("legacy canonical bytes")
        );
        assert_eq!(named.profile_ref(), legacy.profile_ref());
        assert_eq!(
            finality_profile_commitment(&named).expect("named commitment"),
            finality_profile_commitment(&legacy).expect("legacy commitment")
        );
    }

    #[test]
    fn named_rail_and_network_fields_remain_commitment_significant() {
        let original = FinalityProfile::try_from_draft(draft()).expect("original profile");
        let mut swapped = draft();
        std::mem::swap(&mut swapped.rail, &mut swapped.network);
        let swapped = FinalityProfile::try_from_draft(swapped).expect("swapped profile");

        assert_ne!(original.profile_ref().digest, swapped.profile_ref().digest);
    }

    #[test]
    fn named_draft_reuses_registered_v1_validation() {
        let mut zero_revision = draft();
        zero_revision.revision = 0;
        assert_eq!(
            FinalityProfile::try_from_draft(zero_revision),
            Err(FinalityProfileError::ZeroRevision)
        );

        let mut no_kinds = draft();
        no_kinds.required_evidence_kinds.clear();
        assert_eq!(
            FinalityProfile::try_from_draft(no_kinds),
            Err(FinalityProfileError::NoEvidenceRequirements)
        );

        let mut zero_sources = draft();
        zero_sources.min_distinct_sources = 0;
        assert_eq!(
            FinalityProfile::try_from_draft(zero_sources),
            Err(FinalityProfileError::ZeroDistinctSources)
        );

        let mut zero_age = draft();
        zero_age.max_observation_age_ms = 0;
        assert_eq!(
            FinalityProfile::try_from_draft(zero_age),
            Err(FinalityProfileError::ZeroObservationAge)
        );
    }
}
