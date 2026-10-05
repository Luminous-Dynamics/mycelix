//! Runtime-neutral D6X resolution adapter boundary.
//!
//! Status: ReferenceModelOnly.
//!
//! This module describes retrieval evidence without importing Holochain runtime
//! types into semantic closure identity. A runtime adapter may translate
//! ActionHash/EntryHash/ExternalHash (or another address system) into these
//! audit-only forms.

use serde::{Deserialize, Serialize};

fn non_empty(v: &str) -> bool { !v.trim().is_empty() }

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ResolutionAddressKindV1 {
    Action,
    Entry,
    External,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ResolutionAttemptOutcomeV1 {
    Retrieved,
    Unavailable,
    Historical,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResolutionAttemptV1 {
    pub address_kind: ResolutionAddressKindV1,
    pub address: String,
    pub outcome: ResolutionAttemptOutcomeV1,
    pub observed_commitment: Option<String>,
    pub qualification_context_commitment: Option<String>,
}

impl ResolutionAttemptV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.address)
            && self.observed_commitment.as_deref().map_or(true, non_empty)
            && self.qualification_context_commitment.as_deref().map_or(true, non_empty)
            && match self.outcome {
                ResolutionAttemptOutcomeV1::Retrieved =>
                    self.observed_commitment.is_some(),
                ResolutionAttemptOutcomeV1::Unavailable =>
                    // Unavailable means no concrete semantic object was observed;
                    // carrying an observed commitment would contradict the derived
                    // Missing resolution state and create ambiguous audit evidence.
                    self.observed_commitment.is_none(),
                // A historical resolution is still a concrete resolution. It
                // must carry the exact observed commitment so a stale result
                // cannot be represented as an identified object using only a
                // retrieval address. D6X will still map it to Stale and block
                // current qualification; this is an audit/provenance integrity
                // fence, not a currentness upgrade.
                ResolutionAttemptOutcomeV1::Historical =>
                    self.observed_commitment.is_some(),
            }
    }

    /// Maps the runtime retrieval outcome to the D6X semantic resolution state.
    ///
    /// A retrieved observation is only structurally Present here; commitment
    /// equality remains a certificate-level semantic check and is intentionally
    /// not hidden inside the runtime adapter.
    pub fn semantic_resolution(
        &self,
    ) -> crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionV1 {
        match self.outcome {
            ResolutionAttemptOutcomeV1::Retrieved =>
                crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Present,
            ResolutionAttemptOutcomeV1::Unavailable =>
                crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Missing,
            ResolutionAttemptOutcomeV1::Historical =>
                crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Stale,
        }
    }

    /// Converts this runtime-neutral attempt into the opaque evidence payload
    /// used by D6X. The address domain and outcome are preserved by the
    /// envelope below; the legacy evidence payload intentionally remains
    /// semantic-identity-neutral.
    pub fn to_resolution_evidence(
        &self,
    ) -> crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionEvidenceV1 {
        crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionEvidenceV1 {
            retrieval_reference: Some(self.address.clone()),
            observed_commitment: self.observed_commitment.clone(),
            qualification_context_commitment: self.qualification_context_commitment.clone(),
        }
    }
}

/// Lossless audit envelope for runtime resolution attempts.
///
/// The legacy D6X evidence struct is deliberately opaque and does not encode
/// address domain or retrieval outcome. This envelope keeps those fields
/// alongside the derived D6X evidence so an adapter can round-trip the runtime
/// attempt without changing closure_identity_commitment.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResolutionEvidenceEnvelopeV1 {
    pub attempt: ResolutionAttemptV1,
    pub evidence:
        crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionEvidenceV1,
}

impl ResolutionEvidenceEnvelopeV1 {
    pub fn from_attempt(attempt: ResolutionAttemptV1) -> Option<Self> {
        if !attempt.structurally_valid() {
            return None;
        }
        let evidence = attempt.to_resolution_evidence();
        Some(Self { attempt, evidence })
    }

    /// Validates that the envelope has not drifted from its source attempt.
    ///
    /// This protects the audit boundary from carrying contradictory typed and
    /// legacy evidence representations. It does not validate semantic
    /// commitment equality; that remains a D6X certificate invariant.
    pub fn structurally_valid(&self) -> bool {
        self.attempt.structurally_valid()
            && self.evidence == self.attempt.to_resolution_evidence()
    }

    pub fn to_attempt(&self) -> ResolutionAttemptV1 {
        self.attempt.clone()
    }

    pub fn to_resolution_evidence(
        &self,
    ) -> &crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionEvidenceV1 {
        &self.evidence
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retrieved_attempt_requires_an_observation() {
        let attempt = ResolutionAttemptV1 {
            address_kind: ResolutionAddressKindV1::Entry,
            address: "entry-address".into(),
            outcome: ResolutionAttemptOutcomeV1::Retrieved,
            observed_commitment: None,
            qualification_context_commitment: None,
        };
        assert!(!attempt.structurally_valid());
    }

    #[test]
    #[test]
    fn unavailable_attempt_rejects_an_observed_commitment() {
        let attempt = ResolutionAttemptV1 {
            address_kind: ResolutionAddressKindV1::Entry,
            address: "entry-address".into(),
            outcome: ResolutionAttemptOutcomeV1::Unavailable,
            observed_commitment: Some("unexpected-observation".into()),
            qualification_context_commitment: None,
        };
        assert!(!attempt.structurally_valid());
    }

    fn unavailable_attempt_can_record_retrieval_address_without_observation() {
        let attempt = ResolutionAttemptV1 {
            address_kind: ResolutionAddressKindV1::Action,
            address: "action-address".into(),
            outcome: ResolutionAttemptOutcomeV1::Unavailable,
            observed_commitment: None,
            qualification_context_commitment: None,
        };
        assert!(attempt.structurally_valid());
    }

    #[test]
    fn runtime_attempt_round_trip_preserves_address_domain_and_outcome() {
        for (address_kind, outcome, observed) in [
            (
                ResolutionAddressKindV1::Action,
                ResolutionAttemptOutcomeV1::Retrieved,
                Some("commitment"),
            ),
            (
                ResolutionAddressKindV1::Entry,
                ResolutionAttemptOutcomeV1::Unavailable,
                None,
            ),
            (
                ResolutionAddressKindV1::External,
                ResolutionAttemptOutcomeV1::Historical,
                Some("historical-commitment"),
            ),
        ] {
            let attempt = ResolutionAttemptV1 {
                address_kind,
                address: "runtime-address".into(),
                outcome,
                observed_commitment: observed.map(str::to_owned),
                qualification_context_commitment: Some("qualification-context".into()),
            };
            let envelope = ResolutionEvidenceEnvelopeV1::from_attempt(attempt.clone())
                .expect("valid runtime attempt should envelope");
            assert_eq!(envelope.to_attempt(), attempt);
            assert_eq!(
                envelope.to_resolution_evidence(),
                &attempt.to_resolution_evidence()
            );
        }
    }

    #[test]
    fn runtime_outcome_maps_monotonically_to_d6x_resolution() {
        let retrieved = ResolutionAttemptV1 {
            address_kind: ResolutionAddressKindV1::Action,
            address: "action-address".into(),
            outcome: ResolutionAttemptOutcomeV1::Retrieved,
            observed_commitment: Some("commitment".into()),
            qualification_context_commitment: None,
        };
        let unavailable = ResolutionAttemptV1 {
            address_kind: ResolutionAddressKindV1::Entry,
            address: "entry-address".into(),
            outcome: ResolutionAttemptOutcomeV1::Unavailable,
            observed_commitment: None,
            qualification_context_commitment: None,
        };
        let historical = ResolutionAttemptV1 {
            address_kind: ResolutionAddressKindV1::External,
            address: "external-address".into(),
            outcome: ResolutionAttemptOutcomeV1::Historical,
            observed_commitment: Some("historical-commitment".into()),
            qualification_context_commitment: None,
        };

        assert_eq!(
            retrieved.semantic_resolution(),
            crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Present
        );
        assert_eq!(
            unavailable.semantic_resolution(),
            crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Missing
        );
        assert_eq!(
            historical.semantic_resolution(),
            crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Stale
        );
    }

    #[test]
    fn historical_attempt_requires_an_observed_commitment() {
        let attempt = ResolutionAttemptV1 {
            address_kind: ResolutionAddressKindV1::External,
            address: "historical-address".into(),
            outcome: ResolutionAttemptOutcomeV1::Historical,
            observed_commitment: None,
            qualification_context_commitment: None,
        };
        assert!(!attempt.structurally_valid());

        let valid = ResolutionAttemptV1 {
            observed_commitment: Some("historical-commitment".into()),
            ..attempt
        };
        assert!(valid.structurally_valid());
        assert_eq!(
            valid.semantic_resolution(),
            crate::qualified_dependency_closure_d6x::SemanticDependencyResolutionV1::Stale
        );
    }

    #[test]
    fn envelope_rejects_drift_between_typed_attempt_and_legacy_evidence() {
        let attempt = ResolutionAttemptV1 {
            address_kind: ResolutionAddressKindV1::Action,
            address: "action-address".into(),
            outcome: ResolutionAttemptOutcomeV1::Retrieved,
            observed_commitment: Some("commitment".into()),
            qualification_context_commitment: None,
        };
        let mut envelope = ResolutionEvidenceEnvelopeV1::from_attempt(attempt).unwrap();
        envelope.evidence.observed_commitment = Some("different".into());
        assert!(!envelope.structurally_valid());
    }

    #[test]
    fn invalid_runtime_attempt_cannot_enter_lossless_envelope() {
        let attempt = ResolutionAttemptV1 {
            address_kind: ResolutionAddressKindV1::Entry,
            address: "entry-address".into(),
            outcome: ResolutionAttemptOutcomeV1::Retrieved,
            observed_commitment: None,
            qualification_context_commitment: None,
        };
        assert!(ResolutionEvidenceEnvelopeV1::from_attempt(attempt).is_none());
    }

    #[test]
    fn external_address_remains_runtime_evidence_not_semantic_identity() {
        let attempt = ResolutionAttemptV1 {
            address_kind: ResolutionAddressKindV1::External,
            address: "external-address".into(),
            outcome: ResolutionAttemptOutcomeV1::Retrieved,
            observed_commitment: Some("commitment".into()),
            qualification_context_commitment: None,
        };
        let evidence = attempt.to_resolution_evidence();
        assert_eq!(evidence.retrieval_reference.as_deref(), Some("external-address"));
        assert_eq!(evidence.observed_commitment.as_deref(), Some("commitment"));
    }
}
