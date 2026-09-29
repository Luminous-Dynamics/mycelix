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
                ResolutionAttemptOutcomeV1::Unavailable |
                ResolutionAttemptOutcomeV1::Historical =>
                    true,
            }
    }

    /// Converts this runtime-neutral attempt into the opaque evidence payload
    /// used by D6X. The address is intentionally retained as audit evidence,
    /// never semantic dependency identity.
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
