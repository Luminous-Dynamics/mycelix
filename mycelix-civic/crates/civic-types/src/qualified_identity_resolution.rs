// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Public AC-006 authority boundary for entity reconciliation.
//!
//! The low-level matcher is crate-private. External callers must provide the
//! AC-005 result together with the exact versioned ingestion policy that
//! authorized its public/legal-entity identifier bindings.

use serde::{Deserialize, Serialize};

use crate::identity_resolution::{
    EntityIdentityLink, IdentityResolutionContract as InternalIdentityResolutionContract,
    IdentityResolutionError, IdentityResolutionViolation,
};
use crate::standards_ingestion::{StandardsIngestionPolicy, StandardsIngestionResult};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct IdentityResolutionInput {
    pub result: StandardsIngestionResult,
    pub policy: StandardsIngestionPolicy,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum QualifiedIdentityResolutionError {
    MissingPolicyReference { input_index: usize },
    PolicyReferenceMismatch { input_index: usize },
    InvalidPolicyScheme { input_index: usize, scheme: String },
    InvalidSourceEnvelope { input_index: usize },
    BindingSchemeNotAuthorized {
        input_index: usize,
        binding_index: usize,
        scheme: String,
    },
    Reconciliation(IdentityResolutionError),
}

/// Public AC-006 contract.
///
/// This facade deliberately prevents callers from handing arbitrary fabricated
/// `EntityIdentifierBinding` records directly to the proposal engine without
/// also supplying the AC-005 policy that authorized those schemes.
#[derive(Debug, Default, Clone, Copy)]
pub struct IdentityResolutionContract;

impl IdentityResolutionContract {
    pub fn validate_link(
        link: &EntityIdentityLink,
    ) -> Result<(), Vec<IdentityResolutionViolation>> {
        InternalIdentityResolutionContract::validate_link(link)
    }

    pub fn aggregation_eligible(link: &EntityIdentityLink) -> bool {
        InternalIdentityResolutionContract::aggregation_eligible(link)
    }

    pub fn propose_exact_identifier_links(
        inputs: &[IdentityResolutionInput],
    ) -> Result<Vec<EntityIdentityLink>, Vec<QualifiedIdentityResolutionError>> {
        let mut errors = Vec::new();

        for (input_index, input) in inputs.iter().enumerate() {
            if input.policy.policy_ref.trim().is_empty() {
                errors.push(QualifiedIdentityResolutionError::MissingPolicyReference {
                    input_index,
                });
            }
            if input.result.policy_ref != input.policy.policy_ref {
                errors.push(QualifiedIdentityResolutionError::PolicyReferenceMismatch {
                    input_index,
                });
            }
            for scheme in &input.policy.public_entity_identifier_schemes {
                if scheme.is_empty() || scheme.trim() != scheme {
                    errors.push(QualifiedIdentityResolutionError::InvalidPolicyScheme {
                        input_index,
                        scheme: scheme.clone(),
                    });
                }
            }
            if !valid_source_envelope(&input.result) {
                errors.push(QualifiedIdentityResolutionError::InvalidSourceEnvelope {
                    input_index,
                });
            }
            for (binding_index, binding) in input.result.entity_identifiers.iter().enumerate() {
                if !input
                    .policy
                    .public_entity_identifier_schemes
                    .contains(&binding.scheme)
                {
                    errors.push(
                        QualifiedIdentityResolutionError::BindingSchemeNotAuthorized {
                            input_index,
                            binding_index,
                            scheme: binding.scheme.clone(),
                        },
                    );
                }
            }
        }

        if !errors.is_empty() {
            return Err(errors);
        }

        let results: Vec<_> = inputs.iter().map(|input| input.result.clone()).collect();
        InternalIdentityResolutionContract::propose_exact_identifier_links(&results).map_err(
            |internal_errors| {
                internal_errors
                    .into_iter()
                    .map(QualifiedIdentityResolutionError::Reconciliation)
                    .collect()
            },
        )
    }
}

fn valid_source_envelope(result: &StandardsIngestionResult) -> bool {
    let source = &result.source_evidence;
    let valid_hash = source
        .content_hash
        .split_once(':')
        .is_some_and(|(algorithm, digest)| {
            !algorithm.trim().is_empty() && !digest.trim().is_empty()
        });
    !source.source_ref.trim().is_empty()
        && valid_hash
        && !source.validation_receipt_ref.trim().is_empty()
        && !result.policy_ref.trim().is_empty()
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;
    use crate::institutional_graph::{InstitutionalNodeKind, InstitutionalNodeRef};
    use crate::standards_ingestion::{
        EntityIdentifierBinding, ExternalStandard, StandardsIngestionWarning,
        StandardsSourceEvidence,
    };

    fn input(source: &str, node_id: &str, scheme: &str, identifier: &str) -> IdentityResolutionInput {
        let node = InstitutionalNodeRef {
            id: node_id.into(),
            kind: InstitutionalNodeKind::Organization,
        };
        let mut allowed = BTreeSet::new();
        allowed.insert(scheme.into());
        IdentityResolutionInput {
            result: StandardsIngestionResult {
                source_evidence: StandardsSourceEvidence {
                    source_ref: source.into(),
                    content_hash: format!("sha256:{source}"),
                    validation_receipt_ref: format!("receipt:{source}"),
                    ingested_at: 100,
                },
                policy_ref: "policy:v1".into(),
                nodes: vec![node.clone()],
                edges: vec![],
                entity_identifiers: vec![EntityIdentifierBinding {
                    node_id: node.id,
                    scheme: scheme.into(),
                    identifier: identifier.into(),
                    standard: ExternalStandard::Ocds11SchemaRevision115,
                }],
                warnings: Vec::<StandardsIngestionWarning>::new(),
            },
            policy: StandardsIngestionPolicy {
                policy_ref: "policy:v1".into(),
                public_entity_identifier_schemes: allowed,
            },
        }
    }

    #[test]
    fn exact_authorized_bindings_can_reach_the_internal_proposal_engine() {
        let inputs = vec![
            input("source:a", "node:a", "GB-COH", "09506232"),
            input("source:b", "node:b", "GB-COH", "09506232"),
        ];
        let links = IdentityResolutionContract::propose_exact_identifier_links(&inputs)
            .expect("policy-qualified inputs can propose links");
        assert_eq!(links.len(), 1);
        assert_eq!(links[0].status, crate::identity_resolution::IdentityLinkStatus::Proposed);
    }

    #[test]
    fn fabricated_binding_cannot_bypass_identifier_policy() {
        let mut malicious = input("source:a", "node:a", "GB-COH", "09506232");
        malicious.result.entity_identifiers[0].scheme = "PRIVATE-PASSPORT".into();
        let errors = IdentityResolutionContract::propose_exact_identifier_links(&[malicious])
            .expect_err("binding scheme absent from policy must fail closed");
        assert!(errors.iter().any(|error| matches!(
            error,
            QualifiedIdentityResolutionError::BindingSchemeNotAuthorized { .. }
        )));
    }

    #[test]
    fn policy_reference_mismatch_fails_closed() {
        let mut bad = input("source:a", "node:a", "GB-COH", "09506232");
        bad.result.policy_ref = "policy:other".into();
        let errors = IdentityResolutionContract::propose_exact_identifier_links(&[bad])
            .expect_err("result and policy must remain bound");
        assert!(errors.iter().any(|error| matches!(
            error,
            QualifiedIdentityResolutionError::PolicyReferenceMismatch { .. }
        )));
    }

    #[test]
    fn malformed_claimed_digest_is_rejected_before_reconciliation() {
        let mut bad = input("source:a", "node:a", "GB-COH", "09506232");
        bad.result.source_evidence.content_hash = "not-a-digest".into();
        let errors = IdentityResolutionContract::propose_exact_identifier_links(&[bad])
            .expect_err("source envelope must retain digest shape");
        assert!(errors.iter().any(|error| matches!(
            error,
            QualifiedIdentityResolutionError::InvalidSourceEnvelope { .. }
        )));
    }
}
