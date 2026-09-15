// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-006 reversible public-entity identity reconciliation.
//!
//! AC-006 links institutional nodes without rewriting, deleting, or merging their
//! original identities. Exact allowlisted identifiers can create proposals;
//! aggregation eligibility requires explicit review plus authoritative evidence.
//! Natural-person credentials are never eligible for this reconciliation path.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::capture_observation::ProvenanceRef;
use crate::institutional_graph::{InstitutionalNodeKind, InstitutionalNodeRef};
use crate::standards_ingestion::{
    EntityIdentifierBinding, ExternalStandard, StandardsIngestionResult,
};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct EntityBindingEvidence {
    pub node: InstitutionalNodeRef,
    pub scheme: String,
    pub identifier: String,
    pub standard: ExternalStandard,
    pub source_ref: String,
    pub content_hash: String,
    pub validation_receipt_ref: String,
    pub policy_ref: String,
    pub observed_at: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum IdentityVerificationKind {
    AuthoritativeRegistryLookup {
        scheme: String,
        identifier: String,
        registry_ref: String,
    },
    AuthoritativeCrosswalk {
        left_scheme: String,
        left_identifier: String,
        right_scheme: String,
        right_identifier: String,
        crosswalk_ref: String,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct IdentityVerification {
    pub verification_ref: String,
    pub verifier_ref: String,
    pub kind: IdentityVerificationKind,
    pub provenance: Vec<ProvenanceRef>,
    pub verified_at: u64,
}

/// `Corroborated` means qualified for institutional aggregation under AC-006.
/// It is not a legal adjudication and never replaces the source node identities.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum IdentityLinkStatus {
    Proposed,
    Corroborated,
    Challenged,
    Rejected,
    Superseded,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct EntityIdentityLink {
    pub id: String,
    pub left: InstitutionalNodeRef,
    pub right: InstitutionalNodeRef,
    pub status: IdentityLinkStatus,
    pub binding_evidence: Vec<EntityBindingEvidence>,
    pub verification_evidence: Vec<IdentityVerification>,
    pub challenge_refs: Vec<String>,
    pub review_ref: Option<String>,
    pub review_rationale: Option<String>,
    pub superseded_by: Option<String>,
    pub reversible: bool,
    pub recorded_at: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum IdentityResolutionViolation {
    MissingLinkId,
    MissingNodeId,
    SameEndpoint,
    IneligibleNodeKind,
    NonReversibleIdentityLink,
    MissingBindingEvidence,
    InvalidBindingEvidence,
    BindingEvidenceOutsideEndpoints,
    BindingEvidenceMissingEndpoint,
    CorroborationRequiresDistinctSources,
    CorroborationRequiresAuthoritativeVerification,
    AuthoritativeVerificationDoesNotSupportEndpoints,
    InvalidVerificationEvidence,
    CorroborationRequiresReview,
    CorroboratedLinkHasUnresolvedChallenge,
    ProposedLinkHasChallengeReference,
    ChallengedLinkRequiresChallengeReference,
    InvalidChallengeReference,
    TerminalStatusRequiresReview,
    SupersededStatusRequiresReplacementReference,
    SupersededBySelf,
    UnexpectedReplacementReference,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum IdentityResolutionError {
    InvalidResultEnvelope { result_index: usize },
    BindingReferencesUnknownNode {
        result_index: usize,
        binding_index: usize,
    },
    BindingTargetsIneligibleNode {
        result_index: usize,
        binding_index: usize,
    },
    NodeKindConflict { node_id: String },
    IdentifierCollisionWithinSource {
        result_index: usize,
        scheme: String,
    },
}

#[derive(Debug, Default, Clone, Copy)]
pub struct IdentityResolutionContract;

impl IdentityResolutionContract {
    pub fn validate_link(
        link: &EntityIdentityLink,
    ) -> Result<(), Vec<IdentityResolutionViolation>> {
        let mut violations = Vec::new();

        if link.id.trim().is_empty() {
            violations.push(IdentityResolutionViolation::MissingLinkId);
        }
        for node in [&link.left, &link.right] {
            if node.id.trim().is_empty() {
                violations.push(IdentityResolutionViolation::MissingNodeId);
            }
            if !eligible_entity_kind(&node.kind) {
                violations.push(IdentityResolutionViolation::IneligibleNodeKind);
            }
        }
        if link.left.id == link.right.id {
            violations.push(IdentityResolutionViolation::SameEndpoint);
        }
        if !link.reversible {
            violations.push(IdentityResolutionViolation::NonReversibleIdentityLink);
        }

        validate_binding_evidence(link, &mut violations);
        for verification in &link.verification_evidence {
            validate_verification(verification, &mut violations);
        }

        match &link.status {
            IdentityLinkStatus::Proposed => {
                if !link.challenge_refs.is_empty() {
                    violations.push(IdentityResolutionViolation::ProposedLinkHasChallengeReference);
                }
                if link.superseded_by.is_some() {
                    violations.push(IdentityResolutionViolation::UnexpectedReplacementReference);
                }
            }
            IdentityLinkStatus::Corroborated => {
                let distinct_sources: BTreeSet<_> = link
                    .binding_evidence
                    .iter()
                    .map(|evidence| evidence.source_ref.as_str())
                    .collect();
                if distinct_sources.len() < 2 {
                    violations.push(
                        IdentityResolutionViolation::CorroborationRequiresDistinctSources,
                    );
                }
                if link.verification_evidence.is_empty() {
                    violations.push(
                        IdentityResolutionViolation::CorroborationRequiresAuthoritativeVerification,
                    );
                } else if !link
                    .verification_evidence
                    .iter()
                    .any(|verification| verification_supports_link(link, verification))
                {
                    violations.push(
                        IdentityResolutionViolation::AuthoritativeVerificationDoesNotSupportEndpoints,
                    );
                }
                if !present(link.review_ref.as_deref()) || !present(link.review_rationale.as_deref()) {
                    violations.push(IdentityResolutionViolation::CorroborationRequiresReview);
                }
                if !link.challenge_refs.is_empty() {
                    violations.push(
                        IdentityResolutionViolation::CorroboratedLinkHasUnresolvedChallenge,
                    );
                }
                if link.superseded_by.is_some() {
                    violations.push(IdentityResolutionViolation::UnexpectedReplacementReference);
                }
            }
            IdentityLinkStatus::Challenged => {
                if link.challenge_refs.is_empty()
                    || link.challenge_refs.iter().any(|reference| reference.trim().is_empty())
                {
                    violations.push(
                        IdentityResolutionViolation::ChallengedLinkRequiresChallengeReference,
                    );
                }
                if link.superseded_by.is_some() {
                    violations.push(IdentityResolutionViolation::UnexpectedReplacementReference);
                }
            }
            IdentityLinkStatus::Rejected => {
                require_terminal_review(link, &mut violations);
                if link.superseded_by.is_some() {
                    violations.push(IdentityResolutionViolation::UnexpectedReplacementReference);
                }
            }
            IdentityLinkStatus::Superseded => {
                require_terminal_review(link, &mut violations);
                if !present(link.superseded_by.as_deref()) {
                    violations.push(
                        IdentityResolutionViolation::SupersededStatusRequiresReplacementReference,
                    );
                } else if link.superseded_by.as_deref() == Some(link.id.as_str()) {
                    violations.push(IdentityResolutionViolation::SupersededBySelf);
                }
            }
        }

        if link.challenge_refs.iter().any(|reference| reference.trim().is_empty()) {
            violations.push(IdentityResolutionViolation::InvalidChallengeReference);
        }

        if violations.is_empty() {
            Ok(())
        } else {
            Err(violations)
        }
    }

    pub fn aggregation_eligible(link: &EntityIdentityLink) -> bool {
        link.status == IdentityLinkStatus::Corroborated && Self::validate_link(link).is_ok()
    }

    /// Exact identifier matching creates proposals only. There is intentionally
    /// no fuzzy/name-matching API in the authoritative AC-006 path.
    pub fn propose_exact_identifier_links(
        results: &[StandardsIngestionResult],
    ) -> Result<Vec<EntityIdentityLink>, Vec<IdentityResolutionError>> {
        let mut errors = Vec::new();
        let mut global_nodes = BTreeMap::<String, InstitutionalNodeRef>::new();
        let mut grouped = BTreeMap::<(String, String), Vec<EntityBindingEvidence>>::new();

        for (result_index, result) in results.iter().enumerate() {
            if !valid_result_envelope(result) {
                errors.push(IdentityResolutionError::InvalidResultEnvelope { result_index });
                continue;
            }

            let local_nodes: BTreeMap<_, _> = result
                .nodes
                .iter()
                .map(|node| (node.id.clone(), node.clone()))
                .collect();
            for node in &result.nodes {
                match global_nodes.get(&node.id) {
                    Some(existing) if existing.kind != node.kind => {
                        errors.push(IdentityResolutionError::NodeKindConflict {
                            node_id: node.id.clone(),
                        });
                    }
                    _ => {
                        global_nodes.entry(node.id.clone()).or_insert_with(|| node.clone());
                    }
                }
            }

            let mut local_identifier_nodes = BTreeMap::<(String, String), BTreeSet<String>>::new();
            for (binding_index, binding) in result.entity_identifiers.iter().enumerate() {
                let Some(node) = local_nodes.get(&binding.node_id) else {
                    errors.push(IdentityResolutionError::BindingReferencesUnknownNode {
                        result_index,
                        binding_index,
                    });
                    continue;
                };
                if !eligible_entity_kind(&node.kind) {
                    errors.push(IdentityResolutionError::BindingTargetsIneligibleNode {
                        result_index,
                        binding_index,
                    });
                    continue;
                }
                if binding.scheme.trim().is_empty() || binding.identifier.trim().is_empty() {
                    errors.push(IdentityResolutionError::InvalidResultEnvelope { result_index });
                    continue;
                }

                let key = (binding.scheme.clone(), binding.identifier.clone());
                local_identifier_nodes
                    .entry(key.clone())
                    .or_default()
                    .insert(node.id.clone());
                grouped.entry(key).or_default().push(binding_evidence(
                    result,
                    binding,
                    node.clone(),
                ));
            }

            for ((scheme, _), node_ids) in local_identifier_nodes {
                if node_ids.len() > 1 {
                    errors.push(IdentityResolutionError::IdentifierCollisionWithinSource {
                        result_index,
                        scheme,
                    });
                }
            }
        }

        if !errors.is_empty() {
            return Err(errors);
        }

        let mut links = Vec::new();
        for ((scheme, identifier), mut evidence) in grouped {
            evidence.sort_by(|a, b| {
                (&a.node.id, &a.source_ref, &a.content_hash)
                    .cmp(&(&b.node.id, &b.source_ref, &b.content_hash))
            });
            evidence.dedup_by(|a, b| {
                a.node.id == b.node.id
                    && a.source_ref == b.source_ref
                    && a.content_hash == b.content_hash
            });

            let source_refs: BTreeSet<_> = evidence
                .iter()
                .map(|item| item.source_ref.clone())
                .collect();
            if source_refs.len() < 2 {
                continue;
            }
            let node_ids: Vec<_> = evidence
                .iter()
                .map(|item| item.node.id.clone())
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect();
            if node_ids.len() < 2 {
                continue;
            }

            for left_index in 0..node_ids.len() {
                for right_index in (left_index + 1)..node_ids.len() {
                    let left_id = &node_ids[left_index];
                    let right_id = &node_ids[right_index];
                    let pair_evidence: Vec<_> = evidence
                        .iter()
                        .filter(|item| item.node.id == *left_id || item.node.id == *right_id)
                        .cloned()
                        .collect();
                    let pair_sources: BTreeSet<_> = pair_evidence
                        .iter()
                        .map(|item| item.source_ref.as_str())
                        .collect();
                    if pair_sources.len() < 2 {
                        continue;
                    }
                    let left = global_nodes.get(left_id).expect("indexed node").clone();
                    let right = global_nodes.get(right_id).expect("indexed node").clone();
                    let recorded_at = pair_evidence
                        .iter()
                        .map(|item| item.observed_at)
                        .max()
                        .unwrap_or_default();
                    links.push(EntityIdentityLink {
                        id: stable_key(
                            "entity-identity-proposal",
                            &[&scheme, &identifier, left_id, right_id],
                        ),
                        left,
                        right,
                        status: IdentityLinkStatus::Proposed,
                        binding_evidence: pair_evidence,
                        verification_evidence: vec![],
                        challenge_refs: vec![],
                        review_ref: None,
                        review_rationale: None,
                        superseded_by: None,
                        reversible: true,
                        recorded_at,
                    });
                }
            }
        }
        links.sort_by(|a, b| a.id.cmp(&b.id));
        Ok(links)
    }
}

fn eligible_entity_kind(kind: &InstitutionalNodeKind) -> bool {
    matches!(kind, InstitutionalNodeKind::Organization | InstitutionalNodeKind::LegalEntity)
}

fn validate_binding_evidence(
    link: &EntityIdentityLink,
    violations: &mut Vec<IdentityResolutionViolation>,
) {
    if link.binding_evidence.is_empty() {
        violations.push(IdentityResolutionViolation::MissingBindingEvidence);
        return;
    }
    let mut left_present = false;
    let mut right_present = false;
    for evidence in &link.binding_evidence {
        if evidence.node.id == link.left.id {
            left_present = true;
            if evidence.node.kind != link.left.kind {
                violations.push(IdentityResolutionViolation::InvalidBindingEvidence);
            }
        } else if evidence.node.id == link.right.id {
            right_present = true;
            if evidence.node.kind != link.right.kind {
                violations.push(IdentityResolutionViolation::InvalidBindingEvidence);
            }
        } else {
            violations.push(IdentityResolutionViolation::BindingEvidenceOutsideEndpoints);
        }
        if evidence.node.id.trim().is_empty()
            || !eligible_entity_kind(&evidence.node.kind)
            || evidence.scheme.trim().is_empty()
            || evidence.identifier.trim().is_empty()
            || evidence.source_ref.trim().is_empty()
            || evidence.content_hash.trim().is_empty()
            || evidence.validation_receipt_ref.trim().is_empty()
            || evidence.policy_ref.trim().is_empty()
        {
            violations.push(IdentityResolutionViolation::InvalidBindingEvidence);
        }
    }
    if !left_present || !right_present {
        violations.push(IdentityResolutionViolation::BindingEvidenceMissingEndpoint);
    }
}

fn validate_verification(
    verification: &IdentityVerification,
    violations: &mut Vec<IdentityResolutionViolation>,
) {
    let references_valid = !verification.verification_ref.trim().is_empty()
        && !verification.verifier_ref.trim().is_empty()
        && !verification.provenance.is_empty()
        && verification.provenance.iter().all(|source| {
            !source.source_ref.trim().is_empty()
                && source
                    .content_hash
                    .as_deref()
                    .is_some_and(|hash| !hash.trim().is_empty())
        });
    let kind_valid = match &verification.kind {
        IdentityVerificationKind::AuthoritativeRegistryLookup {
            scheme,
            identifier,
            registry_ref,
        } => !scheme.trim().is_empty()
            && !identifier.trim().is_empty()
            && !registry_ref.trim().is_empty(),
        IdentityVerificationKind::AuthoritativeCrosswalk {
            left_scheme,
            left_identifier,
            right_scheme,
            right_identifier,
            crosswalk_ref,
        } => !left_scheme.trim().is_empty()
            && !left_identifier.trim().is_empty()
            && !right_scheme.trim().is_empty()
            && !right_identifier.trim().is_empty()
            && !crosswalk_ref.trim().is_empty(),
    };
    if !references_valid || !kind_valid {
        violations.push(IdentityResolutionViolation::InvalidVerificationEvidence);
    }
}

fn verification_supports_link(
    link: &EntityIdentityLink,
    verification: &IdentityVerification,
) -> bool {
    match &verification.kind {
        IdentityVerificationKind::AuthoritativeRegistryLookup {
            scheme,
            identifier,
            ..
        } => endpoint_has_identifier(link, &link.left.id, scheme, identifier)
            && endpoint_has_identifier(link, &link.right.id, scheme, identifier),
        IdentityVerificationKind::AuthoritativeCrosswalk {
            left_scheme,
            left_identifier,
            right_scheme,
            right_identifier,
            ..
        } => {
            let forward = endpoint_has_identifier(
                link,
                &link.left.id,
                left_scheme,
                left_identifier,
            ) && endpoint_has_identifier(
                link,
                &link.right.id,
                right_scheme,
                right_identifier,
            );
            let reverse = endpoint_has_identifier(
                link,
                &link.left.id,
                right_scheme,
                right_identifier,
            ) && endpoint_has_identifier(
                link,
                &link.right.id,
                left_scheme,
                left_identifier,
            );
            forward || reverse
        }
    }
}

fn endpoint_has_identifier(
    link: &EntityIdentityLink,
    node_id: &str,
    scheme: &str,
    identifier: &str,
) -> bool {
    link.binding_evidence.iter().any(|evidence| {
        evidence.node.id == node_id
            && evidence.scheme == scheme
            && evidence.identifier == identifier
    })
}

fn require_terminal_review(
    link: &EntityIdentityLink,
    violations: &mut Vec<IdentityResolutionViolation>,
) {
    if !present(link.review_ref.as_deref()) || !present(link.review_rationale.as_deref()) {
        violations.push(IdentityResolutionViolation::TerminalStatusRequiresReview);
    }
}

fn present(value: Option<&str>) -> bool {
    value.is_some_and(|value| !value.trim().is_empty())
}

fn valid_result_envelope(result: &StandardsIngestionResult) -> bool {
    !result.source_evidence.source_ref.trim().is_empty()
        && !result.source_evidence.content_hash.trim().is_empty()
        && !result
            .source_evidence
            .validation_receipt_ref
            .trim()
            .is_empty()
        && !result.policy_ref.trim().is_empty()
}

fn binding_evidence(
    result: &StandardsIngestionResult,
    binding: &EntityIdentifierBinding,
    node: InstitutionalNodeRef,
) -> EntityBindingEvidence {
    EntityBindingEvidence {
        node,
        scheme: binding.scheme.clone(),
        identifier: binding.identifier.clone(),
        standard: binding.standard.clone(),
        source_ref: result.source_evidence.source_ref.clone(),
        content_hash: result.source_evidence.content_hash.clone(),
        validation_receipt_ref: result.source_evidence.validation_receipt_ref.clone(),
        policy_ref: result.policy_ref.clone(),
        observed_at: result.source_evidence.ingested_at,
    }
}

fn stable_key(namespace: &str, parts: &[&str]) -> String {
    let mut output = format!("{namespace}|");
    for part in parts {
        output.push_str(&format!("{}:{}|", part.len(), part));
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::standards_ingestion::{StandardsIngestionWarning, StandardsSourceEvidence};

    fn result(
        source_ref: &str,
        hash: &str,
        node_id: &str,
        kind: InstitutionalNodeKind,
        scheme: &str,
        identifier: &str,
    ) -> StandardsIngestionResult {
        let node = InstitutionalNodeRef {
            id: node_id.into(),
            kind,
        };
        StandardsIngestionResult {
            source_evidence: StandardsSourceEvidence {
                source_ref: source_ref.into(),
                content_hash: hash.into(),
                validation_receipt_ref: format!("receipt:{hash}"),
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
        }
    }

    fn exact_results() -> Vec<StandardsIngestionResult> {
        vec![
            result(
                "source:a",
                "sha256:a",
                "node:a",
                InstitutionalNodeKind::Organization,
                "GB-COH",
                "09506232",
            ),
            result(
                "source:b",
                "sha256:b",
                "node:b",
                InstitutionalNodeKind::LegalEntity,
                "GB-COH",
                "09506232",
            ),
        ]
    }

    #[test]
    fn exact_identifier_matching_only_proposes_reversible_links() {
        let links = IdentityResolutionContract::propose_exact_identifier_links(&exact_results())
            .expect("exact public identifier can propose a link");
        assert_eq!(links.len(), 1);
        let link = &links[0];
        assert_eq!(link.status, IdentityLinkStatus::Proposed);
        assert!(link.reversible);
        assert!(!IdentityResolutionContract::aggregation_eligible(link));
        assert_eq!(IdentityResolutionContract::validate_link(link), Ok(()));
    }

    #[test]
    fn duplicate_identifier_inside_one_source_fails_closed() {
        let mut first = exact_results().remove(0);
        first.nodes.push(InstitutionalNodeRef {
            id: "node:b".into(),
            kind: InstitutionalNodeKind::Organization,
        });
        first.entity_identifiers.push(EntityIdentifierBinding {
            node_id: "node:b".into(),
            scheme: "GB-COH".into(),
            identifier: "09506232".into(),
            standard: ExternalStandard::Ocds11SchemaRevision115,
        });
        let errors = IdentityResolutionContract::propose_exact_identifier_links(&[first])
            .expect_err("same source assigning one ID to two nodes is ambiguous");
        assert!(errors.iter().any(|error| matches!(
            error,
            IdentityResolutionError::IdentifierCollisionWithinSource { .. }
        )));
    }

    #[test]
    fn private_person_binding_is_never_eligible() {
        let private = result(
            "source:a",
            "sha256:a",
            "person:a",
            InstitutionalNodeKind::PrivatePersonCredential,
            "PRIVATE",
            "secret",
        );
        let errors = IdentityResolutionContract::propose_exact_identifier_links(&[private])
            .expect_err("natural-person identity is outside AC-006");
        assert!(errors.iter().any(|error| matches!(
            error,
            IdentityResolutionError::BindingTargetsIneligibleNode { .. }
        )));
    }

    #[test]
    fn corroboration_cannot_self_promote() {
        let mut links = IdentityResolutionContract::propose_exact_identifier_links(&exact_results())
            .unwrap();
        let mut link = links.remove(0);
        link.status = IdentityLinkStatus::Corroborated;
        let errors = IdentityResolutionContract::validate_link(&link)
            .expect_err("proposal cannot self-promote");
        assert!(errors.contains(
            &IdentityResolutionViolation::CorroborationRequiresAuthoritativeVerification
        ));
        assert!(errors.contains(&IdentityResolutionViolation::CorroborationRequiresReview));
    }

    #[test]
    fn reviewed_registry_verification_can_enable_aggregation() {
        let mut links = IdentityResolutionContract::propose_exact_identifier_links(&exact_results())
            .unwrap();
        let mut link = links.remove(0);
        link.status = IdentityLinkStatus::Corroborated;
        link.review_ref = Some("review:identity:1".into());
        link.review_rationale = Some("exact public registry identifier independently verified".into());
        link.verification_evidence.push(IdentityVerification {
            verification_ref: "verification:registry:1".into(),
            verifier_ref: "verifier:independent:1".into(),
            kind: IdentityVerificationKind::AuthoritativeRegistryLookup {
                scheme: "GB-COH".into(),
                identifier: "09506232".into(),
                registry_ref: "registry:companies-house".into(),
            },
            provenance: vec![ProvenanceRef {
                source_ref: "registry:companies-house:record".into(),
                content_hash: Some("sha256:registry".into()),
            }],
            verified_at: 200,
        });
        assert_eq!(IdentityResolutionContract::validate_link(&link), Ok(()));
        assert!(IdentityResolutionContract::aggregation_eligible(&link));

        link.challenge_refs = vec!["challenge:1".into()];
        assert!(!IdentityResolutionContract::aggregation_eligible(&link));
        assert!(IdentityResolutionContract::validate_link(&link)
            .expect_err("unresolved challenge must revoke corroborated usability")
            .contains(&IdentityResolutionViolation::CorroboratedLinkHasUnresolvedChallenge));
    }

    #[test]
    fn challenged_link_preserves_evidence_but_cannot_aggregate() {
        let mut links = IdentityResolutionContract::propose_exact_identifier_links(&exact_results())
            .unwrap();
        let mut link = links.remove(0);
        let evidence_len = link.binding_evidence.len();
        link.status = IdentityLinkStatus::Challenged;
        link.challenge_refs = vec!["challenge:1".into()];
        assert_eq!(IdentityResolutionContract::validate_link(&link), Ok(()));
        assert!(!IdentityResolutionContract::aggregation_eligible(&link));
        assert_eq!(link.binding_evidence.len(), evidence_len);
    }
}
