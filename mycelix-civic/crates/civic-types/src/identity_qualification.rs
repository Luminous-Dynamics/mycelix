// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-008 trust-rooted public identity-equivalence boundary.
//!
//! AC-006 and AC-007 provide structural evidence and reversible projection engines.
//! AC-008 is the public deployment boundary: every identity link must carry a
//! time-bounded qualification receipt and an injected verifier must validate that
//! receipt against the exact link before the internal equivalence engine is called.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::capture_metrics::ObservationContext;
use crate::capture_observation::ProvenanceRef;
use crate::equivalence_view::{
    EquivalenceViewContract, EquivalenceViewError, IdentityProjectedObservation,
    QualifiedEquivalenceView,
};
use crate::identity_resolution::EntityIdentityLink;
use crate::institutional_graph::InstitutionalEdge;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct IdentityQualificationReceipt {
    pub receipt_ref: String,
    pub link_ref: String,
    /// Commitment to the exact serialized/canonical link subject. AC-008 does not
    /// interpret this field; the injected verifier must verify its binding.
    pub subject_commitment: String,
    pub authority_ref: String,
    pub qualification_policy_ref: String,
    pub verification_method_ref: String,
    pub evidence: Vec<ProvenanceRef>,
    pub issued_at: u64,
    pub expires_at: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct QualifiedIdentityLinkInput {
    pub link: EntityIdentityLink,
    pub receipt: IdentityQualificationReceipt,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct QualificationVerifierFailure {
    /// Stable non-secret failure code. Implementations should not place private
    /// identifiers or raw credential material here.
    pub code: String,
}

/// Trust-root interface supplied by deployment integration.
///
/// A production implementation can verify Xenia signatures, Mycelix authority
/// receipts, registry attestations, policy authorization, revocation status, and
/// canonical subject commitments. The civic-types crate intentionally does not
/// pretend to possess those trust roots itself.
pub trait IdentityQualificationVerifier {
    fn verify(
        &self,
        link: &EntityIdentityLink,
        receipt: &IdentityQualificationReceipt,
    ) -> Result<(), QualificationVerifierFailure>;
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum IdentityQualificationError {
    MissingRequiredPolicyReference,
    DuplicateReceiptReference { receipt_ref: String },
    InvalidReceipt { input_index: usize },
    ReceiptLinkMismatch { input_index: usize },
    ReceiptPolicyMismatch { input_index: usize },
    ReceiptOutsideValidityWindow { input_index: usize },
    VerifierRejected { input_index: usize, code: String },
    MissingProjectionId,
    EquivalenceView(Vec<EquivalenceViewError>),
}

#[derive(Debug, Default, Clone, Copy)]
pub struct IdentityQualificationContract;

impl IdentityQualificationContract {
    pub fn build_equivalence_view<V: IdentityQualificationVerifier>(
        inputs: &[QualifiedIdentityLinkInput],
        required_policy_ref: &str,
        evaluated_at: u64,
        verifier: &V,
    ) -> Result<QualifiedEquivalenceView, Vec<IdentityQualificationError>> {
        let links = qualify_links(inputs, required_policy_ref, evaluated_at, verifier)?;
        EquivalenceViewContract::build(&links)
            .map_err(|errors| vec![IdentityQualificationError::EquivalenceView(errors)])
    }

    pub fn procurement_supplier_concentration<V: IdentityQualificationVerifier>(
        edges: &[InstitutionalEdge],
        inputs: &[QualifiedIdentityLinkInput],
        required_policy_ref: &str,
        context: ObservationContext,
        verifier: &V,
    ) -> Result<IdentityProjectedObservation, Vec<IdentityQualificationError>> {
        if context.observation_id.trim().is_empty() {
            return Err(vec![IdentityQualificationError::MissingProjectionId]);
        }
        let links = qualify_links(inputs, required_policy_ref, context.observed_at, verifier)?;
        EquivalenceViewContract::procurement_supplier_concentration(edges, &links, context)
            .map_err(|errors| vec![IdentityQualificationError::EquivalenceView(errors)])
    }
}

fn qualify_links<V: IdentityQualificationVerifier>(
    inputs: &[QualifiedIdentityLinkInput],
    required_policy_ref: &str,
    evaluated_at: u64,
    verifier: &V,
) -> Result<Vec<EntityIdentityLink>, Vec<IdentityQualificationError>> {
    let mut errors = Vec::new();
    if required_policy_ref.trim().is_empty() {
        errors.push(IdentityQualificationError::MissingRequiredPolicyReference);
    }

    let mut receipt_refs = BTreeSet::new();
    for (input_index, input) in inputs.iter().enumerate() {
        let receipt = &input.receipt;
        if !receipt_refs.insert(receipt.receipt_ref.clone()) {
            errors.push(IdentityQualificationError::DuplicateReceiptReference {
                receipt_ref: receipt.receipt_ref.clone(),
            });
        }
        if !receipt_structurally_valid(receipt) {
            errors.push(IdentityQualificationError::InvalidReceipt { input_index });
            continue;
        }
        if receipt.link_ref != input.link.id {
            errors.push(IdentityQualificationError::ReceiptLinkMismatch { input_index });
        }
        if receipt.qualification_policy_ref != required_policy_ref {
            errors.push(IdentityQualificationError::ReceiptPolicyMismatch { input_index });
        }
        if evaluated_at < receipt.issued_at || evaluated_at >= receipt.expires_at {
            errors.push(IdentityQualificationError::ReceiptOutsideValidityWindow {
                input_index,
            });
        }

        if let Err(failure) = verifier.verify(&input.link, receipt) {
            let code = if failure.code.trim().is_empty() {
                "unspecified-verifier-rejection".to_string()
            } else {
                failure.code
            };
            errors.push(IdentityQualificationError::VerifierRejected {
                input_index,
                code,
            });
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    Ok(inputs.iter().map(|input| input.link.clone()).collect())
}

fn receipt_structurally_valid(receipt: &IdentityQualificationReceipt) -> bool {
    !receipt.receipt_ref.trim().is_empty()
        && !receipt.link_ref.trim().is_empty()
        && !receipt.subject_commitment.trim().is_empty()
        && !receipt.authority_ref.trim().is_empty()
        && !receipt.qualification_policy_ref.trim().is_empty()
        && !receipt.verification_method_ref.trim().is_empty()
        && receipt.expires_at > receipt.issued_at
        && !receipt.evidence.is_empty()
        && receipt.evidence.iter().all(|evidence| {
            !evidence.source_ref.trim().is_empty()
                && evidence
                    .content_hash
                    .as_deref()
                    .is_some_and(|hash| !hash.trim().is_empty())
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture_observation::{CaptureSubject, ConfidenceAssessment, ConfidenceLevel};
    use crate::identity_resolution::{
        EntityBindingEvidence, IdentityLinkStatus, IdentityVerification,
        IdentityVerificationKind,
    };
    use crate::institutional_graph::{
        AssertionStatus, DisclosureClass, InstitutionalNodeKind, InstitutionalNodeRef,
        InstitutionalRelationKind, ProcurementRole,
    };
    use crate::standards_ingestion::ExternalStandard;

    struct TestVerifier;

    impl IdentityQualificationVerifier for TestVerifier {
        fn verify(
            &self,
            link: &EntityIdentityLink,
            receipt: &IdentityQualificationReceipt,
        ) -> Result<(), QualificationVerifierFailure> {
            if receipt.subject_commitment == format!("commit:{}", link.id)
                && receipt.authority_ref == "authority:test"
            {
                Ok(())
            } else {
                Err(QualificationVerifierFailure {
                    code: "subject-or-authority-invalid".into(),
                })
            }
        }
    }

    fn node(id: &str) -> InstitutionalNodeRef {
        InstitutionalNodeRef {
            id: id.into(),
            kind: InstitutionalNodeKind::Organization,
        }
    }

    fn link(id: &str, left: &str, right: &str) -> EntityIdentityLink {
        let left_node = node(left);
        let right_node = node(right);
        EntityIdentityLink {
            id: id.into(),
            left: left_node.clone(),
            right: right_node.clone(),
            status: IdentityLinkStatus::Corroborated,
            binding_evidence: vec![
                EntityBindingEvidence {
                    node: left_node,
                    scheme: "GB-COH".into(),
                    identifier: "09506232".into(),
                    standard: ExternalStandard::Ocds11SchemaRevision115,
                    source_ref: format!("source:{id}:left"),
                    content_hash: format!("sha256:{id}:left"),
                    validation_receipt_ref: format!("validation:{id}:left"),
                    policy_ref: "ingestion-policy:v1".into(),
                    observed_at: 100,
                },
                EntityBindingEvidence {
                    node: right_node,
                    scheme: "GB-COH".into(),
                    identifier: "09506232".into(),
                    standard: ExternalStandard::Bods04,
                    source_ref: format!("source:{id}:right"),
                    content_hash: format!("sha256:{id}:right"),
                    validation_receipt_ref: format!("validation:{id}:right"),
                    policy_ref: "ingestion-policy:v1".into(),
                    observed_at: 100,
                },
            ],
            verification_evidence: vec![IdentityVerification {
                verification_ref: format!("verification:{id}"),
                verifier_ref: "verifier:registry".into(),
                kind: IdentityVerificationKind::AuthoritativeRegistryLookup {
                    scheme: "GB-COH".into(),
                    identifier: "09506232".into(),
                    registry_ref: "registry:companies-house".into(),
                },
                provenance: vec![ProvenanceRef {
                    source_ref: "registry:companies-house:record".into(),
                    content_hash: Some("sha256:registry".into()),
                }],
                verified_at: 150,
            }],
            challenge_refs: vec![],
            review_ref: Some(format!("review:{id}")),
            review_rationale: Some("reviewed exact public identifier".into()),
            superseded_by: None,
            reversible: true,
            recorded_at: 160,
        }
    }

    fn receipt(link_id: &str) -> IdentityQualificationReceipt {
        IdentityQualificationReceipt {
            receipt_ref: format!("qualification:{link_id}"),
            link_ref: link_id.into(),
            subject_commitment: format!("commit:{link_id}"),
            authority_ref: "authority:test".into(),
            qualification_policy_ref: "qualification-policy:v1".into(),
            verification_method_ref: "test-verifier:v1".into(),
            evidence: vec![ProvenanceRef {
                source_ref: format!("qualification-source:{link_id}"),
                content_hash: Some(format!("sha256:qualification:{link_id}")),
            }],
            issued_at: 200,
            expires_at: 400,
        }
    }

    fn award(id: &str, supplier: &str) -> InstitutionalEdge {
        InstitutionalEdge {
            id: id.into(),
            from: node(supplier),
            to: InstitutionalNodeRef {
                id: format!("procedure:{id}"),
                kind: InstitutionalNodeKind::ProcurementProcedure,
            },
            relation: InstitutionalRelationKind::ProcurementParticipation {
                role: ProcurementRole::Awardee,
            },
            disclosure: DisclosureClass::PublicMetadata,
            assertion_status: AssertionStatus::Declared,
            provenance: vec![ProvenanceRef {
                source_ref: format!("source:{id}"),
                content_hash: Some(format!("sha256:{id}")),
            }],
            challenge_refs: vec![],
            recorded_at: 100,
            valid_from: None,
            valid_until: None,
        }
    }

    fn context(id: &str) -> ObservationContext {
        ObservationContext {
            observation_id: id.into(),
            subject: CaptureSubject::ContractingProcedure("municipal-procurement".into()),
            observed_at: 300,
            confidence: ConfidenceAssessment::Qualitative {
                level: ConfidenceLevel::Moderate,
                basis: "qualified identity projection test".into(),
            },
            limitations: vec!["test fixture".into()],
        }
    }

    #[test]
    fn valid_external_verifier_unlocks_internal_equivalence_engine() {
        let identity_link = link("link:ab", "supplier:a", "supplier:b");
        let input = QualifiedIdentityLinkInput {
            receipt: receipt(&identity_link.id),
            link: identity_link,
        };
        let view = IdentityQualificationContract::build_equivalence_view(
            &[input],
            "qualification-policy:v1",
            300,
            &TestVerifier,
        )
        .expect("verified receipt can unlock a structural view");
        assert_eq!(view.components.len(), 1);
    }

    #[test]
    fn expired_receipt_fails_before_equivalence_use() {
        let identity_link = link("link:ab", "supplier:a", "supplier:b");
        let mut old_receipt = receipt(&identity_link.id);
        old_receipt.expires_at = 250;
        let errors = IdentityQualificationContract::build_equivalence_view(
            &[QualifiedIdentityLinkInput {
                link: identity_link,
                receipt: old_receipt,
            }],
            "qualification-policy:v1",
            300,
            &TestVerifier,
        )
        .expect_err("stale qualification must fail closed");
        assert!(errors.iter().any(|error| matches!(
            error,
            IdentityQualificationError::ReceiptOutsideValidityWindow { .. }
        )));
    }

    #[test]
    fn verifier_rejection_is_authoritative_for_public_boundary() {
        let identity_link = link("link:ab", "supplier:a", "supplier:b");
        let mut bad_receipt = receipt(&identity_link.id);
        bad_receipt.subject_commitment = "wrong-commitment".into();
        let errors = IdentityQualificationContract::build_equivalence_view(
            &[QualifiedIdentityLinkInput {
                link: identity_link,
                receipt: bad_receipt,
            }],
            "qualification-policy:v1",
            300,
            &TestVerifier,
        )
        .expect_err("external verifier must gate the public path");
        assert!(errors.iter().any(|error| matches!(
            error,
            IdentityQualificationError::VerifierRejected { .. }
        )));
    }

    #[test]
    fn receipt_must_bind_the_exact_link_reference() {
        let identity_link = link("link:ab", "supplier:a", "supplier:b");
        let mut wrong = receipt(&identity_link.id);
        wrong.link_ref = "link:other".into();
        let errors = IdentityQualificationContract::build_equivalence_view(
            &[QualifiedIdentityLinkInput {
                link: identity_link,
                receipt: wrong,
            }],
            "qualification-policy:v1",
            300,
            &TestVerifier,
        )
        .expect_err("receipt cannot be replayed onto another link");
        assert!(errors.iter().any(|error| matches!(
            error,
            IdentityQualificationError::ReceiptLinkMismatch { .. }
        )));
    }

    #[test]
    fn public_metric_path_requires_nonempty_projection_identity() {
        let identity_link = link("link:ab", "supplier:a", "supplier:b");
        let input = QualifiedIdentityLinkInput {
            receipt: receipt(&identity_link.id),
            link: identity_link,
        };
        let edges = vec![award("a1", "supplier:a"), award("b1", "supplier:b")];
        let errors = IdentityQualificationContract::procurement_supplier_concentration(
            &edges,
            &[input],
            "qualification-policy:v1",
            context(""),
            &TestVerifier,
        )
        .expect_err("projection ID cannot be synthesized from empty caller identity");
        assert_eq!(errors, vec![IdentityQualificationError::MissingProjectionId]);
    }

    #[test]
    fn verified_public_metric_path_preserves_before_and_after_observations() {
        let identity_link = link("link:ab", "supplier:a", "supplier:b");
        let input = QualifiedIdentityLinkInput {
            receipt: receipt(&identity_link.id),
            link: identity_link,
        };
        let edges = vec![
            award("a1", "supplier:a"),
            award("b1", "supplier:b"),
            award("c1", "supplier:c"),
            award("c2", "supplier:c"),
        ];
        let result = IdentityQualificationContract::procurement_supplier_concentration(
            &edges,
            &[input],
            "qualification-policy:v1",
            context("projection:verified"),
            &TestVerifier,
        )
        .expect("verified public path can compute reversible projection");
        assert_eq!(result.baseline.observation.measurement.value.numerator, 6);
        assert_eq!(result.projected.observation.measurement.value.numerator, 8);
        assert_eq!(result.applied_identity_link_refs, vec!["link:ab"]);
    }
}
