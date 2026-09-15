// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use civic_types::*;

const QUALIFICATION_POLICY: &str = "identity-qualification:v1";
const AUTHORITY: &str = "authority:ac016-fixture";

fn provenance(id: &str) -> ProvenanceRef {
    ProvenanceRef {
        source_ref: format!("source:{id}"),
        content_hash: Some(format!("sha256:{id}")),
    }
}

fn node(id: &str) -> InstitutionalNodeRef {
    InstitutionalNodeRef {
        id: id.into(),
        kind: InstitutionalNodeKind::Organization,
    }
}

fn descriptor(id: &str) -> ProcurementAssumptionDescriptor {
    ProcurementAssumptionDescriptor {
        id: id.into(),
        statement: format!("AC-016 admissible perturbation {id}"),
        admissibility_ref: format!("ac016-rule:{id}"),
        provenance: vec![provenance(id)],
    }
}

fn award(id: &str, supplier: &str) -> InstitutionalEdge {
    InstitutionalEdge {
        id: id.into(),
        from: node(supplier),
        to: InstitutionalNodeRef {
            id: "procedure:ac016".into(),
            kind: InstitutionalNodeKind::ProcurementProcedure,
        },
        relation: InstitutionalRelationKind::ProcurementParticipation {
            role: ProcurementRole::Awardee,
        },
        disclosure: DisclosureClass::PublicMetadata,
        assertion_status: AssertionStatus::Declared,
        provenance: vec![provenance(id)],
        challenge_refs: vec![],
        recorded_at: 100,
        valid_from: None,
        valid_until: None,
    }
}

fn qualified_input(
    link_id: &str,
    left_id: &str,
    right_id: &str,
    identifier: &str,
) -> QualifiedIdentityLinkInput {
    let left = node(left_id);
    let right = node(right_id);
    let link = EntityIdentityLink {
        id: link_id.into(),
        left: left.clone(),
        right: right.clone(),
        status: IdentityLinkStatus::Corroborated,
        binding_evidence: vec![
            EntityBindingEvidence {
                node: left,
                scheme: "ZA-CIPC".into(),
                identifier: identifier.into(),
                standard: ExternalStandard::Ocds11SchemaRevision115,
                source_ref: format!("source:{link_id}:left"),
                content_hash: format!("sha256:{link_id}:left"),
                validation_receipt_ref: format!("validation:{link_id}:left"),
                policy_ref: "ingestion-policy:v1".into(),
                observed_at: 100,
            },
            EntityBindingEvidence {
                node: right,
                scheme: "ZA-CIPC".into(),
                identifier: identifier.into(),
                standard: ExternalStandard::Bods04,
                source_ref: format!("source:{link_id}:right"),
                content_hash: format!("sha256:{link_id}:right"),
                validation_receipt_ref: format!("validation:{link_id}:right"),
                policy_ref: "ingestion-policy:v1".into(),
                observed_at: 110,
            },
        ],
        verification_evidence: vec![IdentityVerification {
            verification_ref: format!("registry-verification:{link_id}"),
            verifier_ref: "registry:ac016".into(),
            kind: IdentityVerificationKind::AuthoritativeRegistryLookup {
                scheme: "ZA-CIPC".into(),
                identifier: identifier.into(),
                registry_ref: "registry:za-cipc:ac016".into(),
            },
            provenance: vec![provenance(&format!("registry:{link_id}"))],
            verified_at: 150,
        }],
        challenge_refs: vec![],
        review_ref: Some(format!("review:{link_id}")),
        review_rationale: Some("AC-016 independent registry review".into()),
        superseded_by: None,
        reversible: true,
        recorded_at: 160,
    };

    QualifiedIdentityLinkInput {
        receipt: IdentityQualificationReceipt {
            receipt_ref: format!("qualification:{link_id}"),
            link_ref: link.id.clone(),
            subject_commitment: format!("commit:{link_id}"),
            authority_ref: AUTHORITY.into(),
            qualification_policy_ref: QUALIFICATION_POLICY.into(),
            verification_method_ref: "ac016-verifier:v1".into(),
            evidence: vec![provenance(&format!("qualification:{link_id}"))],
            issued_at: 200,
            expires_at: 500,
        },
        link,
    }
}

struct AcceptingVerifier;

impl IdentityQualificationVerifier for AcceptingVerifier {
    fn verify(
        &self,
        link: &EntityIdentityLink,
        receipt: &IdentityQualificationReceipt,
    ) -> Result<(), QualificationVerifierFailure> {
        if receipt.link_ref == link.id
            && receipt.subject_commitment == format!("commit:{}", link.id)
            && receipt.authority_ref == AUTHORITY
            && receipt.qualification_policy_ref == QUALIFICATION_POLICY
        {
            Ok(())
        } else {
            Err(QualificationVerifierFailure {
                code: "ac016-binding-rejected".into(),
            })
        }
    }
}

struct RevokingVerifier {
    revoked_receipt_ref: String,
}

impl IdentityQualificationVerifier for RevokingVerifier {
    fn verify(
        &self,
        link: &EntityIdentityLink,
        receipt: &IdentityQualificationReceipt,
    ) -> Result<(), QualificationVerifierFailure> {
        if receipt.receipt_ref == self.revoked_receipt_ref {
            return Err(QualificationVerifierFailure {
                code: "receipt-revoked".into(),
            });
        }
        AcceptingVerifier.verify(link, receipt)
    }
}

fn context(id: &str) -> ObservationContext {
    ObservationContext {
        observation_id: id.into(),
        subject: CaptureSubject::ContractingProcedure("ac016-procurement".into()),
        observed_at: 300,
        confidence: ConfidenceAssessment::Qualitative {
            level: ConfidenceLevel::Moderate,
            basis: "AC-016 adversarial property fixture".into(),
        },
        limitations: vec!["synthetic adversarial property fixture".into()],
    }
}

fn values_for(edges: &[InstitutionalEdge]) -> ProcurementAwardValueSet {
    ProcurementAwardValueSet {
        snapshot_ref: "ac016-values:v1".into(),
        value_semantics_ref: "supplier-attributed-award-values:v1".into(),
        currency_registry_ref: "iso4217:fixture".into(),
        currency_registry_provenance: vec![provenance("iso4217")],
        records: edges
            .iter()
            .enumerate()
            .map(|(index, edge)| SupplierAttributedAwardValue {
                value_ref: format!("value:{index}"),
                award_edge_ref: edge.id.clone(),
                award_ref: format!("award:{index}"),
                amount: ExactCurrencyAmount {
                    coefficient: (index as u64 + 1) * 100,
                    scale: 0,
                    currency: "ZAR".into(),
                },
                provenance: vec![provenance(&format!("value:{index}"))],
            })
            .collect(),
    }
}

fn full_plan() -> FullProcurementRobustnessPlan {
    FullProcurementRobustnessPlan {
        base: ProcurementRobustnessPlan {
            matrix_id: "ac016-order-invariance".into(),
            subject: CaptureSubject::ContractingProcedure("ac016-procurement".into()),
            evaluated_at: 300,
            confidence: ConfidenceAssessment::Qualitative {
                level: ConfidenceLevel::Moderate,
                basis: "AC-016 ordering invariant".into(),
            },
            limitations: vec!["synthetic ordering fixture".into()],
            identity_resolution: Some(descriptor("identity")),
            corroborated_only: None,
            time_windows: vec![],
        },
        value_weighting: Some(ProcurementWeightingAssumptions {
            weighting: descriptor("award-value-weighting"),
            method_choice: descriptor("value-hhi-method"),
        }),
    }
}

fn exploratory() -> RobustnessCoverage {
    RobustnessCoverage::Exploratory {
        limitation: "AC-016 property corpus is intentionally non-exhaustive".into(),
    }
}

#[test]
fn transitive_same_scheme_identifier_conflict_fails_closed() {
    let inputs = vec![
        qualified_input("link:ab", "supplier:a", "supplier:b", "SUP-X"),
        qualified_input("link:bc", "supplier:b", "supplier:c", "SUP-Y"),
    ];

    let errors = IdentityQualificationContract::build_equivalence_view(
        &inputs,
        QUALIFICATION_POLICY,
        300,
        &AcceptingVerifier,
    )
    .expect_err("transitive component with conflicting same-scheme identifiers must fail");

    assert!(errors.iter().any(|error| matches!(
        error,
        IdentityQualificationError::EquivalenceView(view_errors)
            if view_errors.iter().any(|view_error| matches!(
                view_error,
                EquivalenceViewError::ConflictingIdentifiers { scheme, .. }
                    if scheme == "ZA-CIPC"
            ))
    )));
}

#[test]
fn challenged_link_immediately_loses_aggregation_eligibility() {
    let mut input = qualified_input("link:ab", "supplier:a", "supplier:b", "SUP-X");
    input.link.status = IdentityLinkStatus::Challenged;
    input.link.challenge_refs = vec!["challenge:registry-disagreement".into()];

    let errors = IdentityQualificationContract::build_equivalence_view(
        &[input],
        QUALIFICATION_POLICY,
        300,
        &AcceptingVerifier,
    )
    .expect_err("challenged identity must not remain aggregation eligible");

    assert!(errors.iter().any(|error| matches!(
        error,
        IdentityQualificationError::EquivalenceView(view_errors)
            if view_errors.iter().any(|view_error| matches!(
                view_error,
                EquivalenceViewError::IdentityLinkNotAggregationEligible { .. }
            ))
    )));
}

#[test]
fn verifier_revocation_blocks_equivalence_before_analysis() {
    let input = qualified_input("link:ab", "supplier:a", "supplier:b", "SUP-X");
    let revoked = input.receipt.receipt_ref.clone();
    let errors = IdentityQualificationContract::build_equivalence_view(
        &[input],
        QUALIFICATION_POLICY,
        300,
        &RevokingVerifier {
            revoked_receipt_ref: revoked,
        },
    )
    .expect_err("revoked qualification receipt must fail before equivalence use");

    assert!(errors.iter().any(|error| matches!(
        error,
        IdentityQualificationError::VerifierRejected { code, .. }
            if code == "receipt-revoked"
    )));
}

#[test]
fn incomplete_joint_supplier_value_attribution_fails_closed() {
    let edges = vec![award("edge:a", "supplier:a"), award("edge:b", "supplier:b")];
    let values = ProcurementAwardValueSet {
        snapshot_ref: "joint-award-values:v1".into(),
        value_semantics_ref: "supplier-attributed-award-values:v1".into(),
        currency_registry_ref: "iso4217:fixture".into(),
        currency_registry_provenance: vec![provenance("iso4217")],
        records: vec![SupplierAttributedAwardValue {
            value_ref: "joint-value:only-one-supplier".into(),
            award_edge_ref: "edge:a".into(),
            award_ref: "upstream-joint-award".into(),
            amount: ExactCurrencyAmount {
                coefficient: 1_000,
                scale: 0,
                currency: "ZAR".into(),
            },
            provenance: vec![provenance("joint-value")],
        }],
    };

    let errors = ProcurementValueContract::value_weighted_supplier_concentration(
        &edges,
        &values,
        context("joint-award-incomplete"),
    )
    .expect_err("a joint total cannot be silently assigned to one supplier");

    assert!(errors.iter().any(|error| matches!(
        error,
        ProcurementValueError::MissingValueForAwardEdge { award_edge_ref }
            if award_edge_ref == "edge:b"
    )));
}

#[test]
fn full_matrix_is_invariant_to_input_ordering() {
    let edges = vec![
        award("edge:a", "supplier:a"),
        award("edge:b", "supplier:b"),
        award("edge:c", "supplier:c"),
        award("edge:d", "supplier:d"),
    ];
    let inputs = vec![
        qualified_input("link:ab", "supplier:a", "supplier:b", "SUP-GROUP"),
        qualified_input("link:bc", "supplier:b", "supplier:c", "SUP-GROUP"),
    ];
    let values = values_for(&edges);

    let forward = FullProcurementRobustnessContract::build(
        &edges,
        &inputs,
        Some(QUALIFICATION_POLICY),
        Some(&values),
        full_plan(),
        exploratory(),
        &AcceptingVerifier,
    )
    .expect("forward ordering should produce a matrix");

    let mut reversed_edges = edges.clone();
    reversed_edges.reverse();
    let mut reversed_inputs = inputs.clone();
    reversed_inputs.reverse();
    let mut reversed_values = values.clone();
    reversed_values.records.reverse();

    let reversed = FullProcurementRobustnessContract::build(
        &reversed_edges,
        &reversed_inputs,
        Some(QUALIFICATION_POLICY),
        Some(&reversed_values),
        full_plan(),
        exploratory(),
        &AcceptingVerifier,
    )
    .expect("reversed ordering should produce a matrix");

    assert_eq!(forward, reversed, "canonical output must not depend on input ordering");
}
