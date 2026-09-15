// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use std::cell::Cell;

use civic_types::*;

const PRIVATE_RECORD_ID: &str = "PASSPORT-LIKE-PRIVATE-RECORD-9981";
const PRIVATE_IDENTIFIER: &str = "SECRET-PASSPORT-9981";

fn policy() -> StandardsIngestionPolicy {
    StandardsIngestionPolicy {
        policy_ref: "policy:municipal-public-identifiers:v1".into(),
        public_entity_identifier_schemes: ["ZA-CIPC".to_string()].into_iter().collect(),
    }
}

fn source(name: &str, ingested_at: u64) -> StandardsSourceEvidence {
    StandardsSourceEvidence {
        source_ref: format!("https://municipality.example.test/{name}.json"),
        content_hash: format!("sha256:{name}"),
        validation_receipt_ref: format!("validation:{name}"),
        ingested_at,
    }
}

fn provenance(id: &str) -> ProvenanceRef {
    ProvenanceRef {
        source_ref: format!("evidence:{id}"),
        content_hash: Some(format!("sha256:{id}")),
    }
}

fn descriptor(id: &str) -> ProcurementAssumptionDescriptor {
    ProcurementAssumptionDescriptor {
        id: id.into(),
        statement: format!("adversarial fixture assumption {id}"),
        admissibility_ref: format!("fixture-rule:{id}"),
        provenance: vec![provenance(id)],
    }
}

fn ocds_a() -> &'static str {
    r#"{
      "uri":"https://municipality.example.test/ocds-a.json",
      "version":"1.1",
      "releases":[{
        "ocid":"ocds-za-muni-001",
        "id":"release-a",
        "parties":[
          {"id":"buyer-a","roles":["buyer"]},
          {"id":"alpha-a","roles":["supplier"],"identifier":{"scheme":"ZA-CIPC","id":"SUP-ALPHA"}},
          {"id":"beta","roles":["supplier"],"identifier":{"scheme":"ZA-CIPC","id":"SUP-BETA"}}
        ],
        "buyer":{"id":"buyer-a"},
        "awards":[
          {"id":"award-a","suppliers":[{"id":"alpha-a"}],"value":{"amount":100,"currency":"ZAR"}},
          {"id":"award-b","suppliers":[{"id":"beta"}],"value":{"amount":200,"currency":"ZAR"}}
        ]
      }]
    }"#
}

fn ocds_b() -> &'static str {
    r#"{
      "uri":"https://municipality.example.test/ocds-b.json",
      "version":"1.1",
      "releases":[{
        "ocid":"ocds-za-muni-002",
        "id":"release-b",
        "parties":[
          {"id":"buyer-b","roles":["buyer"]},
          {"id":"alpha-alias","roles":["supplier"],"identifier":{"scheme":"ZA-CIPC","id":"SUP-ALPHA"}},
          {"id":"gamma","roles":["supplier"],"identifier":{"scheme":"ZA-CIPC","id":"SUP-GAMMA"}}
        ],
        "buyer":{"id":"buyer-b"},
        "awards":[
          {"id":"award-c","suppliers":[{"id":"alpha-alias"}],"value":{"amount":300,"currency":"ZAR"}},
          {"id":"award-d","suppliers":[{"id":"gamma"}],"value":{"amount":400,"currency":"ZAR"}}
        ]
      }]
    }"#
}

fn bods() -> String {
    format!(
        r#"[
          {{
            "statementId":"11111111-1111-4111-8111-111111111111",
            "recordId":"entity-alpha",
            "recordType":"entity",
            "publicationDetails":{{"bodsVersion":"0.4","publisher":{{"url":"https://registry.example.test"}}}},
            "recordDetails":{{"identifiers":[{{"scheme":"ZA-CIPC","id":"SUP-ALPHA"}}]}}
          }},
          {{
            "statementId":"22222222-2222-4222-8222-222222222222",
            "recordId":"{PRIVATE_RECORD_ID}",
            "recordType":"person",
            "publicationDetails":{{"bodsVersion":"0.4","publisher":{{"url":"https://registry.example.test"}}}},
            "recordDetails":{{"identifiers":[{{"scheme":"710-PASSPORT","id":"{PRIVATE_IDENTIFIER}"}}]}}
          }},
          {{
            "statementId":"33333333-3333-4333-8333-333333333333",
            "recordId":"relationship-alpha",
            "recordType":"relationship",
            "publicationDetails":{{"bodsVersion":"0.4","publisher":{{"url":"https://registry.example.test"}}}},
            "recordDetails":{{
              "subject":"entity-alpha",
              "interestedParty":"{PRIVATE_RECORD_ID}",
              "interests":[{{"type":"shareholding","beneficialOwnershipOrControl":true,"share":{{"exact":25.5}}}}]
            }}
          }}
        ]"#
    )
}

fn award_edges(result: &StandardsIngestionResult) -> Vec<&InstitutionalEdge> {
    result
        .edges
        .iter()
        .filter(|edge| {
            matches!(
                &edge.relation,
                InstitutionalRelationKind::ProcurementParticipation {
                    role: ProcurementRole::Awardee
                }
            )
        })
        .collect()
}

fn promote_link(mut link: EntityIdentityLink) -> QualifiedIdentityLinkInput {
    let binding = link
        .binding_evidence
        .first()
        .expect("proposal has binding evidence")
        .clone();
    link.status = IdentityLinkStatus::Corroborated;
    link.verification_evidence = vec![IdentityVerification {
        verification_ref: format!("registry-verification:{}", link.id),
        verifier_ref: "registry:fixture".into(),
        kind: IdentityVerificationKind::AuthoritativeRegistryLookup {
            scheme: binding.scheme.clone(),
            identifier: binding.identifier.clone(),
            registry_ref: "registry:za-cipc-fixture".into(),
        },
        provenance: vec![provenance(&format!("registry:{}", link.id))],
        verified_at: 220,
    }];
    link.review_ref = Some(format!("review:{}", link.id));
    link.review_rationale = Some("fixture independent registry review".into());

    QualifiedIdentityLinkInput {
        receipt: IdentityQualificationReceipt {
            receipt_ref: format!("qualification:{}", link.id),
            link_ref: link.id.clone(),
            subject_commitment: format!("commit:{}", link.id),
            authority_ref: "authority:municipal-integrity-fixture".into(),
            qualification_policy_ref: "identity-qualification:v1".into(),
            verification_method_ref: "fixture-registry-verifier:v1".into(),
            evidence: vec![provenance(&format!("qualification:{}", link.id))],
            issued_at: 221,
            expires_at: 500,
        },
        link,
    }
}

struct FixtureVerifier {
    calls: Cell<usize>,
}

impl IdentityQualificationVerifier for FixtureVerifier {
    fn verify(
        &self,
        link: &EntityIdentityLink,
        receipt: &IdentityQualificationReceipt,
    ) -> Result<(), QualificationVerifierFailure> {
        self.calls.set(self.calls.get() + 1);
        if receipt.subject_commitment == format!("commit:{}", link.id)
            && receipt.authority_ref == "authority:municipal-integrity-fixture"
            && receipt.qualification_policy_ref == "identity-qualification:v1"
        {
            Ok(())
        } else {
            Err(QualificationVerifierFailure {
                code: "fixture-qualification-rejected".into(),
            })
        }
    }
}

#[derive(Clone)]
struct MunicipalFixture {
    ocds_a: StandardsIngestionResult,
    ocds_b: StandardsIngestionResult,
    bods: StandardsIngestionResult,
    edges: Vec<InstitutionalEdge>,
    identity_inputs: Vec<QualifiedIdentityLinkInput>,
    values: ProcurementAwardValueSet,
}

fn municipal_fixture() -> MunicipalFixture {
    let ingestion_policy = policy();
    let a = StandardsIngestionContract::ingest_ocds_release_package_json(
        ocds_a(),
        source("ocds-a", 100),
        ingestion_policy.clone(),
    )
    .expect("OCDS A fixture must ingest");
    let b = StandardsIngestionContract::ingest_ocds_release_package_json(
        ocds_b(),
        source("ocds-b", 200),
        ingestion_policy.clone(),
    )
    .expect("OCDS B fixture must ingest");
    let beneficial = StandardsIngestionContract::ingest_bods_json(
        &bods(),
        source("bods", 150),
        ingestion_policy.clone(),
    )
    .expect("BODS fixture must ingest");

    let serialized_bods = serde_json::to_string(&beneficial).expect("serialize projected BODS");
    assert!(!serialized_bods.contains(PRIVATE_RECORD_ID));
    assert!(!serialized_bods.contains(PRIVATE_IDENTIFIER));

    let resolution_inputs = vec![
        IdentityResolutionInput {
            result: a.clone(),
            policy: ingestion_policy.clone(),
        },
        IdentityResolutionInput {
            result: b.clone(),
            policy: ingestion_policy.clone(),
        },
        IdentityResolutionInput {
            result: beneficial.clone(),
            policy: ingestion_policy,
        },
    ];
    let proposals = IdentityResolutionContract::propose_exact_identifier_links(&resolution_inputs)
        .expect("authorized public identifiers must reach reconciliation");
    assert_eq!(proposals.len(), 3, "three SUP-ALPHA records create three reversible pair proposals");
    let identity_inputs = proposals.into_iter().map(promote_link).collect::<Vec<_>>();

    let award_refs = award_edges(&a)
        .into_iter()
        .chain(award_edges(&b))
        .map(|edge| edge.id.clone())
        .collect::<Vec<_>>();
    assert_eq!(award_refs.len(), 4);

    let values = ProcurementAwardValueSet {
        snapshot_ref: "municipal-values:snapshot:v1".into(),
        value_semantics_ref: "ocds-1.1.5:award.value:supplier-attributed".into(),
        currency_registry_ref: "iso4217:fixture".into(),
        currency_registry_provenance: vec![provenance("iso4217-fixture")],
        records: award_refs
            .iter()
            .zip([100_u64, 200, 300, 400])
            .enumerate()
            .map(|(index, (edge_ref, amount))| SupplierAttributedAwardValue {
                value_ref: format!("municipal-value:{}", index + 1),
                award_edge_ref: edge_ref.clone(),
                award_ref: format!("municipal-award:{}", index + 1),
                amount: ExactCurrencyAmount {
                    coefficient: amount,
                    scale: 0,
                    currency: "ZAR".into(),
                },
                provenance: vec![provenance(&format!("municipal-value:{}", index + 1))],
            })
            .collect(),
    };

    let mut edges = a
        .edges
        .iter()
        .chain(b.edges.iter())
        .chain(beneficial.edges.iter())
        .cloned()
        .collect::<Vec<_>>();
    let mut award_index = 0_usize;
    for edge in &mut edges {
        if matches!(
            &edge.relation,
            InstitutionalRelationKind::ProcurementParticipation {
                role: ProcurementRole::Awardee
            }
        ) {
            if award_index < 3 {
                edge.assertion_status = AssertionStatus::Corroborated;
                edge.provenance.push(provenance(&format!("independent-audit:{}", edge.id)));
            }
            award_index += 1;
        }
    }
    assert_eq!(award_index, 4);

    MunicipalFixture {
        ocds_a: a,
        ocds_b: b,
        bods: beneficial,
        edges,
        identity_inputs,
        values,
    }
}

fn full_plan() -> FullProcurementRobustnessPlan {
    FullProcurementRobustnessPlan {
        base: ProcurementRobustnessPlan {
            matrix_id: "municipal-adversarial:v1".into(),
            subject: CaptureSubject::ContractingProcedure("municipal-procurement".into()),
            evaluated_at: 250,
            confidence: ConfidenceAssessment::Qualitative {
                level: ConfidenceLevel::Moderate,
                basis: "adversarial integration fixture".into(),
            },
            limitations: vec![
                "fixture uses ingestion time as AC-003 recorded_at rather than economic award date"
                    .into(),
            ],
            identity_resolution: Some(descriptor("identity")),
            corroborated_only: Some(descriptor("corroborated-only")),
            time_windows: vec![ProcurementTimeWindow {
                descriptor: descriptor("early-window"),
                start_recorded_at: 0,
                end_recorded_at: 150,
            }],
        },
        value_weighting: Some(ProcurementWeightingAssumptions {
            weighting: descriptor("award-value-weighting"),
            method_choice: descriptor("value-hhi-method"),
        }),
    }
}

fn no_identity_plan() -> FullProcurementRobustnessPlan {
    let mut plan = full_plan();
    plan.base.identity_resolution = None;
    plan
}

fn exploratory() -> RobustnessCoverage {
    RobustnessCoverage::Exploratory {
        limitation: "adversarial fixture covers selected specifications only".into(),
    }
}

#[test]
fn municipal_fixture_traverses_ac005_through_ac014() {
    let fixture = municipal_fixture();

    assert!(fixture
        .ocds_a
        .edges
        .iter()
        .chain(fixture.ocds_b.edges.iter())
        .all(|edge| edge.assertion_status == AssertionStatus::Declared));
    assert!(fixture
        .bods
        .warnings
        .iter()
        .any(|warning| matches!(warning, StandardsIngestionWarning::BodsPrivateIdentifiersSuppressed { .. })));

    let verifier = FixtureVerifier { calls: Cell::new(0) };
    let matrix = FullProcurementRobustnessContract::build(
        &fixture.edges,
        &fixture.identity_inputs,
        Some("identity-qualification:v1"),
        Some(&fixture.values),
        full_plan(),
        exploratory(),
        &verifier,
    )
    .expect("fully qualified adversarial fixture should produce a matrix");

    assert_eq!(matrix.envelope.scenarios.len(), 16);
    assert_eq!(matrix.scenario_lineage.len(), 16);
    assert_eq!(verifier.calls.get(), fixture.identity_inputs.len());

    let baseline = matrix
        .envelope
        .scenarios
        .iter()
        .find(|scenario| scenario.id == matrix.envelope.baseline_scenario_ref)
        .expect("baseline scenario");
    assert_eq!(baseline.observation.measurement.value.numerator, 4);
    assert_eq!(baseline.observation.measurement.value.denominator, 16);

    let identity_only = matrix
        .envelope
        .scenarios
        .iter()
        .find(|scenario| scenario.assumption_refs == vec!["identity".to_string()])
        .expect("identity-only count scenario");
    assert_eq!(identity_only.observation.measurement.value.numerator, 6);
    assert_eq!(identity_only.observation.measurement.value.denominator, 16);

    let value_scenarios = matrix
        .scenario_lineage
        .iter()
        .filter(|lineage| lineage.weighting_mode == ProcurementWeightingMode::QualifiedAwardValue)
        .collect::<Vec<_>>();
    assert_eq!(value_scenarios.len(), 8);
    assert!(value_scenarios
        .iter()
        .all(|lineage| lineage.value_snapshot_ref.as_deref() == Some("municipal-values:snapshot:v1")));
    assert!(value_scenarios
        .iter()
        .all(|lineage| !lineage.value_record_refs.is_empty()));
}

#[test]
fn duplicated_joint_award_value_fails_at_ac012_preflight() {
    let fixture = municipal_fixture();
    let mut malicious_values = fixture.values.clone();
    malicious_values.records[1].award_ref = malicious_values.records[0].award_ref.clone();

    let error = FullProcurementRobustnessContract::build(
        &fixture.edges,
        &[],
        None,
        Some(&malicious_values),
        no_identity_plan(),
        exploratory(),
        &FixtureVerifier { calls: Cell::new(0) },
    )
    .expect_err("duplicated upstream award total must fail closed");

    assert!(error.iter().any(|item| matches!(
        item,
        FullProcurementRobustnessError::ValuePreflight(errors)
            if errors.iter().any(|error| matches!(
                error,
                ProcurementValueError::DuplicateAwardReference { .. }
            ))
    )));
}

#[test]
fn stale_identity_receipts_fail_before_robustness_analysis() {
    let mut fixture = municipal_fixture();
    for input in &mut fixture.identity_inputs {
        input.receipt.expires_at = 240;
    }
    let error = FullProcurementRobustnessContract::build(
        &fixture.edges,
        &fixture.identity_inputs,
        Some("identity-qualification:v1"),
        Some(&fixture.values),
        full_plan(),
        exploratory(),
        &FixtureVerifier { calls: Cell::new(0) },
    )
    .expect_err("expired identity authority must fail closed");

    assert!(error.iter().any(|item| matches!(
        item,
        FullProcurementRobustnessError::IdentityQualification(errors)
            if errors.iter().any(|error| matches!(
                error,
                IdentityQualificationError::ReceiptOutsideValidityWindow { .. }
            ))
    )));
}

#[test]
fn unsupported_ocds_extension_fails_at_ingestion_boundary() {
    let malicious = ocds_a().replace(
        "\"version\":\"1.1\",",
        "\"version\":\"1.1\",\"extensions\":[\"https://attacker.example/ext.json\"],",
    );
    let errors = StandardsIngestionContract::ingest_ocds_release_package_json(
        &malicious,
        source("ocds-malicious-extension", 100),
        policy(),
    )
    .expect_err("unsupported extension must not pass AC-005");

    assert!(errors.iter().any(|error| matches!(
        error,
        StandardsIngestionViolation::UnsupportedOcdsExtension(_)
    )));
}

#[test]
fn private_bods_identifiers_never_reach_public_reconciliation_bindings() {
    let projected = StandardsIngestionContract::ingest_bods_json(
        &bods(),
        source("bods-private-check", 150),
        policy(),
    )
    .expect("BODS fixture projects");

    let serialized = serde_json::to_string(&projected).expect("serialize projected BODS");
    assert!(!serialized.contains(PRIVATE_RECORD_ID));
    assert!(!serialized.contains(PRIVATE_IDENTIFIER));
    assert!(projected
        .entity_identifiers
        .iter()
        .all(|binding| binding.identifier != PRIVATE_IDENTIFIER));
    assert!(projected
        .warnings
        .iter()
        .any(|warning| matches!(warning, StandardsIngestionWarning::BodsPrivateIdentifiersSuppressed { .. })));
}
