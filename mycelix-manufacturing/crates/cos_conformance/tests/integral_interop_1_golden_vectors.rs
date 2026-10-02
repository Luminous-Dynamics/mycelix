use cos_conformance::canonical_derivation_receipt::{canonical_bytes, canonical_sha256, D6S_REFERENCE_CANONICALIZATION_VERSION};
use cos_conformance::contestable_finality::{
    ObservationAssessmentV1, ObservationIndependenceV1, ObservationClassificationV1,
    ExternalObserverProfileV1, ExternalObservationSetV1, ExternalObservedEvidenceV1,
    ExternalObserverRoleV1,
};
use cos_conformance::effect_finality::{
    ExternalEffectObservationV1, ExternalFinalityStateV1, ExternalObservedStateV1,
    ExternalObservationSourceV1,
};
use cos_conformance::finality_eligibility_composition::{
    verify_witness_join_binding, CurrentFinalityEligibilityReceiptV1,
    FinalityEligibilityCompositionV1, FinalityWitnessEligibilityV1,
};
use cos_conformance::observer_lifecycle::EvidenceEligibilityReceiptV1;
use serde_json::Value;
use sha2::{Digest, Sha256};


#[derive(Debug, serde::Deserialize)]
struct Expected {
    fixture_id: String,
    canonicalization_version: String,
    hash_domain: String,
    canonical_utf8: String,
    domain_separated_sha256: String,
}

#[test]
fn integral_interop_1_design_vector_is_cross_runtime_stable() {
    let value: Value = serde_json::from_str(include_str!("../testdata/integral_interop_1_design.json"))
        .expect("Integral interop fixture must be valid JSON");
    let expected: Expected =
        serde_json::from_str(include_str!("../testdata/integral_interop_1_design.expected.json"))
            .expect("Integral interop expected vector must be valid JSON");

    assert_eq!(expected.fixture_id, "Integral-Interop-1");
    assert_eq!(expected.canonicalization_version, D6S_REFERENCE_CANONICALIZATION_VERSION);

    let bytes = canonical_bytes(&value).expect("Integral fixture must be canonicalizable");
    assert_eq!(
        String::from_utf8(bytes.clone()).expect("D6S canonical bytes must be UTF-8"),
        expected.canonical_utf8
    );
    assert_eq!(
        canonical_sha256(&expected.hash_domain, &value),
        expected.domain_separated_sha256
    );
}

#[test]
fn integral_interop_1_object_member_order_is_semantically_irrelevant() {
    let original: Value =
        serde_json::from_str(include_str!("../testdata/integral_interop_1_design.json"))
            .expect("fixture must be valid JSON");

    let mut reordered = original.clone();
    let object = reordered.as_object_mut().expect("fixture root must be an object");

    let source = object.remove("source").expect("source field must exist");
    let certification = object.remove("certification").expect("certification field must exist");
    let design_version = object.remove("design_version").expect("design_version field must exist");
    let status = object.remove("status").expect("status field must exist");
    let fixture_version = object.remove("fixture_version").expect("fixture_version field must exist");
    let fixture_id = object.remove("fixture_id").expect("fixture_id field must exist");

    object.insert("fixture_id".into(), fixture_id);
    object.insert("fixture_version".into(), fixture_version);
    object.insert("status".into(), status);
    object.insert("design_version".into(), design_version);
    object.insert("certification".into(), certification);
    object.insert("source".into(), source);

    assert_eq!(
        canonical_bytes(&original).expect("original must canonicalize"),
        canonical_bytes(&reordered).expect("reordered object must canonicalize")
    );
}

#[test]
fn integral_interop_1_production_step_mutation_changes_identity() {
    let mut value: Value =
        serde_json::from_str(include_str!("../testdata/integral_interop_1_design.json"))
            .expect("fixture must be valid JSON");

    let baseline = canonical_sha256("integral-interop-1", &value);

    value["design_version"]["parameters"]["production_steps"][1]["estimated_hours"] = Value::from(3);

    let mutated = canonical_sha256("integral-interop-1", &value);
    assert_ne!(baseline, mutated, "selected production-step mutation must change identity");
}

#[derive(Debug, serde::Deserialize)]
struct UpstreamFixture {
    schema_version: String,
    status: String,
    serialization_scope: String,
    hash_input: String,
    domain_prefix_hex: std::collections::BTreeMap<String, String>,
    vectors: Vec<UpstreamVector>,
}

#[derive(Debug, serde::Deserialize)]
struct UpstreamVector {
    id: String,
    layer: String,
    domain_id: String,
    serialization: String,
    preimage_payload_utf8: String,
    sha256: String,
}

fn upstream_domain(domain_id: &str) -> &'static [u8] {
    match domain_id {
        "D6M_OBSERVATION_COMMITMENT_DOMAIN" => {
            cos_conformance::effect_finality::D6M_OBSERVATION_COMMITMENT_DOMAIN
        }
        "D6N_ASSESSMENT_COMMITMENT_DOMAIN" => {
            cos_conformance::contestable_finality::D6N_ASSESSMENT_COMMITMENT_DOMAIN
        }
        "D6O_ELIGIBILITY_RECEIPT_COMMITMENT_DOMAIN" => {
            cos_conformance::observer_lifecycle::D6O_ELIGIBILITY_RECEIPT_COMMITMENT_DOMAIN
        }
        "D6P_WITNESS_COMMITMENT_DOMAIN" => {
            cos_conformance::finality_eligibility_composition::D6P_WITNESS_COMMITMENT_DOMAIN
        }
        "D6P_COMPOSITION_COMMITMENT_DOMAIN" => {
            cos_conformance::finality_eligibility_composition::D6P_COMPOSITION_COMMITMENT_DOMAIN
        }
        "D6P_RECEIPT_COMMITMENT_DOMAIN" => {
            cos_conformance::finality_eligibility_composition::D6P_RECEIPT_COMMITMENT_DOMAIN
        }
        other => panic!("unknown upstream golden-vector domain {other}"),
    }
}

fn upstream_hash(domain_id: &str, payload: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(upstream_domain(domain_id));
    hasher.update(payload);
    format!("{:x}", hasher.finalize())
}

fn vector(fixture: &UpstreamFixture, id: &str) -> &UpstreamVector {
    fixture
        .vectors
        .iter()
        .find(|vector| vector.id == id)
        .unwrap_or_else(|| panic!("missing upstream golden vector {id}"))
}

fn golden_d6m_observation() -> ExternalEffectObservationV1 {
    let mut observation = ExternalEffectObservationV1 {
        observation_id: "obs-golden-1".into(),
        effect_id: "effect-golden-1".into(),
        effect_lineage_id: "lineage-golden-1".into(),
        lifecycle_generation_id: "generation-golden-1".into(),
        route_id: "route-golden-1".into(),
        provider_id: "provider-golden-1".into(),
        provider_operation_id: "operation-golden-1".into(),
        provider_profile_root: "provider-profile-golden-1".into(),
        provider_outcome_id: "outcome-golden-1".into(),
        request_commitment: "request-golden-1".into(),
        idempotency_key: "idempotency-golden-1".into(),
        semantic_environment_root: "environment-golden-1".into(),
        observed_frontier_root: "frontier-golden-1".into(),
        observed_state: ExternalObservedStateV1::Applied,
        source: ExternalObservationSourceV1::IndependentObserver,
        evidence_root: "evidence-golden-1".into(),
        observation_commitment: String::new(),
        claim_ceiling: cos_conformance::effect_finality::EXTERNAL_FINALITY_CLAIM_CEILING.into(),
    };
    observation.observation_commitment = observation.recomputed_commitment();
    observation
}

#[test]
fn upstream_d6m_to_d6p_golden_vectors_are_exact_and_linked() {
    let fixture: UpstreamFixture = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_upstream_golden_vectors.json"
    ))
    .expect("upstream golden vectors must be valid JSON");

    assert_eq!(fixture.schema_version, "integral-interop-upstream-golden-v1");
    assert_eq!(fixture.status, "ReferenceModelOnly");
    assert!(fixture.serialization_scope.contains("not a cross-language wire-schema claim"));
    assert_eq!(fixture.hash_input, "domain-prefix-bytes || canonical_payload_utf8");
    assert_eq!(fixture.vectors.len(), 6);

    for entry in &fixture.vectors {
        let expected_domain_hex = match entry.domain_id.as_str() {
            "D6M_OBSERVATION_COMMITMENT_DOMAIN" => "4d5943454c49582d494e54454752414c2d44364d2d4f42534552564154494f4e2d563100",
            "D6N_ASSESSMENT_COMMITMENT_DOMAIN" => "4d5943454c49582d494e54454752414c2d44364e2d4153534553534d454e542d563100",
            "D6O_ELIGIBILITY_RECEIPT_COMMITMENT_DOMAIN" => "4d5943454c49582d494e54454752414c2d44364f2d454c49474942494c4954592d524543454950542d563100",
            "D6P_WITNESS_COMMITMENT_DOMAIN" => "4d5943454c49582d494e54454752414c2d4436502d5749544e4553532d563100",
            "D6P_COMPOSITION_COMMITMENT_DOMAIN" => "4d5943454c49582d494e54454752414c2d4436502d434f4d504f534954494f4e2d563100",
            "D6P_RECEIPT_COMMITMENT_DOMAIN" => "4d5943454c49582d494e54454752414c2d4436502d524543454950542d563100",
            other => panic!("unknown upstream golden-vector domain {other}"),
        };
        assert_eq!(
            fixture.domain_prefix_hex.get(&entry.domain_id),
            Some(&expected_domain_hex.to_owned()),
            "{} domain prefix drifted",
            entry.id
        );
        let payload = entry.preimage_payload_utf8.as_bytes();
        assert_eq!(
            upstream_hash(&entry.domain_id, payload),
            entry.sha256,
            "{} golden commitment drifted",
            entry.id
        );
    }

    let d6m = vector(&fixture, "d6m-observation");
    let observation = golden_d6m_observation();
    let d6m_payload = serde_json::to_vec(&(
        &observation.observation_id,
        &observation.effect_id,
        &observation.effect_lineage_id,
        &observation.lifecycle_generation_id,
        &observation.route_id,
        &observation.provider_id,
        &observation.provider_operation_id,
        &observation.provider_profile_root,
        &observation.provider_outcome_id,
        &observation.request_commitment,
        &observation.idempotency_key,
        &observation.semantic_environment_root,
        &observation.observed_frontier_root,
        &observation.observed_state,
        &observation.source,
        &observation.evidence_root,
        &observation.claim_ceiling,
    )).expect("D6M tuple serialization must succeed");
    assert_eq!(
        String::from_utf8(d6m_payload).expect("D6M payload must be UTF-8"),
        d6m.preimage_payload_utf8
    );
    assert_eq!(observation.observation_commitment, d6m.sha256);
    assert!(observation.commitment_matches());

    let d6n = vector(&fixture, "d6n-assessment-item");
    let mut assessment: ObservationAssessmentV1 =
        serde_json::from_str(&d6n.preimage_payload_utf8).expect("D6N vector must deserialize");
    assert_eq!(
        String::from_utf8(serde_json::to_vec(&assessment).expect("D6N serialization must succeed"))
            .expect("D6N payload must be UTF-8"),
        d6n.preimage_payload_utf8
    );
    assert_eq!(assessment.observation_commitment, observation.observation_commitment);
    assessment.assessment_commitment = d6n.sha256.clone();
    assert_eq!(assessment.recomputed_commitment(), d6n.sha256);
    assert!(assessment.commitment_matches());

    let d6o = vector(&fixture, "d6o-eligibility-receipt");
    let mut eligibility: EvidenceEligibilityReceiptV1 =
        serde_json::from_str(&d6o.preimage_payload_utf8).expect("D6O vector must deserialize");
    assert_eq!(
        String::from_utf8(serde_json::to_vec(&eligibility).expect("D6O serialization must succeed"))
            .expect("D6O payload must be UTF-8"),
        d6o.preimage_payload_utf8
    );
    eligibility.eligibility_commitment = d6o.sha256.clone();
    assert!(eligibility.commitment_matches());

    let d6pw = vector(&fixture, "d6p-witness");
    let mut witness: FinalityWitnessEligibilityV1 =
        serde_json::from_str(&d6pw.preimage_payload_utf8).expect("D6P witness vector must deserialize");
    assert_eq!(
        String::from_utf8(serde_json::to_vec(&witness).expect("D6P witness serialization must succeed"))
            .expect("D6P witness payload must be UTF-8"),
        d6pw.preimage_payload_utf8
    );
    assert_eq!(witness.d6n_assessment_item_commitment, assessment.assessment_commitment);
    assert_eq!(witness.d6o_eligibility_id, Some(eligibility.eligibility_id.clone()));
    witness.witness_commitment = d6pw.sha256.clone();
    assert!(witness.commitment_matches());

    let d6pc = vector(&fixture, "d6p-composition");
    let mut composition: FinalityEligibilityCompositionV1 =
        serde_json::from_str(&d6pc.preimage_payload_utf8).expect("D6P composition vector must deserialize");
    assert_eq!(
        String::from_utf8(serde_json::to_vec(&composition).expect("D6P composition serialization must succeed"))
            .expect("D6P composition payload must be UTF-8"),
        d6pc.preimage_payload_utf8
    );
    assert_eq!(composition.witnesses, vec![witness.clone()]);
    composition.composition_commitment = d6pc.sha256.clone();
    assert!(composition.commitment_matches());
    assert!(composition.semantically_valid());

    let d6pr = vector(&fixture, "d6p-receipt");
    let mut receipt: CurrentFinalityEligibilityReceiptV1 =
        serde_json::from_str(&d6pr.preimage_payload_utf8).expect("D6P receipt vector must deserialize");
    assert_eq!(
        String::from_utf8(serde_json::to_vec(&receipt).expect("D6P receipt serialization must succeed"))
            .expect("D6P receipt payload must be UTF-8"),
        d6pr.preimage_payload_utf8
    );
    assert_eq!(receipt.composition_commitment, composition.composition_commitment);
    receipt.receipt_commitment = d6pr.sha256.clone();
    assert!(receipt.commitment_matches());
    assert!(receipt.semantically_valid());

    let evidence = ExternalObservedEvidenceV1 {
        observation: observation.clone(),
        observer_id: "observer-golden-1".into(),
        observer: ExternalObserverProfileV1 {
            observer_id: "observer-golden-1".into(),
            role: ExternalObserverRoleV1::IndependentObserver,
            observation_method: "independent-state-read".into(),
            provider_relationship: "external".into(),
            evidence_root: "evidence-golden-1".into(),
            custody_root: "custody-golden-1".into(),
            upstream_observer_ids: std::collections::BTreeSet::new(),
            upstream_evidence_roots: std::collections::BTreeSet::new(),
            semantic_environment_root: "environment-golden-1".into(),
            observation_profile_id: "observation-profile-golden-1".into(),
            independence: ObservationIndependenceV1::DeclaredIndependent,
            independence_commitment: "independence-golden-1".into(),
            claim_ceiling:
                cos_conformance::contestable_finality::CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
        },
    };

    let set = ExternalObservationSetV1 {
        set_id: "set-golden-1".into(),
        effect_id: "effect-golden-1".into(),
        effect_lineage_id: "lineage-golden-1".into(),
        lifecycle_generation_id: "generation-golden-1".into(),
        route_id: "route-golden-1".into(),
        provider_id: "provider-golden-1".into(),
        provider_operation_id: "operation-golden-1".into(),
        provider_profile_root: "provider-profile-golden-1".into(),
        semantic_environment_root: "environment-golden-1".into(),
        observation_frontier_root: "frontier-golden-1".into(),
        qualification_profile_id: "finality-profile-golden-1".into(),
        observation_ids: ["obs-golden-1".into()].into_iter().collect(),
        target_state: ExternalFinalityStateV1::Applied,
        set_commitment: "set-golden-commitment-1".into(),
        claim_ceiling:
            cos_conformance::contestable_finality::CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
    };

    assert!(verify_witness_join_binding(
        &witness,
        &assessment,
        &evidence,
        Some(&eligibility),
        &set,
        "life-profile-golden-1",
        "frontier-golden-1",
    ));
}

#[test]
fn recomputed_but_wrong_d6o_receipt_is_rejected_at_d6p_join() {
    let fixture: UpstreamFixture = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_upstream_golden_vectors.json"
    ))
    .expect("upstream golden vectors must be valid JSON");

    let observation = golden_d6m_observation();
    let mut assessment: ObservationAssessmentV1 =
        serde_json::from_str(&vector(&fixture, "d6n-assessment-item").preimage_payload_utf8)
            .expect("D6N vector must deserialize");
    assessment.assessment_commitment = vector(&fixture, "d6n-assessment-item").sha256.clone();

    let mut eligibility: EvidenceEligibilityReceiptV1 =
        serde_json::from_str(&vector(&fixture, "d6o-eligibility-receipt").preimage_payload_utf8)
            .expect("D6O vector must deserialize");
    eligibility.eligibility_commitment = vector(&fixture, "d6o-eligibility-receipt").sha256.clone();

    let mut witness: FinalityWitnessEligibilityV1 =
        serde_json::from_str(&vector(&fixture, "d6p-witness").preimage_payload_utf8)
            .expect("D6P witness vector must deserialize");
    witness.witness_commitment = vector(&fixture, "d6p-witness").sha256.clone();

    let evidence = ExternalObservedEvidenceV1 {
        observation,
        observer_id: "observer-golden-1".into(),
        observer: ExternalObserverProfileV1 {
            observer_id: "observer-golden-1".into(),
            role: ExternalObserverRoleV1::IndependentObserver,
            observation_method: "independent-state-read".into(),
            provider_relationship: "external".into(),
            evidence_root: "evidence-golden-1".into(),
            custody_root: "custody-golden-1".into(),
            upstream_observer_ids: std::collections::BTreeSet::new(),
            upstream_evidence_roots: std::collections::BTreeSet::new(),
            semantic_environment_root: "environment-golden-1".into(),
            observation_profile_id: "observation-profile-golden-1".into(),
            independence: ObservationIndependenceV1::DeclaredIndependent,
            independence_commitment: "independence-golden-1".into(),
            claim_ceiling:
                cos_conformance::contestable_finality::CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
        },
    };

    let set = ExternalObservationSetV1 {
        set_id: "set-golden-1".into(),
        effect_id: "effect-golden-1".into(),
        effect_lineage_id: "lineage-golden-1".into(),
        lifecycle_generation_id: "generation-golden-1".into(),
        route_id: "route-golden-1".into(),
        provider_id: "provider-golden-1".into(),
        provider_operation_id: "operation-golden-1".into(),
        provider_profile_root: "provider-profile-golden-1".into(),
        semantic_environment_root: "environment-golden-1".into(),
        observation_frontier_root: "frontier-golden-1".into(),
        qualification_profile_id: "finality-profile-golden-1".into(),
        observation_ids: ["obs-golden-1".into()].into_iter().collect(),
        target_state: ExternalFinalityStateV1::Applied,
        set_commitment: "set-golden-commitment-1".into(),
        claim_ceiling:
            cos_conformance::contestable_finality::CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
    };

    assert!(eligibility.commitment_matches());
    eligibility.qualification_profile_id = "life-profile-substituted".into();
    eligibility.eligibility_commitment = eligibility.recomputed_commitment();
    assert!(eligibility.commitment_matches(), "mutation should be locally self-consistent");

    assert!(
        !verify_witness_join_binding(
            &witness,
            &assessment,
            &evidence,
            Some(&eligibility),
            &set,
            "life-profile-golden-1",
            "frontier-golden-1",
        ),
        "D6P must reject a self-consistent D6O receipt whose profile is substituted"
    );
}

#[test]
fn upstream_rejection_matrix_is_self_describing() {
    let fixture: UpstreamFixture = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_upstream_golden_vectors.json"
    ))
    .expect("upstream golden vectors must be valid JSON");

    let raw: Value = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_upstream_golden_vectors.json"
    ))
    .expect("upstream golden corpus must be valid JSON");
    let rejection_vectors = raw["rejection_vectors"]
        .as_array()
        .expect("rejection_vectors must be an array");
    assert_eq!(rejection_vectors.len(), 7);

    for rejection in rejection_vectors {
        assert!(rejection["id"].is_string(), "rejection id must be a string");
        assert!(rejection["layer"].is_string(), "rejection layer must be a string");
        assert!(rejection["mutated_field"].is_string(), "mutated_field must be a string");
        assert!(rejection["stored_commitment"].is_string(), "stored_commitment must be a vector id");
        assert!(rejection["earliest_rejection"].is_string(), "earliest_rejection must be a boundary");
        assert!(
            fixture.vectors.iter().any(|v| v.id == rejection["stored_commitment"].as_str().unwrap()),
            "{} references an unknown golden vector",
            rejection["id"].as_str().unwrap()
        );
        assert!(
            rejection["earliest_rejection"]
                .as_str()
                .unwrap()
                .starts_with(rejection["layer"].as_str().unwrap()),
            "{} earliest rejection must remain at or below its declared layer",
            rejection["id"].as_str().unwrap()
        );
        if rejection["id"] == "d6p-recomputed-d6o-profile-substitution" {
            assert_eq!(
                rejection["property"].as_str(),
                Some("local-integrity-preserved-cross-layer-provenance-rejected")
            );
        }
    }
}

#[test]
fn upstream_stale_bindings_fail_at_the_earliest_commitment_boundary() {
    let fixture: UpstreamFixture = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_upstream_golden_vectors.json"
    ))
    .expect("upstream golden vectors must be valid JSON");

    let d6m = vector(&fixture, "d6m-observation");
    let mut observation = golden_d6m_observation();
    observation.observed_state = ExternalObservedStateV1::Reversed;
    assert_eq!(observation.observation_commitment, d6m.sha256);
    assert!(!observation.commitment_matches(), "D6M must reject a stale commitment first");

    let d6n = vector(&fixture, "d6n-assessment-item");
    let mut assessment: ObservationAssessmentV1 =
        serde_json::from_str(&d6n.preimage_payload_utf8).expect("D6N vector must deserialize");
    assessment.assessment_commitment = d6n.sha256.clone();
    assessment.classification =
        cos_conformance::contestable_finality::ObservationClassificationV1::ContradictoryIndependent;
    assert!(!assessment.commitment_matches(), "D6N must reject its stale commitment");

    let d6o = vector(&fixture, "d6o-eligibility-receipt");
    let mut eligibility: EvidenceEligibilityReceiptV1 =
        serde_json::from_str(&d6o.preimage_payload_utf8).expect("D6O vector must deserialize");
    eligibility.eligibility_commitment = d6o.sha256.clone();
    eligibility.disposition =
        cos_conformance::observer_lifecycle::EvidenceEligibilityDispositionV1::BlockedLifecycle;
    assert!(!eligibility.commitment_matches(), "D6O must reject its stale commitment");

    let d6pw = vector(&fixture, "d6p-witness");
    let mut witness: FinalityWitnessEligibilityV1 =
        serde_json::from_str(&d6pw.preimage_payload_utf8).expect("D6P witness vector must deserialize");
    witness.witness_commitment = d6pw.sha256.clone();
    witness.d6n_classification =
        cos_conformance::contestable_finality::ObservationClassificationV1::ContradictoryIndependent;
    assert!(!witness.commitment_matches(), "D6P witness must reject its stale commitment");

    let d6pc = vector(&fixture, "d6p-composition");
    let mut composition: FinalityEligibilityCompositionV1 =
        serde_json::from_str(&d6pc.preimage_payload_utf8).expect("D6P composition vector must deserialize");
    composition.composition_commitment = d6pc.sha256.clone();
    composition.disposition =
        cos_conformance::finality_eligibility_composition::FinalityEligibilityDispositionV1::Contested;
    assert!(!composition.commitment_matches(), "D6P composition must reject its stale commitment");

    let d6pr = vector(&fixture, "d6p-receipt");
    let mut receipt: CurrentFinalityEligibilityReceiptV1 =
        serde_json::from_str(&d6pr.preimage_payload_utf8).expect("D6P receipt vector must deserialize");
    receipt.receipt_commitment = d6pr.sha256.clone();
    receipt.eligible_independent_count = 2;
    assert!(!receipt.commitment_matches(), "D6P receipt must reject its stale commitment");
}
