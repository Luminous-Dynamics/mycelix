use commons_types::{
    MaritimeEvidenceKind, RegenerativeViabilityEvidenceV1, RegenerativeViabilityHorizonV1,
};

const FIXTURE: &str = include_str!("../fixtures/regenerative-viability-v1-manta.json");

#[test]
fn canonical_manta_viability_fixture_preserves_static_dynamic_and_lineage_bindings() {
    let evidence: RegenerativeViabilityEvidenceV1 = serde_json::from_str(FIXTURE).unwrap();
    evidence.validate().unwrap();

    assert_eq!(evidence.roles.len(), 3);
    assert_eq!(evidence.roles[0].role_id, "operation");
    assert_eq!(
        evidence.roles[0].static_horizon,
        RegenerativeViabilityHorizonV1::FinitePeriods(100)
    );
    assert_eq!(evidence.roles[0].dynamic_first_unavailable_tick, Some(101));

    assert_eq!(evidence.roles[1].role_id, "successor_construction");
    assert_eq!(
        evidence.roles[1].static_horizon,
        RegenerativeViabilityHorizonV1::FinitePeriods(40)
    );
    assert_eq!(evidence.roles[1].dynamic_first_unavailable_tick, Some(41));

    assert_eq!(evidence.roles[2].role_id, "successor_qualification");
    assert_eq!(
        evidence.roles[2].static_horizon,
        RegenerativeViabilityHorizonV1::FinitePeriods(30)
    );
    assert_eq!(evidence.roles[2].dynamic_first_unavailable_tick, Some(31));
    assert_eq!(
        evidence.roles[2].limiting_dependency_refs,
        vec!["dependency:metrology"]
    );

    assert_eq!(
        evidence.regenerative_viability_horizon,
        RegenerativeViabilityHorizonV1::FinitePeriods(30)
    );
    assert_eq!(
        evidence.limiting_role_ids,
        vec!["successor_qualification"]
    );
    assert!(evidence.fully_modeled_regenerative_viability);

    for binding in [
        &evidence.genome_binding,
        &evidence.lineage_evidence_binding,
        &evidence.viability_profile_binding,
        &evidence.static_viability_report_binding,
        &evidence.dynamic_simulation_binding,
        &evidence.closure_model_binding,
        &evidence.flow_support_binding,
    ] {
        assert!(binding.contains(':'));
    }

    let digest = evidence.content_digest().unwrap();
    assert_eq!(digest.len(), 64);

    let envelope = evidence
        .to_maritime_envelope(
            "manta-civil-demo-01",
            5,
            0,
            1_788_900_002_000_000,
            "evidence:regenerative-viability-fixture",
        )
        .unwrap();
    assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
    let decoded: RegenerativeViabilityEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert_eq!(decoded, evidence);
}

#[test]
fn fixture_rejects_semantic_claim_injection_through_unknown_fields() {
    let injected = FIXTURE.replacen(
        "\"schema_version\": 1,",
        "\"schema_version\": 1, \"qualification_authority_granted\": true,",
        1,
    );
    assert!(serde_json::from_str::<RegenerativeViabilityEvidenceV1>(&injected).is_err());
}
