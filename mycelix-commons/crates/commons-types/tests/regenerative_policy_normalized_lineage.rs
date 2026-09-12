// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use commons_types::{
    verify_regenerative_policy_normalized_lineage_evidence, MaritimeEvidenceKind,
    RegenerativeGenerationCountEvidenceV1, RegenerativePolicyNormalizedLineageEvidenceV1,
    RegenerativeViabilityEvidenceV1, RegenerativeViabilityHorizonV1,
    RegenerativeViabilityRoleEvidenceV1, REGENERATIVE_POLICY_NORMALIZED_LINEAGE_SCHEMA_V1,
    REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1,
};

fn viability() -> RegenerativeViabilityEvidenceV1 {
    RegenerativeViabilityEvidenceV1 {
        schema_version: REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1,
        genome_binding: "genome:manta-v3:sha256:example".into(),
        lineage_evidence_binding: "lineage:blake3:manta-v3-example".into(),
        viability_profile_binding: "viability-profile:manta-v3:v1".into(),
        static_viability_report_binding: "symthaea-report:manta-v3:sha256:example".into(),
        dynamic_simulation_binding: "symtropy-run:manta-v3:sha256:example".into(),
        closure_model_binding: "closure-model:manta-v3:sha256:example".into(),
        flow_support_binding: "flow-support:manta-v3:sha256:example".into(),
        period_duration_ms: 1,
        roles: vec![
            RegenerativeViabilityRoleEvidenceV1 {
                role_id: "operation".into(),
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(20),
                dynamic_first_unavailable_tick: Some(21),
                fully_modeled_support: true,
                limiting_dependency_refs: vec!["dependency:platform-spares".into()],
            },
            RegenerativeViabilityRoleEvidenceV1 {
                role_id: "successor_construction".into(),
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(4),
                dynamic_first_unavailable_tick: Some(5),
                fully_modeled_support: true,
                limiting_dependency_refs: vec!["dependency:forge-tooling".into()],
            },
            RegenerativeViabilityRoleEvidenceV1 {
                role_id: "successor_qualification".into(),
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(5),
                dynamic_first_unavailable_tick: Some(6),
                fully_modeled_support: true,
                limiting_dependency_refs: vec!["dependency:reactor-service".into()],
            },
        ],
        regenerative_viability_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(4),
        limiting_role_ids: vec!["successor_construction".into()],
        fully_modeled_regenerative_viability: true,
    }
}

fn normalized(viability: &RegenerativeViabilityEvidenceV1) -> RegenerativePolicyNormalizedLineageEvidenceV1 {
    RegenerativePolicyNormalizedLineageEvidenceV1 {
        schema_version: REGENERATIVE_POLICY_NORMALIZED_LINEAGE_SCHEMA_V1,
        normalized_profile_id: "manta-v3-policy-normalized-v1".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        symthaea_normalized_profile_binding:
            "git:Luminous-Dynamics/symthaea:78ad974603ed2ac6ff28dff32fb18dc82cdcaf94"
                .into(),
        reproduction_policy_id: "minimum-descendant-maturity-v2".into(),
        reproduction_policy_evidence_binding: "policy-evidence:maturity-periods:2".into(),
        maturity_periods: 2,
        physical_successor_reproduction_horizon:
            RegenerativeViabilityHorizonV1::FinitePeriods(4),
        physical_regenerative_viability_horizon:
            RegenerativeViabilityHorizonV1::FinitePeriods(4),
        fully_modeled_successor_reproduction: true,
        fully_modeled_regenerative_viability: true,
        founded_descendant_generations: RegenerativeGenerationCountEvidenceV1::Finite(2),
        maturity_completed_descendant_generations:
            RegenerativeGenerationCountEvidenceV1::Finite(2),
        descendant_reproduction_transitions: RegenerativeGenerationCountEvidenceV1::Finite(1),
        terminal_generation_residual_periods: Some(2),
    }
}

#[test]
fn normalized_profile_agrees_with_existing_physical_viability_provenance() {
    let viability = viability();
    let normalized = normalized(&viability);
    assert_eq!(
        verify_regenerative_policy_normalized_lineage_evidence(&viability, &normalized),
        Ok(())
    );
    assert_eq!(normalized.content_digest().unwrap().len(), 64);
}

#[test]
fn physical_claim_drift_fails_closed() {
    let viability = viability();

    let mut bad_digest = normalized(&viability);
    bad_digest.viability_evidence_content_digest = "0".repeat(64);
    assert!(verify_regenerative_policy_normalized_lineage_evidence(&viability, &bad_digest).is_err());

    let mut bad_reproduction = normalized(&viability);
    bad_reproduction.physical_successor_reproduction_horizon =
        RegenerativeViabilityHorizonV1::FinitePeriods(5);
    assert!(verify_regenerative_policy_normalized_lineage_evidence(&viability, &bad_reproduction).is_err());

    let mut bad_overall = normalized(&viability);
    bad_overall.physical_regenerative_viability_horizon =
        RegenerativeViabilityHorizonV1::FinitePeriods(5);
    assert!(verify_regenerative_policy_normalized_lineage_evidence(&viability, &bad_overall).is_err());

    let mut bad_modeling = normalized(&viability);
    bad_modeling.fully_modeled_successor_reproduction = false;
    assert!(verify_regenerative_policy_normalized_lineage_evidence(&viability, &bad_modeling).is_err());
}

#[test]
fn terminal_generation_metrics_are_strict_and_conditioning_consistent() {
    let viability = viability();

    let mut impossible_order = normalized(&viability);
    impossible_order.maturity_completed_descendant_generations =
        RegenerativeGenerationCountEvidenceV1::Finite(3);
    assert!(impossible_order.validate().is_err());

    let mut zero_residual = normalized(&viability);
    zero_residual.terminal_generation_residual_periods = Some(0);
    assert!(zero_residual.validate().is_err());

    let mut mixed_conditioning = normalized(&viability);
    mixed_conditioning.founded_descendant_generations =
        RegenerativeGenerationCountEvidenceV1::IndefiniteUnderStaticModel;
    assert!(mixed_conditioning.validate().is_err());
}

#[test]
fn normalized_profile_round_trips_over_existing_maritime_transport() {
    let viability = viability();
    let normalized = normalized(&viability);
    let envelope = normalized
        .to_maritime_envelope(
            "manta-civil-demo-01",
            7,
            0,
            1_788_900_004_000_000,
            "evidence:policy-normalized-lineage-event-01",
        )
        .unwrap();
    assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
    let decoded: RegenerativePolicyNormalizedLineageEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert_eq!(decoded, normalized);
}

#[test]
fn unknown_claim_injection_is_rejected() {
    let viability = viability();
    let json = normalized(&viability).to_payload_json().unwrap();
    let injected = json.replacen(
        "{",
        "{\"manufacturing_authority\":true,",
        1,
    );
    assert!(serde_json::from_str::<RegenerativePolicyNormalizedLineageEvidenceV1>(&injected).is_err());
}
