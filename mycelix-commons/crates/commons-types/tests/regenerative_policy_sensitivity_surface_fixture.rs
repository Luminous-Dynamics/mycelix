// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use commons_types::{
    verify_regenerative_policy_sensitivity_surface_evidence, MaritimeEvidenceKind,
    RegenerativeGenerationCountEvidenceV1, RegenerativePolicySensitivityPointEvidenceV1,
    RegenerativePolicySensitivitySurfaceEvidenceV1, RegenerativeViabilityEvidenceV1,
    RegenerativeViabilityHorizonV1, RegenerativeViabilityRoleEvidenceV1,
    REGENERATIVE_POLICY_SENSITIVITY_SURFACE_SCHEMA_V1,
    REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1,
};

const FIXTURE: &str = include_str!("../fixtures/manta-forge-policy-sensitivity-surface-v1.txt");

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
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(8),
                dynamic_first_unavailable_tick: Some(9),
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

fn fixture_points() -> Vec<RegenerativePolicySensitivityPointEvidenceV1> {
    FIXTURE
        .lines()
        .filter_map(|line| line.strip_prefix("policy="))
        .map(|line| {
            let mut maturity_periods = None;
            let mut founded = None;
            let mut matured = None;
            let mut transitions = None;
            let mut residual = None;
            for field in line.split('|') {
                let (key, value) = field.split_once(':').unwrap();
                let value: u64 = value.parse().unwrap();
                match key {
                    "maturity_periods" => maturity_periods = Some(value),
                    "founded_descendants" => founded = Some(value),
                    "maturity_completed_descendants" => matured = Some(value),
                    "descendant_reproduction_transitions" => transitions = Some(value),
                    "terminal_residual_periods" => residual = Some(value),
                    other => panic!("unknown policy-surface fixture field {other}"),
                }
            }
            let maturity_periods = maturity_periods.unwrap();
            RegenerativePolicySensitivityPointEvidenceV1 {
                reproduction_policy_id: format!("manta-v3-maturity-{maturity_periods}"),
                reproduction_policy_evidence_binding: format!(
                    "policy:manta-v3:maturity:{maturity_periods}:v1"
                ),
                maturity_periods,
                founded_descendant_generations:
                    RegenerativeGenerationCountEvidenceV1::Finite(founded.unwrap()),
                maturity_completed_descendant_generations:
                    RegenerativeGenerationCountEvidenceV1::Finite(matured.unwrap()),
                descendant_reproduction_transitions:
                    RegenerativeGenerationCountEvidenceV1::Finite(transitions.unwrap()),
                terminal_generation_residual_periods: Some(residual.unwrap()),
            }
        })
        .collect()
}

fn surface(viability: &RegenerativeViabilityEvidenceV1) -> RegenerativePolicySensitivitySurfaceEvidenceV1 {
    RegenerativePolicySensitivitySurfaceEvidenceV1 {
        schema_version: REGENERATIVE_POLICY_SENSITIVITY_SURFACE_SCHEMA_V1,
        surface_id: "manta-v3-policy-sensitivity-surface-v1".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        symthaea_surface_binding:
            "git:Luminous-Dynamics/symthaea:bedbd880490b34d29e0608b1b6b0780896574822"
                .into(),
        symtropy_surface_binding:
            "git:Luminous-Dynamics/symtropy:644dbac035bf516b4651c916b5e5631cb45d3f62"
                .into(),
        shared_surface_fixture_binding:
            "git-blob:f1bbe16a316017cf366a441071ed4a2378d0e696".into(),
        physical_successor_reproduction_horizon:
            RegenerativeViabilityHorizonV1::FinitePeriods(4),
        physical_regenerative_viability_horizon:
            RegenerativeViabilityHorizonV1::FinitePeriods(4),
        fully_modeled_successor_reproduction: true,
        fully_modeled_regenerative_viability: true,
        policy_points: fixture_points(),
    }
}

#[test]
fn shared_surface_preserves_one_physical_coordinate_and_four_policy_points() {
    let viability = viability();
    let surface = surface(&viability);
    assert_eq!(surface.policy_points.len(), 4);
    assert_eq!(
        verify_regenerative_policy_sensitivity_surface_evidence(&viability, &surface),
        Ok(())
    );
    assert_eq!(surface.content_digest().unwrap().len(), 64);
}

#[test]
fn physical_provenance_drift_fails_closed() {
    let viability = viability();

    let mut wrong_digest = surface(&viability);
    wrong_digest.viability_evidence_content_digest = "0".repeat(64);
    assert!(verify_regenerative_policy_sensitivity_surface_evidence(&viability, &wrong_digest).is_err());

    let mut wrong_successor_horizon = surface(&viability);
    wrong_successor_horizon.physical_successor_reproduction_horizon =
        RegenerativeViabilityHorizonV1::FinitePeriods(5);
    assert!(verify_regenerative_policy_sensitivity_surface_evidence(
        &viability,
        &wrong_successor_horizon
    )
    .is_err());
}

#[test]
fn policy_family_shape_and_monotonicity_fail_closed() {
    let viability = viability();

    let mut unsorted = surface(&viability);
    unsorted.policy_points.swap(1, 2);
    assert!(unsorted.validate().is_err());

    let mut duplicate_policy = surface(&viability);
    duplicate_policy.policy_points[1].reproduction_policy_id =
        duplicate_policy.policy_points[0].reproduction_policy_id.clone();
    assert!(duplicate_policy.validate().is_err());

    let mut improves_under_stricter_policy = surface(&viability);
    improves_under_stricter_policy.policy_points[2].founded_descendant_generations =
        RegenerativeGenerationCountEvidenceV1::Finite(3);
    assert!(improves_under_stricter_policy.validate().is_err());
}

#[test]
fn surface_round_trips_over_existing_maritime_transport() {
    let viability = viability();
    let surface = surface(&viability);
    let envelope = surface
        .to_maritime_envelope(
            "manta-civil-demo-01",
            8,
            0,
            1_788_900_005_000_000,
            "evidence:policy-sensitivity-surface-event-01",
        )
        .unwrap();
    assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
    let decoded: RegenerativePolicySensitivitySurfaceEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert_eq!(decoded, surface);
}

#[test]
fn surface_fixture_binding_and_unknown_field_shape_are_strict() {
    let viability = viability();
    let surface = surface(&viability);
    assert_eq!(
        surface.shared_surface_fixture_binding,
        "git-blob:f1bbe16a316017cf366a441071ed4a2378d0e696"
    );

    let json = surface.to_payload_json().unwrap();
    let injected = json.replacen("{", "{\"authority_override\":true,", 1);
    assert!(serde_json::from_str::<RegenerativePolicySensitivitySurfaceEvidenceV1>(&injected).is_err());
}
