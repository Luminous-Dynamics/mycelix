// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use commons_types::{
    verify_regenerative_policy_surface_revision_evidence, MaritimeEvidenceKind,
    RegenerativeGenerationCountEvidenceV1, RegenerativePolicySensitivityPointEvidenceV1,
    RegenerativePolicySensitivitySurfaceEvidenceV1, RegenerativePolicySurfaceRevisionCauseV1,
    RegenerativePolicySurfaceRevisionEvidenceV1, RegenerativeViabilityEvidenceV1,
    RegenerativeViabilityHorizonV1, RegenerativeViabilityRoleEvidenceV1,
    REGENERATIVE_POLICY_SENSITIVITY_SURFACE_SCHEMA_V1,
    REGENERATIVE_POLICY_SURFACE_REVISION_SCHEMA_V1, REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1,
};

fn viability(horizon: u64, subject: &str) -> RegenerativeViabilityEvidenceV1 {
    RegenerativeViabilityEvidenceV1 {
        schema_version: REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1,
        genome_binding: "genome:manta-v3:sha256:stable-subject".into(),
        lineage_evidence_binding: "lineage:blake3:manta-v3-stable-subject".into(),
        viability_profile_binding: format!("viability-profile:manta-v3:{subject}"),
        static_viability_report_binding: format!("symthaea-report:manta-v3:{subject}"),
        dynamic_simulation_binding: format!("symtropy-run:manta-v3:{subject}"),
        closure_model_binding: format!("closure-model:manta-v3:{subject}"),
        flow_support_binding: format!("flow-support:manta-v3:{subject}"),
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
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(horizon),
                dynamic_first_unavailable_tick: Some(horizon + 1),
                fully_modeled_support: true,
                limiting_dependency_refs: vec!["dependency:forge-tooling".into()],
            },
            RegenerativeViabilityRoleEvidenceV1 {
                role_id: "successor_qualification".into(),
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(horizon + 1),
                dynamic_first_unavailable_tick: Some(horizon + 2),
                fully_modeled_support: true,
                limiting_dependency_refs: vec!["dependency:reactor-service".into()],
            },
        ],
        regenerative_viability_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(horizon),
        limiting_role_ids: vec!["successor_construction".into()],
        fully_modeled_regenerative_viability: true,
    }
}

fn point(
    maturity: u64,
    founded: u64,
    matured: u64,
    transitions: u64,
    residual: u64,
) -> RegenerativePolicySensitivityPointEvidenceV1 {
    RegenerativePolicySensitivityPointEvidenceV1 {
        reproduction_policy_id: format!("manta-v3-maturity-{maturity}"),
        reproduction_policy_evidence_binding: format!("policy:manta-v3:maturity:{maturity}:v1"),
        maturity_periods: maturity,
        founded_descendant_generations: RegenerativeGenerationCountEvidenceV1::Finite(founded),
        maturity_completed_descendant_generations:
            RegenerativeGenerationCountEvidenceV1::Finite(matured),
        descendant_reproduction_transitions:
            RegenerativeGenerationCountEvidenceV1::Finite(transitions),
        terminal_generation_residual_periods: Some(residual),
    }
}

fn h4_surface(viability: &RegenerativeViabilityEvidenceV1) -> RegenerativePolicySensitivitySurfaceEvidenceV1 {
    RegenerativePolicySensitivitySurfaceEvidenceV1 {
        schema_version: REGENERATIVE_POLICY_SENSITIVITY_SURFACE_SCHEMA_V1,
        surface_id: "manta-v3-policy-surface-h4".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        symthaea_surface_binding:
            "git:Luminous-Dynamics/symthaea:bedbd880490b34d29e0608b1b6b0780896574822".into(),
        symtropy_surface_binding:
            "git:Luminous-Dynamics/symtropy:644dbac035bf516b4651c916b5e5631cb45d3f62".into(),
        shared_surface_fixture_binding:
            "git-blob:f1bbe16a316017cf366a441071ed4a2378d0e696".into(),
        physical_successor_reproduction_horizon:
            RegenerativeViabilityHorizonV1::FinitePeriods(4),
        physical_regenerative_viability_horizon:
            RegenerativeViabilityHorizonV1::FinitePeriods(4),
        fully_modeled_successor_reproduction: true,
        fully_modeled_regenerative_viability: true,
        policy_points: vec![
            point(1, 4, 4, 3, 1),
            point(2, 2, 2, 1, 2),
            point(3, 2, 1, 1, 1),
            point(4, 1, 1, 0, 4),
        ],
    }
}

fn h3_surface(viability: &RegenerativeViabilityEvidenceV1) -> RegenerativePolicySensitivitySurfaceEvidenceV1 {
    RegenerativePolicySensitivitySurfaceEvidenceV1 {
        schema_version: REGENERATIVE_POLICY_SENSITIVITY_SURFACE_SCHEMA_V1,
        surface_id: "manta-v3-policy-surface-h3-after-reserve-loss".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        symthaea_surface_binding:
            "git:Luminous-Dynamics/symthaea:5f80e33a9db458e9c5924ea0acaa6bd9dbd2b55f".into(),
        symtropy_surface_binding:
            "git:Luminous-Dynamics/symtropy:ed96021386a6130119420e1db987d7b7f90a3b33".into(),
        shared_surface_fixture_binding:
            "control-contract:git:Luminous-Dynamics/symtropy:ed96021386a6130119420e1db987d7b7f90a3b33".into(),
        physical_successor_reproduction_horizon:
            RegenerativeViabilityHorizonV1::FinitePeriods(3),
        physical_regenerative_viability_horizon:
            RegenerativeViabilityHorizonV1::FinitePeriods(3),
        fully_modeled_successor_reproduction: true,
        fully_modeled_regenerative_viability: true,
        policy_points: vec![
            point(1, 3, 3, 2, 1),
            point(2, 2, 1, 1, 1),
            point(3, 1, 1, 0, 3),
            point(4, 1, 0, 0, 3),
        ],
    }
}

fn revision(
    prior: &RegenerativePolicySensitivitySurfaceEvidenceV1,
    successor: &RegenerativePolicySensitivitySurfaceEvidenceV1,
) -> RegenerativePolicySurfaceRevisionEvidenceV1 {
    RegenerativePolicySurfaceRevisionEvidenceV1 {
        schema_version: REGENERATIVE_POLICY_SURFACE_REVISION_SCHEMA_V1,
        revision_id: "manta-v3-h4-to-h3-post-handoff-reserve-loss".into(),
        cause: RegenerativePolicySurfaceRevisionCauseV1::AssumptionChange,
        prior_surface_content_digest: prior.content_digest().unwrap(),
        successor_surface_content_digest: successor.content_digest().unwrap(),
        dynamic_assumption_change_binding:
            "git:Luminous-Dynamics/symtropy:ed96021386a6130119420e1db987d7b7f90a3b33".into(),
        calibration_evidence_binding:
            "git:Luminous-Dynamics/symthaea:5f80e33a9db458e9c5924ea0acaa6bd9dbd2b55f".into(),
    }
}

#[test]
fn assumption_change_supersedes_without_mutating_prior_surface() {
    let prior_viability = viability(4, "pre-shock-h4");
    let successor_viability = viability(3, "post-shock-h3");
    let prior = h4_surface(&prior_viability);
    let successor = h3_surface(&successor_viability);
    let revision = revision(&prior, &successor);

    assert_eq!(
        verify_regenerative_policy_surface_revision_evidence(
            &prior_viability,
            &prior,
            &successor_viability,
            &successor,
            &revision,
        ),
        Ok(())
    );
    assert_ne!(prior.content_digest().unwrap(), successor.content_digest().unwrap());
    assert_eq!(revision.content_digest().unwrap().len(), 64);
}

#[test]
fn revision_rejects_digest_drift_policy_change_and_same_subject_rewrite() {
    let prior_viability = viability(4, "pre-shock-h4");
    let successor_viability = viability(3, "post-shock-h3");
    let prior = h4_surface(&prior_viability);
    let successor = h3_surface(&successor_viability);

    let mut bad_digest = revision(&prior, &successor);
    bad_digest.prior_surface_content_digest = "0".repeat(64);
    assert!(verify_regenerative_policy_surface_revision_evidence(
        &prior_viability,
        &prior,
        &successor_viability,
        &successor,
        &bad_digest,
    )
    .is_err());

    let mut changed_policy = successor.clone();
    changed_policy.policy_points[0].reproduction_policy_id = "different-policy".into();
    let changed_policy_revision = revision(&prior, &changed_policy);
    assert!(verify_regenerative_policy_surface_revision_evidence(
        &prior_viability,
        &prior,
        &successor_viability,
        &changed_policy,
        &changed_policy_revision,
    )
    .is_err());

    let mut same_id = successor.clone();
    same_id.surface_id = prior.surface_id.clone();
    let same_id_revision = revision(&prior, &same_id);
    assert!(verify_regenerative_policy_surface_revision_evidence(
        &prior_viability,
        &prior,
        &successor_viability,
        &same_id,
        &same_id_revision,
    )
    .is_err());
}

#[test]
fn revision_requires_changed_physical_coordinate_on_same_genome_lineage() {
    let prior_viability = viability(4, "pre-shock-h4");
    let prior = h4_surface(&prior_viability);

    let same_physics_viability = viability(4, "new-evidence-same-h4");
    let mut same_physics_surface = h4_surface(&same_physics_viability);
    same_physics_surface.surface_id = "manta-v3-policy-surface-h4-new-evidence".into();
    let same_physics_revision = revision(&prior, &same_physics_surface);
    assert!(verify_regenerative_policy_surface_revision_evidence(
        &prior_viability,
        &prior,
        &same_physics_viability,
        &same_physics_surface,
        &same_physics_revision,
    )
    .is_err());

    let mut other_lineage_viability = viability(3, "other-lineage-h3");
    other_lineage_viability.genome_binding = "genome:other-v3:sha256:subject".into();
    let other_surface = h3_surface(&other_lineage_viability);
    let other_revision = revision(&prior, &other_surface);
    assert!(verify_regenerative_policy_surface_revision_evidence(
        &prior_viability,
        &prior,
        &other_lineage_viability,
        &other_surface,
        &other_revision,
    )
    .is_err());
}

#[test]
fn revision_round_trips_over_maritime_transport() {
    let prior_viability = viability(4, "pre-shock-h4");
    let successor_viability = viability(3, "post-shock-h3");
    let prior = h4_surface(&prior_viability);
    let successor = h3_surface(&successor_viability);
    let revision = revision(&prior, &successor);

    let envelope = revision
        .to_maritime_envelope(
            "manta-civil-demo-01",
            9,
            0,
            1_788_900_006_000_000,
            "evidence:policy-surface-revision-event-01",
        )
        .unwrap();
    assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
    let decoded: RegenerativePolicySurfaceRevisionEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert_eq!(decoded, revision);
}
