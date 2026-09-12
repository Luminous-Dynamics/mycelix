use mycelix_commons_types::*;

const FIXTURE: &str =
    include_str!("../fixtures/manta-forge-intergenerational-support-basis-v1.txt");

fn scalar(key: &str) -> u64 {
    FIXTURE
        .lines()
        .find_map(|line| line.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("missing fixture scalar {key}"))
        .parse()
        .unwrap()
}

fn bool_scalar(key: &str) -> bool {
    FIXTURE
        .lines()
        .find_map(|line| line.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("missing fixture bool {key}"))
        .parse()
        .unwrap()
}

fn viability() -> RegenerativeViabilityEvidenceV1 {
    RegenerativeViabilityEvidenceV1 {
        schema_version: REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1,
        genome_binding: "genome:manta-v3:basis".into(),
        lineage_evidence_binding: "lineage:manta-v3:basis".into(),
        viability_profile_binding: "viability-profile:manta-v3:basis".into(),
        static_viability_report_binding: "symthaea:manta-v3:basis-static".into(),
        dynamic_simulation_binding: "symtropy:manta-v3:basis-dynamic".into(),
        closure_model_binding: "closure-model:manta-v3:basis".into(),
        flow_support_binding: "flow-support:manta-v3:basis".into(),
        period_duration_ms: scalar("source_period_duration_ms"),
        roles: vec![
            RegenerativeViabilityRoleEvidenceV1 {
                role_id: "operation".into(),
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(100),
                dynamic_first_unavailable_tick: None,
                fully_modeled_support: true,
                limiting_dependency_refs: vec!["dependency:operation-support".into()],
            },
            RegenerativeViabilityRoleEvidenceV1 {
                role_id: "successor_construction".into(),
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(
                    scalar("source_horizon_periods"),
                ),
                dynamic_first_unavailable_tick: Some(5),
                fully_modeled_support: true,
                limiting_dependency_refs: vec!["dependency:forge-tooling-v3".into()],
            },
            RegenerativeViabilityRoleEvidenceV1 {
                role_id: "successor_qualification".into(),
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(
                    scalar("source_horizon_periods"),
                ),
                dynamic_first_unavailable_tick: Some(5),
                fully_modeled_support: true,
                limiting_dependency_refs: vec!["dependency:reactor-service-v3".into()],
            },
        ],
        regenerative_viability_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(
            scalar("source_horizon_periods"),
        ),
        limiting_role_ids: vec![
            "successor_construction".into(),
            "successor_qualification".into(),
        ],
        fully_modeled_regenerative_viability: true,
    }
}

fn preserved_surface(
    viability: &RegenerativeViabilityEvidenceV1,
) -> RegenerativePolicySensitivitySurfaceEvidenceV1 {
    let policy_points = FIXTURE
        .lines()
        .filter_map(|line| line.strip_prefix("preserved_basis_policy="))
        .map(|line| {
            let mut maturity = 0;
            let mut founded = 0;
            let mut matured = 0;
            let mut transitions = 0;
            let mut residual = 0;
            for field in line.split('|') {
                let (key, value) = field.split_once(':').unwrap();
                let value: u64 = value.parse().unwrap();
                match key {
                    "maturity_periods" => maturity = value,
                    "founded_descendants" => founded = value,
                    "maturity_completed_descendants" => matured = value,
                    "descendant_reproduction_transitions" => transitions = value,
                    "terminal_residual_periods" => residual = value,
                    other => panic!("unknown fixture policy field {other}"),
                }
            }
            RegenerativePolicySensitivityPointEvidenceV1 {
                reproduction_policy_id: format!("maturity-{maturity}"),
                reproduction_policy_evidence_binding: format!(
                    "reproduction-policy:maturity-{maturity}:v1"
                ),
                maturity_periods: maturity,
                founded_descendant_generations: RegenerativeGenerationCountEvidenceV1::Finite(
                    founded,
                ),
                maturity_completed_descendant_generations:
                    RegenerativeGenerationCountEvidenceV1::Finite(matured),
                descendant_reproduction_transitions:
                    RegenerativeGenerationCountEvidenceV1::Finite(transitions),
                terminal_generation_residual_periods: Some(residual),
            }
        })
        .collect();

    RegenerativePolicySensitivitySurfaceEvidenceV1 {
        schema_version: REGENERATIVE_POLICY_SENSITIVITY_SURFACE_SCHEMA_V1,
        surface_id: "manta-v3-support-basis-safe-surface".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        symthaea_surface_binding: "symthaea:pr-2052:safe-surface".into(),
        symtropy_surface_binding: "symtropy:pr-821:preserved-basis".into(),
        shared_surface_fixture_binding:
            "git-blob:d434bc443f9d2800a518c231ad8d8c031e956835".into(),
        physical_successor_reproduction_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(
            scalar("source_horizon_periods"),
        ),
        physical_regenerative_viability_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(
            scalar("source_horizon_periods"),
        ),
        fully_modeled_successor_reproduction: true,
        fully_modeled_regenerative_viability: true,
        policy_points,
    }
}

fn assessment(
    source_id: &str,
    successor_id: &str,
    source_draw: u64,
    successor_draw: u64,
    successor_period_ms: u64,
) -> RegenerativeSupportBasisAssessmentEvidenceV1 {
    let source_period_ms = scalar("source_period_duration_ms");
    let preserved = u128::from(source_draw) * u128::from(successor_period_ms)
        == u128::from(successor_draw) * u128::from(source_period_ms);
    RegenerativeSupportBasisAssessmentEvidenceV1 {
        source_dependency_id: source_id.into(),
        successor_dependency_id: successor_id.into(),
        quantity_basis_equivalence_binding: format!(
            "quantity-basis:{source_id}:{successor_id}"
        ),
        source_stockpile_draw_units_per_period: source_draw,
        successor_stockpile_draw_units_per_period: successor_draw,
        source_period_duration_ms: source_period_ms,
        successor_period_duration_ms: successor_period_ms,
        normalized_stockpile_draw_rate_preserved: preserved,
    }
}

fn basis(
    viability: &RegenerativeViabilityEvidenceV1,
    successor_tooling_draw: u64,
    successor_period_ms: u64,
    authorized_surface: Option<String>,
) -> RegenerativeSupportBasisEvidenceV1 {
    let assessments = vec![
        assessment(
            "forge-tooling-v3",
            "forge-tooling-v4",
            scalar("source_tooling_demand_units_per_period"),
            successor_tooling_draw,
            successor_period_ms,
        ),
        assessment(
            "reactor-service-v3",
            "reactor-service-v4",
            1,
            1,
            successor_period_ms,
        ),
    ];
    let safe = assessments
        .iter()
        .all(|assessment| assessment.normalized_stockpile_draw_rate_preserved);
    RegenerativeSupportBasisEvidenceV1 {
        schema_version: REGENERATIVE_SUPPORT_BASIS_EVIDENCE_SCHEMA_V1,
        basis_evidence_id: if safe {
            "manta-v3-v4-support-basis-preserved".into()
        } else {
            "manta-v3-v4-support-basis-changed".into()
        },
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        symthaea_support_basis_binding:
            "symthaea:pr-2052:fcb546e0456f164d5bedeee27e65a7ccc09945ea".into(),
        symtropy_support_basis_binding:
            "symtropy:pr-821:bc074d70697bd1ea156efff820c9f266a6a54402".into(),
        shared_support_basis_fixture_binding:
            "git-blob:d434bc443f9d2800a518c231ad8d8c031e956835".into(),
        basis_policy_id: "manta-forge-bootstrap-support-basis-v1".into(),
        basis_policy_evidence_binding: "basis-policy:manta-forge:v1".into(),
        source_model_binding: "symthaea:model:closure-v3-basis".into(),
        successor_model_binding: "symthaea:model:closure-v4-basis".into(),
        assessments,
        scalar_runway_projection_safe: safe,
        authorized_policy_surface_content_digest: authorized_surface,
    }
}

#[test]
fn unsafe_counterexample_is_preserved_but_cannot_authorize_a_surface() {
    assert!(!bool_scalar("changed_basis_scalar_projection_safe"));
    let viability = viability();
    let unsafe_basis = basis(
        &viability,
        scalar("changed_successor_tooling_demand_units_per_period"),
        scalar("changed_successor_period_duration_ms"),
        None,
    );
    assert_eq!(unsafe_basis.validate(), Ok(()));
    assert_eq!(
        verify_regenerative_support_basis_evidence(&viability, &unsafe_basis),
        Ok(())
    );
    assert!(!unsafe_basis.scalar_runway_projection_safe);

    let surface = preserved_surface(&viability);
    assert!(verify_regenerative_support_basis_surface_authorization(
        &unsafe_basis,
        &surface
    )
    .is_err());

    let mut illegal = unsafe_basis.clone();
    illegal.authorized_policy_surface_content_digest = Some(surface.content_digest().unwrap());
    assert!(illegal.validate().is_err());
}

#[test]
fn preserved_basis_can_bind_the_exact_authorized_surface() {
    assert!(bool_scalar("preserved_basis_scalar_projection_safe"));
    let viability = viability();
    let surface = preserved_surface(&viability);
    assert_eq!(surface.validate(), Ok(()));
    let safe_basis = basis(
        &viability,
        scalar("preserved_successor_tooling_demand_units_per_period"),
        scalar("preserved_successor_period_duration_ms"),
        Some(surface.content_digest().unwrap()),
    );
    assert_eq!(safe_basis.validate(), Ok(()));
    assert!(safe_basis.scalar_runway_projection_safe);
    assert_eq!(
        verify_regenerative_support_basis_evidence(&viability, &safe_basis),
        Ok(())
    );
    assert_eq!(
        verify_regenerative_support_basis_surface_authorization(&safe_basis, &surface),
        Ok(())
    );
}

#[test]
fn arithmetic_or_subject_tampering_fails_closed() {
    let viability = viability();
    let surface = preserved_surface(&viability);
    let safe_basis = basis(
        &viability,
        scalar("preserved_successor_tooling_demand_units_per_period"),
        scalar("preserved_successor_period_duration_ms"),
        Some(surface.content_digest().unwrap()),
    );

    let mut bad_flag = safe_basis.clone();
    bad_flag.assessments[0].normalized_stockpile_draw_rate_preserved = false;
    assert!(bad_flag.validate().is_err());

    let mut bad_viability = safe_basis.clone();
    bad_viability.viability_evidence_content_digest = "0".repeat(64);
    assert!(verify_regenerative_support_basis_evidence(&viability, &bad_viability).is_err());

    let mut bad_surface = safe_basis.clone();
    bad_surface.authorized_policy_surface_content_digest = Some("1".repeat(64));
    assert!(verify_regenerative_support_basis_surface_authorization(&bad_surface, &surface).is_err());
}

#[test]
fn support_basis_evidence_round_trips_over_existing_maritime_transport() {
    let viability = viability();
    let surface = preserved_surface(&viability);
    let evidence = basis(
        &viability,
        scalar("preserved_successor_tooling_demand_units_per_period"),
        scalar("preserved_successor_period_duration_ms"),
        Some(surface.content_digest().unwrap()),
    );
    let envelope = evidence
        .to_maritime_envelope(
            "manta-civil-demo-01",
            7,
            0,
            1_788_900_005_000_000,
            "evidence:regenerative-support-basis-event-01",
        )
        .unwrap();
    let decoded: RegenerativeSupportBasisEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert_eq!(decoded, evidence);
}
