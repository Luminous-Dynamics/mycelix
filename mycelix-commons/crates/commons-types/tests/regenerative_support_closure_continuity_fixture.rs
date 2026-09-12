use commons_types::*;

const FIXTURE: &str =
    include_str!("../fixtures/manta-forge-support-topology-adversary-v1.txt");

fn scalar(key: &str) -> u64 {
    FIXTURE
        .lines()
        .find_map(|line| line.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("missing fixture scalar {key}"))
        .parse()
        .unwrap()
}

fn viability() -> RegenerativeViabilityEvidenceV1 {
    RegenerativeViabilityEvidenceV1 {
        schema_version: REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1,
        genome_binding: "genome:manta-v3:topology".into(),
        lineage_evidence_binding: "lineage:manta-v3:topology".into(),
        viability_profile_binding: "profile:manta-v3:topology".into(),
        static_viability_report_binding: "symthaea:manta-v3:topology-static".into(),
        dynamic_simulation_binding: "symtropy:manta-v3:topology-dynamic".into(),
        closure_model_binding: "model:closure-v3-topology".into(),
        flow_support_binding: "flow-support:v3:direct".into(),
        period_duration_ms: scalar("period_duration_ms"),
        roles: vec![
            RegenerativeViabilityRoleEvidenceV1 {
                role_id: "operation".into(),
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(100),
                dynamic_first_unavailable_tick: None,
                fully_modeled_support: true,
                limiting_dependency_refs: vec!["dependency:reactor-service-v3".into()],
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
                static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(100),
                dynamic_first_unavailable_tick: None,
                fully_modeled_support: true,
                limiting_dependency_refs: vec!["dependency:reactor-service-v3".into()],
            },
        ],
        regenerative_viability_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(
            scalar("source_horizon_periods"),
        ),
        limiting_role_ids: vec!["successor_construction".into()],
        fully_modeled_regenerative_viability: true,
    }
}

fn surface(viability: &RegenerativeViabilityEvidenceV1) -> RegenerativePolicySensitivitySurfaceEvidenceV1 {
    let policy_points = FIXTURE
        .lines()
        .filter_map(|line| line.strip_prefix("direct_policy="))
        .map(|line| {
            let mut values = [0u64; 5];
            for field in line.split('|') {
                let (key, raw) = field.split_once(':').unwrap();
                let value: u64 = raw.parse().unwrap();
                match key {
                    "maturity_periods" => values[0] = value,
                    "founded_descendants" => values[1] = value,
                    "maturity_completed_descendants" => values[2] = value,
                    "descendant_reproduction_transitions" => values[3] = value,
                    "terminal_residual_periods" => values[4] = value,
                    other => panic!("unknown fixture field {other}"),
                }
            }
            RegenerativePolicySensitivityPointEvidenceV1 {
                reproduction_policy_id: format!("maturity-{}", values[0]),
                reproduction_policy_evidence_binding: format!(
                    "reproduction-policy:maturity-{}:v1",
                    values[0]
                ),
                maturity_periods: values[0],
                founded_descendant_generations: RegenerativeGenerationCountEvidenceV1::Finite(
                    values[1],
                ),
                maturity_completed_descendant_generations:
                    RegenerativeGenerationCountEvidenceV1::Finite(values[2]),
                descendant_reproduction_transitions:
                    RegenerativeGenerationCountEvidenceV1::Finite(values[3]),
                terminal_generation_residual_periods: Some(values[4]),
            }
        })
        .collect();
    RegenerativePolicySensitivitySurfaceEvidenceV1 {
        schema_version: REGENERATIVE_POLICY_SENSITIVITY_SURFACE_SCHEMA_V1,
        surface_id: "manta-v3-topology-direct-surface".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        symthaea_surface_binding: "symthaea:pr-2160:direct-surface".into(),
        symtropy_surface_binding: "symtropy:pr-840:direct-surface".into(),
        shared_surface_fixture_binding:
            "git-blob:64a17015d8fab3a30eae95f34e327c705f47087f".into(),
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

fn basis(
    viability: &RegenerativeViabilityEvidenceV1,
    surface: &RegenerativePolicySensitivitySurfaceEvidenceV1,
) -> RegenerativeSupportBasisEvidenceV1 {
    let period = scalar("period_duration_ms");
    RegenerativeSupportBasisEvidenceV1 {
        schema_version: REGENERATIVE_SUPPORT_BASIS_EVIDENCE_SCHEMA_V1,
        basis_evidence_id: "manta-v3-v4-topology-basis".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        symthaea_support_basis_binding: "symthaea:pr-2052:a9344c".into(),
        symtropy_support_basis_binding: "symtropy:pr-821:bc074d".into(),
        shared_support_basis_fixture_binding:
            "git-blob:d434bc443f9d2800a518c231ad8d8c031e956835".into(),
        basis_policy_id: "basis-policy-topology-v1".into(),
        basis_policy_evidence_binding: "basis-policy:topology:v1".into(),
        source_model_binding: "model:closure-v3-topology".into(),
        successor_model_binding: "model:closure-v4-topology".into(),
        assessments: vec![
            RegenerativeSupportBasisAssessmentEvidenceV1 {
                source_dependency_id: "forge-tooling-v3".into(),
                successor_dependency_id: "forge-tooling-v4".into(),
                quantity_basis_equivalence_binding: "quantity-basis:forge:v3-v4".into(),
                source_stockpile_draw_units_per_period: 1,
                successor_stockpile_draw_units_per_period: 1,
                source_period_duration_ms: period,
                successor_period_duration_ms: period,
                normalized_stockpile_draw_rate_preserved: true,
            },
            RegenerativeSupportBasisAssessmentEvidenceV1 {
                source_dependency_id: "reactor-service-v3".into(),
                successor_dependency_id: "reactor-service-v4".into(),
                quantity_basis_equivalence_binding: "quantity-basis:reactor:v3-v4".into(),
                source_stockpile_draw_units_per_period: 1,
                successor_stockpile_draw_units_per_period: 1,
                source_period_duration_ms: period,
                successor_period_duration_ms: period,
                normalized_stockpile_draw_rate_preserved: true,
            },
        ],
        scalar_runway_projection_safe: true,
        authorized_policy_surface_content_digest: Some(surface.content_digest().unwrap()),
    }
}

fn closure_refs(hidden: bool, generation: u64) -> Vec<String> {
    let mut refs = vec![
        format!("dependency:forge-tooling-v{generation}"),
        format!("dependency:local-controller-v{generation}"),
        format!("dependency:metrology-v{generation}"),
        format!("dependency:reactor-service-v{generation}"),
        format!("dependency:structural-stock-v{generation}"),
    ];
    if hidden {
        refs.push(format!("dependency:controller-support-v{generation}"));
    }
    refs.sort();
    refs
}

fn direct_continuity(
    viability: &RegenerativeViabilityEvidenceV1,
    basis: &RegenerativeSupportBasisEvidenceV1,
    surface: &RegenerativePolicySensitivitySurfaceEvidenceV1,
) -> RegenerativeSupportClosureContinuityEvidenceV1 {
    RegenerativeSupportClosureContinuityEvidenceV1 {
        schema_version: REGENERATIVE_SUPPORT_CLOSURE_CONTINUITY_SCHEMA_V1,
        continuity_evidence_id: "manta-v3-v4-direct-support-closure".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        support_basis_evidence_content_digest: basis.content_digest().unwrap(),
        symthaea_continuity_binding: "symthaea:pr-2160:31a3a96".into(),
        symtropy_topology_binding: "symtropy:pr-840:direct".into(),
        shared_topology_fixture_binding:
            "git-blob:64a17015d8fab3a30eae95f34e327c705f47087f".into(),
        source_model_binding: basis.source_model_binding.clone(),
        successor_model_binding: basis.successor_model_binding.clone(),
        source_support_binding: "flow-support:v3:direct".into(),
        successor_support_binding: "flow-support:v4:direct".into(),
        source_role_support_dependency_refs: closure_refs(false, 3),
        successor_role_support_dependency_refs: closure_refs(false, 4),
        finite_root_pairs: vec![
            RegenerativeSupportClosureRootPairEvidenceV1 {
                source_dependency_id: "forge-tooling-v3".into(),
                successor_dependency_id: "forge-tooling-v4".into(),
                transfer_qualified: true,
                basis_qualified: true,
            },
            RegenerativeSupportClosureRootPairEvidenceV1 {
                source_dependency_id: "reactor-service-v3".into(),
                successor_dependency_id: "reactor-service-v4".into(),
                transfer_qualified: true,
                basis_qualified: true,
            },
        ],
        topology_isomorphic: true,
        scalar_runway_projection_safe: true,
        rejection_binding: None,
        authorized_policy_surface_content_digest: Some(surface.content_digest().unwrap()),
    }
}

fn hidden_continuity(
    viability: &RegenerativeViabilityEvidenceV1,
    basis: &RegenerativeSupportBasisEvidenceV1,
) -> RegenerativeSupportClosureContinuityEvidenceV1 {
    RegenerativeSupportClosureContinuityEvidenceV1 {
        schema_version: REGENERATIVE_SUPPORT_CLOSURE_CONTINUITY_SCHEMA_V1,
        continuity_evidence_id: "manta-v3-v4-hidden-support-closure".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        support_basis_evidence_content_digest: basis.content_digest().unwrap(),
        symthaea_continuity_binding: "symthaea:pr-2160:finite-root-refusal".into(),
        symtropy_topology_binding: "symtropy:pr-840:hidden".into(),
        shared_topology_fixture_binding:
            "git-blob:64a17015d8fab3a30eae95f34e327c705f47087f".into(),
        source_model_binding: basis.source_model_binding.clone(),
        successor_model_binding: basis.successor_model_binding.clone(),
        source_support_binding: "flow-support:v3:hidden".into(),
        successor_support_binding: "flow-support:v4:hidden".into(),
        source_role_support_dependency_refs: closure_refs(true, 3),
        successor_role_support_dependency_refs: closure_refs(true, 4),
        finite_root_pairs: vec![
            RegenerativeSupportClosureRootPairEvidenceV1 {
                source_dependency_id: "controller-support-v3".into(),
                successor_dependency_id: "controller-support-v4".into(),
                transfer_qualified: false,
                basis_qualified: false,
            },
            RegenerativeSupportClosureRootPairEvidenceV1 {
                source_dependency_id: "forge-tooling-v3".into(),
                successor_dependency_id: "forge-tooling-v4".into(),
                transfer_qualified: true,
                basis_qualified: true,
            },
            RegenerativeSupportClosureRootPairEvidenceV1 {
                source_dependency_id: "reactor-service-v3".into(),
                successor_dependency_id: "reactor-service-v4".into(),
                transfer_qualified: true,
                basis_qualified: true,
            },
        ],
        topology_isomorphic: true,
        scalar_runway_projection_safe: false,
        rejection_binding: Some("symthaea:pr-2160:finite-root-not-transferred".into()),
        authorized_policy_surface_content_digest: None,
    }
}

#[test]
fn direct_closure_can_authorize_exact_surface() {
    let viability = viability();
    let surface = surface(&viability);
    let basis = basis(&viability, &surface);
    let continuity = direct_continuity(&viability, &basis, &surface);
    assert_eq!(
        verify_regenerative_support_closure_continuity_evidence(
            &viability,
            &basis,
            &continuity,
        ),
        Ok(())
    );
    assert_eq!(
        verify_regenerative_support_closure_surface_authorization(
            &basis,
            &continuity,
            &surface,
        ),
        Ok(())
    );
}

#[test]
fn hidden_finite_root_refusal_is_auditable_but_cannot_authorize_surface() {
    let viability = viability();
    let surface = surface(&viability);
    let basis = basis(&viability, &surface);
    let continuity = hidden_continuity(&viability, &basis);
    assert_eq!(
        verify_regenerative_support_closure_continuity_evidence(
            &viability,
            &basis,
            &continuity,
        ),
        Ok(())
    );
    assert!(!continuity.scalar_runway_projection_safe);
    assert!(verify_regenerative_support_closure_surface_authorization(
        &basis,
        &continuity,
        &surface,
    )
    .is_err());
}

#[test]
fn forged_safe_root_or_surface_digest_fails_closed() {
    let viability = viability();
    let surface = surface(&viability);
    let basis = basis(&viability, &surface);
    let mut hidden = hidden_continuity(&viability, &basis);
    hidden.finite_root_pairs[0].basis_qualified = true;
    hidden.finite_root_pairs[0].transfer_qualified = true;
    hidden.scalar_runway_projection_safe = true;
    hidden.rejection_binding = None;
    hidden.authorized_policy_surface_content_digest = Some(surface.content_digest().unwrap());
    assert!(verify_regenerative_support_closure_continuity_evidence(
        &viability,
        &basis,
        &hidden,
    )
    .is_err());

    let mut direct = direct_continuity(&viability, &basis, &surface);
    direct.authorized_policy_surface_content_digest = Some("0".repeat(64));
    assert!(verify_regenerative_support_closure_surface_authorization(
        &basis,
        &direct,
        &surface,
    )
    .is_err());
}

#[test]
fn support_closure_evidence_round_trips_over_maritime_transport() {
    let viability = viability();
    let surface = surface(&viability);
    let basis = basis(&viability, &surface);
    let evidence = direct_continuity(&viability, &basis, &surface);
    let envelope = evidence
        .to_maritime_envelope(
            "manta-civil-demo-01",
            8,
            0,
            1_788_900_006_000_000,
            "evidence:support-closure-continuity-event-01",
        )
        .unwrap();
    let decoded: RegenerativeSupportClosureContinuityEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert_eq!(decoded, evidence);
}
