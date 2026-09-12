use commons_types::{
    RegenerativeSupportBasisAssessmentEvidenceV1, RegenerativeSupportBasisEvidenceV1,
    REGENERATIVE_SUPPORT_BASIS_EVIDENCE_SCHEMA_V1,
};

fn record(safe: bool) -> RegenerativeSupportBasisEvidenceV1 {
    RegenerativeSupportBasisEvidenceV1 {
        schema_version: REGENERATIVE_SUPPORT_BASIS_EVIDENCE_SCHEMA_V1,
        basis_evidence_id: "period-basis-control".into(),
        viability_evidence_content_digest: "0".repeat(64),
        symthaea_support_basis_binding: "symthaea:period-basis-control".into(),
        symtropy_support_basis_binding: "symtropy:period-basis-control".into(),
        shared_support_basis_fixture_binding: "fixture:period-basis-control".into(),
        basis_policy_id: "period-basis-policy".into(),
        basis_policy_evidence_binding: "policy:period-basis-control".into(),
        source_model_binding: "model:source-period-basis".into(),
        successor_model_binding: "model:successor-period-basis".into(),
        assessments: vec![RegenerativeSupportBasisAssessmentEvidenceV1 {
            source_dependency_id: "tooling-v3".into(),
            successor_dependency_id: "tooling-v4".into(),
            quantity_basis_equivalence_binding: "quantity-basis:tooling:v3-v4".into(),
            source_stockpile_draw_units_per_period: 2,
            successor_stockpile_draw_units_per_period: 1,
            source_period_duration_ms: 2,
            successor_period_duration_ms: 1,
            // Physical draw rate is equal, but period-count basis is not.
            normalized_stockpile_draw_rate_preserved: true,
        }],
        scalar_runway_projection_safe: safe,
        authorized_policy_surface_content_digest: None,
    }
}

#[test]
fn equal_physical_rate_across_unlike_periods_is_not_scalar_period_safe() {
    assert!(record(false).validate().is_ok());
    assert!(record(true).validate().is_err());
}
