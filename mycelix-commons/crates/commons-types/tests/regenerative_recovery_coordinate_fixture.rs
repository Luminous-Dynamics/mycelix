include!("regenerative_support_closure_continuity_fixture.rs");

const RECOVERY_FIXTURE: &str =
    include_str!("../fixtures/regenerative-recovery-coordinate-v1.txt");

fn recovery_scalar(key: &str) -> u64 {
    RECOVERY_FIXTURE
        .lines()
        .find_map(|line| line.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("missing recovery-coordinate fixture scalar {key}"))
        .parse()
        .unwrap()
}

fn recovery_text(key: &str) -> &str {
    RECOVERY_FIXTURE
        .lines()
        .find_map(|line| line.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("missing recovery-coordinate fixture text {key}"))
}

fn recovery_evidence(
    viability: &RegenerativeViabilityEvidenceV1,
    continuity: &RegenerativeSupportClosureContinuityEvidenceV1,
) -> RegenerativeRecoveryCoordinateEvidenceV1 {
    let external_units = recovery_scalar("external_recovery_reserve_units");
    let recovery_cost = recovery_scalar("reserve_units_per_recovery");
    RegenerativeRecoveryCoordinateEvidenceV1 {
        schema_version: REGENERATIVE_RECOVERY_COORDINATE_EVIDENCE_SCHEMA_V1,
        recovery_evidence_id: "manta-v4-metrology-recovery-coordinate".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        support_closure_continuity_content_digest: continuity.content_digest().unwrap(),
        symthaea_recovery_binding:
            "symthaea:pr-2235:93beeb6ac163e56eeaf3bfbb7b53529eeca16a5f".into(),
        symtropy_recovery_binding:
            "symtropy:pr-862:e48f48a0c3ff35b5071647c2352fe62115c6f4b9".into(),
        semantic_recovery_fixture_binding:
            "git-blob:725880ba8affd94efd6a9cfb798c39e6c08c9c49".into(),
        dynamic_recovery_fixture_binding:
            "git-blob:4c043fdf476f4811ac89e82d46b327919ceffd27".into(),
        successor_profile_id: "profile-v4-topology".into(),
        successor_profile_evidence_binding: "profile-evidence:v4:topology".into(),
        successor_model_binding: continuity.successor_model_binding.clone(),
        successor_support_binding: continuity.successor_support_binding.clone(),
        recovery_policy_id: "recovery-policy:metrology-v4".into(),
        recovery_policy_evidence_binding: "recovery-policy:metrology-v4:v1".into(),
        recovery_qualification_binding: "recovery-qualification:metrology-v4:v1".into(),
        disturbance_id: "disturbance:metrology-production-loss-v4".into(),
        disturbance_evidence_binding: "disturbance:metrology-production-loss-v4:v1".into(),
        target_dependency_id: recovery_text("target_dependency").into(),
        flow_kind: RegenerativeRecoveryFlowKindEvidenceV1::Production,
        healthy_units_per_period: recovery_scalar("healthy_units_per_period"),
        degraded_units_per_period: recovery_scalar("degraded_units_per_period"),
        external_recovery_reserve_id: recovery_text("external_recovery_reserve_id").into(),
        external_recovery_reserve_binding: recovery_text(
            "external_recovery_reserve_binding",
        )
        .into(),
        external_recovery_reserve_units_at_qualification: external_units,
        reserve_units_per_recovery: recovery_cost,
        reserve_units_before: external_units,
        reserve_units_after: external_units - recovery_cost,
        reserve_external_to_nominal_closure: true,
        dynamic_recovery_receipt_binding:
            "symtropy:pr-862:e48f48a0c3ff35b5071647c2352fe62115c6f4b9:recovery-receipt-v4"
                .into(),
        recovery_qualified: true,
        disturbance_conditioned_recovery_authorized: true,
        observed_maturity_periods: recovery_scalar("maturity_periods"),
        unrecoverable_first_unavailable_period: recovery_scalar(
            "unrecoverable_first_unavailable_period",
        ),
        recoverable_maturity_completed: true,
    }
}

#[test]
fn recovery_coordinate_composes_with_exact_safe_nominal_continuity() {
    let viability = viability(false);
    let surface = surface(&viability);
    let basis = basis(&viability, Some(&surface));
    let continuity = direct_continuity(&viability, &basis, &surface);
    let recovery = recovery_evidence(&viability, &continuity);

    assert_eq!(continuity.successor_model_binding, "model:closure-v4-topology");
    assert_eq!(continuity.successor_support_binding, "flow-support:v4:direct");
    assert_eq!(recovery.successor_profile_id, "profile-v4-topology");
    assert_eq!(
        recovery.successor_profile_evidence_binding,
        "profile-evidence:v4:topology"
    );
    assert_eq!(
        verify_regenerative_recovery_coordinate_evidence(
            &viability,
            &basis,
            &continuity,
            &recovery,
        ),
        Ok(())
    );
}

#[test]
fn recovery_coordinate_rejects_parent_substitution_and_external_reserve_collision() {
    let viability = viability(false);
    let surface = surface(&viability);
    let basis = basis(&viability, Some(&surface));
    let continuity = direct_continuity(&viability, &basis, &surface);
    let mut recovery = recovery_evidence(&viability, &continuity);

    recovery.support_closure_continuity_content_digest = "0".repeat(64);
    assert!(verify_regenerative_recovery_coordinate_evidence(
        &viability,
        &basis,
        &continuity,
        &recovery,
    )
    .is_err());

    let mut collision = recovery_evidence(&viability, &continuity);
    collision.external_recovery_reserve_id = "forge-tooling-v4".into();
    assert!(verify_regenerative_recovery_coordinate_evidence(
        &viability,
        &basis,
        &continuity,
        &collision,
    )
    .is_err());
}

#[test]
fn recovery_reserve_receipt_and_externality_verdict_are_fail_closed() {
    let viability = viability(false);
    let surface = surface(&viability);
    let basis = basis(&viability, Some(&surface));
    let continuity = direct_continuity(&viability, &basis, &surface);

    let mut arithmetic_drift = recovery_evidence(&viability, &continuity);
    arithmetic_drift.reserve_units_after += 1;
    assert!(arithmetic_drift.validate().is_err());

    let mut snapshot_drift = recovery_evidence(&viability, &continuity);
    snapshot_drift.reserve_units_before = 0;
    assert!(snapshot_drift.validate().is_err());

    let mut externality_drift = recovery_evidence(&viability, &continuity);
    externality_drift.reserve_external_to_nominal_closure = false;
    assert!(externality_drift.validate().is_err());
}

#[test]
fn recovery_coordinate_round_trips_over_maritime_transport() {
    let viability = viability(false);
    let surface = surface(&viability);
    let basis = basis(&viability, Some(&surface));
    let continuity = direct_continuity(&viability, &basis, &surface);
    let recovery = recovery_evidence(&viability, &continuity);
    let envelope = recovery
        .to_maritime_envelope(
            "manta-civil-demo-01",
            9,
            0,
            1_788_900_007_000_000,
            "evidence:recovery-coordinate-event-01",
        )
        .unwrap();
    let decoded: RegenerativeRecoveryCoordinateEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert_eq!(decoded, recovery);
}
