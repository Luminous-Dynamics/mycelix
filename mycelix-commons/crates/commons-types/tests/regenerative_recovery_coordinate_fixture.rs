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

fn recovery_evidence(
    viability: &RegenerativeViabilityEvidenceV1,
    continuity: &RegenerativeSupportClosureContinuityEvidenceV1,
) -> RegenerativeRecoveryCoordinateEvidenceV1 {
    RegenerativeRecoveryCoordinateEvidenceV1 {
        schema_version: REGENERATIVE_RECOVERY_COORDINATE_EVIDENCE_SCHEMA_V1,
        recovery_evidence_id: "manta-v4-metrology-recovery-coordinate".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        support_closure_continuity_content_digest: continuity.content_digest().unwrap(),
        symthaea_recovery_binding:
            "symthaea:pr-2235:cbcce33872dd1ab2eac09a27922240a94f5d433f".into(),
        symtropy_recovery_binding:
            "symtropy:pr-862:6116da68b30c1151e62208baf631c8fc30ef7efa".into(),
        semantic_recovery_fixture_binding:
            "git-blob:92847f1bc67d0fc96068ba3583069632a025243b".into(),
        dynamic_recovery_fixture_binding:
            "git-blob:7cb27c2990e468e00f93835083f38daa8028a390".into(),
        successor_model_binding: continuity.successor_model_binding.clone(),
        successor_support_binding: continuity.successor_support_binding.clone(),
        disturbance_id: "disturbance:metrology-production-loss-v4".into(),
        disturbance_evidence_binding: "disturbance:metrology-production-loss-v4:v1".into(),
        target_dependency_id: "metrology-v4".into(),
        flow_kind: RegenerativeRecoveryFlowKindEvidenceV1::Production,
        healthy_units_per_period: recovery_scalar("healthy_units_per_period"),
        degraded_units_per_period: recovery_scalar("degraded_units_per_period"),
        reserve_dependency_id: "repair-reserve-v4".into(),
        reserve_units_per_recovery: recovery_scalar("reserve_units_per_recovery"),
        reserve_units_before: recovery_scalar("reserve_stockpile_units"),
        reserve_units_after: recovery_scalar("reserve_stockpile_units")
            - recovery_scalar("reserve_units_per_recovery"),
        dynamic_recovery_receipt_binding:
            "symtropy:pr-862:6116da68b30c1151e62208baf631c8fc30ef7efa:recovery-receipt-v4".into(),
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
fn recovery_coordinate_rejects_parent_substitution_and_reserve_closure_drift() {
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

    let mut drifted_continuity = continuity.clone();
    drifted_continuity
        .successor_role_support_dependency_refs
        .push("dependency:repair-reserve-v4".into());
    drifted_continuity
        .successor_role_support_dependency_refs
        .sort();
    let mut drifted_recovery = recovery_evidence(&viability, &drifted_continuity);
    drifted_recovery.support_closure_continuity_content_digest =
        drifted_continuity.content_digest().unwrap();
    assert!(verify_regenerative_recovery_coordinate_evidence(
        &viability,
        &basis,
        &drifted_continuity,
        &drifted_recovery,
    )
    .is_err());
}

#[test]
fn recovery_receipt_arithmetic_is_fail_closed() {
    let viability = viability(false);
    let surface = surface(&viability);
    let basis = basis(&viability, Some(&surface));
    let continuity = direct_continuity(&viability, &basis, &surface);
    let mut recovery = recovery_evidence(&viability, &continuity);
    recovery.reserve_units_after += 1;
    assert!(recovery.validate().is_err());
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
