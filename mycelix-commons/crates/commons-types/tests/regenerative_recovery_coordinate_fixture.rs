include!("regenerative_support_closure_continuity_fixture.rs");

const RECOVERY_FIXTURE: &str =
    include_str!("../fixtures/regenerative-recovery-coordinate-v1.txt");
const SYMTROPY_RECOVERY_HEAD: &str = "1759282b98a600c4dbde9d3903dee099fcbdeca2";

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
        recovery_evidence_id: "manta-v4-metrology-recovery-coordinate-1".into(),
        viability_evidence_content_digest: viability.content_digest().unwrap(),
        support_closure_continuity_content_digest: continuity.content_digest().unwrap(),
        symthaea_recovery_binding:
            "symthaea:pr-2235:93beeb6ac163e56eeaf3bfbb7b53529eeca16a5f".into(),
        symtropy_recovery_binding: format!("symtropy:pr-862:{SYMTROPY_RECOVERY_HEAD}"),
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
        disturbance_id: "disturbance:metrology-production-loss-v4-1".into(),
        disturbance_evidence_binding: "disturbance:metrology-production-loss-v4:1".into(),
        dynamic_disturbance_observation_binding: format!(
            "symtropy:pr-862:{SYMTROPY_RECOVERY_HEAD}:disturbance-observation-v4-1"
        ),
        target_dependency_id: recovery_text("target_dependency").into(),
        flow_kind: RegenerativeRecoveryFlowKindEvidenceV1::Production,
        healthy_units_per_period: recovery_scalar("healthy_units_per_period"),
        degraded_units_per_period: recovery_scalar("degraded_units_per_period"),
        external_recovery_reserve_id: recovery_text("external_recovery_reserve_id").into(),
        external_recovery_reserve_binding: recovery_text(
            "external_recovery_reserve_binding",
        )
        .into(),
        external_recovery_reserve_initial_units: external_units,
        external_recovery_reserve_units_at_qualification: external_units,
        reserve_units_per_recovery: recovery_cost,
        reserve_units_before: external_units,
        reserve_units_after: external_units - recovery_cost,
        reserve_spend_sequence_before: 0,
        reserve_spend_sequence_after: 1,
        previous_recovery_evidence_content_digest: None,
        reserve_external_to_nominal_closure: true,
        dynamic_recovery_receipt_binding: format!(
            "symtropy:pr-862:{SYMTROPY_RECOVERY_HEAD}:recovery-receipt-v4-1"
        ),
        recovery_qualified: true,
        disturbance_conditioned_recovery_authorized: true,
        observed_maturity_periods: recovery_scalar("maturity_periods"),
        unrecoverable_first_unavailable_period: recovery_scalar(
            "unrecoverable_first_unavailable_period",
        ),
        recoverable_maturity_completed: true,
    }
}

fn second_recovery_evidence(
    first: &RegenerativeRecoveryCoordinateEvidenceV1,
) -> RegenerativeRecoveryCoordinateEvidenceV1 {
    let mut second = first.clone();
    second.recovery_evidence_id = "manta-v4-metrology-recovery-coordinate-2".into();
    second.disturbance_id = "disturbance:metrology-production-loss-v4-2".into();
    second.disturbance_evidence_binding = "disturbance:metrology-production-loss-v4:2".into();
    second.dynamic_disturbance_observation_binding = format!(
        "symtropy:pr-862:{SYMTROPY_RECOVERY_HEAD}:disturbance-observation-v4-2"
    );
    second.dynamic_recovery_receipt_binding = format!(
        "symtropy:pr-862:{SYMTROPY_RECOVERY_HEAD}:recovery-receipt-v4-2"
    );
    second.reserve_units_before = first.reserve_units_after;
    second.reserve_units_after = second.reserve_units_before - second.reserve_units_per_recovery;
    second.reserve_spend_sequence_before = first.reserve_spend_sequence_after;
    second.reserve_spend_sequence_after = second.reserve_spend_sequence_before + 1;
    second.previous_recovery_evidence_content_digest = Some(first.content_digest().unwrap());
    second
}

fn parents() -> (
    RegenerativeViabilityEvidenceV1,
    RegenerativeSupportBasisEvidenceV1,
    RegenerativeSupportClosureContinuityEvidenceV1,
) {
    let viability = viability(false);
    let surface = surface(&viability);
    let basis = basis(&viability, Some(&surface));
    let continuity = direct_continuity(&viability, &basis, &surface);
    (viability, basis, continuity)
}

#[test]
fn recovery_coordinate_composes_with_exact_safe_nominal_continuity() {
    let (viability, basis, continuity) = parents();
    let recovery = recovery_evidence(&viability, &continuity);

    assert_eq!(continuity.successor_model_binding, "model:closure-v4-topology");
    assert_eq!(continuity.successor_support_binding, "flow-support:v4:direct");
    assert_eq!(recovery.successor_profile_id, "profile-v4-topology");
    assert_eq!(recovery.reserve_spend_sequence_before, 0);
    assert_eq!(recovery.reserve_spend_sequence_after, 1);
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
fn second_spend_requires_exact_predecessor_digest_sequence_and_quantity() {
    let (viability, basis, continuity) = parents();
    let first = recovery_evidence(&viability, &continuity);
    let second = second_recovery_evidence(&first);

    assert_eq!(
        verify_regenerative_recovery_coordinate_evidence_with_predecessor(
            &viability,
            &basis,
            &continuity,
            Some(&first),
            &second,
        ),
        Ok(())
    );

    let mut wrong_digest = second.clone();
    wrong_digest.previous_recovery_evidence_content_digest = Some("0".repeat(64));
    assert!(verify_regenerative_recovery_reserve_lineage(Some(&first), &wrong_digest).is_err());

    let mut quantity_reset = second.clone();
    quantity_reset.reserve_units_before = first.reserve_units_before;
    quantity_reset.reserve_units_after = quantity_reset.reserve_units_before - quantity_reset.reserve_units_per_recovery;
    assert!(verify_regenerative_recovery_reserve_lineage(Some(&first), &quantity_reset).is_err());

    let mut sequence_gap = second.clone();
    sequence_gap.reserve_spend_sequence_before += 1;
    sequence_gap.reserve_spend_sequence_after += 1;
    assert!(verify_regenerative_recovery_reserve_lineage(Some(&first), &sequence_gap).is_err());
}

#[test]
fn later_spend_without_predecessor_is_rejected_even_when_arithmetic_is_valid() {
    let (viability, _basis, continuity) = parents();
    let first = recovery_evidence(&viability, &continuity);
    let mut second = second_recovery_evidence(&first);
    second.previous_recovery_evidence_content_digest = None;
    assert!(second.validate().is_err());
}

#[test]
fn recovery_coordinate_rejects_parent_substitution_and_external_reserve_collision() {
    let (viability, basis, continuity) = parents();
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
    let (viability, _basis, continuity) = parents();

    let mut arithmetic_drift = recovery_evidence(&viability, &continuity);
    arithmetic_drift.reserve_units_after += 1;
    assert!(arithmetic_drift.validate().is_err());

    let mut sequence_drift = recovery_evidence(&viability, &continuity);
    sequence_drift.reserve_spend_sequence_after += 1;
    assert!(sequence_drift.validate().is_err());

    let mut externality_drift = recovery_evidence(&viability, &continuity);
    externality_drift.reserve_external_to_nominal_closure = false;
    assert!(externality_drift.validate().is_err());

    let mut missing_observation = recovery_evidence(&viability, &continuity);
    missing_observation.dynamic_disturbance_observation_binding.clear();
    assert!(missing_observation.validate().is_err());
}

#[test]
fn recovery_coordinate_round_trips_over_maritime_transport() {
    let (viability, _basis, continuity) = parents();
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
