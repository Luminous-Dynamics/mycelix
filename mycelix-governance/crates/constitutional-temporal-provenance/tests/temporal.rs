// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use constitutional_consumption::FinalityProfile;
use constitutional_temporal_provenance::{
    AcceptClosureOutcome, ClosureProof, ClosureWitness, EvidenceClosure, FinalityDomainPolicy,
    FinalityEvidence, ObserveFinalityOutcome, ObserveRevocationOutcome, RevocationEvidence,
    TemporalEvidenceState, TemporalIntegrityFault, TemporalOrder, TemporalProvenanceError,
};

fn policy(profile: FinalityProfile) -> FinalityDomainPolicy {
    FinalityDomainPolicy {
        domain_id: "finality:region-a".into(),
        profile,
        policy_version: "policy-v1".into(),
        min_closure_witnesses: if profile == FinalityProfile::WitnessedSingleSpend { 2 } else { 0 },
        min_closure_independence_domains: if profile == FinalityProfile::WitnessedSingleSpend {
            2
        } else {
            0
        },
    }
}

fn evidence(id: &str, profile: FinalityProfile, effective: u64, observed: u64) -> FinalityEvidence {
    FinalityEvidence {
        evidence_id: id.into(),
        claim_id: format!("claim-{id}"),
        proof_id: format!("proof-{id}"),
        profile,
        order: TemporalOrder {
            domain_id: "finality:region-a".into(),
            effective_seq: effective,
            observed_seq: observed,
        },
    }
}

fn revocation(id: &str, effective: u64, observed: u64) -> RevocationEvidence {
    RevocationEvidence {
        evidence_id: format!("evidence-{id}"),
        revocation_id: id.into(),
        order: TemporalOrder {
            domain_id: "finality:region-a".into(),
            effective_seq: effective,
            observed_seq: observed,
        },
    }
}

fn witnessed_closure(id: &str, through: u64, observed: u64, previous: Option<&str>) -> EvidenceClosure {
    EvidenceClosure {
        closure_id: id.into(),
        domain_id: "finality:region-a".into(),
        closed_through_effective_seq: through,
        observed_at_seq: observed,
        profile: FinalityProfile::WitnessedSingleSpend,
        policy_version: "policy-v1".into(),
        previous_closure_id: previous.map(str::to_owned),
        proof: ClosureProof::WitnessQuorum {
            proof_ref: format!("closure-proof-{id}"),
            witnesses: vec![
                ClosureWitness {
                    witness_id: "w1".into(),
                    independence_domain: "integrity".into(),
                },
                ClosureWitness {
                    witness_id: "w2".into(),
                    independence_domain: "deliberative".into(),
                },
            ],
        },
    }
}

#[test]
fn effective_order_may_precede_observation_order() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    assert_eq!(
        state
            .observe_finality(evidence(
                "late",
                FinalityProfile::WitnessedSingleSpend,
                10,
                30,
            ))
            .unwrap(),
        ObserveFinalityOutcome::Accepted
    );
    assert_eq!(state.accepted_finality["late"].order.effective_seq, 10);
    assert_eq!(state.accepted_finality["late"].order.observed_seq, 30);
}

#[test]
fn revocation_also_preserves_effective_and_observation_order() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    assert_eq!(
        state.observe_revocation(revocation("revoke-1", 10, 30)).unwrap(),
        ObserveRevocationOutcome::Accepted
    );
    let recorded = &state.revocations["evidence-revoke-1"];
    assert_eq!(recorded.order.effective_seq, 10);
    assert_eq!(recorded.order.observed_seq, 30);
}

#[test]
fn observation_before_effective_order_is_rejected() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    assert_eq!(
        state.observe_finality(evidence(
            "bad",
            FinalityProfile::WitnessedSingleSpend,
            20,
            10,
        )),
        Err(TemporalProvenanceError::ObservationBeforeEffectiveOrder)
    );
}

#[test]
fn observation_order_cannot_regress_across_evidence_kinds() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    state.observe_revocation(revocation("revoke-1", 5, 20)).unwrap();
    assert_eq!(
        state.observe_finality(evidence(
            "second",
            FinalityProfile::WitnessedSingleSpend,
            6,
            19,
        )),
        Err(TemporalProvenanceError::ObservationOrderRegression)
    );
}

#[test]
fn known_revocation_rejects_finality_effective_at_or_after_revocation() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    state.observe_revocation(revocation("revoke-1", 10, 20)).unwrap();
    assert_eq!(
        state
            .observe_finality(evidence(
                "blocked",
                FinalityProfile::WitnessedSingleSpend,
                10,
                21,
            ))
            .unwrap(),
        ObserveFinalityOutcome::RejectedKnownRevocation
    );
    assert!(state.rejected_finality.contains_key("blocked"));
    assert!(!state.accepted_finality.contains_key("blocked"));
}

#[test]
fn late_observed_pre_revocation_finality_remains_admissible_before_closure() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    state.observe_revocation(revocation("revoke-1", 20, 20)).unwrap();
    assert_eq!(
        state
            .observe_finality(evidence(
                "historical",
                FinalityProfile::WitnessedSingleSpend,
                10,
                30,
            ))
            .unwrap(),
        ObserveFinalityOutcome::Accepted
    );
    assert!(state.integrity_fault.is_none());
}

#[test]
fn detection_only_cannot_create_a_completeness_watermark() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::DetectionOnly)).unwrap();
    let closure = EvidenceClosure {
        closure_id: "closure-1".into(),
        domain_id: "finality:region-a".into(),
        closed_through_effective_seq: 10,
        observed_at_seq: 10,
        profile: FinalityProfile::DetectionOnly,
        policy_version: "policy-v1".into(),
        previous_closure_id: None,
        proof: ClosureProof::LocalCheckpoint {
            checkpoint_ref: "dht-no-conflict-seen".into(),
        },
    };
    assert_eq!(
        state.accept_closure(closure),
        Err(TemporalProvenanceError::ClosureUnsupportedForDetectionOnly)
    );
}

#[test]
fn witnessed_closure_requires_identity_and_domain_diversity() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    let mut closure = witnessed_closure("closure-1", 10, 10, None);
    if let ClosureProof::WitnessQuorum { witnesses, .. } = &mut closure.proof {
        witnesses[1].independence_domain = "integrity".into();
    }
    assert_eq!(
        state.accept_closure(closure),
        Err(TemporalProvenanceError::InsufficientClosureDomains)
    );
}

#[test]
fn closure_chain_is_monotonic_and_parent_bound() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    assert_eq!(
        state
            .accept_closure(witnessed_closure("closure-1", 10, 10, None))
            .unwrap(),
        AcceptClosureOutcome::Accepted
    );
    assert_eq!(
        state.accept_closure(witnessed_closure("closure-2", 9, 11, Some("closure-1"))),
        Err(TemporalProvenanceError::ClosureRegression)
    );
    assert_eq!(
        state.accept_closure(witnessed_closure("closure-3", 12, 12, Some("wrong-parent"))),
        Err(TemporalProvenanceError::ClosureChainMismatch)
    );
}

#[test]
fn admissible_finality_after_closed_watermark_is_quarantined_and_faults() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    state
        .accept_closure(witnessed_closure("closure-1", 10, 10, None))
        .unwrap();

    assert_eq!(
        state
            .observe_finality(evidence(
                "late",
                FinalityProfile::WitnessedSingleSpend,
                8,
                11,
            ))
            .unwrap(),
        ObserveFinalityOutcome::QuarantinedContradiction
    );
    assert!(state.accepted_finality.is_empty());
    assert!(state.quarantined_finality.contains_key("late"));
    assert_eq!(
        state.integrity_fault,
        Some(TemporalIntegrityFault::FinalityAfterClosedWatermark {
            evidence_id: "late".into(),
            closure_id: "closure-1".into(),
            evidence_effective_seq: 8,
            closed_through_effective_seq: 10,
        })
    );
}

#[test]
fn inadmissible_post_revocation_finality_does_not_falsely_fault_closure() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    state.observe_revocation(revocation("revoke-1", 5, 5)).unwrap();
    state
        .accept_closure(witnessed_closure("closure-1", 10, 10, None))
        .unwrap();

    assert_eq!(
        state
            .observe_finality(evidence(
                "invalid",
                FinalityProfile::WitnessedSingleSpend,
                8,
                11,
            ))
            .unwrap(),
        ObserveFinalityOutcome::RejectedKnownRevocation
    );
    assert!(state.integrity_fault.is_none());
    assert!(state.rejected_finality.contains_key("invalid"));
}

#[test]
fn evidence_after_existing_fault_is_retained_but_not_accepted() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    state
        .accept_closure(witnessed_closure("closure-1", 10, 10, None))
        .unwrap();
    state
        .observe_finality(evidence(
            "contradiction",
            FinalityProfile::WitnessedSingleSpend,
            8,
            11,
        ))
        .unwrap();

    assert_eq!(
        state
            .observe_finality(evidence(
                "later",
                FinalityProfile::WitnessedSingleSpend,
                12,
                12,
            ))
            .unwrap(),
        ObserveFinalityOutcome::QuarantinedAfterFault
    );
    assert!(state.quarantined_finality.contains_key("later"));
    assert_eq!(
        state.observe_revocation(revocation("revoke-after-fault", 12, 13)).unwrap(),
        ObserveRevocationOutcome::AcceptedAfterFault
    );
}

#[test]
fn closure_through_r_minus_one_is_sufficient_for_terminal_revocation_decision() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    assert!(!state.pre_revocation_interval_is_closed(20));
    state
        .accept_closure(witnessed_closure("closure-1", 19, 19, None))
        .unwrap();
    assert!(state.pre_revocation_interval_is_closed(20));
    assert!(!state.pre_revocation_interval_is_closed(21));
}

#[test]
fn sequence_one_revocation_has_no_positive_pre_revocation_interval() {
    let state = TemporalEvidenceState::new(policy(FinalityProfile::DetectionOnly)).unwrap();
    assert!(state.pre_revocation_interval_is_closed(1));
}

#[test]
fn strong_consensus_closure_requires_namespace_commitment() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::StrongConsensus)).unwrap();
    let closure = EvidenceClosure {
        closure_id: "closure-1".into(),
        domain_id: "finality:region-a".into(),
        closed_through_effective_seq: 10,
        observed_at_seq: 10,
        profile: FinalityProfile::StrongConsensus,
        policy_version: "policy-v1".into(),
        previous_closure_id: None,
        proof: ClosureProof::StrongConsensus {
            checkpoint_ref: "height:100".into(),
            namespace_commitment: "".into(),
        },
    };
    assert_eq!(
        state.accept_closure(closure),
        Err(TemporalProvenanceError::EmptyProofReference)
    );
}

#[test]
fn closure_from_another_domain_cannot_close_local_history() {
    let mut state = TemporalEvidenceState::new(policy(FinalityProfile::WitnessedSingleSpend)).unwrap();
    let mut closure = witnessed_closure("closure-1", 10, 10, None);
    closure.domain_id = "finality:region-b".into();
    assert_eq!(
        state.accept_closure(closure),
        Err(TemporalProvenanceError::DomainMismatch)
    );
}
