// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use constitutional_closure_coverage::{
    assess_pre_revocation_coverage, permits_terminal_revocation, ClosureCoverageError,
    PreRevocationCoverage,
};
use constitutional_consumption::FinalityProfile;
use constitutional_temporal_provenance::{
    ClosureProof, ClosureWitness, EvidenceClosure, FinalityDomainPolicy, FinalityEvidence,
    ObservationOrderPolicy, TemporalEvidenceState, TemporalOrder,
};

fn policy() -> FinalityDomainPolicy {
    FinalityDomainPolicy {
        domain_id: "finality:region-a".into(),
        profile: FinalityProfile::WitnessedSingleSpend,
        policy_version: "policy-v1".into(),
        min_closure_witnesses: 2,
        min_closure_independence_domains: 2,
    }
}

fn closure(id: &str, through: u64, observed: u64) -> EvidenceClosure {
    EvidenceClosure {
        closure_id: id.into(),
        domain_id: "finality:region-a".into(),
        closed_through_effective_seq: through,
        observed_at_seq: observed,
        profile: FinalityProfile::WitnessedSingleSpend,
        policy_version: "policy-v1".into(),
        previous_closure_id: None,
        proof: ClosureProof::WitnessQuorum {
            proof_ref: format!("proof:{id}"),
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

fn late_finality(id: &str, effective: u64, observed: u64) -> FinalityEvidence {
    FinalityEvidence {
        evidence_id: id.into(),
        claim_id: format!("claim:{id}"),
        proof_id: format!("proof:{id}"),
        profile: FinalityProfile::WitnessedSingleSpend,
        order: TemporalOrder {
            domain_id: "finality:region-a".into(),
            effective_seq: effective,
            observed_seq: observed,
        },
    }
}

#[test]
fn zero_revocation_sequence_is_invalid() {
    let state = TemporalEvidenceState::new(policy()).unwrap();
    assert_eq!(
        assess_pre_revocation_coverage(&state, 0),
        Err(ClosureCoverageError::ZeroRevocationEffectiveSequence)
    );
}

#[test]
fn no_closure_is_open() {
    let state = TemporalEvidenceState::new(policy()).unwrap();
    let coverage = assess_pre_revocation_coverage(&state, 20).unwrap();
    assert!(matches!(
        coverage,
        PreRevocationCoverage::Open {
            revocation_effective_seq: 20,
            required_closed_through_effective_seq: 19,
            ..
        }
    ));
    assert!(!permits_terminal_revocation(&coverage));
}

#[test]
fn sequence_one_has_an_empty_pre_revocation_interval_with_policy_provenance() {
    let state = TemporalEvidenceState::new(policy()).unwrap();
    let coverage = assess_pre_revocation_coverage(&state, 1).unwrap();
    assert!(matches!(
        coverage,
        PreRevocationCoverage::EmptyPreRevocationInterval {
            revocation_effective_seq: 1,
            ref finality_domain_id,
            ref policy_version,
        } if finality_domain_id == "finality:region-a" && policy_version == "policy-v1"
    ));
    assert!(permits_terminal_revocation(&coverage));
}

#[test]
fn exact_cover_returns_the_authoritative_closure_object() {
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    state.accept_closure(closure("closure-19", 19, 19)).unwrap();

    let coverage = assess_pre_revocation_coverage(&state, 20).unwrap();
    match &coverage {
        PreRevocationCoverage::Closed {
            revocation_effective_seq,
            required_closed_through_effective_seq,
            closure,
            finality_domain_id,
            policy_version,
            ..
        } => {
            assert_eq!(*revocation_effective_seq, 20);
            assert_eq!(*required_closed_through_effective_seq, 19);
            assert_eq!(closure.closure_id, "closure-19");
            assert_eq!(closure.closed_through_effective_seq, 19);
            assert_eq!(finality_domain_id, "finality:region-a");
            assert_eq!(policy_version, "policy-v1");
        }
        other => panic!("expected Closed, got {other:?}"),
    }
    assert!(permits_terminal_revocation(&coverage));
}

#[test]
fn insufficient_watermark_remains_open() {
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    state.accept_closure(closure("closure-18", 18, 18)).unwrap();

    let coverage = assess_pre_revocation_coverage(&state, 20).unwrap();
    assert!(matches!(coverage, PreRevocationCoverage::Open { .. }));
    assert!(!permits_terminal_revocation(&coverage));
}

#[test]
fn healthy_domain_truth_table_is_exactly_watermark_reaches_r_minus_one() {
    for revocation_seq in 2_u64..=8 {
        for through in 1_u64..=8 {
            let mut state = TemporalEvidenceState::new(policy()).unwrap();
            state
                .accept_closure(closure(
                    &format!("closure-{revocation_seq}-{through}"),
                    through,
                    through,
                ))
                .unwrap();
            let coverage = assess_pre_revocation_coverage(&state, revocation_seq).unwrap();
            assert_eq!(
                permits_terminal_revocation(&coverage),
                through >= revocation_seq - 1,
                "R={revocation_seq}, closure_through={through}, coverage={coverage:?}"
            );
        }
    }
}

#[test]
fn contradiction_fault_revokes_closure_authority_without_erasing_history() {
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    state.accept_closure(closure("closure-19", 19, 19)).unwrap();

    state.observe_finality(late_finality("late", 10, 20)).unwrap();
    assert!(state.integrity_fault.is_some());
    assert_eq!(state.latest_closure().unwrap().closure_id, "closure-19");

    let coverage = assess_pre_revocation_coverage(&state, 20).unwrap();
    assert!(matches!(
        coverage,
        PreRevocationCoverage::IntegrityFault {
            revocation_effective_seq: 20,
            latest_closure_id: Some(ref id),
            ..
        } if id == "closure-19"
    ));
    assert!(!permits_terminal_revocation(&coverage));
}

#[test]
fn integrity_fault_dominates_even_empty_interval_shortcut() {
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    state.accept_closure(closure("closure-2", 2, 2)).unwrap();
    state.observe_finality(late_finality("late", 1, 3)).unwrap();

    let coverage = assess_pre_revocation_coverage(&state, 1).unwrap();
    assert!(matches!(coverage, PreRevocationCoverage::IntegrityFault { .. }));
    assert!(!permits_terminal_revocation(&coverage));
}

#[test]
fn independent_observation_order_does_not_change_effective_interval_coverage() {
    let mut state = TemporalEvidenceState::new_with_observation_order(
        policy(),
        ObservationOrderPolicy::independent("observer:intake-a"),
    )
    .unwrap();

    state
        .accept_closure(closure("closure-million", 1_000_000, 42))
        .unwrap();

    let coverage = assess_pre_revocation_coverage(&state, 1_000_001).unwrap();
    match coverage {
        PreRevocationCoverage::Closed {
            cross_order_relation,
            observation_domain_id,
            closure,
            ..
        } => {
            assert_eq!(observation_domain_id, "observer:intake-a");
            assert_eq!(
                cross_order_relation,
                constitutional_temporal_provenance::CrossOrderRelation::Independent
            );
            assert_eq!(closure.closed_through_effective_seq, 1_000_000);
        }
        other => panic!("expected Closed, got {other:?}"),
    }
}
