// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use constitutional_consumption::FinalityProfile;
use constitutional_temporal_provenance::{
    CrossOrderRelation, FinalityDomainPolicy, FinalityEvidence, ObservationOrderPolicy,
    ObserveFinalityOutcome, TemporalEvidenceState, TemporalOrder, TemporalProvenanceError,
};

fn policy() -> FinalityDomainPolicy {
    FinalityDomainPolicy {
        domain_id: "consensus:chain-a".into(),
        profile: FinalityProfile::StrongConsensus,
        policy_version: "policy-v1".into(),
        min_closure_witnesses: 0,
        min_closure_independence_domains: 0,
    }
}

fn evidence(effective: u64, observed: u64) -> FinalityEvidence {
    FinalityEvidence {
        evidence_id: format!("e-{effective}-{observed}"),
        claim_id: "claim-a".into(),
        proof_id: "proof-a".into(),
        profile: FinalityProfile::StrongConsensus,
        order: TemporalOrder {
            domain_id: "consensus:chain-a".into(),
            effective_seq: effective,
            observed_seq: observed,
        },
    }
}

#[test]
fn independent_order_allows_raw_effective_height_above_local_observation_counter() {
    let observation = ObservationOrderPolicy::independent("observer:region-a");
    let mut state = TemporalEvidenceState::new_with_observation_order(policy(), observation).unwrap();

    assert_eq!(
        state.observe_finality(evidence(1_000_000, 42)).unwrap(),
        ObserveFinalityOutcome::Accepted
    );
    assert_eq!(state.last_observed_seq, 42);
    assert_eq!(state.observation_order.relation, CrossOrderRelation::Independent);
}

#[test]
fn shared_comparable_order_rejects_the_same_numeric_history() {
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    assert_eq!(
        state.observe_finality(evidence(1_000_000, 42)),
        Err(TemporalProvenanceError::ObservationBeforeEffectiveOrder)
    );
}

#[test]
fn shared_comparable_policy_requires_same_order_domain() {
    let observation = ObservationOrderPolicy {
        domain_id: "observer:region-a".into(),
        relation: CrossOrderRelation::SharedComparable,
    };
    assert_eq!(
        TemporalEvidenceState::new_with_observation_order(policy(), observation),
        Err(TemporalProvenanceError::SharedOrderDomainMismatch)
    );
}

#[test]
fn independent_order_still_requires_strict_local_observation_monotonicity() {
    let observation = ObservationOrderPolicy::independent("observer:region-a");
    let mut state = TemporalEvidenceState::new_with_observation_order(policy(), observation).unwrap();
    state.observe_finality(evidence(1_000_000, 42)).unwrap();

    let mut second = evidence(1_000_001, 42);
    second.evidence_id = "e-second".into();
    second.claim_id = "claim-b".into();
    second.proof_id = "proof-b".into();
    assert_eq!(
        state.observe_finality(second),
        Err(TemporalProvenanceError::ObservationOrderRegression)
    );
}
