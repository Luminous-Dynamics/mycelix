// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use constitutional_consumption::FinalityProfile;
use constitutional_temporal_provenance::{
    ClosureProof, ClosureWitness, EvidenceClosure, FinalityDomainPolicy, FinalityEvidence,
    ObserveFinalityOutcome, RevocationEvidence, TemporalEvidenceState, TemporalOrder,
    TemporalProvenanceError,
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

fn finality(id: &str, effective: u64, observed: u64) -> FinalityEvidence {
    FinalityEvidence {
        evidence_id: id.into(),
        claim_id: format!("claim-{id}"),
        proof_id: format!("proof-{id}"),
        profile: FinalityProfile::WitnessedSingleSpend,
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

fn closure(observed: u64) -> EvidenceClosure {
    EvidenceClosure {
        closure_id: "closure-1".into(),
        domain_id: "finality:region-a".into(),
        closed_through_effective_seq: 10,
        observed_at_seq: observed,
        profile: FinalityProfile::WitnessedSingleSpend,
        policy_version: "policy-v1".into(),
        previous_closure_id: None,
        proof: ClosureProof::WitnessQuorum {
            proof_ref: "closure-proof-1".into(),
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
fn distinct_finality_evidence_cannot_reuse_observation_sequence() {
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    state.observe_finality(finality("a", 5, 20)).unwrap();
    assert_eq!(
        state.observe_finality(finality("b", 6, 20)),
        Err(TemporalProvenanceError::ObservationOrderRegression)
    );
}

#[test]
fn distinct_revocation_cannot_reuse_observation_sequence() {
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    state.observe_finality(finality("a", 5, 20)).unwrap();
    assert_eq!(
        state.observe_revocation(revocation("r1", 10, 20)),
        Err(TemporalProvenanceError::ObservationOrderRegression)
    );
}

#[test]
fn closure_cannot_reuse_previous_observation_sequence() {
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    state.observe_finality(finality("a", 5, 20)).unwrap();
    assert_eq!(
        state.accept_closure(closure(20)),
        Err(TemporalProvenanceError::ObservationOrderRegression)
    );
}

#[test]
fn exact_duplicate_replay_remains_idempotent_without_advancing_order() {
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    let item = finality("a", 5, 20);
    assert_eq!(
        state.observe_finality(item.clone()).unwrap(),
        ObserveFinalityOutcome::Accepted
    );
    assert_eq!(
        state.observe_finality(item).unwrap(),
        ObserveFinalityOutcome::AlreadyObserved
    );
    assert_eq!(state.last_observed_seq, 20);
}
