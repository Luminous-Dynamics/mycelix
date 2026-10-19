use constitutional_consumption::{
    ClaimBinding, ConsumptionClaim, ConsumptionKey, FinalityProfile,
};
use constitutional_envelope::MatterId;
use constitutional_temporal_provenance::{
    BoundFinalityEvidence, FinalityDomainPolicy, FinalityEvidence, ObserveFinalityOutcome,
    TemporalEvidenceState, TemporalOrder, TemporalProvenanceError,
};

fn claim() -> ConsumptionClaim {
    ConsumptionClaim {
        claim_id: "claim-a".into(),
        key: ConsumptionKey {
            envelope_digest: "sha256:envelope-a".into(),
            nonce: "nonce-a".into(),
            use_index: 0,
            jurisdiction: "finality:region-a".into(),
        },
        matter: MatterId {
            namespace: "test".into(),
            stable_id: "matter-a".into(),
        },
        target_digest: "sha256:target-a".into(),
        payload_digest: "sha256:payload-a".into(),
        budget_id: "budget-a".into(),
    }
}

fn policy() -> FinalityDomainPolicy {
    FinalityDomainPolicy {
        domain_id: "finality:region-a".into(),
        profile: FinalityProfile::WitnessedSingleSpend,
        policy_version: "policy-v1".into(),
        min_closure_witnesses: 2,
        min_closure_independence_domains: 2,
    }
}

fn raw_evidence(id: &str, observed_seq: u64) -> FinalityEvidence {
    FinalityEvidence {
        evidence_id: id.into(),
        claim_id: "claim-a".into(),
        proof_id: "proof-a".into(),
        profile: FinalityProfile::WitnessedSingleSpend,
        order: TemporalOrder {
            domain_id: "finality:region-a".into(),
            effective_seq: 1,
            observed_seq,
        },
    }
}

fn bound(id: &str, observed_seq: u64, binding: ClaimBinding) -> BoundFinalityEvidence {
    BoundFinalityEvidence {
        evidence: raw_evidence(id, observed_seq),
        claim_binding: binding,
    }
}

#[test]
fn bound_finality_registers_exact_claim_binding() {
    let c = claim();
    let mut state = TemporalEvidenceState::new(policy()).unwrap();

    assert_eq!(
        state
            .observe_bound_finality(bound("fe-a", 1, c.binding()))
            .unwrap(),
        ObserveFinalityOutcome::Accepted
    );
    assert_eq!(state.finality_binding("fe-a"), Some(&c.binding()));
}

#[test]
fn duplicate_bound_evidence_with_same_binding_is_idempotent() {
    let c = claim();
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    let evidence = bound("fe-a", 1, c.binding());

    assert_eq!(
        state.observe_bound_finality(evidence.clone()).unwrap(),
        ObserveFinalityOutcome::Accepted
    );
    assert_eq!(
        state.observe_bound_finality(evidence).unwrap(),
        ObserveFinalityOutcome::AlreadyObserved
    );
}

#[test]
fn duplicate_evidence_id_with_different_binding_is_rejected() {
    let c = claim();
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    state
        .observe_bound_finality(bound("fe-a", 1, c.binding()))
        .unwrap();

    let mut wrong = c.binding();
    wrong.payload_digest = "sha256:attacker".into();

    assert_eq!(
        state.observe_bound_finality(bound("fe-a", 1, wrong)),
        Err(TemporalProvenanceError::DuplicateFinalityBindingConflict)
    );
    assert_eq!(state.finality_binding("fe-a"), Some(&c.binding()));
}

#[test]
fn binding_claim_identity_must_match_evidence_claim_identity() {
    let c = claim();
    let mut binding = c.binding();
    binding.claim_id = "claim-b".into();

    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    assert_eq!(
        state.observe_bound_finality(bound("fe-a", 1, binding)),
        Err(TemporalProvenanceError::ClaimBindingIdentityMismatch)
    );
    assert!(state.accepted_finality.is_empty());
    assert!(state.finality_binding("fe-a").is_none());
}

#[test]
fn legacy_unbound_evidence_does_not_gain_binding_authority() {
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    assert_eq!(
        state.observe_finality(raw_evidence("legacy", 1)).unwrap(),
        ObserveFinalityOutcome::Accepted
    );
    assert!(state.finality_binding("legacy").is_none());
}


#[test]
fn legacy_unbound_evidence_cannot_be_retroactively_upgraded() {
    let c = claim();
    let mut state = TemporalEvidenceState::new(policy()).unwrap();
    let legacy = raw_evidence("legacy-upgrade", 1);

    assert_eq!(
        state.observe_finality(legacy.clone()).unwrap(),
        ObserveFinalityOutcome::Accepted
    );
    assert!(state.finality_binding("legacy-upgrade").is_none());

    assert_eq!(
        state.observe_bound_finality(BoundFinalityEvidence {
            evidence: legacy,
            claim_binding: c.binding(),
        }),
        Err(TemporalProvenanceError::UnboundFinalityCannotBeUpgraded)
    );
    assert!(state.finality_binding("legacy-upgrade").is_none());
}
