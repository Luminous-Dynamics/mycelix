use constitutional_claim_lifecycle::{
    ClaimLifecycleError, ClaimLifecycleState, ClaimLifecycleStatus, CoverageResolutionOutcome,
    LifecycleRevokeOutcome,
};
use constitutional_consumption::{
    ConsumptionClaim, ConsumptionKey, EvidenceAvailability, FinalityProfile, FinalityProof,
    FinalityRequirement, RevocationCutoff, UsageBudget, WitnessAttestation,
};
use constitutional_envelope::MatterId;
use constitutional_temporal_provenance::{
    ClosureProof, ClosureWitness, EvidenceClosure, FinalityDomainPolicy, FinalityEvidence,
    ObserveFinalityOutcome, RevocationEvidence, TemporalEvidenceState, TemporalOrder,
};

const DOMAIN: &str = "domain-a";

fn requirement() -> FinalityRequirement {
    FinalityRequirement {
        minimum_profile: FinalityProfile::WitnessedSingleSpend,
        min_witnesses: 2,
        min_distinct_domains: 2,
        revocation_cutoff: RevocationCutoff::Finality,
    }
}

fn budget(max_uses: u32) -> UsageBudget {
    UsageBudget {
        budget_id: "budget-a".into(),
        max_uses,
    }
}

fn temporal() -> TemporalEvidenceState {
    TemporalEvidenceState::new(FinalityDomainPolicy {
        domain_id: DOMAIN.into(),
        profile: FinalityProfile::WitnessedSingleSpend,
        policy_version: "policy-v1".into(),
        min_closure_witnesses: 2,
        min_closure_independence_domains: 2,
    })
    .unwrap()
}

fn lifecycle(max_uses: u32) -> ClaimLifecycleState {
    ClaimLifecycleState::new(temporal(), budget(max_uses), requirement()).unwrap()
}

fn claim(id: &str, use_index: u32) -> ConsumptionClaim {
    ConsumptionClaim {
        claim_id: id.into(),
        key: ConsumptionKey {
            envelope_digest: format!("envelope-{id}"),
            nonce: format!("nonce-{id}"),
            use_index,
            jurisdiction: DOMAIN.into(),
        },
        matter: MatterId {
            namespace: "lifecycle-test".into(),
            stable_id: format!("matter-{id}"),
        },
        target_digest: format!("target-{id}"),
        payload_digest: format!("payload-{id}"),
        budget_id: "budget-a".into(),
    }
}

fn witnesses() -> Vec<WitnessAttestation> {
    vec![
        WitnessAttestation {
            witness_id: "w1".into(),
            domain_id: "d1".into(),
        },
        WitnessAttestation {
            witness_id: "w2".into(),
            domain_id: "d2".into(),
        },
    ]
}

fn proof(claim_id: &str, proof_id: &str, effective_seq: u64) -> FinalityProof {
    FinalityProof {
        proof_id: proof_id.into(),
        claim_id: claim_id.into(),
        profile: FinalityProfile::WitnessedSingleSpend,
        finalized_at_seq: effective_seq,
        dependency_state: EvidenceAvailability::Complete,
        witnesses: witnesses(),
        consensus_ref: None,
    }
}

fn finality_evidence(
    evidence_id: &str,
    claim_id: &str,
    proof_id: &str,
    effective_seq: u64,
    observed_seq: u64,
) -> FinalityEvidence {
    FinalityEvidence {
        evidence_id: evidence_id.into(),
        claim_id: claim_id.into(),
        proof_id: proof_id.into(),
        profile: FinalityProfile::WitnessedSingleSpend,
        order: TemporalOrder {
            domain_id: DOMAIN.into(),
            effective_seq,
            observed_seq,
        },
    }
}

fn revocation_evidence(
    evidence_id: &str,
    effective_seq: u64,
    observed_seq: u64,
) -> RevocationEvidence {
    RevocationEvidence {
        evidence_id: evidence_id.into(),
        revocation_id: format!("revocation-{evidence_id}"),
        order: TemporalOrder {
            domain_id: DOMAIN.into(),
            effective_seq,
            observed_seq,
        },
    }
}

fn closure(
    closure_id: &str,
    closed_through: u64,
    observed_seq: u64,
) -> EvidenceClosure {
    EvidenceClosure {
        closure_id: closure_id.into(),
        domain_id: DOMAIN.into(),
        closed_through_effective_seq: closed_through,
        observed_at_seq: observed_seq,
        profile: FinalityProfile::WitnessedSingleSpend,
        policy_version: "policy-v1".into(),
        previous_closure_id: None,
        proof: ClosureProof::WitnessQuorum {
            proof_ref: format!("closure-proof-{closure_id}"),
            witnesses: vec![
                ClosureWitness {
                    witness_id: "cw1".into(),
                    independence_domain: "cd1".into(),
                },
                ClosureWitness {
                    witness_id: "cw2".into(),
                    independence_domain: "cd2".into(),
                },
            ],
        },
    }
}

#[test]
fn finalizing_winner_terminalizes_competitor_without_deleting_evidence() {
    let mut lifecycle = lifecycle(1);
    lifecycle.submit_claim(claim("a", 0)).unwrap();
    lifecycle.submit_claim(claim("b", 0)).unwrap();

    assert_eq!(
        lifecycle
            .observe_finality_evidence(finality_evidence("fe-a", "a", "proof-a", 1, 1))
            .unwrap(),
        ObserveFinalityOutcome::Accepted
    );
    lifecycle
        .finalize("fe-a", "a", proof("a", "proof-a", 1))
        .unwrap();

    assert!(matches!(
        lifecycle.claim("a").unwrap().status,
        ClaimLifecycleStatus::Finalized {
            ref finality_evidence_id,
            ..
        } if finality_evidence_id == "fe-a"
    ));
    assert!(matches!(
        lifecycle.claim("b").unwrap().status,
        ClaimLifecycleStatus::RejectedConflict {
            ref winning_claim_id,
            ref winning_finality_evidence_id,
            ..
        } if winning_claim_id == "a" && winning_finality_evidence_id == "fe-a"
    ));
    assert_eq!(lifecycle.remaining_uses(), 0);
    assert_eq!(lifecycle.consumption().pending.len(), 2);

    let err = lifecycle
        .finalize("fe-a", "b", proof("b", "proof-b", 1))
        .unwrap_err();
    assert_eq!(err, ClaimLifecycleError::TerminalClaim);
}

#[test]
fn blocked_claim_can_still_finalize_with_late_observed_pre_revocation_proof() {
    let mut lifecycle = lifecycle(1);
    lifecycle.submit_claim(claim("a", 0)).unwrap();

    assert_eq!(
        lifecycle
            .observe_revocation_evidence(revocation_evidence("re-1", 3, 3))
            .unwrap(),
        LifecycleRevokeOutcome::Revoked
    );
    assert!(matches!(
        lifecycle.claim("a").unwrap().status,
        ClaimLifecycleStatus::BlockedAwaitingEvidenceClosure { .. }
    ));

    assert_eq!(
        lifecycle
            .observe_finality_evidence(finality_evidence("fe-a", "a", "proof-a", 2, 4))
            .unwrap(),
        ObserveFinalityOutcome::Accepted
    );
    lifecycle
        .finalize("fe-a", "a", proof("a", "proof-a", 2))
        .unwrap();
    assert!(matches!(
        lifecycle.claim("a").unwrap().status,
        ClaimLifecycleStatus::Finalized {
            finalized_effective_seq: 2,
            ..
        }
    ));
}

#[test]
fn closure_terminalizes_blocked_claim_with_exact_provenance() {
    let mut lifecycle = lifecycle(1);
    lifecycle.submit_claim(claim("a", 0)).unwrap();
    lifecycle
        .observe_revocation_evidence(revocation_evidence("re-1", 3, 3))
        .unwrap();
    lifecycle.accept_closure(closure("closure-1", 2, 4)).unwrap();

    let outcome = lifecycle.resolve_revocation_coverage().unwrap();
    assert_eq!(
        outcome,
        CoverageResolutionOutcome::Terminalized {
            claim_ids: vec!["a".into()]
        }
    );
    assert!(matches!(
        lifecycle.claim("a").unwrap().status,
        ClaimLifecycleStatus::RevokedClosed { ref resolution, .. }
            if matches!(
                resolution,
                constitutional_claim_lifecycle::RevocationResolutionEvidence::Closed {
                    closure,
                    ..
                } if closure.closure_id == "closure-1"
            )
    ));
}

#[test]
fn post_closure_temporal_fault_halts_unresolved_work_but_preserves_closed_history() {
    let mut lifecycle = lifecycle(2);
    lifecycle.submit_claim(claim("closed", 0)).unwrap();
    lifecycle
        .observe_revocation_evidence(revocation_evidence("re-1", 3, 3))
        .unwrap();
    lifecycle.accept_closure(closure("closure-1", 2, 4)).unwrap();
    lifecycle.resolve_revocation_coverage().unwrap();

    lifecycle.submit_claim(claim("later", 1)).unwrap();
    let outcome = lifecycle
        .observe_finality_evidence(finality_evidence(
            "contradiction",
            "ghost",
            "ghost-proof",
            2,
            5,
        ))
        .unwrap();
    assert_eq!(outcome, ObserveFinalityOutcome::QuarantinedContradiction);

    assert!(matches!(
        lifecycle.claim("closed").unwrap().status,
        ClaimLifecycleStatus::RevokedClosed { .. }
    ));
    assert!(matches!(
        lifecycle.claim("later").unwrap().status,
        ClaimLifecycleStatus::IntegrityHalted { .. }
    ));
    assert!(lifecycle.check_invariants().is_ok());
}

#[test]
fn effect_requires_finalized_lifecycle_state() {
    let mut lifecycle = lifecycle(1);
    lifecycle.submit_claim(claim("a", 0)).unwrap();
    assert_eq!(
        lifecycle.apply_effect("a", 2, "out").unwrap_err(),
        ClaimLifecycleError::ClaimNotFinalized
    );

    lifecycle
        .observe_finality_evidence(finality_evidence("fe-a", "a", "proof-a", 1, 1))
        .unwrap();
    lifecycle
        .finalize("fe-a", "a", proof("a", "proof-a", 1))
        .unwrap();
    lifecycle.apply_effect("a", 2, "out").unwrap();
}

#[test]
fn duplicate_applied_effect_remains_readable_after_later_temporal_fault() {
    let mut lifecycle = lifecycle(1);
    lifecycle.submit_claim(claim("a", 0)).unwrap();
    lifecycle
        .observe_finality_evidence(finality_evidence("fe-a", "a", "proof-a", 1, 1))
        .unwrap();
    lifecycle
        .finalize("fe-a", "a", proof("a", "proof-a", 1))
        .unwrap();
    lifecycle.apply_effect("a", 2, "out-a").unwrap();

    lifecycle.accept_closure(closure("closure-1", 1, 2)).unwrap();
    let outcome = lifecycle
        .observe_finality_evidence(finality_evidence(
            "late-contradiction",
            "ghost",
            "ghost-proof",
            1,
            3,
        ))
        .unwrap();
    assert_eq!(outcome, ObserveFinalityOutcome::QuarantinedContradiction);

    assert!(matches!(
        lifecycle.apply_effect("a", 99, "ignored").unwrap(),
        constitutional_consumption::EffectOutcome::AlreadyApplied { ref output_ref }
            if output_ref == "out-a"
    ));
}

#[test]
fn constructor_rejects_prepopulated_temporal_state() {
    let mut prepopulated = temporal();
    prepopulated
        .observe_revocation(revocation_evidence("external-revocation", 2, 2))
        .unwrap();

    assert_eq!(
        ClaimLifecycleState::new(prepopulated, budget(1), requirement()).unwrap_err(),
        ClaimLifecycleError::NonPristineTemporalState
    );
}

#[test]
fn owned_temporal_state_rejects_external_snapshot_substitution() {
    let mut lifecycle = lifecycle(1);
    lifecycle.submit_claim(claim("a", 0)).unwrap();

    let mut external = temporal();
    external
        .observe_finality(finality_evidence("external-fe", "a", "proof-a", 1, 1))
        .unwrap();

    assert_eq!(
        lifecycle
            .finalize("external-fe", "a", proof("a", "proof-a", 1))
            .unwrap_err(),
        ClaimLifecycleError::UnknownFinalityEvidence
    );
    assert!(lifecycle.temporal().accepted_finality.is_empty());
    assert_eq!(lifecycle.remaining_uses(), 1);

    lifecycle
        .observe_finality_evidence(finality_evidence("owned-fe", "a", "proof-a", 1, 1))
        .unwrap();
    lifecycle
        .finalize("owned-fe", "a", proof("a", "proof-a", 1))
        .unwrap();
    assert_eq!(lifecycle.remaining_uses(), 0);
}

#[test]
fn transition_receipts_have_unique_contiguous_global_ordinals() {
    let mut lifecycle = lifecycle(1);
    lifecycle.submit_claim(claim("a", 0)).unwrap();
    lifecycle.submit_claim(claim("b", 0)).unwrap();
    lifecycle
        .observe_finality_evidence(finality_evidence("fe-a", "a", "proof-a", 1, 1))
        .unwrap();
    lifecycle
        .finalize("fe-a", "a", proof("a", "proof-a", 1))
        .unwrap();

    let mut ordinals: Vec<u64> = lifecycle
        .claims()
        .values()
        .flat_map(|record| record.transitions.iter().map(|receipt| receipt.ordinal))
        .collect();
    ordinals.sort_unstable();
    assert_eq!(ordinals, vec![1, 2]);
    assert_eq!(lifecycle.next_transition_ordinal(), 2);
    assert!(lifecycle.check_invariants().is_ok());
}
