// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use constitutional_consumption::{
    ConsumptionClaim, ConsumptionError, ConsumptionKey, ConsumptionState, EffectOutcome,
    EvidenceAvailability, FinalityProfile, FinalityProof, FinalityRequirement, FinalizeOutcome,
    IntegrityFault, RevocationCutoff, RevokeOutcome, UsageBudget, WitnessAttestation,
};
use constitutional_envelope::MatterId;

fn matter() -> MatterId {
    MatterId {
        namespace: "appropriation".into(),
        stable_id: "matter-42".into(),
    }
}

fn claim(id: &str, use_index: u32, budget_id: &str) -> ConsumptionClaim {
    ConsumptionClaim {
        claim_id: id.into(),
        key: ConsumptionKey {
            envelope_digest: "sha256:envelope".into(),
            nonce: "nonce-42".into(),
            use_index,
            jurisdiction: "region-a".into(),
        },
        matter: matter(),
        target_digest: "sha256:target".into(),
        payload_digest: "sha256:payload".into(),
        budget_id: budget_id.into(),
    }
}

fn witnesses() -> Vec<WitnessAttestation> {
    vec![
        WitnessAttestation {
            witness_id: "witness-a".into(),
            domain_id: "integrity".into(),
        },
        WitnessAttestation {
            witness_id: "witness-b".into(),
            domain_id: "deliberative".into(),
        },
    ]
}

fn proof(claim_id: &str, finalized_at_seq: u64) -> FinalityProof {
    FinalityProof {
        proof_id: format!("proof-{claim_id}"),
        claim_id: claim_id.into(),
        profile: FinalityProfile::WitnessedSingleSpend,
        finalized_at_seq,
        dependency_state: EvidenceAvailability::Complete,
        witnesses: witnesses(),
        consensus_ref: None,
    }
}

fn state_with_cutoff(max_uses: u32, revocation_cutoff: RevocationCutoff) -> ConsumptionState {
    ConsumptionState::new(
        UsageBudget {
            budget_id: "budget-root".into(),
            max_uses,
        },
        FinalityRequirement {
            minimum_profile: FinalityProfile::WitnessedSingleSpend,
            min_witnesses: 2,
            min_distinct_domains: 2,
            revocation_cutoff,
        },
    )
    .unwrap()
}

fn state(max_uses: u32) -> ConsumptionState {
    state_with_cutoff(max_uses, RevocationCutoff::Finality)
}

#[test]
fn competing_claims_for_one_use_index_cannot_both_finalize() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    state.submit_claim(claim("claim-b", 0, "budget-root")).unwrap();

    assert_eq!(
        state.finalize("claim-a", proof("claim-a", 10)).unwrap(),
        FinalizeOutcome::Finalized
    );
    assert_eq!(
        state.finalize("claim-b", proof("claim-b", 11)),
        Err(ConsumptionError::ConflictingUseAlreadyFinalized)
    );
    assert_eq!(state.finalized.len(), 1);
    assert!(state.check_invariants().is_ok());
}

#[test]
fn indeterminate_dependencies_never_authorize_finality() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    let mut p = proof("claim-a", 10);
    p.dependency_state = EvidenceAvailability::Indeterminate;

    assert_eq!(
        state.finalize("claim-a", p),
        Err(ConsumptionError::IndeterminateDependencies)
    );
    assert!(state.finalized.is_empty());
}

#[test]
fn detection_only_is_not_enough_for_witnessed_single_spend() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    let mut p = proof("claim-a", 10);
    p.profile = FinalityProfile::DetectionOnly;

    assert_eq!(
        state.finalize("claim-a", p),
        Err(ConsumptionError::InsufficientFinalityProfile)
    );
}

#[test]
fn revocation_before_finality_blocks_finalization() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    assert_eq!(state.revoke(10).unwrap(), RevokeOutcome::Revoked);

    assert_eq!(
        state.finalize("claim-a", proof("claim-a", 10)),
        Err(ConsumptionError::AuthorizationRevokedBeforeFinality)
    );
}

#[test]
fn finality_cutoff_allows_committed_effect_after_later_revocation() {
    let mut state = state_with_cutoff(1, RevocationCutoff::Finality);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    state.finalize("claim-a", proof("claim-a", 10)).unwrap();
    assert_eq!(state.revoke(20).unwrap(), RevokeOutcome::Revoked);

    assert_eq!(
        state.apply_effect("claim-a", 30, "output-1").unwrap(),
        EffectOutcome::Applied
    );
}

#[test]
fn effect_cutoff_blocks_effect_if_revoked_after_finality_but_before_effect() {
    let mut state = state_with_cutoff(1, RevocationCutoff::Effect);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    state.finalize("claim-a", proof("claim-a", 10)).unwrap();
    assert_eq!(state.revoke(20).unwrap(), RevokeOutcome::Revoked);

    assert_eq!(
        state.apply_effect("claim-a", 30, "output-1"),
        Err(ConsumptionError::AuthorizationRevokedBeforeEffect)
    );
}

#[test]
fn effect_cutoff_late_revocation_before_finality_cancels_without_integrity_fault() {
    let mut state = state_with_cutoff(1, RevocationCutoff::Effect);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    state.finalize("claim-a", proof("claim-a", 10)).unwrap();

    assert_eq!(state.revoke(5).unwrap(), RevokeOutcome::Revoked);
    assert!(state.integrity_fault.is_none());
    assert!(state.check_invariants().is_ok());
    assert_eq!(
        state.apply_effect("claim-a", 20, "output-1"),
        Err(ConsumptionError::AuthorizationRevokedBeforeEffect)
    );
}

#[test]
fn effect_cutoff_late_revocation_before_applied_effect_raises_integrity_fault() {
    let mut state = state_with_cutoff(1, RevocationCutoff::Effect);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    state.finalize("claim-a", proof("claim-a", 10)).unwrap();
    state.apply_effect("claim-a", 12, "output-1").unwrap();

    assert_eq!(
        state.revoke(5),
        Err(ConsumptionError::LateEarlierRevocationConflict)
    );
    assert_eq!(
        state.integrity_fault,
        Some(IntegrityFault::LateEarlierRevocation {
            revocation_seq: 5,
            conflicting_use_index: 0,
            cutoff: RevocationCutoff::Effect,
            commit_at_seq: 12,
        })
    );
}

#[test]
fn effect_cutoff_preserves_effect_that_logically_precedes_later_revocation() {
    let mut state = state_with_cutoff(1, RevocationCutoff::Effect);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    state.finalize("claim-a", proof("claim-a", 10)).unwrap();
    assert_eq!(state.revoke(30).unwrap(), RevokeOutcome::Revoked);

    assert_eq!(
        state.apply_effect("claim-a", 20, "output-1").unwrap(),
        EffectOutcome::Applied
    );
}

#[test]
fn finality_cutoff_late_earlier_revocation_raises_integrity_fault_and_halts_new_effects() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    state.finalize("claim-a", proof("claim-a", 10)).unwrap();

    assert_eq!(
        state.revoke(5),
        Err(ConsumptionError::LateEarlierRevocationConflict)
    );
    assert_eq!(
        state.integrity_fault,
        Some(IntegrityFault::LateEarlierRevocation {
            revocation_seq: 5,
            conflicting_use_index: 0,
            cutoff: RevocationCutoff::Finality,
            commit_at_seq: 10,
        })
    );
    assert_eq!(
        state.apply_effect("claim-a", 20, "output-1"),
        Err(ConsumptionError::IntegrityFaultActive)
    );
}

#[test]
fn irreversible_effect_requires_finality() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();

    assert_eq!(
        state.apply_effect("claim-a", 20, "output-1"),
        Err(ConsumptionError::EffectBeforeFinality)
    );
}

#[test]
fn duplicate_delivery_of_finalized_use_is_idempotent() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    state.finalize("claim-a", proof("claim-a", 10)).unwrap();

    assert_eq!(
        state.apply_effect("claim-a", 20, "output-1").unwrap(),
        EffectOutcome::Applied
    );
    assert_eq!(
        state
            .apply_effect("claim-a", 30, "different-output-request")
            .unwrap(),
        EffectOutcome::AlreadyApplied {
            output_ref: "output-1".into()
        }
    );
    assert_eq!(state.effects.len(), 1);
}

#[test]
fn historical_idempotent_result_remains_readable_after_later_fault() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    state.finalize("claim-a", proof("claim-a", 10)).unwrap();
    state.apply_effect("claim-a", 12, "output-1").unwrap();
    assert_eq!(
        state.revoke(5),
        Err(ConsumptionError::LateEarlierRevocationConflict)
    );

    assert_eq!(
        state.apply_effect("claim-a", 30, "ignored").unwrap(),
        EffectOutcome::AlreadyApplied {
            output_ref: "output-1".into()
        }
    );
}

#[test]
fn bounded_use_authority_cannot_finalize_out_of_range_index() {
    let mut state = state(2);
    assert_eq!(
        state.submit_claim(claim("claim-c", 2, "budget-root")),
        Err(ConsumptionError::UseIndexOutOfRange)
    );
}

#[test]
fn delegated_child_cannot_reset_usage_budget_by_renaming_it() {
    let mut state = state(2);
    assert_eq!(
        state.submit_claim(claim("child-claim", 0, "fresh-child-budget")),
        Err(ConsumptionError::BudgetMismatch)
    );
}

#[test]
fn witness_count_without_domain_diversity_is_insufficient() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    let mut p = proof("claim-a", 10);
    p.witnesses[1].domain_id = "integrity".into();

    assert_eq!(
        state.finalize("claim-a", p),
        Err(ConsumptionError::InsufficientWitnessDomains)
    );
}

#[test]
fn duplicate_witness_identity_does_not_count_twice() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    let mut p = proof("claim-a", 10);
    p.witnesses[1].witness_id = "witness-a".into();

    assert_eq!(
        state.finalize("claim-a", p),
        Err(ConsumptionError::InsufficientWitnesses)
    );
}

#[test]
fn strong_consensus_profile_requires_external_reference() {
    let mut state = ConsumptionState::new(
        UsageBudget {
            budget_id: "budget-root".into(),
            max_uses: 1,
        },
        FinalityRequirement {
            minimum_profile: FinalityProfile::StrongConsensus,
            min_witnesses: 2,
            min_distinct_domains: 2,
            revocation_cutoff: RevocationCutoff::Finality,
        },
    )
    .unwrap();
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    let mut p = proof("claim-a", 10);
    p.profile = FinalityProfile::StrongConsensus;

    assert_eq!(
        state.finalize("claim-a", p),
        Err(ConsumptionError::MissingConsensusReference)
    );
}

#[test]
fn repeated_finality_delivery_is_idempotent_when_proof_is_identical() {
    let mut state = state(1);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    let p = proof("claim-a", 10);

    assert_eq!(
        state.finalize("claim-a", p.clone()).unwrap(),
        FinalizeOutcome::Finalized
    );
    assert_eq!(
        state.finalize("claim-a", p).unwrap(),
        FinalizeOutcome::AlreadyFinalized
    );
}

#[test]
fn max_finalized_uses_never_exceeds_budget() {
    let mut state = state(2);
    state.submit_claim(claim("claim-a", 0, "budget-root")).unwrap();
    state.submit_claim(claim("claim-b", 1, "budget-root")).unwrap();
    state.finalize("claim-a", proof("claim-a", 10)).unwrap();
    state.finalize("claim-b", proof("claim-b", 11)).unwrap();

    assert_eq!(state.remaining_uses(), 0);
    assert_eq!(state.finalized.len(), 2);
    assert!(state.check_invariants().is_ok());
}
