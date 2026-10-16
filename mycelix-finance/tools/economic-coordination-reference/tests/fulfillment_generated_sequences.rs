//! Deterministic generated-sequence checks for the partial-fulfilment reference model.
//!
//! These are bounded state-machine probes, not a proof of distributed safety.
//! The generator is deliberately dependency-free so the same sequence is replayable
//! on the pinned Rust toolchain and in constrained environments.

use economic_coordination_reference::{
    fulfillment_tranches::{
        DeliveryTrancheClaim, FulfillmentLedger, FulfillmentRequest, TrancheError,
        TrancheJournalEntry, TrancheState, TrancheVerification,
    },
    EvidenceOrigin, EvidenceStatus,
};

const REQUESTED: u64 = 100;

fn next(seed: &mut u64) -> u64 {
    *seed = (*seed)
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    *seed
}

fn assert_invariants(book: &FulfillmentLedger, attempts: usize) {
    let accepted = book.accepted_quantity().expect("bounded accepted sum");
    assert!(accepted <= REQUESTED, "accepted quantity exceeded request");
    assert_eq!(
        book.remaining_quantity().expect("bounded remainder"),
        REQUESTED - accepted
    );

    let record_sum: u64 = (0..attempts)
        .filter_map(|step| book.tranche(&format!("claim-{step}")))
        .map(|record| record.accepted_quantity)
        .sum();
    assert_eq!(accepted, record_sum, "aggregate differs from tranche records");

    let mut previous_time = 0;
    for entry in book.journal() {
        let time = match entry {
            TrancheJournalEntry::ClaimRecorded { recorded_at, .. } => *recorded_at,
            TrancheJournalEntry::VerificationRecorded(result) => result.decided_at,
        };
        assert!(time >= previous_time, "journal time moved backwards");
        previous_time = time;
    }
}

fn run_generated_sequence(seed: u64, attempts: usize) -> (Vec<TrancheJournalEntry>, u64) {
    let mut rng = seed;
    let mut book = FulfillmentLedger::new(FulfillmentRequest {
        request_id: format!("need-seed-{seed}"),
        requester_id: "household-a".to_owned(),
        provider_id: "repair-coop".to_owned(),
        resource_id: "repair-hours".to_owned(),
        unit: "hour".to_owned(),
        requested_quantity: REQUESTED,
        deliver_by: 100_000,
    })
    .expect("valid generated request");

    for step in 0..attempts {
        let choice = next(&mut rng) % 5;
        let quantity = next(&mut rng) % 12 + 1;
        // The over-claim control must reach the verified-quantity bound rather
        // than being diverted earlier by the synthetic-origin guard.
        let claim_origin = if choice == 3 || next(&mut rng) % 11 != 0 {
            EvidenceOrigin::LiveDomain
        } else {
            EvidenceOrigin::SyntheticSimulation
        };
        let recorded_at = step as u64 * 2 + 2;
        let claim = DeliveryTrancheClaim {
            claim_id: format!("claim-{step}"),
            source_id: "repair-coop".to_owned(),
            evidence_id: format!("claim-evidence-{step}"),
            resource_id: "repair-hours".to_owned(),
            unit: "hour".to_owned(),
            quantity,
            origin: claim_origin,
            observed_at: recorded_at - 1,
        };

        let before_record = book.journal().len();
        match book.record_claim(claim.clone(), recorded_at) {
            Ok(()) => {}
            Err(TrancheError::OverCommitment) => {
                assert_eq!(book.journal().len(), before_record);
                assert_invariants(&book, attempts);
                continue;
            }
            Err(other) => panic!("unexpected generated claim error: {other:?}"),
        }

        let (status, origin, accepted_quantity) = match choice {
            0 => (EvidenceStatus::Unknown, EvidenceOrigin::LiveDomain, Some(quantity)),
            1 => (
                EvidenceStatus::Verified,
                EvidenceOrigin::SyntheticSimulation,
                Some(quantity),
            ),
            2 => (
                EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain,
                Some(next(&mut rng) % (quantity + 1)),
            ),
            3 => (
                EvidenceStatus::Verified,
                EvidenceOrigin::LiveDomain,
                Some(quantity + 1),
            ),
            _ => (EvidenceStatus::Verified, EvidenceOrigin::LiveDomain, None),
        };
        let verification = TrancheVerification {
            verifier_id: format!("independent-auditor-{step}"),
            claim_id: claim.claim_id.clone(),
            claim_evidence_id: claim.evidence_id.clone(),
            evidence_id: format!("verification-evidence-{step}"),
            evidence_status: status,
            origin,
            accepted_quantity,
            decided_at: recorded_at + 1,
        };

        let before_verify_journal = book.journal().len();
        let before_verify_accepted = book.accepted_quantity().unwrap();
        let result = book.verify_claim(verification);
        if choice == 3 {
            assert_eq!(result, Err(TrancheError::OverFulfillment));
            assert_eq!(book.journal().len(), before_verify_journal);
            assert_eq!(book.accepted_quantity().unwrap(), before_verify_accepted);
            assert_eq!(
                book.tranche(&claim.claim_id).unwrap().state,
                TrancheState::Claimed,
                "rejected over-claim must not mutate the tranche"
            );
        } else {
            assert!(result.is_ok(), "unexpected verification error: {result:?}");
            if status != EvidenceStatus::Verified
                || origin != EvidenceOrigin::LiveDomain
                || claim.origin != EvidenceOrigin::LiveDomain
                || accepted_quantity.is_none()
            {
                assert_eq!(result.unwrap(), TrancheState::Indeterminate);
            }
            assert_eq!(book.journal().len(), before_verify_journal + 1);
        }

        assert_invariants(&book, attempts);
    }

    (book.journal().to_vec(), book.accepted_quantity().unwrap())
}

#[test]
fn generated_transition_sequences_preserve_quantity_and_journal_invariants() {
    // 4,096 independent ledgers × 32 generated claim attempts = 131,072 attempts.
    // An attempt may be rejected by the model's own bounded reservation rule.
    for seed in 0..4_096_u64 {
        let _ = run_generated_sequence(seed + 1, 32);
    }
}

#[test]
fn same_seed_replays_produce_identical_ordered_journals() {
    for seed in 1..=100_u64 {
        let first = run_generated_sequence(seed, 64);
        let replay = run_generated_sequence(seed, 64);
        assert_eq!(first, replay, "seed {seed} did not replay identically");
    }
}


#[test]
fn isolated_replicas_can_double_allocate_without_a_shared_reservation_protocol() {
    // Deliberate counterexample, not a desired property: each serial replica
    // sees the full request as available and therefore accepts locally valid
    // claims. A production design needs a shared reservation authority or a
    // formally specified escrow/partition protocol before global safety can
    // be claimed.
    fn isolated_replica(replica: &str, auditor: &str) -> FulfillmentLedger {
        let request = FulfillmentRequest {
            request_id: "shared-need-001".to_owned(),
            requester_id: "household-a".to_owned(),
            provider_id: "repair-coop".to_owned(),
            resource_id: "repair-hours".to_owned(),
            unit: "hour".to_owned(),
            requested_quantity: 10,
            deliver_by: 50,
        };
        let mut ledger = FulfillmentLedger::new(request).unwrap();
        let claim = DeliveryTrancheClaim {
            claim_id: format!("claim-{replica}"),
            source_id: "repair-coop".to_owned(),
            evidence_id: format!("claim-evidence-{replica}"),
            resource_id: "repair-hours".to_owned(),
            unit: "hour".to_owned(),
            quantity: 10,
            origin: EvidenceOrigin::LiveDomain,
            observed_at: 2,
        };
        ledger.record_claim(claim.clone(), 3).unwrap();
        assert_eq!(
            ledger
                .verify_claim(TrancheVerification {
                    verifier_id: auditor.to_owned(),
                    claim_id: claim.claim_id,
                    claim_evidence_id: claim.evidence_id,
                    evidence_id: format!("verification-{replica}"),
                    evidence_status: EvidenceStatus::Verified,
                    origin: EvidenceOrigin::LiveDomain,
                    accepted_quantity: Some(10),
                    decided_at: 4,
                })
                .unwrap(),
            TrancheState::Accepted
        );
        ledger
    }

    let region_a = isolated_replica("region-a", "independent-auditor-a");
    let region_b = isolated_replica("region-b", "independent-auditor-b");
    assert_eq!(region_a.accepted_quantity().unwrap(), 10);
    assert_eq!(region_b.accepted_quantity().unwrap(), 10);
    assert!(
        region_a.accepted_quantity().unwrap() + region_b.accepted_quantity().unwrap() > 10,
        "independent serial validity does not imply global reservation safety"
    );
}
