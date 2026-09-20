// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//!
//! MPC-001B RED corpus.
//!
//! These tests intentionally encode fail-closed properties that the parent
//! implementation did not establish at audit time. This branch is evidence of
//! the demonstrated defects, not a qualification PASS and not a merge-ready
//! product change.

use feldman_dkg::{
    CommitmentScheme, DkgCeremony, DkgConfig, Dealer, HashCommitmentSet, ParticipantId, Scalar,
    Share,
};
use rand::rngs::OsRng;
use std::panic::{catch_unwind, AssertUnwindSafe};

fn feldman_ceremony(threshold: usize, participants: usize) -> DkgCeremony {
    let config = DkgConfig::new(threshold, participants).unwrap();
    let mut ceremony = DkgCeremony::new(config, 0);
    for id in 1..=participants as u32 {
        ceremony.add_participant(ParticipantId(id), 0).unwrap();
    }
    ceremony
}

fn hash_ceremony(scheme: CommitmentScheme) -> DkgCeremony {
    let config = DkgConfig::new(2, 3)
        .unwrap()
        .with_commitment_scheme(scheme);
    let mut ceremony = DkgCeremony::new(config, 0);
    for id in 1..=3 {
        ceremony.add_participant(ParticipantId(id), 0).unwrap();
    }
    ceremony
}

#[test]
fn red_rejects_caller_dealer_substitution() {
    let mut ceremony = feldman_ceremony(2, 3);
    let dealer = Dealer::new(ParticipantId(1), 2, 3, &mut OsRng).unwrap();
    let deal = dealer.generate_deal();

    assert!(
        ceremony.submit_deal(ParticipantId(2), deal, 0).is_err(),
        "caller-supplied dealer identity must equal the dealer identity carried by the deal"
    );
}

#[test]
fn red_rejects_missing_recipient_share() {
    let mut ceremony = feldman_ceremony(2, 3);
    let dealer = Dealer::new(ParticipantId(1), 2, 3, &mut OsRng).unwrap();
    let mut deal = dealer.generate_deal();
    deal.shares.pop();

    assert!(
        ceremony.submit_deal(ParticipantId(1), deal, 0).is_err(),
        "deal admission must require exactly one share for every recipient 1..=n"
    );
}

#[test]
fn red_rejects_duplicate_recipient_index() {
    let mut ceremony = feldman_ceremony(2, 3);
    let dealer = Dealer::new(ParticipantId(1), 2, 3, &mut OsRng).unwrap();
    let mut deal = dealer.generate_deal();

    // Replace recipient 3 with another recipient-2 slot. The value remains the
    // original f(3), so this also demonstrates that detecting an invalid share
    // is not enough if the malformed deal is still inserted into ceremony state.
    deal.shares[2].index = 2;

    assert!(
        ceremony.submit_deal(ParticipantId(1), deal, 0).is_err(),
        "recipient indexes must form the exact unique set 1..=n"
    );
}

#[test]
fn red_rejects_embedded_share_dealer_substitution() {
    let mut ceremony = feldman_ceremony(2, 3);
    let dealer = Dealer::new(ParticipantId(1), 2, 3, &mut OsRng).unwrap();
    let mut deal = dealer.generate_deal();
    deal.shares[0].dealer = 2;

    assert!(
        ceremony.submit_deal(ParticipantId(1), deal, 0).is_err(),
        "every embedded share must bind to the admitted dealer"
    );
}

#[test]
fn red_duplicate_complaint_does_not_mint_independent_complainants() {
    let mut ceremony = feldman_ceremony(2, 3);
    for id in 1..=3u32 {
        let dealer = Dealer::new(ParticipantId(id), 2, 3, &mut OsRng).unwrap();
        ceremony
            .submit_deal(ParticipantId(id), dealer.generate_deal(), 0)
            .unwrap();
    }

    ceremony
        .file_complaint(ParticipantId(1), ParticipantId(3))
        .unwrap();
    let duplicate = ceremony.file_complaint(ParticipantId(1), ParticipantId(3));

    assert!(
        duplicate.is_err(),
        "the same (complainer, accused) pair must not count twice"
    );
}

#[test]
fn red_disqualified_nondealers_cannot_underflow_qualified_count() {
    let mut ceremony = feldman_ceremony(2, 5);

    for id in 1..=2u32 {
        let dealer = Dealer::new(ParticipantId(id), 2, 5, &mut OsRng).unwrap();
        ceremony
            .submit_deal(ParticipantId(id), dealer.generate_deal(), 0)
            .unwrap();
    }

    // Three registered participants did not deal. Excluding them leaves exactly
    // the two submitted dealers, which is still enough to meet t=2.
    ceremony
        .exclude_and_continue(
            &[ParticipantId(3), ParticipantId(4), ParticipantId(5)],
            100,
        )
        .unwrap();

    let result = catch_unwind(AssertUnwindSafe(|| ceremony.finalize()));
    assert!(
        result.is_ok(),
        "qualified-deal accounting must not panic when disqualified nondealers outnumber submitted deals"
    );
    assert!(
        result.unwrap().is_ok(),
        "the two actually submitted, non-disqualified deals satisfy t=2"
    );
}

#[test]
fn red_rejects_hash_reveal_before_commitment_collection_closes() {
    let mut ceremony = hash_ceremony(CommitmentScheme::HashBased);
    let dealer = Dealer::new(ParticipantId(1), 2, 3, &mut OsRng).unwrap();
    let deal = dealer.generate_deal();
    let (commitment, reveal) = HashCommitmentSet::from_deal(&deal, &mut OsRng);

    ceremony
        .submit_hash_commitment(ParticipantId(1), commitment, 0)
        .unwrap();

    assert!(
        ceremony
            .submit_hash_reveal(ParticipantId(1), reveal, 0)
            .is_err(),
        "a reveal must not be admitted while other parties are still committing"
    );
}

#[test]
fn red_rejects_duplicate_hash_reveal() {
    let mut ceremony = hash_ceremony(CommitmentScheme::HashBased);
    let mut reveals = Vec::new();

    for id in 1..=3u32 {
        let dealer = Dealer::new(ParticipantId(id), 2, 3, &mut OsRng).unwrap();
        let deal = dealer.generate_deal();
        let (commitment, reveal) = HashCommitmentSet::from_deal(&deal, &mut OsRng);
        ceremony
            .submit_hash_commitment(ParticipantId(id), commitment, 0)
            .unwrap();
        reveals.push(reveal);
    }

    let first = reveals[0].clone();
    ceremony
        .submit_hash_reveal(ParticipantId(1), first.clone(), 0)
        .unwrap();

    assert!(
        ceremony
            .submit_hash_reveal(ParticipantId(1), first, 0)
            .is_err(),
        "one dealer must not be able to replace an already-admitted reveal"
    );
}

#[test]
fn red_hash_profile_cannot_finalize_without_reveal_verification() {
    let mut ceremony = hash_ceremony(CommitmentScheme::Hybrid);
    let mut deals = Vec::new();

    for id in 1..=3u32 {
        let dealer = Dealer::new(ParticipantId(id), 2, 3, &mut OsRng).unwrap();
        let deal = dealer.generate_deal();
        let (commitment, _reveal) = HashCommitmentSet::from_deal(&deal, &mut OsRng);
        ceremony
            .submit_hash_commitment(ParticipantId(id), commitment, 0)
            .unwrap();
        deals.push((ParticipantId(id), deal));
    }

    for (id, deal) in deals {
        ceremony.submit_deal(id, deal, 0).unwrap();
    }

    assert!(
        ceremony.finalize().is_err(),
        "HashBased/Hybrid profiles must not finalize before required reveal verification succeeds"
    );
}

#[test]
fn red_share_dealer_mismatch_cannot_be_hidden_by_successful_finalization() {
    let mut ceremony = feldman_ceremony(2, 3);

    let d1 = Dealer::new(ParticipantId(1), 2, 3, &mut OsRng)
        .unwrap()
        .generate_deal();
    ceremony.submit_deal(ParticipantId(1), d1, 0).unwrap();

    let mut d2 = Dealer::new(ParticipantId(2), 2, 3, &mut OsRng)
        .unwrap()
        .generate_deal();
    for share in &mut d2.shares {
        share.dealer = 1;
    }

    assert!(
        ceremony.submit_deal(ParticipantId(2), d2, 0).is_err(),
        "malformed embedded dealer identities must fail at admission, not be ignored later"
    );
}

#[test]
fn red_corpus_has_no_security_claim_from_test_presence() {
    // Deliberate theorem marker: a regression corpus is evidence of specified
    // expectations only. It is not evidence that the parent implementation
    // satisfies them until execution demonstrates the expected disposition.
    assert_ne!(Scalar::from_u64(1), Scalar::zero());
}
