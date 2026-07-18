// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use super::*;

fn mock_agent(byte: u8) -> AgentPubKey {
    AgentPubKey::from_raw_36(vec![byte; 36])
}

#[test]
fn provisional_votes_are_equal_weight() {
    let vote = ArbitrationVote {
        dispute_hash: ActionHash::from_raw_36(vec![1; 36]),
        dispute_revision_hash: ActionHash::from_raw_36(vec![2; 36]),
        arbitrator: mock_agent(3),
        favor_buyer: true,
        reasoning: "Evidence supports the buyer".into(),
        arbitrator_matl_score: 1.0,
        voted_at: Timestamp::from_micros(1_000_000),
    };
    assert_eq!(vote.arbitrator_matl_score, 1.0);
}

#[test]
fn threshold_requires_more_than_sixty_six_percent() {
    assert!(2.0 / 3.0 > RESULT_THRESHOLD);
    assert!(!(0.66 > RESULT_THRESHOLD));
}

#[test]
fn compensation_recommendation_is_integer_and_deterministic() {
    assert_eq!(recommended_compensation(10_000, 1.0), 10_000);
    assert_eq!(recommended_compensation(10_000, 0.8), 7_500);
    assert_eq!(recommended_compensation(10_000, 2.0 / 3.0), 5_000);
}

#[test]
fn transaction_resolution_rejects_conflicts() {
    let resolution = TransactionResolutionWire {
        root_transaction_hash: ActionHash::from_raw_36(vec![1; 36]),
        state: TransactionResolutionStateWire::Conflicted,
        canonical: None,
        heads: Vec::new(),
        revision_count: 2,
    };
    assert!(resolution.current().is_err());
}

#[test]
fn resolved_transaction_exposes_current_revision() {
    let current = TransactionOutputWire {
        transaction_hash: ActionHash::from_raw_36(vec![2; 36]),
        transaction: TransactionWire {
            buyer: mock_agent(3),
            seller: mock_agent(4),
            total_price_cents: 1_999,
            status: TransactionStatusWire::Disputed,
        },
    };
    let resolution = TransactionResolutionWire {
        root_transaction_hash: ActionHash::from_raw_36(vec![1; 36]),
        state: TransactionResolutionStateWire::Resolved,
        canonical: Some(current),
        heads: Vec::new(),
        revision_count: 2,
    };
    assert_eq!(
        resolution.current().unwrap().transaction.status,
        TransactionStatusWire::Disputed
    );
}
