// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_business_core::{
    AllocationLine, AllocationRef, AllocationStatement, DomainRef, DomainScopedRef, Quantity,
    SemanticProfileId, SourceEventRef, SubjectRef, UnitId,
};

fn statement() -> AllocationStatement {
    let usd = UnitId::new("USD-cent").expect("valid unit");
    AllocationStatement {
        allocation_ref: DomainScopedRef::new(
            DomainRef::new("finance").expect("valid domain"),
            AllocationRef::new("allocation:1").expect("valid allocation"),
        ),
        source_event: SourceEventRef::new("bank", "settlement:1").expect("valid event"),
        semantic_profile: SemanticProfileId::new("finance.allocation", 1).expect("valid profile"),
        total_available: Quantity::new(usd.clone(), 100_000),
        lines: vec![AllocationLine {
            target: SubjectRef::new("invoice:a").expect("valid target"),
            quantity: Quantity::new(usd, 60_000),
        }],
    }
}

#[test]
fn validated_summary_retains_exact_statement_snapshot() {
    let mut candidate = statement();
    let summary = candidate
        .validate_conservation()
        .expect("candidate conserves declared capacity");

    candidate.lines[0].quantity.amount = 90_000;

    assert_eq!(summary.statement().lines[0].quantity.amount, 60_000);
    assert_eq!(summary.allocated().amount, 60_000);
    assert_eq!(summary.remaining().amount, 40_000);
}

#[test]
fn validation_of_changed_statement_produces_distinct_summary_basis() {
    let first = statement();
    let first_summary = first.validate_conservation().unwrap();

    let mut second = statement();
    second.lines[0].quantity.amount = 70_000;
    let second_summary = second.validate_conservation().unwrap();

    assert_ne!(first_summary.statement(), second_summary.statement());
    assert_ne!(first_summary.allocated(), second_summary.allocated());
}
