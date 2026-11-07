# LIFE-COMPOSITE-BUDGET-001 — Overlapping-scope disclosure budgets

## Question

Can a federation preserve disclosure-budget invariants when one optional release consumes risk across multiple simultaneously visible scopes—person, household/cooperative/community, purpose and observer/collusion domain—and when newly visible relationship edges make older releases linkable?

This is a deterministic, synthetic Rust state machine, not a production ledger and not differential privacy.

## Paradigm

A per-account budget alone misses composition across people. A group budget alone misses overlapping groups and across-purpose releases. Charging only the groups known at original release time misses historical disclosures that become correlatable after an edge is added.

The benchmark therefore treats a successful disclosure as an **atomic admission across every currently applicable scope**. It also treats a once-public relation edge as monotone: removing it later does not imply observers forget the historical association. New edges are evaluated against historical accepted releases before the edge becomes visible.

## Scope semantics in v0.1

- **Person / observer domain:** total abstract risk units for one subject's disclosures that a named observer or colluding coalition can associate.
- **Group / observer domain:** total abstract risk units across all currently linked members' accepted releases in that observer domain, across purposes.
- **Purpose / observer domain:** total abstract risk units for that purpose as seen by the observer domain, across subjects.
- **Observer domain:** total risk units that the coalition can compose within the domain.
- **Membership epoch:** global canonical relation-registry version. Requests bind to it; stale snapshots fail closed.
- **Policy epoch:** version of future admission rules. It does not refund historical spending.
- **Request identity:** stable request ID plus all subject/domain/purpose/cost/policy/membership/digest fields. Exact retry is idempotent; changed reuse is a conflict.

Budgets use **abstract integer risk units**. They are not epsilon/delta, not calibrated privacy-loss units and not a differential-privacy guarantee. The person-only profile is intentionally unsafe and exists as a negative control.

## Scenarios

1. unsafe_person_only_control: four cost-3 releases each fit the person's budget but the common group reaches 12 against a group limit of 8.
2. composite_atomic_gate: all applicable scopes are checked; the third/fourth release are rejected when group spend would exceed the cap, with no partial spend.
3. overlapping_groups_atomic_gate: a subject belongs to two groups; one release consumes budget in both. A later release rejected by either scope consumes no budget in either.
4. retroactive_relinking_guard: three subjects each release cost 3 before they share a visible group edge. The first two edges fit historical group spend at 3 and 6; the third would make 9 against a limit of 8 and is rejected/quarantined. Previously released records remain in the audit; the model does not claim to undo them.
5. policy_rollover_no_refund: a policy epoch increment does not refund a previously accepted cost-7 disclosure; the stale old-epoch request is rejected and the new cost-2 request still fails the group cap.

## Independent audit

The auditor rebuilds total accepted spend and current group/observer-domain exposure from accepted-request records and the canonical membership registry. It never reads Metrics or reducer-side spend counters. This makes it possible to compare reducer behavior against an independent replay.

## Run

~~~sh
cargo +1.96.0 fmt --manifest-path research/life-composite-disclosure-budgets/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-composite-disclosure-budgets/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-composite-disclosure-budgets/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-composite-disclosure-budgets/Cargo.toml
~~~

The dedicated GitHub Actions workflow checks out the exact PR head and asserts the checked-out SHA, runs format/tests/strict Clippy, compares two report files byte-for-byte, and stores the report, frozen manifest, SHA-256 sums and qualification receipt.

## Current verification note

The first exact-head run passed formatting but exposed two test-fixture errors. The composite rejection assertion tried a cost-2 request against 6 spent / budget 8, so it was correctly accepted at the boundary; the fixture now tries cost 3 to exceed the cap. The manifest assertion now matches the actual pretty-printed JSON punctuation. The next exact-head run must qualify these corrections; the earlier run is not a pass.

## Proposed acceptance criteria (not achieved results)

- Zero accepted over-budget releases under the composite policy.
- Zero partial spend on any rejected request.
- Exact replays do not spend again; changed reuse is rejected.
- Any new public group edge recomputes the historical releases newly made linkable before it can commit.
- No membership or policy epoch increment refunds accepted disclosure history.
- Independent audit totals agree with accepted history; a deliberate person-only negative control exposes the expected group overrun.
- Fixed source + fixtures + manifest produce byte-identical reports.

## Boundaries and next step

A passing run qualifies only this exact synthetic state machine. It does not establish production consensus, authenticated membership authority, tamper-proof persistence, calibrated privacy risk units, DP, anonymity or a real-world group-privacy guarantee. Production adoption would require threat-modeling the observer/collusion domain, authenticated membership changes, concurrency/race semantics, durable idempotency and an independently reviewed implementation.

Related:
- #5466 / PR #5468: federated single-budget conservation
- #5474 / PR #5475: relational disclosure spillover
- #5477: privacy-unit and group-guarantee mismatch

