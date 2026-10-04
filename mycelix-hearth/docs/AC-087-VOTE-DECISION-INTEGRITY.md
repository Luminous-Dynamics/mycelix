# AC-087: Vote → Decision integrity qualification

## Scope

AC-087 hardens the Hearth Decisions integrity boundary for newly created Vote records.

The Vote now has deterministic structural dependencies on the exact Decision Create action named by decision_hash.

## Required invariants

For a new Vote:

1. voter must equal the Holochain action author.
2. decision_hash must resolve to a valid application Decision Create action in this integrity zome.
3. choice must be within the Decision's immutable option vector.
4. created_at must be on or after the Decision's created_at.
5. created_at must be on or before the Decision's immutable deadline.

The checks use hash-addressed content only. Validation does not inspect mutable link collections, wall-clock time, live membership state, or coordinator call provenance.

## Dependency semantics

The Decision is retrieved with must_get_valid_record.

A missing Decision therefore remains an unresolved dependency and can be retried after DHT delivery. An existing record of the wrong action/entry type is an explicit validation failure.

This distinction is intentional: incomplete DHT delivery is not evidence that the Vote itself is invalid.

## Test coverage

Pure integrity tests cover:

- a two-option Decision rejecting choice 2;
- a twenty-option Decision accepting choice 19;
- a Vote preceding Decision creation being rejected;
- a Vote after the Decision deadline being rejected;
- a Vote exactly at the deadline being accepted;
- a spoofed voter field being rejected.

The host-dependent reference-resolution path is kept separate from pure value validation so the failure semantics remain explicit.

## Qualification boundary

AC-087 establishes structural Vote integrity. It does not establish:

- active membership eligibility;
- one-vote-per-agent in adversarial distributed conditions;
- DHT-wide vote-set completeness;
- finalization legitimacy;
- governance legitimacy.

Those remain separate acceptance criteria.