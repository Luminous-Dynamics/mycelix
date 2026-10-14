# AC-087: Vote to Decision integrity qualification

## Scope

AC-087 hardens the Hearth Decisions integrity boundary for newly created Vote records.

The Vote now has a deterministic structural dependency on the exact Decision Create action named by decision_hash.

## Required invariants

For a new Vote:

1. decision_hash must resolve to a valid application Decision Create action in this integrity zome.
2. choice must be within the Decision's immutable option vector.
3. created_at must be on or after the Decision's created_at.
4. created_at must be on or before the Decision's immutable deadline.

The checks use hash-addressed Decision content plus the Holochain action timestamps carried by the Decision Create and Vote Create actions. These timestamps are authoritative for record provenance, but Holochain documents them as self-reported, so this is not a trusted wall-clock deadline proof. Validation does not use the Vote's self-reported created_at field, mutable link collections, current wall-clock time, live membership state, or coordinator call provenance.

## Dependency semantics

The Decision is retrieved with must_get_valid_record.

A missing Decision remains an unresolved dependency and can be retried after DHT delivery. An existing record of the wrong action or entry type is an explicit validation failure.

This distinction is intentional: incomplete DHT delivery is not evidence that the Vote itself is invalid.

## Test coverage

Pure integrity tests cover:

- a two-option Decision rejecting choice 2;
- a twenty-option Decision accepting choice 19;
- a Vote action timestamp preceding the Decision action timestamp being rejected;
- a Vote action timestamp after the Decision deadline being rejected;
- a Vote action timestamp exactly at the deadline being accepted;
- an explicit characterization that action timestamps do not establish external wall-clock time;
- a forged Vote.created_at value being unable to bypass a valid action-timestamp deadline check.

The host-dependent reference-resolution path is kept separate from pure value validation so failure semantics remain explicit.

## Qualification boundary

AC-087 establishes the structural relationship between Vote and Decision. It does not establish authorship (AC-068), active membership eligibility, DHT-wide vote-set completeness, distributed exactly-once semantics, finalization legitimacy, or governance legitimacy.

Those remain separate acceptance criteria.