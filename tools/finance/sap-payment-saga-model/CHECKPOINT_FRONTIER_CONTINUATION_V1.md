# Owner-scoped checkpoint frontier continuation v1

**Status:** effects-disabled, cross-language structural continuation verifier. It composes with the existing MXSAP-JOURNAL-CHECKPOINT-SUBJECT-V1 subject and does not add a checkpoint format, journal, identity algorithm, detached signature or production Holochain adapter.

## Problem

The checkpoint subject already binds an immutable operation/intent, owner domain, event-log identity, event count/tail digest, checkpoint sequence/prior digest and opaque source-action/frontier references. Subject validation alone does not establish that checkpoint N+1 is a valid continuation of checkpoint N.

This increment defines the narrow structural relation required for a one-step frontier advance. It reuses the current canonical checkpoint subject and its intent-bound digest.

## Contract

Given previous checkpoint P, next checkpoint N and the exact immutable payment intent I:

1. P and N must each pass the existing checkpoint subject validation against I.
2. `operation_id` and `intent_digest` must remain identical.
3. Owner domain and `event_log_id` must remain identical. Payments and Treasury remain separate checkpoint chains.
4. N's `checkpoint_sequence` must equal P's sequence plus one, with overflow rejected.
5. N's `prior_checkpoint_digest` must equal the exact canonical digest of P, calculated through the intent-bound checkpoint API.
6. N's `event_count` must strictly increase; `tail_digest` and `source_chain_frontier` must be distinct from P's.
7. N's `source_action_hash` must be distinct from P's source-action reference.

Issuer DID, verification method and authority-grant reference are allowed to change. Key rotation and grant succession are valid research cases, but an external verifier must independently establish historical signature validity, current authority, revocation status and the correct scope for each checkpoint. A string change is not a proof of rotation or authorization.

## Why not compare timestamps or opaque hashes?

The checkpoint's `issued_at_unix_ms`, source ActionHash and source-chain frontier are not sortable logical counters. This verifier does not compare their lexical/numeric order or trust wall clocks. It checks only exact predecessor digest linkage, owner-local sequence and event-count advancement, and rejects obvious reuse of identical opaque references.

A later Holochain adapter must validate the actual signed Create action, its author, exact source action and predecessor/source-chain semantics. This module cannot prove that a changed ActionHash or frontier string is a genuine successor.

## Cross-language implementation

- Rust: `CheckpointSubjectV1::validate_against_intent` plus `validate_checkpoint_frontier_advance_v1`.
- Go: `CheckpointSubjectV1.ValidateAgainstIntent` plus `ValidateCheckpointFrontierAdvanceV1`.
- Both implementations recompute the previous digest with the existing canonical intent-bound API.
- Negative cases include wrong owner/log, skipped sequence, wrong prior digest, no event-count/tail/frontier advance, reused source-action identity and u64 sequence overflow.
- Positive rotation case changes issuer DID, verification method and authority-grant reference while preserving structural continuity.

## Safety boundary

The validator establishes only:

> The supplied next subject is a structurally contiguous, intent-bound checkpoint subject that references the exact digest of the supplied previous subject and advances the declared owner-local frontier fields.

It does not establish:

- issuer/authorship authenticity or Holochain ActionHash validity;
- owner-domain authorization or grant correctness;
- revocation, key rotation or grant succession validity;
- DHT availability/completeness, rollback resistance, competing-fork resolution or global uniqueness;
- truth of any event, balance movement, payment settlement, legal discharge or beneficiary receipt.

A correct digest chain can still be fabricated by a party able to rewrite both checkpoints. Native Holochain action validation and source-action verification remain mandatory before relying on this as an authenticated frontier.

## Next qualification

1. Bind each checkpoint entry to its native signed Holochain Create action and actual action author.
2. Verify issuer DID mapping and scope-bound owner grant from deterministic, hash-addressed dependencies.
3. Verify the exact source ActionHash, entry type, owner source-chain predecessor and source frontier.
4. Preserve Holochain `Valid`, `Invalid` and `UnresolvedDependencies` separately; unresolved is not absence or acceptance.
5. Qualify stale checkpoints, partial DHT visibility, competing forks, key revocation, grant supersession and restart/recovery across multiple conductors.
6. Keep Payments and Treasury independent; an unavailable Treasury checkpoint must not be filled in by a Payments checkpoint or global status projection.

## Benchmarks

Measure structural digest verification separately from Holochain host calls and DHT retrieval:

- 1, 10, 100, 1,000 and 10,000 sequential checkpoints per owner;
- p50/p95/p99 canonical digest and continuation-check latency;
- bytes/checkpoint, memory and replay cost;
- checkpoint-chain truncation/rollback detection;
- stale-authority acceptance and revocation-propagation latency in the later runtime adapter;
- partial-owner reconciliation time after restart and network partition.

Hard safety targets: zero acceptance of a skipped sequence, wrong previous digest, owner/log substitution, stale tail or mismatched immutable intent. Performance cannot compensate for any false acceptance.

No performance result or production qualification is claimed by this reference implementation.
