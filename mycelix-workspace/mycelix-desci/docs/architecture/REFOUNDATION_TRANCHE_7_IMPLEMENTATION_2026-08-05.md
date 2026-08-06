# Mycelix-DeSci Refoundation Tranche 7

**Date:** 2026-08-05
**Patchset:** v0.8
**Theme:** risk-tiered quorums, organization-diverse approvals, external transparency witnesses, and process-safe reference storage

## Purpose

The v0.7 threshold layer prevented one administrator from unilaterally changing scientific authority, but every action still used one threshold and one delay. It also published locally generated checkpoints without a governed way for independent organizations to attest to them. Finally, the JSON reference stores protected one process but could allow two separately started writers to overwrite a stale prefix.

Tranche 7 closes those gaps without rewriting or invalidating earlier signed governance records.

## 1. Risk-tiered quorum policy

A new additive action installs a three-tier policy:

- routine;
- sensitive;
- critical.

Each tier specifies:

- unique active-administrator threshold;
- minimum distinct administrator organizations;
- activation delay.

The policy validates monotonic strengthening across all three dimensions. Action classification is deterministic code and cannot be supplied by a proposal client.

Accepted proposals snapshot their tier and requirements. Later policy changes cannot weaken an open proposal. At execution, administrators are re-resolved against the credential registry at execution time; revoked roles or keys stop counting.

The first risk policy is installed through the existing base threshold. Every later risk-policy change is itself critical under the previously active policy.

## 2. Organization-diverse approval

Approvals continue to count unique actors. The projection now additionally derives organization diversity from active administrator credentials.

One actor contributes at most one organization, and multiple administrators in the same organization contribute one diversity slot. This prevents account multiplication inside one institution from satisfying a supposedly independent quorum.

Readiness now evaluates the routine tier rather than the bootstrap threshold, ensuring a governed risk policy cannot lower normal production authority beneath the deployment minimum.

## 3. Governed transparency witnesses

The action protocol now supports threshold-governed witness authorization and revocation. A witness authority binds:

- actor;
- organization;
- Ed25519 public key;
- validity interval;
- revocation time.

Witness keys must remain distinct from actor and acceptance-service keys.

A signed witness attestation binds the exact checkpoint hash, actor, organization, witness time, and public key. The governance envelope must use the same actor, signer, and timestamp. Duplicate witness actors for one checkpoint fail closed.

The projection records witness attestations by checkpoint and reports distinct witness organizations for the latest checkpoint. The API exposes checkpoint hashes and proposal approval status. The CLI can append a witness attestation after independently verifying an exported checkpoint.

`DESCI_MIN_CHECKPOINT_WITNESS_ORGANIZATIONS` provides a deployment readiness gate. Repository deployment examples require two organizations; embedded library behavior defaults to zero for backwards compatibility.

## 4. Process-independent optimistic file transactions

The reference JSON journals now use a shared transaction helper that:

1. creates an exclusive sidecar lock;
2. persists lock metadata and fsyncs the parent;
3. rereads the durable expected prefix;
4. rejects stale state;
5. writes a unique temporary file;
6. fsyncs and atomically renames it;
7. fsyncs the parent directory;
8. verifies lock ownership during release.

The helper protects:

- scientific credential registry;
- credential-governance journal;
- individual canonical scientific event streams.

A regression test opens two event-log instances before either writes and proves that the stale second writer is rejected after the first commits.

Lock files are not automatically reclaimed. Operator inspection is required after a crash.

## 5. Compatibility

Earlier governance event codes and canonical bytes remain unchanged. New action and payload codes are appended. Historical v0.7 records therefore retain their signatures and record hashes.

Proposals opened before a risk policy continue to use the legacy base threshold and delay. Proposals opened after installation snapshot the appropriate tier rule.

## 6. Tests added

The tranche adds tests for:

- stale second-process scientific-event writers;
- active transaction-lock exclusion;
- stale expected-state rejection;
- monotonic risk-policy validation;
- critical action classification;
- organization diversity counting institutions rather than accounts;
- transparency witness signature binding.

## 7. Remaining limitations

- The Rust workspace could not be compiled in the patch-generation environment because no Rust toolchain is installed and the standalone archive omits `../crates/mycelix-zkp-core`.
- The authority-receipt chain uses immutable create-new files but does not yet provide one serializable transaction across all receipts and scientific-event side effects.
- Cross-journal credential execution is crash-recovered rather than committed by one database transaction.
- Risk classification is code-defined and does not yet have its own governed version identifier.
- Witness compromise notices and historical uncertainty intervals are not yet modeled.
- Organization identity remains a registry assertion rather than an externally verified institutional credential.

## 8. Next tranche

The next deepest improvement should implement a transactional SQL reference backend with:

- serializable append procedures;
- unique stream-sequence and actor-idempotency constraints;
- transactional governance execution across registry and governance tables;
- authority-receipt outbox/finalization;
- advisory-lock or row-lock semantics;
- deterministic replay parity tests against the file adapter;
- externally operated checkpoint mirror and cosigning protocol.
