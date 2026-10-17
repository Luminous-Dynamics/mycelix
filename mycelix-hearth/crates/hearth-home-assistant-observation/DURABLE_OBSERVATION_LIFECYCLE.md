# Durable observation lifecycle journal (HA-0.2C-D)

**Status: append-and-sync interface plus in-memory test store; no production storage backend or crash-durability qualification is claimed.**

This tranche adds a durable commit boundary around the pure [HA-0.2C observation lifecycle](OBSERVATION_LIFECYCLE.md). It is intentionally separate from the provider projector and from Hearth's physical-effect journal.

## Commit theorem

```text
candidate lifecycle transition
+ bounded value-minimized journal envelope
-> store.append_and_sync(record)
-> only after success:
   publish candidate lifecycle state and typed observations
```

Every call is applied to a private clone. The store must append the entire envelope and sync it before returning success. Only then does the wrapper replace its active lifecycle/currentness view or return positive observations. If append-and-sync returns an error, the outcome is treated as unknown: the wrapper preserves its prior lifecycle and currentness maps internally, latches unhealthy, reports no current positive revision, and rejects further operations. Reopen and recover from the store; do not retry through the faulted instance.

A partially written record is not repaired optimistically. A backend must either provide whole-record append semantics or allow recovery to detect a truncated/invalid final record and fail closed.

## Record and recovery contract

The journal envelope binds a schema version, strictly increasing global ledger sequence, immutable profile reference, transition kind, phase-before/after, session-epoch metadata, local timestamp, typed failure category, and zero or more entity lifecycle records.

Recovery validates:

- exact sequence starting at 1 with no gaps/duplicates;
- supported schema and exact profile reference;
- continuous prior-phase and epoch metadata;
- legal transition order;
- per-entity revisions advancing exactly one step;
- each invalidation naming the precise positive revision it revokes;
- each positive revision either starting an entity's current chain or superseding the exact current revision;
- entity records limited to the profile allowlist.

Recovery reconstructs revision/epoch high-water marks and a value-free audit chain, **never positive currentness**. It appends a recovery barrier into `Dormant` before accepting new events. A strictly higher session epoch and fresh snapshot are required to become live again.

## Privacy choices

Journal records never contain raw Home Assistant state strings, arbitrary attribute bags, source timestamps, credentials, URLs, or unsalted value hashes. Low-entropy hashes of binary or categorical state can reveal the original value via dictionary attacks. A positive is identified by entity/profile/session/revision, its observed/freshness timestamps, and the global ledger sequence that created it—not by publishing a digest of the state value.

The store interface's success contract is durable append-and-sync. This repository includes only a deterministic in-memory test store; its success is **not** evidence of filesystem sync, atomic replacement, power-loss durability, anti-rollback protection, or process-crash recovery for a concrete backend.

## Required next backend work

A production backend must bind records to the exact file/database identity, define framing and torn-tail detection, sync data and required directory metadata, recover without restoring positive currentness, exercise process-kill/power-loss boundaries, and prove that acknowledged sequence numbers cannot roll back. Measure persistence latency separately from lifecycle reducer overhead. Never weaken sync-before-publish semantics to meet a latency target.

## Test command

`cargo test --manifest-path mycelix-hearth/Cargo.toml -p hearth-home-assistant-observation`

The command was not run locally in this work session. Check the exact-head dedicated workflow before claiming a pass.
