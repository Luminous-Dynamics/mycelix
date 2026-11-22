# Mycelix Directory Fabric History Oracle

Independent, dependency-free Rust qualification checker for captured Directory Fabric execution histories.

This crate is intentionally separate from `myc-dir-fabric-reference`. The implementation being qualified must not be the only implementation judging its own safety.

## Current scope

The first slice checks:
- unique event identities and causal-parent graph structure;
- owner-local durable sequence uniqueness and consistency with causal order;
- owner bootstrap and monotonic `(owner_epoch, generation)` transitions;
- effect commits against the owner position current at the owner-local commit sequence; owner-scoped logical-operation and idempotency-key uniqueness;
- worker-claim epoch freshness at the mutation commit point;
- exact attempt, logical-operation and idempotency bindings;
- Unknown outcome reconciliation and retry-after-confirmed-no-effect;
- fence commit -> durable read-back -> owner acknowledgement chains;
- exact owner-set equality for global barrier acknowledgement;
- post-barrier route staging and activation against current inventory, owner read-back and atomic fence enforcement.

The checker returns `Valid`, `Violation` or `Inconclusive`. Incomplete source evidence, missing causal parents and unresolved Unknown attempts are never coerced into a pass. A counterexample takes precedence over inconclusive findings in the final report.

## Run

From the repository root:

```sh
cargo test --manifest-path tools/myc-dir-fabric-history-oracle/Cargo.toml
```

There are currently 17 authored tests. They have **not yet been compiled or executed** in the editing environment.

## Evidence adapter contract

Before calling `audit_history`, the ingestion adapter must:
1. authenticate each source and verifier/build identity;
2. independently verify signed decision, claim, owner and barrier receipts;
3. establish the completeness of the event bundle and each required owner stream;
4. preserve owner-local durable sequence/commit positions; the exact route-to-owner map; and the single realm/resource/action scope commitment for the bundle;
5. canonicalize digests and event IDs exactly as specified by the producer protocol;
6. supply the required owner-stream set from the verified inventory closure.

`event_set_complete` and `complete_owner_streams` are assertions by the ingestion adapter. The checker cannot independently prove those assertions. A production adapter must not set them optimistically.

## Ordering model

Wall-clock timestamps are not used for safety ordering. Within one effect owner, the oracle uses durable owner-local sequence positions. The causal graph constrains cross-owner dependencies: fence commit, read-back, owner acknowledgement, inventory closure, global acknowledgement and route activation.

There is deliberately no invented total order across independent owners. The global barrier is checked as a causal certificate over the exact owner-acknowledgement set.

## Explicit non-claims

This crate does not:
- authenticate signatures or establish trust roots;
- prove that the effect-route inventory is complete;
- implement broker/database adapters;
- provide a distributed consensus or fencing service;
- establish PostgreSQL/WAL commit durability;
- verify cryptographic hashes from the supplied digests;
- replace TLC, Porcupine, Jepsen or real-system fault injection;
- establish any runtime result or SOTA claim.

Porcupine and Jepsen remain generic linearizability/consistency comparators. The value of this checker is the Directory Fabric-specific sequential contract around worker claims, owner epochs, resource fences, durable acknowledgement chains and strict revocation barriers. The next stage is an independent event-ingest adapter and conformance corpus, followed by comparison against Porcupine/Jepsen-compatible models.
