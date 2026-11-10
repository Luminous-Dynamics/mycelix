# Payment-channel fork reference model v0.1

**Status:** research/reference model; effects-disabled; synthetic-only. This is a child qualification slice for Mycelix [#5712 — concurrent channel updates, forks and settlement semantics](https://github.com/Luminous-Dynamics/mycelix/issues/5712), stacked on [#5714 — participant-bound channel updates](https://github.com/Luminous-Dynamics/mycelix/pull/5714).

## Why this exists

The current shared Finance update-chain helper chooses the lowest ActionHash when several updates branch from one predecessor. That produces a deterministic read projection, but it cannot establish that a losing branch did not receive a successful local write response or that the selected branch had exclusive spend authority.

Holochain's own entry guidance says that applications must define whether two updates are conflicts and choose a retrieval-time resolution strategy when one canonical version is required. Its countersigning documentation distinguishes agreement from consensus and says countersigning is not a double-spend prevention mechanism. The page currently labels countersigning unstable and disabled by default in Holochain 0.4 and newer.

This model makes a deliberate semantic change: **the complete competing-claim set is the input to reconciliation; it is never reduced to a single branch by timestamp, ActionHash order, arrival order or caller preference.**

## Model

The standalone Rust library has no external dependencies and no runtime or ledger side effects.

It models:
- immutable channel identity, participants, currency and initial total;
- an exact predecessor reference and monotonic candidate sequence;
- transfer and close transitions with checked arithmetic and conservation;
- stable sender-scoped operation IDs;
- exact duplicate delivery versus a second DHT action reference;
- participant-consent claim shape;
- single-writer epoch claims;
- mutual-consent claims;
- witness-quorum claims with the intersection condition (2q > n + f), where (n) is the witness roster size, (q) the threshold and (f) the declared Byzantine bound;
- deterministic, order-invariant conflict census.

A distinct, structurally valid candidate from the same predecessor produces `Conflicted`, including a close racing a transfer. The resolution retains the full distinct candidate claim payloads as well as the action-reference list, including both payloads if one action reference is reused for different content. Exact repeat delivery of the same candidate reference and payload is counted as a duplicate observation and stored only once in the distinct census. Reusing one operation ID with a changed payload is a conflict.

A single candidate whose policy claims are structurally consistent yields `PolicyClaimConsistentPendingFinality`. That name is intentionally weaker than `Authorized`, `Committed` or `Settled`. No settlement-finality state exists in this model.

## Authorization and trust boundaries

The `AuthorizationClaim` values are synthetic claims. The model checks their shape, signer membership, sequencing role and threshold arithmetic. It does **not**:
- verify Ed25519 or other signatures;
- authenticate DID-to-agent-key bindings;
- resolve or validate real Holochain ActionHash records;
- prove source-chain continuity, DHT uniqueness or non-equivocation;
- prove that the witness roster is independent or that the Byzantine bound is true;
- simulate network partitions, conductor restarts, DHT gossip or Sweettest;
- contact a bank, payment service provider, escrow or external rail;
- establish funded backing, ledger conservation across agents, or settlement finality.

A production adapter must verify exact domain-separated evidence before presenting a typed authorization result to any later enforcement layer. A string-shaped evidence reference is not a signature, and a quorum-count check is not proof that the witnesses are independent or honest.

## Strategy comparison

| Strategy | What it can contribute | What it cannot establish alone |
|---|---|---|
| Deterministic projection | Stable display / convergent read choice | Exclusive spend authority, absence of a losing effect, or settlement |
| Single writer + epoch | Reduces ordinary concurrent writers and supplies a clear sequencing role | A distributed lock, compare-and-swap, or immunity to the writer's own fork |
| Mutual participant consent | Both participants' consent to the exact transition | Prevention of both participants signing competing transitions; availability when either party is offline |
| Intersecting witness quorum | Can make conflicting certificates impossible under a declared roster, honest-intersection assumption and non-equivocation rule | The truth of the roster/fault bound, independence of witnesses, or progress when a partition lacks a quorum |
| External rail / escrow authority | Can provide a separately defined settlement-finality source | A generic guarantee that applies to every provider or instrument |

For a quorum roster of (n) witnesses and at most (f) Byzantine witnesses, requiring (2q > n+f) guarantees that two quorums intersect in more than (f) members. That only helps if the roster is authentic, the fault bound is defensible, and honest witnesses will not sign two conflicting transitions for the same channel, epoch and predecessor. The model deliberately returns conflict if two quorum-claimed branches are observed anyway; it does not pick a winner or hide a failed assumption.

### Industry comparison

- **Holochain:** its current entry guide explicitly leaves update-conflict semantics to the application. Its countersigning guide states that countersigning synchronizes an agreed write but is not itself double-spend prevention. The same guide currently labels countersigning unstable and disabled by default in Holochain 0.4+. Mycelix must not assume this feature is a universally available payment-finality primitive.
- **Lightning:** BOLT #2 specifies a commitment-signature / revocation exchange for channel updates; BOLT #5 addresses on-chain handling of commitment states. It explicitly accounts for a window in which more than one valid, unrevoked commitment transaction may exist and requires nodes to handle either. This is a stronger, rail-specific dispute model because it is tied to enforceable on-chain transactions and revocation rules; it is not a direct drop-in for Holochain or a metric to compare without matched conditions.
- **Consensus/quorum systems:** these can provide stronger uniqueness/finality within explicit membership, fault and availability assumptions, but pay for that with coordination and partition sensitivity. Mycelix should report the safety/liveness trade-off rather than claiming universal availability.

References:
- [Holochain — Entries and resolving update conflicts](https://developer.holochain.org/build/entries/)
- [Holochain — Countersigning: agreement, consensus and double spending](https://developer.holochain.org/concepts///10_countersigning)
- [Holochain — Validation](https://developer.holochain.org/build/validation/)
- [Lightning BOLT #2 — Peer protocol](https://github.com/lightning/bolts/blob/master/02-peer-protocol.md)
- [Lightning BOLT #5 — On-chain transaction handling](https://github.com/lightning/bolts/blob/master/05-onchain.md)

## Test corpus

The 25 current unit tests cover:
- empty observation set and one policy-claim-consistent branch;
- exact duplicate delivery and duplicate action references;
- two competing transfers from the same predecessor;
- same operation ID with changed amount;
- close-versus-transfer races;
- stale predecessor, sequence gap, zero transfer, overdraw and closed channel;
- projection-only, single-writer, mutual-consent and intersecting-quorum claim rules;
- close requiring both participants;
- unexplained base-state balance changes and integer overflow;
- candidate-order permutation invariance.

These are deterministic synthetic unit tests, not multi-agent integration results. The next integration gate is a real multi-conductor test that creates competing candidate updates from the same predecessor, observes partial DHT views in both orders, reconnects peers, and verifies that the complete branch census converges to `Conflicted` rather than selecting a financially final winner.

## Benchmark programme

### Correctness gates (non-compensatory)

Targets:
- 100% of frozen positive/negative scenarios produce their declared disposition;
- zero silent discard of a structurally valid competing branch;
- zero conversion of a deterministic projection into an authorized or settled outcome;
- zero same-operation-ID / changed-payload acceptance;
- exact duplicate delivery does not create a second logical operation;
- candidate arrival/order permutations preserve the same conflict disposition and candidate census;
- no value-conservation or integer-overflow escape.

### Performance and liveness measurements

Measure separately, per strategy and workload:
- candidate count per predecessor: 1, 2, 4, 16, 64, 256, 1,024 and 10,000 where memory permits;
- duplicate-delivery ratio and conflict width;
- p50/p95/p99 reconciliation latency, CPU and peak RSS;
- healthy-network authorization and finality latency;
- time to indeterminate / time to recovery after partition;
- percent of operations that remain indeterminate after reconnect;
- availability when a single writer, one participant, or a quorum is unavailable;
- throughput under same-channel contention and independent-channel concurrency.

Retain raw samples, exact candidate bytes/fixture digests, source SHA, Rust toolchain, OS/CPU and build profile. Do not merge correctness, latency and availability into a single score. Do not set a universal latency threshold until the first exact-host baseline has been captured.

### Required distributed scenarios

1. A and B propose distinct transitions from the same predecessor concurrently.
2. Same operation ID and same payload retries after response loss.
3. Same operation ID is reused with a changed amount, recipient or currency.
4. Same semantic transition appears under two different action references.
5. One peer sees branch A while another sees branch B; both exchange DHT operations in either arrival order.
6. Close races a transfer.
7. Stale-predecessor update arrives after a newer state.
8. One party or the sequencer restarts after local write but before response.
9. Witness network is partitioned with and without quorum availability.
10. Two quorum claims are presented despite the assumed honest-intersection rule.
11. A candidate is policy-consistent but an external rail returns timeout/unknown.
12. A replay arrives after close or dispute.

## Run

```sh
cargo fmt --manifest-path tools/finance/payment-channel-fork-reference/Cargo.toml --all --check
cargo test --manifest-path tools/finance/payment-channel-fork-reference/Cargo.toml --all-targets
cargo clippy --manifest-path tools/finance/payment-channel-fork-reference/Cargo.toml --all-targets -- -D warnings
```

The dedicated exact-head workflow pins Rust 1.96.0. No local compile/test/Clippy result is claimed from the authoring environment.

## Relationship to existing work

- [#5711 / PR #5714](https://github.com/Luminous-Dynamics/mycelix/pull/5714): participant-bound channel debit and integrity-layer transition validation. This model is stacked on that draft and does not replace its prerequisite.
- [#5712](https://github.com/Luminous-Dynamics/mycelix/issues/5712): this reference model addresses the first software-model/conformance slice only.
- [#5713](https://github.com/Luminous-Dynamics/mycelix/issues/5713): funded backing remains separate and blocked on SAP provenance/conservation, concurrency and idempotency prerequisites.
- [#5744 / PR #5745](https://github.com/Luminous-Dynamics/mycelix/pull/5745): payment saga / recovery; this model does not duplicate the payer-to-recipient saga.
- [#5716 / PR #5755](https://github.com/Luminous-Dynamics/mycelix/pull/5755): payment-receipt authenticity; receipt authenticity does not prove channel spend uniqueness or external settlement.

This is a research reference, not a production payment-channel implementation, funded custody system, regulatory assessment or security audit.
