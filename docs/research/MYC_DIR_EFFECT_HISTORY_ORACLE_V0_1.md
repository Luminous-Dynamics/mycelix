# Mycelix Directory Fabric: independent effect-history oracle v0.1

**Profile:** MYC-DIR-EFFECT-HISTORY-ORACLE-0.1  
**Status:** research specification; not implemented or executed  
**Tracking:** proposed follow-on to [MYC-DIR-REVOCATION-BARRIER-001 / #6476](https://github.com/Luminous-Dynamics/mycelix/issues/6476)

## 1. Why a separate oracle is necessary

The component under test must not be the sole judge of its own safety.

A worker can log "success" before a database commit. A broker can confirm publication without a worker running. A coordinator can log "revoked" before every owner has installed its fence. Wall-clock timestamps can be skewed or reordered. Therefore the qualification oracle must reconstruct the actual protected-effect history from independent owner evidence and check that history against a sequential specification.

The linearization point for a PostgreSQL-local effect is the transaction's durable commit, not:
- request receipt;
- worker start or completion log;
- broker delivery;
- publisher confirm;
- consumer acknowledgement;
- an uncommitted row update;
- a client-side success response.

The owner fence's linearization point is its durable commit, followed by the exact read-back required before the owner acknowledgement.

## 2. Do not reinvent generic linearizability checkers

There are established tools and techniques to use as baselines:

- [Porcupine](https://github.com/anishathalye/porcupine) is a fast executable-specification linearizability checker. Its model takes sequential operations and a concurrent history, then determines whether the history can be linearized.
- [Jepsen](https://github.com/jepsen-io/jepsen) combines real systems, fault injection (nemesis), captured histories and consistency checkers. Its linearizable checker asks whether each operation can be placed atomically between invocation and completion.
- [FoundationDB's simulation and testing approach](https://apple.github.io/foundationdb/testing.html) combines deterministic simulation, live performance testing and hardware-based failure testing, with reproducible fault-heavy simulations.

A Rust Porcupine implementation also exists ([porcupine-rs](https://github.com/erwin-kok/porcupine-rs)). It should be evaluated as a qualification-harness dependency or comparator, not automatically added to the dependency-free reference semantics crate.

Mycelix should not claim generic linearizability-checking novelty. The domain-specific value is an independently checked model that includes resource owner epochs/generations, acknowledged revocation barriers, exact route inventory, worker claims, unknown outcomes and durable effect receipts.

## 3. Oracle output is three-valued

Every history evaluation returns one of:

- **VALID:** required source evidence is complete and the reconstructed history satisfies every safety invariant.
- **VIOLATION:** a counterexample exists, such as a stale-generation commit after an owner acknowledgement, duplicate logical effect, incomplete global owner set, route activation before read-back or owner epoch reuse.
- **INCONCLUSIVE:** evidence is missing, contradictory, truncated, unauthenticated or insufficient to order events.

INCONCLUSIVE is never a pass. Missing logs or unavailable owners must not be treated as evidence that no effect occurred.

## 4. Canonical event envelope

Every event should bind:
- schema/version and event type;
- event ID and source identity/build digest;
- realm/resource/action and exact effect-owner identity;
- owner epoch and resource generation, where applicable;
- inventory epoch, manifest digest and route-set digest, where applicable;
- revocation digest and global barrier receipt digest, where applicable;
- logical-operation ID, physical-attempt ID/number and idempotency commitment, where applicable;
- worker ID, claim epoch and claim-proof commitment, where applicable;
- broker ID, destination, message key, broker sequence and delivery count as transport-only fields;
- owner-local commit sequence / transaction identifier / WAL or equivalent durable commit reference;
- durable read-back digest and acknowledgement receipt digest;
- causal parent/event references and canonical payload digest.

All optional domain-specific bindings must have explicit absence semantics. A field that is required for one event kind cannot be silently omitted.

## 5. Order evidence: never use wall clocks as the safety oracle

Wall-clock values are useful for latency distributions, but not sufficient to establish the mutation order.

The oracle should use:
- monotonically ordered owner-local commit sequence numbers or equivalent durable commit positions;
- exact transaction/effect IDs and immutable receipt-ledger rows;
- owner epoch and generation transition records;
- causal parent digests linking fence commit → durable read-back → owner acknowledgement;
- owner-ack-set digest linking every exact owner receipt → global acknowledgement;
- invocation/response intervals to constrain histories, but not override authoritative owner commit order.

For PostgreSQL, collect transaction IDs, commit timestamps only as diagnostic data, the relevant durable WAL position/LSN where available, fence generation read-back, effect ledger row and transaction outcome. The chosen commit reference must be qualified against failover and promotion; a local LSN from a lost asynchronous replica is not durable evidence.

Across heterogeneous owners there is no single shared transaction sequence. The global receipt is instead a causal barrier whose exact owner-ack set binds one independently qualified owner history per owner. Do not invent a total order from unsynchronized clocks.

## 6. Required event families

### Authority and delivery
- `DecisionIssued`: exact Allow receipt, request, authority cut, clock profile and validity interval.
- `CommandPublished`: broker/destination/message key/payload digest and logical operation.
- `DeliveryObserved`: transport identity, delivery count and broker sequence; never effect evidence.
- `WorkerClaimInstalled`: worker ID, claim epoch, validity and proof commitment.
- `WorkerClaimSuperseded`: new epoch and exact previous claim.

### Owner mutation and fencing
- `EffectAttemptStarted`: physical attempt identity and exact command bindings.
- `EffectCommitted`: owner-local commit sequence, effect ID, exact owner position, route and transaction commitment.
- `EffectConfirmedNoEffect`: authoritative owner evidence tied to the exact attempt.
- `EffectOutcomeUnknown`: explicit ambiguity; blocks blind retry.
- `EffectReconciled`: exact attempt ID and independent terminal outcome.
- `OwnerFenceCommitted`: previous and installed owner positions plus commit reference.
- `OwnerFenceReadBack`: exact committed position and durable read-back digest.
- `OwnerFenceAcknowledged`: owner ack receipt binding commit/read-back evidence.
- `OwnerRecoveryStarted` / `OwnerRecoveryCompleted`: restored candidate, high-water, new owner epoch and read-back.

### Route and global barrier
- `RouteStaged`: route identity, owner, enforcement mode and inventory identity.
- `RouteActivated`: exact current owner position and exact manifest/route-set digests.
- `InventoryClosed`: frozen route inventory, proof and topology epoch.
- `GlobalBarrierAcknowledged`: coordinator epoch, exact owner-ack-set digest, inventory identity and durable coordinator commit reference.

## 7. Core history checks

For every owner independently:

1. Owner epoch/generation positions never regress.
2. A mutation must use the exact current owner position at its owner-local commit linearization point.
3. An old-generation mutation may commit before a new fence's durable commit if it acquired the owner serialization primitive first; it may not commit after the new fence commit.
4. An owner acknowledgement must follow the fence commit and durable read-back for the exact same position, revocation and inventory.
5. A restored owner cannot accept writes below the acknowledged high-water mark.
6. An owner epoch cannot be reused across restart/failover/restore cycles.
7. A physical retry cannot begin while the prior attempt remains Unknown.
8. An exact replay of a prior committed logical operation is read-only and returns the original effect receipt.

For routes:

1. Every executable route must be present in a currently verified inventory.
2. A newly staged route cannot activate until its owner has adopted and durably read back the current fence.
3. Route admission must bind exact inventory epoch, manifest digest and route-set digest.
4. CAEP-only, expiry-only and best-effort invalidation paths cannot satisfy strict mutation-fence coverage.
5. After a fence advances, any old route capability must fail at the owner mutation boundary.

For the global barrier:

1. The global acknowledgement follows every exact required owner acknowledgement.
2. The owner-ack set must equal the frozen owner set, not merely match a count or quorum.
3. Every owner acknowledgement binds the same revocation and inventory identity.
4. A topology or route-inventory change invalidates the pending closure or is held from execution until a new route-admission proof exists.
5. No global acknowledgement is accepted if any owner or route is missing, best-effort, unknown or awaiting reconciliation.

## 8. Benchmarking the oracle itself

Measure:
- histories checked per second;
- median/p95/p99/max checking time;
- maximum history size and memory;
- counterexample discovery time;
- proportion of histories returned VALID / VIOLATION / INCONCLUSIVE;
- evidence-ingest throughput and storage;
- false acceptance/false rejection against known safe and mutated histories;
- reproducibility of a failure trace from its evidence bundle.

Use matched input histories for Porcupine, Jepsen-compatible models and the Mycelix domain-specific oracle. Keep generic data-structure linearizability checks separate from the distributed revocation-specific checks.

Generate:
- safe histories with effect-lock-first and fence-lock-first orderings;
- stale post-ack commit mutants;
- early owner/global acknowledgement mutants;
- missing/extra owner-ack mutants;
- duplicate effect and idempotency-substitution histories;
- lost-ack commit/unknown-response histories;
- stale restore and owner-epoch-reuse histories;
- route-activation-before-readback and stale-inventory histories;
- incomplete and contradictory evidence histories.

Performance benchmarks are meaningful only after the oracle has passed known-positive and known-negative conformance cases. A fast incorrect checker is not an improvement.

## 9. Fault-injection regime

Adopt a layered approach similar to established rigorous distributed-database testing:
- deterministic state-space exploration for protocol interleavings;
- real broker and database fault injection for concrete implementation behavior;
- process pause/kill and network partition schedules;
- leader failover and restore from stale snapshots;
- independent owner-state read-back after every injected failure;
- reproducible seeds, raw event traces and exact build/configuration digests.

The initial qualification should use one real PostgreSQL owner and one NATS JetStream adapter. Then add heterogeneous owner instances and the full RabbitMQ/Kafka matrix. Run at least ten independent open-loop performance repetitions only after correctness gates pass.

## 10. Initial source implementation (2026-11-22)

The first independent checker source is now present in the standalone, dependency-free crate:

- `tools/myc-dir-fabric-history-oracle/Cargo.toml`
- `tools/myc-dir-fabric-history-oracle/src/lib.rs`
- `tools/myc-dir-fabric-history-oracle/README.md`

The crate is intentionally separate from the production reference crate. It implements a three-valued audit result and checks:
- unique event IDs, causal parent existence and cycle detection;
- owner-local sequence uniqueness and consistency with the causal DAG;
- owner bootstrap and monotonic epoch/generation fencing;
- effect commit against the owner position current at the owner-local commit sequence;
- owner-scoped idempotency-key uniqueness and operation identity;
- worker-claim freshness at that same commit point;
- effect-attempt, idempotency and logical-operation bindings;
- Unknown outcome reconciliation and exact retry-after-confirmed-no-effect;
- fence commit, durable read-back and owner acknowledgement lineage;
- exact global owner-set equality;
- post-barrier route activation with exact inventory membership and owner read-back.

There are currently 19 authored unit tests. These are **not compiled or executed**. Source inspection and GitHub fetch-back are not compiler evidence.

The current oracle deliberately treats source completeness and authentication as upstream trust boundaries. Its caller must independently authenticate source records, verify signatures/proofs and set `event_set_complete` / `complete_owner_streams` only when that completeness is established. A false completeness assertion can invalidate the result. The checker does not prove inventory completeness, cryptographic authenticity, WAL durability or a distributed consensus protocol.

The first slice is not yet a Porcupine or Jepsen replacement. The next step is to build a separate evidence-ingest adapter and an executable safe/unsafe history corpus, then compare generic operation-history checks with Porcupine/Jepsen-compatible models.

## 11. Evidence boundaries

This note is a research contract plus an initial source-present checker. Porcupine, Jepsen and FoundationDB are comparators and sources of methodology; no claim is made that Mycelix currently matches their implementation depth, fault volume or maturity. No history has been checked by an executed checker and no oracle performance is measured yet.

The production reference crate remains dependency-free. The checker may live in a separate qualification harness so that independent tooling does not become part of the production authorization path.


## 12. Machine-readable conformance preregistration

The companion corpus is [MYC_DIR_HISTORY_ORACLE_V0_1.json](../benchmarks/MYC_DIR_HISTORY_ORACLE_V0_1.json), with its [Draft 2020-12 schema](../benchmarks/MYC_DIR_HISTORY_ORACLE_V0_1.schema.json). It contains 19 known-valid, known-violation and incomplete-evidence scenarios and 12 checker/resource/performance metrics. Its status is `preregistered_not_executed`. The conformance set requires zero false-valid results on known violations, zero false-violation results on known-valid histories, and zero missing-evidence histories marked VALID.

All corpus JSON, schema JSON and source artifacts were fetched back from the branch and structurally checked for parseability, unique IDs, matching schema minima and presence of all three verdict classes. This is source-structure evidence only; it is not a checker execution or a performance measurement.
