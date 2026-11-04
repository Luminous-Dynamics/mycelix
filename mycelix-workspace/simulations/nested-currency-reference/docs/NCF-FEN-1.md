# NCF-FEN-1: fencing and safe reallocation of partitioned risk rights

**Status:** proposed protocol contract; not implemented or qualified.  
**Scope:** nested-community currency clearing, building on NCF-1 and FED-CUR-002.  
**Core rule:** a reservation is a bounded right to create gross exposure, not merely a cached balance. Reallocation revokes an old right and creates a new one; it must not duplicate the same risk budget.

## 1. Problem and safety boundary

The NCF-1 ReservationBook assigns disjoint capacity slices to an agreement, instrument, shard and epoch. It deliberately refuses to reuse unused old-epoch capacity automatically. This can strand liquidity during partitions, but avoids assuming that a timed-out owner is dead.

A lease timeout, local clock, missing heartbeat, or failed RPC is **not** proof that a writer can no longer spend. If owner A is partitioned while the coordinator reallocates A's slice to B, both may still accept locally valid operations. A local test, DHT eventual convergence, or signature alone does not make the overlapping rights safe.

The v1 safety/availability choice is explicit:

- **Reachable old owner:** drain and fence it, reconcile accepted operations, then transfer only the proven remainder.
- **Unreachable old owner:** keep its allocation charged and unavailable for reallocation, unless a separately specified authoritative fencing mechanism can make all future old-epoch operations invalid under a declared fault model.
- **No authoritative control path:** reject or return unknown/unavailable; never optimistically claim that a transfer succeeded.

This protocol does not claim that Holochain supplies a global atomic register or serializable cross-agent transaction. A production adapter must name the control-plane authority and prove that all accepting validators enforce the same fencing chain. Without that substrate, only static escrow quotas are in scope.

## 2. Signed, typed receipts

Digests must use canonical versioned serialization and domain separation. Display strings, ticker symbols, floating-point amounts, wall-clock timestamps, and caller-provided profile objects are not authorities.

An allocation receipt must bind: protocol version and receipt type; stable community IDs and immutable agreement/counterparty identity; canonical instrument ID, profile digest and revision; community-cap and agreement-cap policy digests/revisions; reservation ID, shard/owner ID, positive integer capacity and unit; monotonically increasing epoch/fencing token and control-log sequence; predecessor receipt digest; issuer/signer identities and authorization evidence; and a unique operation ID plus canonical request-payload digest.

A transfer/fencing receipt must additionally bind: old and new owner and epoch; exact allocation being moved; the old epoch's terminal operation watermark or durable sequence cut; reconciled gross outstanding, pending exposure and transferable remainder; old and new allocation receipt digests; the control decision that closes the old epoch before the new one becomes spendable; proof mode and its assumptions; and a unique receipt ID/result digest.

Signatures prove who attested to bytes, not that an economic statement is true.

## 3. State machine

Each allocation follows an append-only lifecycle:

1. **ACTIVE:** owner may create new obligations only within its slice.
2. **DRAIN_REQUESTED:** control plane asks owner to stop accepting obligations for this epoch.
3. **DRAINED:** owner durably records a terminal operation sequence/hash and reports gross outstanding. This report is evidence to reconcile, not permission to reallocate.
4. **RECONCILED:** the designated authority verifies all accepted operations through the terminal cut and computes the remaining rights. Missing dependencies keep the state unresolved.
5. **FENCED:** a durable monotonic control decision makes the old epoch invalid for new obligations. Verifiers reject operations that spend an old epoch past the fence's cut.
6. **TRANSFERRED:** a new allocation receipt commits against the fenced predecessor and updated cap revisions. The new owner may spend only after this state is verifiable.

Transitions are idempotent for the same operation ID and payload. Reuse with any different scope, amount, owner, epoch, predecessor or digest fails closed. State changes compare-and-swap against the expected prior revision/control digest. Aborts leave the previous state or a verifiable committed receipt, never a half-transfer that counts capacity twice.

A timeout may trigger drain or escalation. It must not itself create FENCED or TRANSFERRED.

### Transfer arithmetic

For old capacity C, verified terminal gross outstanding O, and separately proven pending exposure P, transferable capacity is at most T = C - O - P, only if O + P <= C, all values share the same canonical instrument and unit, every accepted operation through the terminal cut is reconciled, and the old epoch is fenced. If completeness is not proven, T is **unknown**, not C minus the observed outstanding.

Debit of the old allocation and credit to the new allocation must be one logical control-plane transition. If storage cannot do this atomically, use a recoverable prepare/commit protocol whose intermediate states reserve capacity against both paths and whose recovery is deterministic. Never free capacity between debit and credit.

## 4. Define what qualifies as a fence

Choose and document exactly one v1 proof mode; do not combine these implicitly:

- **Acknowledged-owner mode:** each owner that can spend confirms durable drain/fence. State the assumption about owners that might continue authoring after acknowledgement, and ensure verifiers reject such writes.
- **Linearizable-authority mode:** one named authority serializes control transitions; every operation verifier checks its monotonic control sequence.
- **Quorum mode:** define membership, quorum intersection, signer authentication, fault threshold, conflict resolution and recovery. A collection of signatures without these rules is not a quorum safety proof.

If offline owners must retain spending rights during a partition, those same rights cannot safely be reclaimed in that partition without stronger coordination or a prior allocation change. This is a genuine safety/availability trade-off.

For Holochain, validation must be deterministic and depend on explicitly addressable receipts. Do not infer the current epoch from a changing collection such as a query over whichever links happen to exist. If a validator cannot retrieve or verify a control dependency, the result stays unresolved/rejected under a documented retry policy; it must not silently accept a stale epoch.

## 5. Required negative controls and invariants

Test at least these cases: stale owner submits after fence; new owner submits before transfer commit; identical transfer replay; same ID with modified amount/owner/instrument/epoch/predecessor/policy digest; missing terminal operations or unresolved DHT dependencies; crash after every transition/write; reordered or duplicated receipts; cap lowered below allocated capacity; instrument or cap-policy revision changed mid-transfer; old owner unreachable; concurrent attempts for the last capacity unit; and unlike instrument units mixed as raw numbers.

Minimum invariants:

- **FEN-I-1:** accepted gross outstanding never exceeds valid, disjoint allocations for the exact instrument.
- **FEN-I-2:** each allocation has exactly one owner and active epoch under the verified control chain.
- **FEN-I-3:** after a valid fence, no new obligation under that epoch is accepted.
- **FEN-I-4:** each transferred risk unit is credited to at most one successor allocation.
- **FEN-I-5:** an unresolved terminal operation set cannot yield a successful transfer.
- **FEN-I-6:** identical retries are idempotent; conflicting replay rejects.
- **FEN-I-7:** cap or asset-policy revision mismatch fails closed.
- **FEN-I-8:** recovery cannot increase spendable risk rights beyond the last verified committed transition.

## 6. Qualification and benchmarks

Correctness is a hard gate; performance is measured only after an independent oracle agrees.

- Start with hand-authored transition fixtures, then enumerate all interleavings for small systems (2–4 owners, 1–2 agreements and small integer budgets).
- Run at least 10,000 reproducible generated schedules after independent review of the state-machine oracle. Vary retries, dropped/duplicated/reordered receipts, crashes between durable writes, delayed old-owner messages, cap revisions, partitions and healing.
- Include honest-crash and adversarial/stale-owner models separately; report which fence modes satisfy each model.
- Race 100 contenders for the final capacity unit; at most one successful transition may consume it.
- Target zero cap violations, zero dual-active epochs, zero post-fence accepted obligations, zero double-transfers and zero conflicting-idempotency acceptance. One violation fails the run; averages cannot hide it.

Report accepted/rejected/unknown transitions by reason; allocated, used and stranded capacity; partition duration; drain-to-transfer time; coordination messages; retry rate; throughput; p50/p95/p99 and maximum latency; crash-recovery outcome; and memory/CPU where comparable. Plot **safety against capital utilization**. More utilization is not an improvement if it increases stale-owner risk.

Compare static-quota baseline, acknowledged-drain mode, and any linearizable/quorum adapter under identical traces. Pin commit, toolchain, machine, corpus digest, seed and binary/output digests in receipts. Single-process model numbers must be reported separately from multi-agent/Holochain results.

### Independent bounded model and negative control

An independent finite TLA+ abstraction is now drafted under [`formal/NCF_FEN_1.tla`](../formal/NCF_FEN_1.tla), with small TLC configurations, a second finite model for two-shard escrow, and four deliberately unsafe mutants. The timeout-reallocation mutant should violate `NoTransferWithoutFence`; the stale-owner mutant should violate `NoOldWriteOutsideActive`; the duplicate-allocation mutant should violate `ReservedCapacityBound` by allocating the full capacity to a second shard; the partition-resize mutant should violate `PartitionedReservationsStayFixed` by mutating the reservation vector during a partition. Both safe models should have no invariant violations at `Capacity = 2` and four operation steps; every mutant check should return a counterexample to its named invariant. **None of these outcomes has been executed or confirmed yet.**

See [the formal model instructions](../formal/README.md). This follows the established Mycelix model-checking pattern in [#5091](https://github.com/Luminous-Dynamics/mycelix/issues/5091): the positive model is insufficient unless deliberately unsafe mutations are detected. TLC is useful for exhaustive exploration of the declared finite abstraction; it is not an unbounded proof or a substitute for a distributed runtime test.

## 7. Industry comparison and research anchors

This is not a novelty claim for escrow, fencing, or idempotency.

- [Interledger Settlement Engines](https://interledger.org/developers/rfcs/settlement-engines/) specifies peer credit limits and idempotency keys for retry-safe settlement. NCF should measure the same retry discipline while adding explicit ownership of partitioned risk rights.
- [Bailis et al., Coordination Avoidance in Database Systems](https://amplab.cs.berkeley.edu/publication/coordination-avoidance-in-database-systems/) formalizes when an invariant can be maintained without coordination. Use it to separate safe local operations within escrow quotas from cross-shard reallocation that needs coordination.
- [Holochain validation guidance](https://developer.holochain.org/concepts/7_validation/) requires deterministic validation and explicit dependencies. The implementation must not infer a globally current epoch from changing collections.
- [BIS Project Agorá report, 27 May 2026](https://www.bis.org/publications/project-agora-shared-programmable-platform-wholesale-cross-border-payments) demonstrates atomic multi-currency wholesale settlement in a coordinated platform. It is useful for comparing all-or-nothing transitions, but the institutional context and control assumptions differ from federated community currencies; raw throughput comparisons would mislead.

## 8. Exit criteria for production consideration

Do not call this production-safe until there is a chosen and threat-modeled fencing authority; canonical receipt schema and signature/key lifecycle; deterministic validators with explicit control-chain dependencies; crash/recovery and partition/race integration tests against a real multi-agent topology; independent oracle and reproducible corpus receipts; explicit unresolved-state, dispute, default and cap-change handling; exact-head compile/test/benchmark evidence; and independent review.

Until then, retain static per-shard escrow quotas and keep potentially live old-epoch capacity charged.
