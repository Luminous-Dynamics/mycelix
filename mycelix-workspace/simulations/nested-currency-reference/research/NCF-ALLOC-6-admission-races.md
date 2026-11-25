# NCF-ALLOC-6 — Atomic obligation admission under concurrent retries

**Status:** an independent finite Python interleaving model executed locally; eight tests pass for this model. The Rust/Holochain implementation, durable database, cryptographic identities, actual concurrent processes, and production settlement remain unqualified.

## Research question

NCF-ALLOC-5 adds sequential deduplication by request ID and canonical work digest and checks a per-principal/instrument cap. That is insufficient unless the dedup lookup, cap check, obligation insertion, liability update and receipt persistence form one durable atomic operation (or a separately proved invariant-confluent design).

This campaign tests the time-of-check/time-of-use gap. Two clients each execute four steps in order: read_dedup, read_used, check, commit. There are 70 order-preserving interleavings per case. We compare two deliberately vulnerable commit styles—atomic increments after stale checks, and snapshot overwrite—with a serializable reference oracle that applies check + uniqueness + debit + receipt as one critical section.

## Results

| Scenario | Vulnerable mode | Unsafe schedules / 70 | Cap violations | Duplicate/conflicting accepted requests | Liability counter understatement | Worst overshoot |
|---|---|---:|---:|---:|---:|---:|
| Distinct 60-unit obligations, cap 100 | Atomic increment after stale check | 60 | 60 | 0 | 0 | 20 |
| Distinct 60-unit obligations, cap 100 | Snapshot overwrite after stale check | 60 | 60 | 0 | 60 | 20 |
| Same work, distinct request IDs | Atomic increment after stale check | 60 | 60 | 60 | 0 | 20 |
| Same work, distinct request IDs | Snapshot overwrite after stale check | 60 | 60 | 60 | 60 | 20 |
| Same request ID, conflicting payloads | Atomic increment after stale check | 60 | 0 | 60 | 0 | 0 |
| Same request ID, conflicting payloads | Snapshot overwrite after stale check | 60 | 0 | 60 | 60 | 0 |

The serializable oracle had zero modeled violations in both transaction orders for all three cases. For the 60 + 60 contention example, one 60-unit obligation is accepted and the other is rejected at cap 100. Exact same-work retries return the existing result, while same-ID conflicting payloads are rejected.

These figures are not a claim that a particular database or Mycelix code path has a defect: this is a finite abstract scheduler that demonstrates the class of bug which integration tests must rule out.

## Required paradigm shift

**Treat admission as a single state transition, not a sequence of individually correct helper calls.** The uniqueness key, canonical work digest, principal/instrument cap, accepted obligation, liability counter and durable response/receipt need one atomic serialization point. A process-local lock or in-memory dictionary is not sufficient if multiple conductors/peers can authorize the same principal or obligation.

For a database-backed implementation, test a serializable transaction or equivalent constraint-driven design: unique index on canonical work identity, unique request ID bound to payload digest, atomic cap debit, and response receipt persisted with the transition. Retry the same ID and payload after response loss; changed payload must reject. If the storage substrate cannot provide this globally, explicitly use preallocated non-overlapping rights and prove the invariant-confluence boundary rather than silently relying on eventual validation.

Holochain requires extra care: validation is deterministic and dependencies that cannot be fetched may leave validation indeterminate and be retried, but actions inside an atomic write are not necessarily validated together as a cross-agent atomic transaction. A source-chain write or DHT link must not be treated as a global unique-key constraint without a proof and adversarial multi-agent test.

## Industry comparison and qualification gates

- Interledger's settlement-engine RFC requires idempotency keys, persisted response state, exact-retry semantics and a minimum 24-hour persistence window. NCF must qualify retention and replay semantics for its own risk model instead of assuming a client retry cannot be duplicated.
- BIS Project Agorá's 2026 report and July real-value testing emphasize end-to-end settlement workflow and operational/legal controls. Its reported 17 scenarios and approximately CHF 800,000 of real-value testing are not a throughput baseline for this reference model; the comparison is the discipline of testing workflow failure and settlement semantics, not headline TPS.
- Current BIS Project FuSSE reports 10,000 TPS only for a controlled proof of concept and explicitly says the result is not a performance benchmark or implementation reference. Do not compare it directly to this scheduler.

Hard safety targets for a real implementation:
1. Zero duplicate canonical-work acceptances, zero conflicting idempotency-key acceptance, and zero cap overshoots across at least 100,000 deterministically generated histories and 10,000 fault-injected/reordered histories.
2. Zero lost acknowledged obligations and zero duplicate committed receipts after crash/restart and response loss.
3. The negative-control implementation must reproduce the demonstrated race; the proposed transaction implementation must pass the same histories.
4. Run concurrent multi-agent tests against the actual storage/validation path, with artifact hashes and exact source/toolchain receipts.
5. Only after safety qualification, measure full-path throughput, p50/p95/p99 admission latency, durable-write amplification, idempotency-index/tombstone growth, replay retention, recovery time and capacity denied by policy.

The 1,000 accepted local operations/s and p99 admission below 100 ms remain proposed targets for an explicitly described testbed, not achieved results. End-to-end financial settlement performance should be reported separately from local admission.

## Reproduction

From the research directory:

    python3 -m unittest -v test_ncf_alloc6_admission_races.py
    python3 ncf_alloc6_admission_races.py > ncf-alloc6-admission-races-results.json

The local execution source hash and checked-in source copy hash are recorded separately because those bytes are not identical. No checksum-bound CI receipt is claimed.