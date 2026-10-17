# Revision-Fenced Capacity Reservation v1 — benchmark and claim contract

**Status:** serial in-memory reference protocol; targets are proposed, not measured.  
**Tracking issue:** [PROC-RESERVE-CAS-001 #5153](https://github.com/Luminous-Dynamics/mycelix/issues/5153).  
**Allocation prerequisite:** [joint-capacity checker PR #5150](https://github.com/Luminous-Dynamics/mycelix/pull/5150).

## Paradigm shift

A validated allocation is a **proposal**, not a promise that a supplier's capacity remains available. A pool snapshot can become stale, and two plans can compete for the same final units. The reservation authority must compare-and-swap against a current pool revision and fencing token before issuing a receipt.

The implementation in src/reservation_cas.rs is intentionally a serial reference state machine. Its Rust mutable access gives it one in-memory writer; it does not implement transaction isolation, distributed consensus, durable storage, or a network-partition protocol. Production safety must be supplied by a single authoritative transaction domain or a separately qualified coordination protocol.

## v1 behavior implemented

- Register one versioned capacity snapshot for a pool with stable ID, unit, time bucket, quantity ceiling, digest reference, evidence status/origin and validity interval.
- Require reserve commands to present expected pool revision, fencing token and snapshot digest.
- Reject stale revisions/tokens/snapshots, unknown/stale/synthetic-as-live capacity, invalid quantities, expired lease windows, duplicate identities, and capacity overcommit.
- Exact reserve retries with the same idempotency key and payload return the original immutable receipt without consuming capacity twice; conflicting key reuse fails closed.
- Every successful pool mutation increments both revision and fencing token; events record the before/after revision, fence token and stable command/event identity.
- Support explicit Active → Committed → Consumed transitions, Active → Released, and Active → Expired. Committed or consumed capacity cannot be silently released.
- Expiry is explicit and append-only. A lease whose expiry time has passed does not self-release through a failed command; an authorized Expire transition must be recorded using the current revision/fence.
- Failed compare-and-swap attempts append no event or partial reservation.
- The capacity report keeps active/committed quantity (reserved), consumed quantity, and remaining capacity separate.

## Known scope limits

- One authoritative in-memory state owner; no thread/multi-process/network concurrency implementation, persistence, recovery protocol, quorum protocol, or atomic cross-pool transfer.
- No pool snapshot refresh command yet. A refresh must be designed as its own revision-fenced operation that preserves reservations and rejects a new ceiling below already occupied units.
- Evidence statuses, origins, digests, policies and manifest digests are caller-supplied assertions; no signature, hash preimage, supplier truth or physical capacity is verified here.
- The reserve receipt identifies the snapshot and decision, but a future persisted service must commit the state mutation and receipt/event atomically in the same transaction.
- One currency/quantity unit is inherent in each pool; cross-pool unit conversion and multi-pool reservations are not supported.
- This reference does not call a supplier, commit a contract, create a purchase order, mutate external inventory, dispatch goods, or settle payments.

## Correctness gates

| Property | Target |
|---|---:|
| Concurrent contenders in the same frozen pool revision oversubscribe capacity | 0 accepted over-capacity mutations |
| 100 attempts for one unit using the same expected revision/fence | At most 1 succeeds |
| Exact retry of a reservation creates extra capacity use/events | 0 |
| Conflicting idempotency-key or command-ID reuse is accepted | 0 |
| Stale revision, fencing token or snapshot is allowed to mutate state | 0 |
| Expired/unverified/synthetic-as-live pool evidence produces a reservation | 0 |
| Commit at/after expiry succeeds | 0 |
| Release after commit or consumption succeeds | 0 |
| Failed operation partially mutates capacity/state/journal | 0 |
| Quantity conservation: occupied = reserved + consumed and occupied ≤ ceiling | 100% |
| Event sequence gaps, revision regression or fence regression | 0 |
| Independent state-machine oracle disagreement on tractable sequences | 0 |
| Stable hand-authored fixtures | 40 minimum |
| Generated operation schedules after oracle qualification | 10,000 minimum |

The 100-attempt test in this module deterministically models contenders that all present the same initial revision. It is not a benchmark of actual concurrent threads or network calls. A proper store-level stress benchmark must race concurrent workers against the transaction adapter and observe committed receipts, not trust client-side success responses.

## Independent oracle and schedule campaign

Start with a separate test-local reference reducer that interprets accepted/rejected commands and expected pool state without calling ReservationBook. Freeze input commands and expected states for each fixture. Include exact retry and conflict schedules, delayed responses, stale revision/fence, expiry at exactly the boundary, release/commit races, crash points before/after event append, and restarts from durable snapshots once persistence is added.

Only after that oracle qualifies, run at least 10,000 deterministic command schedules with a pinned generator, seed, toolchain, manifest/corpus digest and source revision. Shrink every counterexample into a named golden fixture. Do not call the production state machine twice and treat equality as independent verification.

## Runtime and operational gates

Only after semantic correctness passes, report p50/p95/max transaction latency, throughput, contention/retry rate, stale-fence rejection rate, capacity-exhaustion rate, journal growth and recovery/replay time on pinned hardware. Keep measurements separate for uncontended, hot single-pool, many-pool, high retry, and expiry-heavy workloads. There is no universal millisecond target before a reproducible reference environment is established.

## Research and implementation constraint

PostgreSQL Serializable transactions are designed to prevent serialization anomalies, but applications must retry the whole transaction after serialization failures; a transaction read is not authoritative until commit succeeds. This is an appropriate contract for a future SQL-backed pool authority, not evidence that the current in-memory module is production safe:

- https://www.postgresql.org/docs/16/transaction-iso.html
- https://www.postgresql.org/docs/16/mvcc-serialization-failure-handling.html

Procurement lifecycle events should also remain attributable across planning, award, contract and implementation. OCDS offers a contracting-process identifier to link those stages; GS1 EPCIS specifies later corrective events rather than silently deleting earlier events. Those standards guide adapters and traceability but do not authenticate the truth of a specific capacity assertion:

- https://standard.open-contracting.org/latest/en/primer/how/
- https://www.gs1.org/standards/epcis-and-cbv-implementation-guideline/current-standardd
