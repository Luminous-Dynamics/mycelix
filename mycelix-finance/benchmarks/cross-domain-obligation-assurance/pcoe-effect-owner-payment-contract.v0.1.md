# PCOE Effect-Owner Payment Execution Contract v0.1

**Status:** proposed protocol and qualification contract; not production authorization, Holochain conformance, or proof of exactly-once execution.  
**Date:** 23 November 2026.  
**Composes:** PCOE kernel #6526, SCITT continuity addendum #6553, and existing Mycelix payment ownership issues #4383 and #5750.

## 1. Design correction: make the effect owner authoritative

A coordinator call is not a transaction spanning every system it calls. A transport response is not the effect. A timeout does not reveal whether the effect occurred.

For SAP settlement, Finance owns the balance mutations and therefore must own the stable operation identity, request fingerprint, durable effect record and retry/reconciliation API. Energy is a caller and obligation owner; it must not invent a second identity or decide that the payment occurred from a local boolean.

This profile uses the term **one economic effect per operation identity**, not exactly-once transport. Requests and responses may be delivered more than once or not at all. The safety objective is that repeated delivery of the same operation identity and same semantic request does not create another economic effect, while an altered request under that identity is rejected.

## 2. Current source-level execution gap

The public-main Finance `send_payment` path inspected for this profile performs, within one Payments zome-function invocation:

1. an author-bound sender SAP debit;
2. a beneficiary SAP credit;
3. a synchronous Treasury fee/compost call through another local cell;
4. Payment entry creation with `TransferStatus::Completed`;
5. payment indexes and a Receipt entry;
6. response delivery to the caller.

The request type has no durable operation identity or canonical request digest. The Payment ID is timestamp-derived. The SAP balance `justified_by` field remains unvalidated by `validate_sap_balance`.

**Important Holochain atomicity boundary:** Holochain documents that multiple records written by one zome function to one source chain are committed atomically. The sender debit, beneficiary credit, Payment entry, Receipt and local links are called from the same Finance invocation / source chain; they must not be described as independently durable steps if that invocation aborts before commit. A real-agent test should confirm the expected rollback and chain behavior. The state-model mutant that permits a debit outside the batch is a deliberately unsafe negative control, not a claim that the current same-call path necessarily leaves a partial local debit.

The real cross-cell window is the synchronous Treasury call: it can commit on the Treasury cell's own source chain while Finance's source-chain transaction is still in flight. If a later Finance operation fails, that Treasury effect is not rolled back with the Finance source chain. Conversely, the current code logs Treasury delivery failure and continues to create a `Completed` Payment; the fee has been deducted from the payer but no exact Treasury receipt / durable outbox disposition is bound to the Payment. A timeout at the Energy→Finance boundary can also lose the Finance response after its local commit; without a stable operation identity, retry may create a second payment.

The next architecture therefore preserves same-call Finance atomicity, moves Treasury delivery behind a fee outbox committed with the Finance operation, and adds a durable effect-owner identity / fingerprint for exact retries and reconciliation. This addendum is a design and test contract. It does not change or qualify the live implementation.

## 3. Identity and canonical request binding

A stable operation identity is created by the obligation owner and consumed by the effect owner. The identity is scoped by the effect owner and authenticated payer; it is not a free-form memo.

The canonical semantic fingerprint MUST bind:

- protocol and schema version;
- effect-owner DNA / payment domain;
- source hApp and source DNA / role;
- immutable source obligation reference, including the exact trade or milestone action hash;
- authenticated payer identity and the payer's authorizing agent identity;
- beneficiary identity;
- currency / instrument identifier and integer amount in the instrument's smallest supported unit;
- purpose / payment type;
- fee schedule / calculation policy version and exact fee amount;
- any required mandate, contract and acceptance references.

The fingerprint MUST exclude transport-only or mutable presentation fields such as request timestamp, retry count, HTTP headers other than the idempotency identity, log correlation IDs, user-facing memo text and response-delivery state.

The key and fingerprint are separate:
- **operation identity** answers "which intended economic operation is this?";
- **fingerprint** answers "what exact economic intent was authorized under that identity?"

Same identity + same fingerprint is an exact retry / lookup. Same identity + different fingerprint is a conflict and MUST NOT return the original result as though the altered request had succeeded. Different identities are different economic intents, even if their fingerprints happen to match.

## 4. Durable operation and journal semantics

The authoritative Finance effect should be one author-bound, immutable **journal batch** committed in one Finance source-chain operation. This is not a proposal to reimplement Holochain's existing all-or-nothing commit of same-chain writes; it adds one canonical, fingerprint-bound economic event and makes the full set of local legs explicit. It contains the complete local value movement:

- exact payer debit;
- exact beneficiary credit;
- exact fee allocation (or explicit zero fee);
- stable operation identity and semantic fingerprint;
- source obligation reference and effect-owner author;
- instrument and integer unit;
- deterministic journal sequence / predecessor reference;
- fee outbox instruction, when value must be delivered to a distinct Treasury owner;
- protocol version and event digest / signature envelope.

The batch is the economic event. Balance entries, Payment, Receipt, history indexes and UI projections are deterministic read models derived from it. They MUST NOT be separately authorized sources of monetary truth.

A journal batch with a missing payer debit, beneficiary credit, or fee allocation is invalid. The three local legs must conserve value exactly:

\[
\text{payer debit} = \text{beneficiary credit} + \text{fee allocation}
\]

No floating-point amount is admitted. Fee policy must be deterministic and version-pinned. The fee allocation is an internal Finance liability/claim; actually delivering the fee to Treasury is a separate effect.

**Holochain boundary:** official Holochain guidance states that multiple records written by one zome function to one source chain are committed atomically, but they are not validated together as one atomic transaction. The Treasury call is a different cell and therefore lies outside the Finance source-chain commit. Keep the Treasury effect behind an outbox which is written in the Finance commit and dispatched only after commit. Validation callbacks are deterministic and may depend only on addressable/available prior records; two different agents' source chains do not become one global serializable transaction. Therefore the batch should be authored and committed by the authenticated effect owner in one source-chain call, and the relevant integrity validators must validate the exact batch and its prior-chain relationship. DHT link-count prechecks are not a singleton proof. Real multi-agent races, forks and dependency availability still require Tryorama/Sweettests. See the [Holochain source-chain contract](https://developer.holochain.org/concepts/3_source_chain/) and [validation guidance](https://developer.holochain.org/build/validation/).

## 5. Operation state, value state, and delivery state are separate

Do not encode every aspect as `Payment.status = Completed`.

### 5.1 Operation state

- `Absent`: no operation identity has been claimed.
- `Prepared`: exact identity and fingerprint are durably registered; no value legs exist.
- `Committed`: the complete local journal batch and all projections / outbox records are committed.
- `Rejected`: the effect owner recorded a terminal no-effect rejection.
- `Conflicted`: this request reused an existing identity with a different fingerprint; the original operation is unchanged.
- `Indeterminate`: the caller cannot currently establish the authoritative result; it must query the same identity, not make a new one.

`Indeterminate` is an observer's knowledge state, not a license to forget the operation. If Finance has not resolved whether the batch committed, it must not create another operation for the same intent.

### 5.2 Local value-effect state

- `NotCommitted`
- `LedgerCommitted`
- `ReconciliationRequired`

A Finance `LedgerCommitted` result is an internal ledger fact only. It is not external bank finality, reserve backing, legal discharge, physical delivery, or contract acceptance.

### 5.3 Outbox / Treasury delivery state

- `NotRequired` when fee is zero;
- `Pending` after the journal batch creates a fee liability/outbox item;
- `Indeterminate` when the Treasury call may have committed but the response is unknown;
- `Delivered` only after an exact Treasury-owned idempotent receipt is verified;
- `Rejected` only after a durable no-effect rejection.

The outbox item has its own stable effect identity and semantic fingerprint. Unknown delivery is reconciled against the Treasury effect owner using the original identity. A new fee request identity must not be invented to work around a timeout.

### 5.4 Response delivery

A response can be lost after `Committed`. The durable operation remains committed. An exact retry returns the same operation reference, journal-batch reference and Payment projection reference. Failure to deliver a response does not roll back or repeat the effect.

## 6. Replay and retention rules

1. Exact same-key/same-fingerprint retry returns the original disposition and original effect references.
2. Same key/different fingerprint is rejected as conflict and does not mutate the original operation.
3. A request received while the original claim is being committed must serialize, converge to the same durable claim, or return an explicit in-progress / indeterminate outcome. It must not proceed to a second batch.
4. A delayed request arriving after the caller's timeout is still associated with the original identity.
5. An expired API idempotency-cache window MUST NOT make a prior financial operation identity reusable. Keep a durable operation tombstone or reject stale identities; do not delete the only evidence that an effect may have occurred.
6. Recovery after agent restart, network partition or source-chain race queries the same operation identity and fingerprint before any further mutation.
7. Manual evidence is a separate disposition; it never becomes a Finance-ledger commit or an external rail finality receipt by boolean coercion.

The 21 November 2026 HTTPAPI Idempotency-Key draft -09 is useful guidance for unique keys, changed-payload conflicts, duplicate requests and documenting expiry behavior. It is still an Internet-Draft and does not provide a distributed ledger's uniqueness mechanism. See [draft-ietf-httpapi-idempotency-key-header-09](https://datatracker.ietf.org/doc/html/draft-ietf-httpapi-idempotency-key-header-09).

## 7. Holochain implementation obligations

The Finance implementation must prove, rather than assume, the following:

- the authenticated payer / author is authorized for the source obligation;
- the stable operation identity is unique within its declared payer and effect-owner scope;
- the operation claim and the journal batch cannot be committed twice by concurrent calls on the same author chain;
- a concurrent call that loses a source-chain-top race re-reads the same operation identity and returns / reconciles the winning result;
- alternate agent keys, cloned agents, forked chains, stale DHT state, delayed operations and identity remapping cannot authorize an independent second value effect;
- integrity validation verifies exact prior operation / batch dependencies using deterministic, addressable records rather than mutable collection cardinality;
- the journal batch is complete and conserves value;
- derived balances and Payment/Receipt projections are reproducible and cannot be used to authorize a second effect;
- Treasury fee routing uses a separate idempotent outbox operation with an exact receipt;
- corrections, refunds, reversals and chargebacks append new linked events and never erase the original batch.

A single source chain serializes writes for one author, and Holochain supports atomic commit of multiple records from one zome function to that chain. Those properties are useful local building blocks, not proof of global uniqueness across different agents, DHT visibility, cells, hApps or external rails. The exact system boundary must be explicit.

## 8. Fault-injection and history-checking plan

Use two complementary qualification methods:

**State-space model:** enumerate bounded transitions and check safety invariants. Include a deliberate mutant that permits a debit before the complete batch and prove the invariant detects it.

**Jepsen-style adapter testing:** run concurrent real-agent operations against a pinned Holochain conductor / DNA, record invocation and completion histories, and inject process, network, DHT-availability and response faults. A history checker should compare observed results with the reference operation model. Jepsen's method explicitly records operation histories while a nemesis induces failures and then checks whether the histories are valid; see [Jepsen core documentation](https://jepsen-io.github.io/jepsen/jepsen.core.html).

Required fault cuts include:
- before operation claim;
- after claim but before journal commit;
- while competing requests race the same claim;
- after journal commit but before caller response;
- after response is lost and before retry;
- during balance / Payment / Receipt projection rebuild;
- after fee outbox commit but before Treasury dispatch;
- after Treasury delivery but before its receipt response;
- during Finance or Treasury restart;
- during DHT partition, delayed gossip, stale reads and source-chain-top races;
- after client-side idempotency cache expiry but while durable financial evidence remains;
- during key rotation, identity change or recovery from a fork.

The run must preserve the full invocation/return/fault history, source-chain action references, exact operation fingerprint, journal batches, per-leg counts, balances, fee liability/outbox and retry responses.

## 9. Acceptance properties and proposed targets

All are proposed research targets, not achieved results, regulatory thresholds or a claim that the current implementation meets them.

| Property | Target |
|---|---:|
| Journal batches per operation identity | At most 1 |
| Payer debits per operation identity | At most 1 |
| Beneficiary credits per operation identity | At most 1 |
| Fee liabilities per operation identity | Exactly 1 when fee > 0; otherwise 0 |
| Changed-payload reuse of an existing key | 100% rejected |
| Exact retry agreement with original disposition/reference | 100% |
| Partial local journal batches | 0 |
| Unknown outcome silently promoted to Completed | 0 |
| Value-conservation violations | 0 |
| Unreconciled unknowns silently discarded | 0 |
| Independent history-replay agreement | 100% over frozen histories |

Performance is measured only after every safety gate passes:
- p50 / p95 / p99 operation-claim and commit latency;
- p50 / p95 / p99 retry lookup latency after response loss;
- p50 / p95 / p99 time to reconcile an Indeterminate outcome;
- fee-outbox age and backlog by status;
- manual recovery minutes per 1,000 unknown outcomes;
- storage / DHT growth per committed journal batch;
- cost per 1,000 correctly completed obligations.

Do not compare raw TPS against banks or payment networks until the same workload semantics, finality definitions, fault model and correctness gates are matched.

## 10. Scenario corpus and model status

The companion corpus defines 18 synthetic adversarial cases. It is an executable-test contract, not 18 passed tests.

The companion TLA+ model specifies a finite effect-owner operation, atomic journal batch, exact retry, conflict, response loss, fee-outbox recovery, durable tombstone and an unsafe partial-debit mutant. The safe and mutant TLC configurations are provided.

**Model-checking status:** authored, not TLC-checked in this authoring environment. The TLA+ files must be run with a pinned TLC release and their exact output / state counts retained before claiming a model-check result. The model is bounded and does not prove the live Holochain runtime or external rail behavior.

## 11. Implementation sequence

1. Preserve the current public-main source finding and keep #5750 open.
2. Implement a Finance-owned operation identity, request fingerprint and claim record.
3. Implement the immutable journal batch in the same Finance author-chain call, with balance / Payment / Receipt as deterministic projections.
4. Give Treasury fee delivery its own outbox identity, receipt and reconciliation lifecycle.
5. Prove source-chain race behavior, author identity constraints, stale DHT and fork / alternate-key cases with real multi-agent tests.
6. Add a Jepsen-style history driver / checker for response-loss, restart, partition and concurrency schedules.
7. Integrate Energy as the operation initiator / query client; it must not own Finance idempotency or finality.
8. Only then connect the Finance ledger result to separate external rail, contract acceptance and physical-delivery evidence.

## References

- [IETF draft-ietf-httpapi-idempotency-key-header-09, 21 November 2026](https://datatracker.ietf.org/doc/html/draft-ietf-httpapi-idempotency-key-header-09)
- [AWS Builders' Library: Making retries safe with idempotent APIs](https://aws.amazon.com/builders-library/making-retries-safe-with-idempotent-APIs/)
- [Holochain source chain and atomic commit semantics](https://developer.holochain.org/concepts/3_source_chain/)
- [Holochain deterministic validation and validation dependencies](https://developer.holochain.org/build/validation/)
- [Jepsen core: histories, nemesis, and checkers](https://jepsen-io.github.io/jepsen/jepsen.core.html)
- [TLC explicit-state model checker](https://docs.tlapl.us/using:tlc:start)
