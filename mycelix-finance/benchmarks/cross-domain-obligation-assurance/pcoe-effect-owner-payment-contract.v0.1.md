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

## 4. Two-sided transfer protocol and owner-authored effects

Do not model a payment as one atomic update to two independently owned balances. Holochain atomicity is per source-chain transaction; it does not create a transaction spanning the payer's and beneficiary's chains.

The reference protocol is a **two-sided, identity-bound transfer claim**:

### 4.1 Sender-owned commit

The payer's authenticated source chain commits one immutable transfer intent and its local balance successor in the same zome-function transaction. The transaction binds:

- stable operation identity and canonical semantic fingerprint;
- exact payer identity / author and source obligation reference;
- exact beneficiary identity;
- instrument and integer principal amount;
- version-pinned fee policy and exact fee allocation;
- exact payer predecessor ActionHash and successor ActionHash;
- protocol version and operation digest;
- fee liability and durable Treasury outbox item, if the fee is nonzero.

The sender intent is not a DHT lookup hint. It is the immutable economic command and evidence that the payer authorized the exact transfer. The payer debit and intent must commit together or roll back together. A retry with the same identity and fingerprint returns the original intent; a changed fingerprint is a conflict.

### 4.2 Recipient-owned claim

The beneficiary does not have its balance mutated by a third-party payer or arbitrary observer. The beneficiary claims the exact transfer intent by authoring its own balance successor on its own source chain. The recipient claim must bind:

- exact transfer-intent ActionHash and semantic fingerprint;
- authenticated beneficiary / source-chain author;
- exact recipient predecessor ActionHash;
- exact positive principal amount and instrument;
- exact recipient successor ActionHash and resulting balance;
- replay ancestry and any observed competing successor set.

The recipient claim and recipient balance credit commit atomically on the beneficiary's chain. Exact replay returns the same claim reference and cannot create another credit. A missing recipient claim means the transfer remains `Debited / ClaimPending`, not `Settled`.

### 4.3 Reconciliation and conservation

The operation is fully settled at the internal principal-transfer boundary only when the exact sender intent / debit and the beneficiary-authored claim / credit have both been observed and independently bound to the same operation identity.

For exact integer amounts, the economic conservation equation across the two owners and the fee owner is:

[
	ext{payer debit}
=
	ext{beneficiary credit}
+
	ext{outstanding recipient claim}
+
	ext{Treasury fee delivered}
+
	ext{outstanding fee liability}
]

The sender's local commit creates the recipient claim liability and, where applicable, fee liability. The recipient claim consumes the recipient liability and credits the beneficiary. Treasury delivery consumes the fee liability. Do not create or destroy value by treating a pending claim as a completed credit or an undelivered fee as though Treasury received it.

No floating-point amount is admitted. The exact fee calculation is integer-only and version-pinned. Refunds, reversals, corrections and chargebacks append new linked events; they never delete the original intent or claim.

### 4.4 Treasury outbox is a third effect owner

The Finance sender transaction commits the fee liability and durable outbox item. Dispatch occurs only after commit. Treasury consumes the stable outbox identity idempotently and returns an exact receipt. Treasury's source chain is independent; Finance cannot roll back a Treasury commit.

The Finance `post_commit` callback is only a wake-up hint. A restart-safe scanner reconstructs pending outbox work from committed records. Delivery may be at least once; the Treasury consumer must be idempotent.

### 4.5 What this does not claim

There is no atomic transaction spanning sender, beneficiary and Treasury source chains. The protocol is a sequence of separately authored, identity-bound effects with explicit pending / indeterminate / conflict states. Cross-owner uniqueness, cloned-agent identity and fork resolution require separate authority proofs and adversarial multi-agent tests. Never infer global serializability from a local source-chain commit or DHT link count.

This direction composes with the existing [AC-099 owner-authenticated transfer design / PR #4178](https://github.com/Luminous-Dynamics/mycelix/pull/4178), the [predecessor-bound balance-effect work / PR #657](https://github.com/Luminous-Dynamics/mycelix/pull/657), and [FIN-SAFE-012 / #654](https://github.com/Luminous-Dynamics/mycelix/issues/654). These are open, stale-branch design / implementation workstreams, not current-main guarantees. PCOE adds a cross-domain fault / history qualification profile; it must not replace their owner-authored balance semantics.

## 5. Operation, owner-leg and delivery states are separate

Do not encode every aspect as `Payment.status = Completed`.

### 5.1 Operation state

- `Absent`: no Finance operation record exists for the supplied identity.
- `Prepared`: the exact identity and fingerprint have been accepted for evaluation; no sender value effect exists.
- `Debited`: the payer's intent and debit committed together; the beneficiary claim may still be outstanding.
- `Settled`: the beneficiary-authored claim and exact credit are observed and no competing-successor conflict exists.
- `Rejected`: a terminal no-effect rejection was durably recorded.
- `Conflicted`: changed-payload reuse or competing valid recipient successors prevent authoritative progression.
- `Indeterminate`: the caller cannot currently establish the authoritative result; it must query / retry the original identity, never mint a replacement.

`Debited` is a valid durable intermediate state, not a successful recipient settlement. `Settled` means the internal principal-transfer legs reconcile. It does not prove external bank finality, reserve backing, legal discharge, physical delivery, or contract acceptance.

### 5.2 Owner-leg state

**Sender leg**
- `NotCommitted`
- `Debited`: immutable transfer intent and exact payer balance successor committed on the payer's chain.

**Recipient leg**
- `Unclaimed`
- `Claimed`: recipient-authored claim and exact balance successor committed.
- `Indeterminate`: recipient claim may have committed but the caller has not established its ActionHash / disposition.
- `ConflictFrozen`: competing owner-authored successors exist for one predecessor; the balance is non-spendable until an explicit resolution theorem.

A response loss does not undo a claim. Query the same operation identity and exact predecessor / successor evidence. Never add competing successors together or pick whichever DHT link is returned last.

### 5.3 Outbox / Treasury delivery state

- `NotRequired` when fee is zero;
- `Pending` after the sender commit creates a fee liability / outbox item;
- `Indeterminate` when Treasury may have committed but its response is unknown;
- `Delivered` only after an exact Treasury-owned idempotent receipt is verified;
- `Rejected` only after a durable no-effect rejection.

The outbox item has its own stable effect identity and semantic fingerprint. The identity is reused for every retry. Unknown delivery is reconciled against Treasury using the original identity; a new fee request identity must not be invented to work around a timeout.

### 5.4 Response delivery and recovery

A sender response can be lost after its intent / debit commit. An exact retry returns the original sender-intent and debit references. The recipient claim may still be Unclaimed; do not fabricate a beneficiary credit or report `Settled`.

A recipient claim response can also be lost after the beneficiary chain commits. Re-query the same intent and recipient operation; return the original claim or remain Indeterminate. Do not repeat the credit.

This is the transactional-outbox pattern for Treasury: the sender transaction and outbox record commit together; only after commit does a dispatcher deliver the message. Delivery may be at least once, so Treasury must consume the stable outbox identity idempotently. Holochain's `post_commit` callback is a wake-up hint, not the durable queue. A restart-safe scanner reconstructs Pending / Indeterminate work from committed records. See [AWS Prescriptive Guidance: Transactional outbox](https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/transactional-outbox.html) and [Holochain zome-function commit / post-commit lifecycle](https://developer.holochain.org/build/zome-functions/).

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

The integration must prove, rather than assume, the following:

- the authenticated payer / author is authorized for the immutable source obligation;
- sender transfer intent, payer debit, fee liability and Treasury outbox commit or roll back together on the payer's source chain;
- the sender-side stable operation identity and fingerprint cannot create a second debit under exact retry or source-chain-top races;
- the beneficiary, not the payer or a third-party observer, authors the recipient balance successor;
- the recipient claim is bound to the exact sender intent, exact recipient predecessor and exact principal amount;
- a recipient's same-predecessor competing valid successors produce `BalanceConflict` / `ConflictFrozen`, not last-write-wins or summed balances;
- a losing `chain head moved` call refreshes authoritative state and looks up / retries the **same** operation identity;
- exact record dependencies are loaded by ActionHash and typed entry identity; mutable `get_links()` cardinality is never a uniqueness theorem;
- payment and receipt projections reference both owner-authored legs and are reproducible without mutating balances;
- Treasury fee dispatch is outside the sender's source-chain transaction, uses the durable outbox, and has its own idempotent receipt;
- corrections, refunds, reversals and chargebacks append new linked events and never erase the original intent / claim.

A single source chain serializes writes for one author and commits multiple writes from one zome function atomically. Those properties do not create a transaction spanning payer, beneficiary and Treasury. Alternate agent keys, cloned agents, DHT partitions and sibling chains require explicit authority / fork handling. The PCOE model is a bounded reference; real-agent Tryorama / Sweettests are still required.

## 8. Fault-injection and history-checking plan

Use two complementary qualification methods:

**State-space model:** enumerate bounded operation, sender-leg, recipient-leg, Treasury-outbox, response-loss, chain-head-race and fork-conflict transitions. Include a deliberate mutant that separates sender debit from its immutable transfer intent and prove the sender intent/debit invariant detects it.

**Jepsen-style adapter testing:** run concurrent real-agent operations against a pinned Holochain conductor / DNA, record invocation and completion histories, and inject process, network, DHT-availability and response faults. A history checker should compare observed results with the reference operation model. Jepsen's method records operation histories while a nemesis induces failures and then checks whether the histories are valid; see [Jepsen core documentation](https://jepsen-io.github.io/jepsen/jepsen.core.html).

Required fault cuts include:
- before sender operation claim;
- during sender intent + payer debit atomic commit;
- after sender commit but before sender response;
- after sender commit while recipient claim remains absent;
- during recipient claim / recipient-owned balance successor commit;
- after recipient claim commits but before its response / ActionHash is observed;
- competing recipient successor / source-chain fork and conflict freeze;
- after fee outbox commit but before Treasury dispatch;
- after Treasury delivery but before its receipt response;
- loss of post_commit wake-up and restart-safe outbox scan;
- Finance, beneficiary or Treasury restart;
- DHT partition, delayed gossip, stale reads and source-chain-top races;
- client idempotency cache expiry while durable sender / recipient evidence remains;
- alternate agent keys, key rotation, identity remapping and fork recovery.

The run must preserve the full invocation / return / fault history, sender and recipient ActionHashes, exact operation fingerprint, sender debit, recipient claim / credit, unresolved claim and fee liabilities, Treasury outbox / receipt, balance predecessor / successor references and every retry response. A missing remote record in a partition is not evidence that the record does not exist.

## 9. Acceptance properties and proposed targets

All are proposed research targets, not achieved results, regulatory thresholds or claims that current main meets them.

| Property | Target |
|---|---:|
| Sender intents per operation identity | At most 1 |
| Payer debits per operation identity | At most 1 |
| Recipient claims / credits per operation identity | At most 1 |
| Settled operations lacking an observed recipient claim | 0 |
| Conflicted recipient successors treated as spendable | 0 |
| Fee liabilities per nonzero-fee operation | Exactly 1 |
| Treasury deliveries per outbox identity | At most 1 |
| Changed-payload reuse of an existing key | 100% rejected |
| Exact retry agreement with original disposition / references | 100% |
| Partial sender intent/debit commits | 0 |
| Unknown outcome silently promoted to Settled / Delivered | 0 |
| Value-conservation violations | 0 |
| Unreconciled unknowns silently discarded | 0 |
| Independent history-replay agreement | 100% over frozen histories |

Performance is measured only after every safety gate passes:
- p50 / p95 / p99 sender-intent and debit commit latency;
- p50 / p95 / p99 recipient-claim latency and reconciliation latency;
- p50 / p95 / p99 Treasury outbox delivery / reconciliation latency;
- fee-outbox age and backlog by status;
- manual recovery minutes per 1,000 unknown outcomes;
- storage / DHT growth per transfer intent and recipient claim;
- cost per 1,000 correctly completed obligations.

Do not compare raw TPS against banks or payment networks until the same owner-leg semantics, finality boundary, fault model and correctness gates are matched.

## 9.1 Industry baseline and how to exceed it

The comparison is about semantics, not raw transaction-per-second marketing:

| System / pattern | Publicly documented behavior | What this contract should match | Where Mycelix should go further |
|---|---|---|---|
| Stripe API v1 | Saves the first endpoint status/body after execution begins; returns the saved response for a same-key retry, including 500 responses; compares retry parameters; keys may be pruned after at least 24 hours. API v2 documents different retry behavior for failed/partially failed work. | Stable key, original-result replay, parameter/fingerprint mismatch rejection, explicit in-progress/concurrent behavior. | Durable obligation identity and audit evidence outlive the request cache; reconcile effects after local cell, process and cross-hApp recovery. Never use cache expiry as permission to execute again. |
| Adyen API | Up to 64-character key; company-account scope; 7–14 day validity; regional endpoints do not share duplicate checks; concurrent requests may return transient / already-in-progress responses. | Same-key retry, explicit concurrency response, backoff and status query / webhook recovery. | Bind operation identity to the payer chain / effect owner and obligation fingerprint; do not assume multi-region or alternate-agent uniqueness without an authority proof. |
| AWS transactional outbox | Commit business state and outbox message together; dispatcher may deliver at least once; consumer must be idempotent; ordering and transaction rollback need explicit handling. | Durable outbox and idempotent Treasury consumer. | Source-chain-native evidence, operation-scoped receipt verification and an independently reproducible replay of the outbox-to-effect history. |
| Holochain source chain | Writes in one zome-function call are atomic on that source chain; concurrent source-chain writes can lose with a chain-head-moved error. | Roll back failed same-chain calls; refresh after chain-head movement. | Turn the concurrency loss into an explicit same-identity lookup / retry protocol, not a fresh payment request. Cross-cell effects remain separate transactions. |

These are documented vendor / platform behaviors, not a uniform benchmark run. Stripe API v1 and v2 are not interchangeable. Adyen's stated regional limitation is particularly important: a key checked within one regional store is not a global cross-region uniqueness theorem. Mycelix must likewise scope its claim to the exact DNA, payer / author chain, effect owner and failure model it has actually proven.

### Proposed qualification tiers

**Tier A — semantic safety (hard release gates, no compensating score):**
- 100 concurrent exact-key submissions by one authenticated payer: at most one sender intent/debit; exact retries resolve to the same sender ActionHashes; beneficiary credit remains absent until the beneficiary authors a claim.
- 1,000 altered-payload attempts against existing operation identities: 100% conflict; zero new value effects.
- 10,000 outbox redeliveries / duplicate wake-ups: at most one Treasury effect per outbox identity.
- Every fault cut in the frozen corpus: zero split sender intent/debit commits, zero unauthorized recipient credits, zero silently discarded Indeterminate outcomes, zero false Settled / external-finality promotions.
- Every source-chain-top race: losing call creates no second effect and the subsequent same-identity lookup resolves the winner or remains Indeterminate.

**Tier B — recovery quality:**
- 100% of committed-but-response-lost sender operations resolve to the original intent/debit reference after process restart.
- 100% of lost post_commit wake-ups are recovered by scanning the durable outbox.
- 100% of recipient-claim and Treasury delivery-success / response-loss cases resolve to their original owner-authored references.
- Report p50 / p95 / p99 recovery latency, the number of DHT / source-chain reads, and operator intervention minutes per 1,000 ambiguous outcomes.

**Tier C — efficiency and competitiveness (only after Tier A passes):**
- On three frozen, representative workloads, target at least 20% lower p95 compute-and-storage cost per correctly completed obligation than the matched baseline, or 20% greater sustainable throughput at the same p99 latency and resource budget.
- Report p50 / p95 / p99 latency, sustainable throughput, CPU, memory, storage / DHT growth, retries, outbox age, and the percentage of operations requiring human reconciliation.
- Compare providers only under matched semantics: exact same-key retry, changed-payload conflict, concurrent duplicate, response loss, retention / expiry, process restart and region / cell failover. Do not compare Mycelix raw TPS with a provider's API benchmark if the finality boundary, retry contract or failure model differs.

All numerical targets above are preregistered research goals, not measured results or current SOTA claims. Any safety failure invalidates the performance comparison.

## 10. Scenario corpus and model status

The companion corpus defines **22 synthetic adversarial cases**. These are executable-test contracts, not 22 passed tests.

The companion TLA+ model represents the payer's sender-intent/debit commit, the beneficiary's separately authored claim/credit, Treasury's independent fee effect, sender and recipient response loss, operation-identity replay, source-chain-top movement, durable outbox scanning and recipient fork conflict freeze. The unsafe mutant splits sender intent from debit or duplicates the debit. Safe and mutant TLC configurations are provided.

**Model-checking status:** the earlier single-batch model's TLC results are superseded by this two-owner rewrite. This revision is authored and statically frame-audited but has not yet passed TLC. The exact-head workflow must complete with no model warning, no safe invariant violation, and the expected mutant counterexample before this revision can claim a model-check result. The model is bounded and does not prove the live Holochain runtime, alternate-agent uniqueness, real fork-resolution protocol, Treasury adapter or external rail behavior.

## 11. Implementation sequence

1. Reconcile the existing [AC-099 owner-authenticated transfer protocol / PR #4178](https://github.com/Luminous-Dynamics/mycelix/pull/4178) with current main and the open #6505/#5750/#4383 obligations; do not merge a stale branch without a fresh source audit.
2. Qualify and integrate the predecessor-bound balance transition / fork semantics from [FIN-SAFE-012 / PR #657](https://github.com/Luminous-Dynamics/mycelix/pull/657) and [#654](https://github.com/Luminous-Dynamics/mycelix/issues/654). A competing same-predecessor beneficiary successor must be frozen, not last-write-wins.
3. Bind the sender transfer intent to a stable operation identity and canonical fingerprint. Commit intent + payer debit + fee liability / durable outbox on the payer's own chain in one atomic zome-function call.
4. Implement beneficiary-authored claim + credit on the beneficiary's own chain, bound to the exact intent and recipient predecessor. Do not promote Debited to Settled until the exact claim is observed and fork-free.
5. Give Treasury fee delivery its own outbox identity, exact receipt and reconciliation lifecycle. Dispatch only after the sender source-chain commit; treat post_commit as a wake-up hint.
6. Add real-agent Tryorama / Sweettests for same-key races, chain-head movement, sender rollback, recipient claim response loss, duplicate claim, alternate-key attempts, recipient sibling successors, Treasury delivery-success / response-loss and restart / DHT partition.
7. Add a Jepsen-style history driver / checker for sender / recipient / Treasury operations, preserving exact ActionHashes and independent owner-state observations.
8. Connect internal recipient settlement to separately qualified external rail, reserve/custody, legal acceptance and physical-delivery evidence. None is implied by the internal SAP transfer.

## References
- [Stripe API v1 and v2 — Idempotent requests](https://docs.stripe.com/api/idempotent_requests)
- [Adyen — API idempotency, 7–14 day scope and regional behavior](https://docs.adyen.com/development-resources/api-idempotency)
- [AWS Prescriptive Guidance — Transactional outbox pattern](https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/transactional-outbox.html)
- [Holochain — Source-chain atomic commits and chain-head movement](https://developer.holochain.org/concepts/3_source_chain/)


- [AWS Prescriptive Guidance — Transactional outbox pattern](https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/transactional-outbox.html)
- [Holochain — Zome Functions and post_commit lifecycle](https://developer.holochain.org/build/zome-functions/)

- [IETF draft-ietf-httpapi-idempotency-key-header-09, 21 November 2026](https://datatracker.ietf.org/doc/html/draft-ietf-httpapi-idempotency-key-header-09)
- [AWS Builders' Library: Making retries safe with idempotent APIs](https://aws.amazon.com/builders-library/making-retries-safe-with-idempotent-APIs/)
- [Holochain source chain and atomic commit semantics](https://developer.holochain.org/concepts/3_source_chain/)
- [Holochain deterministic validation and validation dependencies](https://developer.holochain.org/build/validation/)
- [Jepsen core: histories, nemesis, and checkers](https://jepsen-io.github.io/jepsen/jepsen.core.html)
- [TLC explicit-state model checker](https://docs.tlapl.us/using:tlc:start)
