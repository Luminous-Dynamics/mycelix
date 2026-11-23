# Hearth authorization and SAP withdrawal hardening v0.1

**Status:** implementation candidate; exact-head focused qualification pending  
**Audited base snapshot:** `e949664ceb2f9ddde26704c70e23a19380f33dde`  
**Tracking:** [#6560 — P0 SAP withdrawal](https://github.com/Luminous-Dynamics/mycelix/issues/6560), [#6565 — P0 TEND membership](https://github.com/Luminous-Dynamics/mycelix/issues/6565)

## 1. Source findings

At the audited main snapshot:

- Payments `withdraw_from_hearth_pool` debited the pool and incremented its lifetime withdrawal total, then credited the member only when a personal `SapBalance` happened to exist. Absence silently skipped the credit while the function returned success.
- Payments and TEND membership helpers rejected an explicit `false`, but accepted malformed response data, unexpected cross-zome response variants, and call errors as a “standalone” fallback.
- Hearth contribution arithmetic used unchecked additions for the pool balance and lifetime contribution total.
- The old withdrawal path could create an empty pool through a getter/helper, rather than treating absence as an inability to establish a withdrawable asset.

These are source-level observations against the exact snapshot above. They do not assert that a particular deployed DNA is reachable or exploitable.

## 2. Implemented containment

### SAP withdrawal

The updated Payments coordinator:

1. Requires a positive amount and an explicit, decoded Hearth membership result.
2. Resolves the caller's personal balance before touching the pool; missing or mismatched DID and a record not authored by the caller's agent key reject.
3. Resolves an already-existing pool; it does not create one as a withdrawal side effect.
4. Rejects zero or multiple locally observed root links, malformed/missing entries, and a pool whose embedded Hearth DID differs from the requested DID.
5. Applies pending pool demurrage in memory.
6. Computes the next pool balance, lifetime withdrawal total and member balance with checked arithmetic before writing.
7. Updates the pool and personal balance in one Payments zome invocation.

The contribution path now uses checked addition for the pool balance and lifetime contributed total.

### SAP and TEND membership

Both membership helpers now authorize only a successfully decoded `true`. A decoded `false`, decode error, unexpected response variant, and cross-zome call error reject the operation. There is no implicit “standalone mode” permission when authority evidence is unavailable.

### Local transaction boundary

The pool and personal SAP writes are in the same Payments cell and one zome function call. Holochain documents these writes as atomic and transactional: they commit together or fail together. Concurrent source-chain activity may still cause a chain-head-moved rollback. This guarantee is local to this one cell and does not create an atomic transaction across cells or independent owners.

## 3. Focused unit checks added

Payments pure guard tests cover:
- exact debit/credit conservation;
- zero amount;
- insufficient pool balance;
- lifetime withdrawal overflow;
- personal balance overflow;
- explicit membership true versus false/unavailable.

TEND pure guard tests cover explicit true versus false, malformed/unavailable evidence.

The mirrored public source copies are kept byte-identical and are checked by the focused workflow. The 20 cases are also stored in `benchmarks/sap/hearth-auth-withdrawal-scenarios.v0.1.json` with a companion schema; the workflow validates JSON syntax, required case shape, unique IDs and mirror parity. These structural checks do not replace JSON-Schema validator coverage, multi-agent Sweettests or HDI validation tests.

## 4. Explicit claim ceiling / remaining work

This change is a **containment fix**, not full closure of SAP economic-cause integrity.

Still required:

- A typed immutable cause for every monetary successor, cryptographically/structurally bound to exact predecessor, owner, amount, currency, economic operation and effect profile.
- Integrity-level binding of `SapBalance` create/update to the actual Holochain `Action.author` across every producer. This PR checks `Record.action().author()` for the withdrawal's personal balance only; it does not establish the global invariant.
- Completeness-qualified DHT/fork resolution. One visible link is not proof that no unseen concurrent root or fork exists; this helper rejects multiple locally observed roots but cannot prove global absence.
- Durable operation identity, canonical payload fingerprint, same-key/same-payload reconciliation and same-key/changed-payload rejection across cache expiry and agent restarts.
- A consume-once durable outbox for any separate-cell/remote effect; retries must reconcile against the original operation rather than mint a new effect.
- Real Holochain Sweettests for absent member balance, membership bridge unavailable, malformed response, ambiguous pool roots, fork/head races, response loss and rollback after each write.
- Review whether `contribute_to_hearth_pool` should be rebuilt with the same typed transfer/effect model. Its same-call writes are atomic, but it still lacks a typed monetary cause and full fork/operation identity.

Unknown, stale or incomplete observations must produce an explicit indeterminate/reconciliation state, never be interpreted as absence, permission or settlement.

## 5. Adversarial qualification corpus

| Case | Fault / input | Required result |
|---|---|---|
| HSA-001 | Explicit membership `true` | Continue to further checks |
| HSA-002 | Explicit membership `false` | Reject; no write |
| HSA-003 | Hearth bridge unavailable | Reject; no write |
| HSA-004 | Malformed membership response | Reject; no write |
| HSA-005 | Unexpected response variant | Reject; no write |
| HSA-006 | Personal SAP balance missing | Reject before pool write |
| HSA-007 | Personal balance owner mismatch | Reject before pool write |
| HSA-008 | No existing pool root visible | Reject; do not create a pool |
| HSA-009 | Multiple pool roots observed | Reject as ambiguous |
| HSA-010 | Embedded Hearth DID mismatch | Reject |
| HSA-011 | Pool balance below withdrawal amount | Reject |
| HSA-012 | Personal balance addition overflow | Reject before writes |
| HSA-013 | Lifetime withdrawal total overflow | Reject before writes |
| HSA-014 | Zero withdrawal | Reject |
| HSA-015 | Concurrent source-chain head move | Atomic rollback; caller must reconcile/retry same operation |
| HSA-016 | Failure after first staged write | Both pool/member writes roll back |
| HSA-017 | TEND provider is not a member | Reject before exchange mutation |
| HSA-018 | TEND receiver is not a member | Reject before exchange mutation |
| HSA-019 | TEND membership evidence unavailable | Reject before exchange mutation |
| HSA-020 | Personal balance DID matches caller, but its signed record was authored by another agent | Reject before pool read/write |

The current pure unit tests cover arithmetic and the shared membership decision function. The caller-authorship predicate is present in the coordinator preflight, but it still needs a real multi-agent author-spoof Sweettest. The remaining cases are required next integration/host-fault qualification, not claimed as already executed.

## 6. Industry comparison and research direction

| Reference pattern | Established behavior | Mycelix opportunity |
|---|---|---|
| Adyen idempotency | Keys are company-account scoped and retained for 7–14 days; regional endpoints do not share duplicate detection. The service documents transient in-progress conflicts and unavailable idempotency storage. | Bind a durable operation ID to the owner-authored effect record, not only a time-bounded request cache. |
| IETF Idempotency-Key Internet-Draft -09 (21 Nov 2026) | Active Internet-Draft, not an RFC. Describes key uniqueness, optional payload fingerprinting, mismatch rejection, duplicate handling and published expiry policy. | Extend from HTTP retry semantics to owner-authored economic event identity and causal reconciliation. |
| AWS transactional outbox | Local state plus outbox are committed together; delivery may be at least once, so consumers must be idempotent. | Keep the producer's outbox record with the exact source-chain effect and make the receiving owner return a durable receipt keyed by that operation. |
| Holochain source-chain transactions | All writes in one zome function call are atomic within one cell; concurrent calls can cause chain-head-moved rollback. | Preserve this native local atomicity, then model cross-cell and cross-owner effects as explicit protocols—not as one imaginary distributed transaction. |

## 7. Qualification targets (targets, not measured results)

Safety gates precede throughput comparisons:

- 100 concurrent attempts with one operation ID and one payload: at most one owner debit / effect.
- 1,000 attempts reusing that operation ID with changed payloads: 100% conflict rejection and zero new effects.
- 10,000 duplicate outbox deliveries: at most one recipient/treasury effect per operation ID; every duplicate reconciles to the same receipt.
- Inject response loss before commit, after local commit, after remote commit and before acknowledgement: zero false `Settled` or `Delivered` results.
- Run at least 100,000 generated schedules across retries, concurrency, stale DHT observations, forks and service outages: zero unauthorized credits, zero negative balances, zero uncaused balance successors and zero silent loss of a debit.
- Every indeterminate observation remains non-spendable / non-settled until reconciliation.

Only after safety gates pass, compare p50/p95/p99 latency, sustainable throughput, CPU, storage and DHT growth, outbox age, recovery time and total cost. Match the same effect semantics, key scope, retention horizon, retry load and failure injection against a ledger transaction baseline and an outbox-based service baseline. A candidate target is at least 20% better p95 cost or sustainable throughput at equal p99 latency/resource budgets; this is a research target, not a current performance claim.

## 8. Paradigm shift

The proposed primitive is an **effect-caused balance transition**:

`Owner-authored intent → authority check → predecessor-bound typed cause → atomic local balance transition → independently owned remote effect/claim → durable receipt → reconciliation`

The balance is not merely a mutable number with an optional pointer. Each delta becomes a typed, reviewable causal event. The operation identity survives retries and cache expiry; the operation's effect owner authors the evidence for the effect they control. The state machine distinguishes `CommittedLocally`, `EffectPending`, `EffectObserved`, `Reconciled`, `Rejected` and `Indeterminate`. In particular, no timeout, missing response or incomplete DHT query promotes an operation to success.

This direction combines the strongest native Holochain local transaction property with the durable-retry lessons of payment APIs and transactional outboxes, while preserving the boundary between sender authority and recipient authority.
