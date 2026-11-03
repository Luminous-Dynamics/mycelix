# Bank Migration, Reconciliation and Reversible Cutover Contract v1

**Status:** proposed reference contract; synthetic qualification only  
**Owner:** Mycelix Finance  
**Parent work:** [BANK-MIGRATION-001 / #5715](https://github.com/Luminous-Dynamics/mycelix/issues/5715)  
**Related:** #5302, #4957, #3343, #4478, #5292  
**Version:** 1.0.0

## 1. Purpose and claim boundary

Define a portable, reversible migration contract for an institution moving a bounded financial workflow from an incumbent core/rail integration to a Mycelix-connected workflow.

Migration is not a database copy. It is a controlled transition between independently authoritative systems whose records may differ in representation, timing, status vocabulary, correction semantics and legal scope.

This v1 is an **effects-disabled synthetic reference contract**. It does not connect to a bank, ingest customer records, initiate payments, alter accounts, authorize a cutover, establish regulatory compliance or prove production migration readiness.

## 2. Governing distinctions

The implementation MUST preserve these distinctions:

```
source bytes != parsed source record
parsed source record != canonical financial event
translated event != booked ledger entry
payment message != payment instruction
payment instruction != provider acceptance
provider acceptance != clearing
clearing != settlement
settlement != final settlement
final settlement != legal discharge
reconciliation PASS != cutover authorization
cutover authorization != external-effect authorization
rollback request != safe rollback
```

No adapter, model, dashboard or import receipt may promote one stage into a stronger stage by inference.

## 3. Migration state machine

A migration run has one explicit state and immutable transition history.

Allowed high-level states:

- `Draft`
- `Inventoried`
- `Captured`
- `Translated`
- `Shadowing`
- `ReconciliationReady`
- `Reconciled`
- `CutoverProposed`
- `CutoverAuthorized`
- `CanaryActive`
- `Paused`
- `RollbackProposed`
- `RolledBack`
- `Completed`
- `Indeterminate`

A transition binds the prior state digest, next state, actor/role, policy profile, exact run identity, evidence refs and timestamp. State transitions are append-only; corrections create new evidence and never rewrite a prior transition.

`CutoverAuthorized` requires an institution-controlled approval record outside the benchmark candidate. A synthetic verifier may check that the required approval evidence is present and exact; it cannot create institutional authority.

`Indeterminate` is a first-class terminal-for-automation disposition until an authorized human reconciliation action resolves it. It is never coerced to PASS.

## 4. Record disposition accounting

Every source record in the declared source scope MUST receive exactly one terminal import disposition:

- `Imported`
- `ExcludedWithReason`
- `Quarantined`
- `Unavailable`
- `Indeterminate`

The run MUST account for the source-declared record count and every page/cursor boundary. Missing pages, an incomplete export, a changed source snapshot or unenumerated scope prevents a completeness claim.

For each source item preserve:
- source system and exact product/profile version;
- institution/legal-entity scope;
- privacy-preserving source account/party token;
- stable source event/record reference and revision/sequence;
- capture timestamp and source-declared effective/recorded times as distinct values;
- exact raw byte length and cryptographic digest;
- raw payload reference or approved retention/deletion disposition;
- parser and mapping profile/version;
- source currency/instrument identity and exact amount/unit representation;
- source lifecycle state and mapped target state;
- mapping loss, unsupported fields, exclusions and ambiguity;
- import operation ID and idempotency scope;
- target event/ledger references and reconciliation lineage.

Never place raw account numbers, credentials, bearer tokens, private keys or full personal records in public DHT evidence. A privacy-preserving token is a join key, not proof of ownership or authorization.

## 5. Financial representation rules

1. Amounts use integer minor units or a declared exact decimal representation. Binary floating point is prohibited for monetary quantities.
2. Currency identity is not just a display symbol. Preserve ISO 4217 identity where applicable and the issuer/network/contract/instrument profile for non-fiat assets.
3. Unit scaling is not FX conversion. FX requires a separate versioned quote, rate source, execution and settlement record.
4. Debit/credit direction and legal-entity scope are mandatory where applicable.
5. Opening and closing positions reconcile per account-token, legal entity and instrument/currency. A total across different currencies is meaningless without a separately qualified valuation.
6. Booked balances, pending obligations, holds, reserves, fees, returns and disputed amounts remain separate buckets.
7. Corrections, reversals and returns append lineage. A later event does not overwrite the original event or erase the earlier disposition.
8. Unknown source lifecycle states remain `Indeterminate` or `Quarantined`; they are not mapped to a favorable state.

## 6. Replay and idempotency

A logical import identity binds at minimum:
- source institution/system/profile;
- source record/event identity and revision;
- exact raw-content digest;
- parser and mapping profile versions;
- declared idempotency scope.

Same identity + same fingerprint MUST resolve to the same import result with no additional posting/effect. Same identity + different fingerprint MUST fail closed as a conflict. A digest alone does not prove truth, authority, completeness or settlement.

Pagination retries, duplicated pages, process restarts and out-of-order delivery are expected inputs to qualification.

## 7. Reconciliation contract

Reconciliation compares equivalent scopes at a frozen source/target boundary.

Required independent comparisons:
- source record census vs import disposition census;
- opening position + booked movements = closing position, per currency/instrument and account/entity scope;
- pending instructions, clearing items, holds and reserves;
- fees, FX, returns, reversals and disputes;
- transaction lifecycle state and finality semantics;
- source and target reference lineage;
- known exclusions, missing evidence and unresolved exceptions.

The arithmetic identity MUST use checked integer arithmetic. A reconciliation PASS requires zero unexplained differences and zero unresolved semantic promotions. A matched aggregate total cannot conceal per-account or per-currency differences.

An independent verifier should reconstruct the result from frozen inputs and the mapping profile. The candidate system cannot generate the oracle labels or alter holdouts.

## 8. Cutover and rollback

Cutover is a separate institutional decision, not an automatic consequence of technical reconciliation.

Before any canary cutover:
- institution and legal entity are explicitly scoped;
- incumbent remains authoritative until the approved handover point;
- exact source snapshot, target state, mapping profile and policy are frozen;
- all in-scope records have explicit dispositions;
- per-account/per-currency reconciliation has zero unexplained differences;
- in-flight instructions, scheduled transactions, returns, disputes and reversals have an owner and continuity plan;
- required human approvals and independent review are recorded;
- pause, rollback, incident and data-export/exit paths are rehearsed.

Rollback is not safe when an external effect may already have occurred but its outcome is unknown. In that case the system MUST pause in `Indeterminate`, query/reconcile the original effect identity, and must not create a new logical transaction or blindly replay.

A rollback record binds the exact last-known-safe source and target snapshots, all possible effects, reconciliation state, actor/authority and resulting disposition. Rollback never deletes or rewrites prior evidence.

## 9. Benchmark protocol

Compare three candidates on the same frozen fixtures, scenario budget, information budget and hardware:

1. conventional ETL plus incumbent reconciliation/runbook;
2. provider-neutral adapter plus operational reconciliation;
3. Mycelix migration contract and evidence pipeline.

Report separately:
- source-record disposition coverage;
- semantic field preservation and explicit loss;
- duplicate, stale, missing, conflicting and out-of-order evidence detection;
- false-safe / false-final / false-completed outcomes;
- exact per-account/per-instrument reconciliation;
- deterministic replay and rollback completeness;
- operator time and exception-resolution time;
- p50/p95/p99 latency, throughput, peak memory, CPU and storage;
- fully loaded integration, support, compute and maintenance cost;
- privacy exposure and retention footprint.

A single unexplained financial difference, false finality promotion or duplicate effect is a hard failure regardless of speed or cost. Report all failed, excluded, invalid and indeterminate runs. Synthetic scenario frequency is not a real-world probability estimate.

## 10. v1 hard gates

- 100% of source records in declared scope have one explicit disposition.
- Zero unexplained per-account/per-currency balance differences.
- 100% rejection of seeded duplicate-effect, semantic-promotion, scope-substitution and replay-conflict negative controls.
- Same idempotency key and same fingerprint creates no second effect.
- Same idempotency key and changed fingerprint fails closed.
- Missing pages, stale snapshots and incomplete pagination prevent a completeness PASS.
- Replaying pinned deterministic inputs reproduces declared deterministic outputs.
- Rollback after a definitely-no-effect failure is reconstructable.
- A possible external effect with unknown outcome remains indeterminate and is never blindly replayed.
- No live customer data or external effects in v1.

These are proposed engineering acceptance gates, not industry certifications or achieved Mycelix results.

## 11. v1 synthetic corpus

The companion `bank-migration-corpus-v1.json` contains 12 frozen cases for complete capture, duplicate replay, conflicting source identity, incomplete pagination, late return, instrument/scale substitution, unknown status, false settlement promotion, legal-entity substitution, safe pre-effect rollback, uncertain possible effect, and timestamp ambiguity.

The JSON Schema file defines the wire shape. The corpus is synthetic and intentionally small; expansion requires stable case IDs, known-truth oracle ownership, explicit mutation operators and versioned holdout governance.

## 12. Standards and incumbent comparison

Use external specifications as exact, versioned mapping inputs rather than authorities over Mycelix semantics:
- ISO 20022 message definitions and market practice profiles;
- CPMI harmonized ISO 20022 requirements for cross-border payments;
- institution-specific core and payment-provider API contracts;
- applicable local legal, record-retention, data-protection and operational-resilience requirements;
- Basel operational-resilience principles for critical operations and third-party dependencies.

This contract is not a claim of ISO 20022 conformance, Basel conformity, payment-rail access, bank sponsorship or regulatory authorization.

## 13. Non-claims

A passing synthetic corpus establishes only the tested migration-contract semantics for the exact version and cases. It does not establish real bank compatibility, complete source extraction, legal ownership, customer consent, custody, external settlement finality, regulatory compliance, production readiness, zero migration risk or superiority to an incumbent.