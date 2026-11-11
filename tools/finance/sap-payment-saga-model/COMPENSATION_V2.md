# Effect-level financial compensation V2

Status: independent Rust/Go effects-disabled reference model; not a live refund or settlement adapter.

This profile is separate from the frozen SAP Saga Trace V1 and Oracle V1 contracts. It models a compensation as a new, independently authorized financial effect linked to one positively settled forward effect.

## Paradigm

Do not treat compensation as an inverse database mutation or a saga-wide `rolled_back` flag. A forward effect can be irreversible, partly reversible, or manually recoverable. A compensation can be partial, separately funded, delayed, indeterminate, and independently settled.

The V2 model binds six canonical subjects:

1. `ForwardEffectV2`: original source and destination owners/accounts, asset, amount, settlement class and reversible ceiling.
2. `CompensationPlanV2`: the exact forward-effect digest, funding account, beneficiary account, policy digest, recovery reason and aggregate plan ceiling.
3. `CompensationAuthorizationV2`: issuer/grantee, plan, exact account/asset scope, cumulative amount ceiling, nonce and expiry/revocation state.
4. `LiquidityReservationV2`: one unique reservation ID, funding owner/account, asset and exact amount.
5. `CompensationRequestV2`: a fresh compensation identity, the acting owner, amount, authorization and reservation.
6. `CompensationOutcomeV2`: a new effect ID and outcome kind bound to the exact forward effect, plan, authority, reservation, funding account, beneficiary account, asset and amount.

Canonical SHA-256 profiles use domain separation, fixed field order, u32-BE length-prefixed UTF-8 strings and u64-BE minor units. The shared independent known-answer file is `fixtures/sap-compensation-v2.kat`; Rust and Go test every subject digest against it.

## State machine

```text
Reserved
  ├─ cancel before dispatch ───────────────> Cancelled
  └─ authorized grantee + live capability ─> DispatchAuthorized
        ├─ confirmed final settlement ─────> Settled
        └─ timeout/uncertain response ─────> Indeterminate
              ├─ authoritative absence ────> AuthoritativelyAbsent
              │      ├─ same effect + live grant -> Reserved
              │      └─ cancel before retry ────> Cancelled
              ├─ observed but unsettled ───> ObservedUnsettled
              │      └─ final settlement ───────> Settled
              └─ contradictory evidence ──> reject / retain encumbrance
```

The original forward effect is immutable. Settlement creates a new compensation movement; no transition rewrites the original debit, credit, receipt or history.

## Safety properties in the reference model

- Only a forward effect marked Settled with a positive reversible ceiling can produce a compensation plan.
- The refund beneficiary is exactly the forward source owner/account; the asset matches.
- An acting owner must match the grant's grantee at reservation and dispatch.
- A capability ID has one immutable scope; changing its amount ceiling, nonce, expiry or other scope under the same ID is rejected.
- A reusable capability has one cumulative settled-plus-encumbered ceiling across all partial compensation requests.
- A reservation ID is unique for the lifetime of the book, including after cancellation.
- Per-forward total settled plus encumbered compensation never exceeds the plan's reversible ceiling.
- Aggregate settled plus encumbered compensation never exceeds the modelled source liquidity.
- Indeterminate, authoritative-absence and observed-unsettled states retain encumbrance.
- Retry is possible only after an authoritative-absence disposition and reuses the original compensation effect ID and still-live grant.
- A positive/possible effect count contradicts an authoritative-absence claim.
- Exact request and outcome retries are idempotent; changed terms or evidence conflict.
- Cancellation is available only before dispatch (or after confirmed absence before retry) and releases both encumbrance and capability consumption.

The Go book serializes its in-memory transition methods with a mutex. The Rust model uses exclusive mutable access. This is a reference state machine, not proof of a distributed account lock or a real Holochain source-chain reservation.

## Indexed admission

The first prototype scanned every compensation record for reservation-ID uniqueness, cumulative capability use and settled/encumbered totals. That produced O(n) admission and O(n²) work for a batch of n refunds.

The current reference uses:
- a reservation-ID set;
- capability-ID → immutable scope digest;
- capability-ID → cumulative amount use;
- incrementally maintained settled and encumbered totals;
- direct compensation-ID lookup.

Admission is therefore O(log n) for the Rust BTree indexes and expected O(1) for Go hash indexes, plus canonical hashing and mutex contention. The Go test corpus includes 10,000 partial reservations, a 50-caller concurrent ceiling test, and single-threaded/parallel admission benchmarks.

This is an in-memory algorithmic property. It does not include Holochain validation, persistence, external account holds, provider latency or DHT convergence.

## Industry comparison and what this is trying to add

The industry already has strong primitives; Mycelix must integrate them, not claim to replace them:

- Temporal recommends idempotent compensations, including handling cases where the forward activity may not have executed. It notes that some operations do not have meaningful compensation. The new profile requires a positively settled forward subject before a financial payout is eligible, instead of assuming that a generic no-op compensator is sufficient. Temporal's workflow durability and retries remain complementary: https://docs.temporal.io/design-patterns and https://github.com/temporalio/documentation/blob/main/docs/design-patterns/saga-pattern.mdx
- AWS describes saga eventual consistency, missing transaction isolation and the need for semantic locking under concurrent orchestration. The indexed local book makes one admission boundary explicit; a production adapter still needs a single authoritative funding-account reservation authority or an equivalent atomic primitive: https://docs.aws.amazon.com/prescriptive-guidance/latest/cloud-design-patterns/saga-orchestration.html
- Azure states that compensation is not necessarily an exact inverse, can apply business-specific rules, can fail and may require manual intervention. This motivates effect-level reversibility and an explicit noncompensable/manual-recovery class rather than a generic rollback: https://learn.microsoft.com/en-us/azure/architecture/patterns/compensating-transaction
- Adyen already supports idempotent POST requests and prevents aggregate refunds from exceeding the captured amount by default. Its idempotency keys are company-account scoped, retained 7–14 days, and are not cross-region duplicate checks. Mycelix should use those provider guarantees rather than duplicate them, while binding the provider's actual scope/retention/region/finality contract into the adapter and independently enforcing owner grants and funding-account exposure: https://docs.adyen.com/development-resources/api-idempotency

The competitive hypothesis is a stronger evidence and authority boundary around refunds across owners/providers, not a claim that this in-memory model is faster or more correct than Temporal, AWS Step Functions or Adyen.

## Benchmark protocol

Run the Rust and Go exact-head qualification suites, then benchmark on controlled, identified hardware.

### Scales and contention

- Existing regression: 10,000 one-minor-unit partial refund reservations.
- Target corpus: 1k, 10k, 100k and 1m compensation records.
- Parallel admission: 1, 2, 4, 8, 16, 32 and 64 workers.
- Mix: 1–16 partial components per plan; 1–1,000 simultaneous requests against one forward effect; independent forward effects across a shared funding account.
- Recovery mix: 0%, 1%, 10% and 50% indeterminate outcomes; duplicate delivery; authoritative absence; observed-unsettled; final settlement and cancellation.

### Measurements

- Admission p50/p95/p99 and throughput at each retained corpus size.
- CPU, allocations/bytes per admission, index size and mutex contention.
- False liquidity denials and stale-reservation conflicts.
- Time from indeterminate observation to reconciled disposition.
- Exact request replay and duplicate delivery rate.
- Rust/Go canonical digest agreement and disposition agreement.
- Number of safety invariant violations (target: zero); manual recovery and noncompensable residual exposure must be measured separately.

### Acceptance targets

- Zero overcompensation, duplicate reservation use, double settlement, retry under uncertain outcome or unauthorized dispatch in the complete adversarial suite.
- 100% shared KAT agreement across Rust and Go.
- 100% exact-head formatting, tests, race detector and static analysis.
- Admission scaling should track the indexed algorithm rather than the former full-history scan; publish actual latency/throughput only with raw samples, hardware identity and source SHA.
- No absolute latency or SOTA superiority claim until a matched public baseline, equivalent workload, same machine and identical correctness envelope have been run.

## Qualification limits and next work

The profile does not verify:
- a bank/provider signature or authenticated settlement/absence receipt;
- that the liquidity reservation actually created an account hold or reduced available funds;
- that the initial liquidity snapshot is fresh or globally exclusive;
- real multi-agent Holochain serialization, fork exclusion, source-chain ownership, DHT convergence or restart durability;
- provider idempotency retention, region scope, refund eligibility or legal finality.

Next required stages:
1. Add provider-contract adapters with exact idempotency scope, retention, region, partial-refund policy, settlement/absence evidence and version pinning.
2. Replace the model's supplied liquidity snapshot with an owner-authorized, durable reservation receipt from the funding-account authority.
3. Verify signed provider outcomes with pinned trust roots, revocation and replay protection.
4. Add an exhaustive bounded interleaving/model-checking specification and fault injection at every reserve/dispatch/observe/settle persistence boundary.
5. Differentially execute the Rust and Go models against the actual Mycelix/Holochain implementation; do not infer production behavior from reference agreement.
