# Bounded Agentic Payment Delegation Profile v1

**Status:** synthetic/effects-disabled research contract. This is not a payment protocol, credential, legal standing mandate, or production authorization service.

## Why this is the next boundary

Agentic payment infrastructure is moving from human-present checkout to pre-authorized human-not-present execution. Google donated AP2 to FIDO on 28 April 2026; AP2 v0.2 adds human-not-present payments using pre-authorized instructions. Mastercard's Verifiable Intent is aligned with AP2. Visa, BILL and Billtrust announced a live business-invoice agent-payment pilot on 20 October 2026. FIDO's agentic work remains under active development; W3C SPC is a Candidate Recommendation Draft for a one-time transaction-confirmation ceremony, not a standing spend mandate.

Mycelix should not fork AP2, Verifiable Intent, SPC, or generic identity/authority objects. The opportunity is a deterministic safety kernel that proves an exact proposed payment action stays within a separately qualified and current authorization envelope.

## Reuse boundaries

- The parent QualifiedIntentEnvelope (QIE) is an opaque reference owned by AGENT-004/#801. This evaluator does not define or authorize a QIE.
- The exact generic Intent/ActionRequest and its payload digest remain owned by institutional-core authority ancestry and #5769. Here they are opaque exact-action references; the payment evaluator does not create a second ActionRequest type or reimplement its canonical digest.
- Principal approval is referenced through a typed receipt. A model-generated proposal, authenticated agent session, merchant credential or AP2-compatible envelope does not itself prove principal approval.
- Payment instruction, provider acceptance, recipient availability, settlement/finality and recourse remain separate outcomes.

## Deterministic sequence

`principal approval → active bounded mandate → typed action proposal → deterministic constraint evaluation → atomic budget reservation → optional required human confirmation → dispatch authorization → provider outcome → independent reconciliation`

The v1 evaluator deliberately requires human confirmation before reservation where the beneficiary is new or the profile/mandate threshold is reached. Unknown fee or FX information stops automatic authorization. An exact candidate cannot widen the mandate or its parent QIE.

## Budget and reservation semantics

The budget is denominated in one explicit currency and integer minor units. For each candidate:

`all_in_budget_debit = budget_equivalent_principal + known_fee + known_FX_cost`

For a same-currency action, budget-equivalent principal equals the original amount and FX cost is zero. A cross-currency action requires a source-referenced quote with base amount/currency, quote amount/currency, observation time and expiry. Unknown or expired quotes are not eligible.

Reservation is a linearized, atomic event. Each action can consume at most one reservation identity. Active reservations include reserved, dispatched and unknown-outcome payments. A timeout does not release budget. Only an independent reconciliation event proving no economic effect releases the reservation. A confirmed effect consumes budget. A later return is a successor event and does not replenish mandate budget by default.

The event sequence is the synthetic reference linearization order for concurrency tests. A production implementation would need a durable serializable reservation store or equivalent compare-and-swap/fencing contract; this fixture is not proof of a distributed database implementation.

## Revocation and delegation

A revocation request suspends new reservations while authentication/effectiveness is resolved. An effective revocation blocks later reservations and dispatches. It does not reverse an already dispatched transfer; that transfer remains an in-flight outcome requiring reconciliation.

Sub-agent scope must be a subset of its parent at each edge: maximum single spend, total spend, fee/FX caps, action classes, purposes, currencies and beneficiaries. Depth is bounded. A sub-agent cannot create a broader mandate, change the principal, or reuse another agent's exact ActionRequest.

## Industry comparison

- **AP2/FIDO:** signed mandate and verifiable intent architecture, including HNP scenarios; standards work is active, not a claim of final global conformance.
- **Mastercard Verifiable Intent:** tamper-resistant record of authorized agent actions, aligned with AP2.
- **Visa Intelligent Commerce / Trusted Agent Protocol:** issuer/agent identity, tokenized payment credentials and merchant visibility; Visa announced a live approved business-invoice pilot in October 2026.
- **W3C Secure Payment Confirmation:** cryptographic evidence that a user confirmed exact payee, amount/currency and instrument in a one-time ceremony; not a long-lived autonomous budget.
- **Mycelix S0 target:** independently testable constraint inclusion, all-in price ceilings, atomic reservation accounting, stale-outcome retention, revocation and non-expanding sub-delegation. This is a research contract, not a competing agent-payment network.

## Hard benchmark gates

- 0 payment reservations outside the exact mandate/QIE reference and approved scope.
- 0 self-authorized or agent-generated mandate approvals.
- 0 single, total, period or concurrency budget overruns.
- 0 duplicate reservations/effects from replay or parallel actions.
- 0 unknown fee/FX claims treated as known or zero.
- 0 dispatch after effective or pending revocation.
- 0 budget release after timeout/unknown outcome without independent no-effect reconciliation.
- 0 return events that silently replenish mandate budget.
- 0 sub-agent scope expansion or depth overrun.
- 100% deterministic replay of frozen events and reservation outcomes.
- 100% exact minor-unit arithmetic; overflow fails closed.

## Utility and operational metrics (separate from safety gates)

- Eligible low-risk action completion without extra human interruption.
- Correct human-escalation rate for new payees, high amounts, changed terms and unknown fees/FX.
- False acceptance / false rejection on independently adjudicated mandate cases.
- p50/p95/p99 authorization latency by risk and approval class.
- Budget contention and reservation recovery time.
- Revocation propagation latency and stale-authority exposure.
- Task-completion utility and unnecessary escalation.
- Audit reproduction time and completeness of non-repudiable evidence.

Do not optimise autonomous completion rate without reporting false authorization, user-control, unknown outcomes, fee/FX overruns, and denominator definitions. A 100% refusal system is not useful; a high-throughput system with one unauthorized effect fails the safety objective.

## Claim ceiling

The checked-in corpus is synthetic. Passing tests demonstrates only that this reference evaluator rejects seeded violations. It does not prove legal validity of standing mandates, a real provider's authorization, cryptographic credential security, production atomicity, agent competence, regulatory approval or safe real-money operation.


## S0 reference evaluator and benchmark corpus

The synthetic v1 corpus contains 32 cases and four profiles: two South African research-policy profiles, an AP2/FIDO HNP architecture comparator, and a W3C SPC one-time-confirmation comparator. Comparator profiles cannot be used as authorization policy inputs.

The Rust evaluator reuses opaque QIE and exact ActionRequest references; it does not define parallel generic types. It checks principal approval binding, profile admission, exact action digest references, purpose/action/payee/currency scope, all-in fee/FX budget arithmetic, quote source/freshness, confirmation thresholds, atomic event-order reservation accounting, transaction-count and concurrency limits, revocation, and sub-agent scope inclusion.

Unknown fee/FX data enters `terms_requalification_required`; a human confirmation cannot convert unknown terms into known terms. High-value and new-beneficiary actions use a separate principal confirmation event bound to the exact presented action digest. A provider timeout retains the reservation, an independently reconciled no-effect event may release it, and a later return does not replenish the mandate budget by default.

The corpus adds specific transaction-count exhaustion and revocation-during-indeterminate-payment cases. The evaluator is a deterministic event-log reference model only. A production reservation service still requires a durable serializable/compare-and-swap or fenced store, crash-recovery tests, and independently qualified clock and authority sources.
