# Payment Agent Delegation Profile v1

**Status:** synthetic reference contract; no live financial-agent integration or payment execution.

## Purpose

Permit an AI agent to select among acceptable financial actions within a principal-approved constraint envelope, without treating the natural-language mission, model proposal, agent authentication or a generic credential as payment authority.

## Core composition

`Mission ≠ TypedIntentProposal ≠ QualifiedIntentEnvelope ≠ exact Intent ≠ ActionRequest ≠ provider authority ≠ payment outcome`

This profile does not replace AGENT-004 (#801), exact-action human-visible presentation (#1336), or the payment receipt model (#1215). It defines payment-specific constraints and reservation semantics that must be proven before an exact ActionRequest can proceed.

- AGENT-004 owns how a broader principal-approved set of acceptable outcomes becomes a QualifiedIntentEnvelope.
- Existing `Intent`/`ActionRequest` remains the representation of one exact proposed action, with exact payload digest, authority grant, consent evidence, nonce and validity.
- LEX-NET-037 binds the exact material presentation to human confirmation when the payment profile requires it.
- A successful local constraint evaluation is not a payment instruction, bank authorization, settlement or finality.

## Bounded delegation model

A payment mandate binds:
- principal and agent identities;
- exact parent envelope commitments and versions;
- audience, purpose, action types and validity window;
- allowed beneficiaries, currencies and merchant/provider scope;
- single-action cap, total budget, transaction count and concurrency limits;
- fee, FX and all-in cost constraints;
- new-payee, amount and material-change confirmation thresholds;
- sub-agent chain and maximum delegation depth;
- expiry, revocation and compromise state;
- budget reservation and reconciliation policy.

The allowed action set is the intersection of all parent and child constraints. A child cannot widen a parent mandate.

## Deterministic admission sequence

1. Verify mandate authenticity/status, principal, agent, audience, version, expiry and revocation.
2. Verify every delegation hop and its depth/scope intersection.
3. Verify candidate action, purpose, beneficiary, provider, currency and profile scope.
4. Qualify the total cost in the mandate's budget currency. Unknown fees or stale/unverified FX quotes require a new quote; they never mean zero.
5. Check single-action, cumulative, count and concurrency limits with checked arithmetic.
6. Require exact principal confirmation for new beneficiaries, threshold breaches or any profile-defined high-risk/material change.
7. Bind the final exact ActionRequest to the candidate digest and any required human-visible presentation/consent evidence.
8. Atomically reserve budget and idempotency identity before dispatch. The reference evaluator only simulates this step; it creates no external effect.

A language model cannot interpret arbitrary natural-language restrictions at the security boundary. If a condition such as "refundable only" or "no overnight connection" cannot be represented in the closed constraint vocabulary and independently checked, the candidate remains approval-required or ineligible.

## Budget, unknown outcomes and returns

- Reservations must be atomic across concurrent candidate actions.
- Exact replay with the same idempotency identity and digest deduplicates; the same identity with a different digest rejects.
- A dispatched or indeterminate transaction retains its budget reservation until the outcome is independently reconciled.
- A provider timeout is not proof that no funds moved.
- A reconciled failure or pre-dispatch cancellation may release a reservation under the applicable rule.
- A later refund/return does not automatically replenish the mandate budget. Replenishment requires an explicit principal-approved rule and a separately reconciled successor event.
- Revocation blocks new candidate actions after its effective point. It does not reverse transactions already dispatched and must not erase their outstanding reconciliation state.

## Synthetic benchmark

The 32 scenarios include valid low-value autonomy, per-action/cumulative limits, fees and FX, confirmation thresholds, exact digest binding, new beneficiaries, parent/child scope, prompt injection, idempotent replay, concurrency, unknown provider outcomes, failure reconciliation, returns, revocation and in-flight actions.

Hard gates:
- 0 actions outside any parent/child envelope;
- 0 agent-only scope expansion or self-confirmation;
- 0 spend beyond single, total, fee, count or concurrency constraints;
- 0 duplicate reservation/spend from replay or parallel calls;
- 0 new action after effective revocation;
- 0 release of budget on unknown outcome;
- 0 silent zero-cost interpretation of unknown fees/FX;
- 100% exact ActionRequest digest and required principal confirmation binding;
- 100% deterministic replay of frozen mandates, reservations and candidate order.

Measure separately:
- eligible actions completing without extra human interruption;
- unnecessary confirmation rate;
- required-confirmation recall on new payee, price/fee drift and high-value cases;
- false admission and false rejection;
- overspend/duplicate upper confidence bounds under concurrency;
- p50/p95/p99 mandate-check and reservation latency;
- budget-contention and recovery time;
- revocation propagation and stale-authority exposure;
- unknown-outcome reservation hold duration;
- user task success and total cost/FX compliance.

Do not optimise autonomous completion rate without measuring false admission, user control, appeal and recourse.

## Industry context

The FIDO Alliance, W3C and major payment networks are developing agent authentication, verifiable instructions and trusted agentic commerce mechanisms. This profile is a research contract designed to compose with those efforts; it does not claim conformance. The distinguishing proposition is a deterministically checkable payment mandate, atomic budget consumption and explicit revocation/reconciliation semantics, with human confirmation triggered by material conditions.

## Claim ceiling

All fixtures are synthetic and external effects are disabled. Passing the evaluator demonstrates only schema and seeded constraint conformance. It does not establish that a mandate is legally valid, accepted by a bank, compliant in every jurisdiction or safe for real-money use. Legal authority and participant operating rules must be established separately.
