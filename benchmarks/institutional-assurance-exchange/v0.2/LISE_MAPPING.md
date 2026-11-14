# IAE v0.2 → LISE/IPB-FINALITY owner mapping

**Status:** synthetic adapter contract; not an executable cross-crate adapter yet.

The Institutional Assurance Exchange (IAE) owns the accepted intent baseline, claim/evidence packet, bitemporal replay and decision trace. The LISE/IPB-FINALITY owner (#6136; reference crate in PR #6139) owns exact-operation correspondence, current constitutional/policy identity, execution preflight, retry safety and partial/unknown effect reconciliation.

IAE must not become a competing finality engine. It provides a bounded projection to the LISE owner.

## Distinct approval objects

The fixture deliberately carries two separate references:

- `intent_baseline.approval`: human acceptance of the compiled requirement baseline.
- `operation_finality.approval_ref`: human authorization of the exact lease operation.

Acceptance of requirements does not authorize a transaction. Transaction authority and effectuation must still be checked at the execution boundary.

Both records are synthetic and explicitly unauthenticated in this fixture.

## Field mapping

| IAE v0.2 field | LISE reference type/field | Qualification |
|---|---|---|
| `approved_operation.logical_operation_id` | `OperationBinding.logical_operation_id` | Exact stable logical operation identity |
| `canonical_operation_digest` | `OperationBinding.canonical_operation_digest` | Placeholder only; no hash was calculated or verified |
| `action_digest` | `OperationBinding.action_digest` | Placeholder only; no action digest was calculated or verified |
| `action` | Domain-specific action projection | Must bind the named protected operation |
| `principal_ref` | Approval/authority principal context | Must bind the presenting actor |
| `target_ref` | `OperationBinding.target_ref` | Must match PROPERTY-001 in this fixture |
| `destination_or_audience` | `OperationBinding.destination_or_audience` | Synthetic tenant identifier |
| `amount_minor`, `currency` | Same-named fields | Null because this is not a monetary transfer |
| `purpose` | `OperationBinding.purpose` | Exact approved purpose |
| `resource_revision` | `OperationBinding.resource_revision` | Synthetic resource revision |
| `idempotency_ref` | `OperationBinding.idempotency_ref` | Exact same key across approved/pending operation |
| `operation_finality.constitution` | `ConstitutionRef` | Institution, epoch and digest; digest is placeholder only |
| `operation_finality.approval_state` | `ApprovalState` | `authorized` maps to Authorized |
| `operation_finality.policy_revision` | `ApprovalBinding.policy_revision` / `CurrentProtectedState.policy_revision` | Exact policy version |
| `authority_state_before_dispatch` | `AuthorityState` at original attempt time | Owner-resolved at action time and knowledge cutoff |
| `authority_state_after_timeout` | `AuthorityState` at reattempt time | Now revoked; reattempt is denied |
| `resource_state_before_dispatch` | `ResourceState` | Owner-resolved resource precondition |
| `prior_effect_after_timeout` | `PriorEffectState` for the original operation | `OutcomeUnknownAfterDispatch`; retained independently of retry authorization |
| `retry_safety_after_timeout` | `RetrySafety` | Unknown; no retry permission is inferred |

## Two different post-timeout questions

The fixture intentionally separates the original operation's unresolved outcome from the authorization of any new attempt.

**Original preflight and operation:** the decision-time query has effective/action/knowledge cutoffs at 2026-01-25 10:00Z. The human-approved operation, policy, resource precondition and authority were current at that preflight. The provider submission was attempted at 10:01Z and timed out at 10:01:05Z. The original outcome remains UNKNOWN and requires reconciliation.

**Potential reattempt:** at 2026-02-06 10:00Z, the authority history says the actor's grant is revoked (effective 2026-02-01, observed 2026-02-02). A new attempt is denied. The system must not turn the original unknown outcome into failure-no-effect, and must not retry.

The LISE pre-effect reference is used in two conceptual steps:

1. Original pre-dispatch: `PriorEffectState::NeverAttempted` plus current authority/resource/constitution can produce Permit for the bounded original operation.
2. Reattempt preflight: current authority is revoked. LISE should return `Deny(DenialCode::AuthorityRevoked)`. The prior operation's outcome remains an independent UNKNOWN reconciliation obligation; denial of a retry does not resolve it.

This distinction avoids a misleading single terminal label. "Cannot retry" and "the original effect is known not to have happened" are different claims.

The IAE harness structurally checks both: original operation history is UNKNOWN/HOLD with retry=0, while the current reattempt authority projection is REVOKED/DENY. Historical query knowledge cutoffs precede the 10:01Z dispatch and timeout; later current knowledge includes the 2026-02-02 revocation observation.

## Executable integration status

The IAE harness currently verifies the packet mapping structurally. It does **not** import or call the LISE Rust crate in this PR, because the LISE reference currently exists on a separate draft branch and has not yet merged to `main`. Once #6139 is accepted/merged, add a thin adapter test that constructs `FinalityInput`, calls `evaluate_finality` separately for original preflight and reattempt preflight, and uses the LISE reconciliation API for the original unknown effect. Do not copy the LISE state machine into IAE.

## Remaining gaps before executable cross-crate conformance

1. Replace synthetic operation and constitution digests with owner-generated canonical digests and independently verified receipts.
2. Bind the exact domain-specific canonicalization contract and version; the LISE crate intentionally does not canonicalize arbitrary payloads.
3. Supply the actual current constitutional reference from its owner; do not infer it from entity identity.
4. Supply owner-resolved authority and resource precondition states at the exact effectuation boundary.
5. Supply provider capability evidence for idempotency, cancellation, status queries and reconciliation.
6. Connect the IAE packet to a read-only LISE adapter test once the owner reference crate is available on the target branch.

Until these gaps are closed, the fixture is a coherent synthetic mapping demonstration—not production finality assurance, cryptographic authenticity, legal authority or an exactly-once claim.
