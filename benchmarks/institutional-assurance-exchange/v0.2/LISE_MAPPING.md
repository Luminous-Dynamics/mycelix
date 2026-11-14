# IAE v0.2 → LISE/IPB-FINALITY owner mapping

**Status:** synthetic adapter contract; not an executable cross-crate adapter yet.

The Institutional Assurance Exchange (IAE) owns the accepted intent baseline, claim/evidence packet, bitemporal replay and decision trace. The LISE/IPB-FINALITY owner (#6136; reference crate in PR #6139) owns exact-operation correspondence, current constitutional/policy identity, execution preflight, retry safety and partial/unknown effect reconciliation.

IAE must not become a competing finality engine. It should provide a bounded projection to the LISE owner.

## Distinct approval objects

The fixture deliberately carries two separate references:

- `intent_baseline.approval`: human acceptance of the compiled requirement baseline.
- `operation_finality.approval_ref`: human authorization of the exact lease operation.

Acceptance of requirements does not authorize a transaction. Transaction authority and effectuation must still be checked at the execution boundary.

Both records are synthetic and explicitly unauthenticated in this fixture.

## Field mapping

| IAE v0.2 field | LISE reference type/field | Qualification |
|---|---|---|
| `operation_finality.approved_operation.logical_operation_id` | `OperationBinding.logical_operation_id` | Exact stable logical operation identity |
| `canonical_operation_digest` | `OperationBinding.canonical_operation_digest` | Placeholder only; no hash was calculated or verified |
| `action_digest` | `OperationBinding.action_digest` | Placeholder only; no action digest was calculated or verified |
| `target_ref` | `OperationBinding.target_ref` | Must match PROPERTY-001 in this fixture |
| `destination_or_audience` | `OperationBinding.destination_or_audience` | Synthetic tenant identifier |
| `amount_minor`, `currency` | Same-named fields | Null because this is not a monetary transfer |
| `purpose` | `OperationBinding.purpose` | Exact approved purpose |
| `resource_revision` | `OperationBinding.resource_revision` | Synthetic resource revision |
| `idempotency_ref` | `OperationBinding.idempotency_ref` | Exact same key across approved/pending operation |
| `operation_finality.constitution` | `ConstitutionRef` | Institution, epoch and digest; digest is placeholder only |
| `operation_finality.approval_state` | `ApprovalState` | `authorized` maps to Authorized |
| `operation_finality.policy_revision` | `ApprovalBinding.policy_revision` / `CurrentProtectedState.policy_revision` | Exact policy version |
| `authority_state_before_dispatch` | `AuthorityState` | Owner-resolved state at action time and knowledge cutoff |
| `resource_state_before_dispatch` | `ResourceState` | Owner-resolved resource precondition |
| `execution_control.state=UNKNOWN` plus operation history | `PriorEffectState::OutcomeUnknownAfterDispatch` | Timeout after possible dispatch |
| `retry_safety_after_timeout=unknown` | `RetrySafety::Unknown` | No retry permission is inferred |

## Expected reference result

For the post-timeout packet, the LISE reference should produce:

`FinalityDecision::Hold(HoldCode::PriorOutcomeUnknown)`

The original operation is not to be repeated. A retry can be considered only after owner-qualified reconciliation establishes no effect and a provider contract proves same-operation/same-idempotency-key retry safety.

The IAE harness currently verifies the mapping and the hold/no-retry output structurally. It does **not** import or call the LISE Rust crate in this PR, because the LISE reference currently exists on a separate draft branch and has not yet merged to `main`. Once #6139 is accepted/merged, add a thin adapter test that constructs `FinalityInput` and calls `evaluate_finality` directly. Do not copy the LISE state machine into IAE.

## Remaining gaps before executable cross-crate conformance

1. Replace synthetic operation and constitution digests with owner-generated canonical digests and independently verified receipts.
2. Bind the exact domain-specific canonicalization contract and version; the LISE crate intentionally does not canonicalize arbitrary payloads.
3. Supply the actual current constitutional reference from its owner; do not infer it from entity identity.
4. Supply owner-resolved authority and resource precondition states at the exact effectuation boundary.
5. Supply provider capability evidence for idempotency, cancellation, status queries and reconciliation.
6. Connect the IAE packet to a read-only LISE adapter test once the owner reference crate is available on the target branch.

Until these gaps are closed, the fixture is a coherent synthetic mapping demonstration—not production finality assurance, cryptographic authenticity, legal authority or an exactly-once claim.
