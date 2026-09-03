# Execution Lifecycle Runtime v0.1 — Containment Invariants

Status: **normative draft runtime contract**

This runtime is a containment layer for issue #62. It does not yet replace the legacy execution coordinator and it deliberately exposes no external-effect executor.

## Candidate != authority

`LifecycleEventCandidate` is immutable audit/provenance data.

A candidate is never binding merely because:

- it exists on the DHT;
- integrity validation accepted its shape;
- its publisher is the claimed actor; or
- its event/domain hashes recompute.

`get_verified_execution_lifecycle` must independently resolve the exact current execution domain and independently verify every current-domain event before constructing `VerifiedLifecycleEvent` values for the pure projector.

A rejected candidate is excluded from authority. A verifier outage/call/decode failure is a denial, not an exclusion/fallback.

## Intentional verifier absence

The checked-in DNA packages `execution_lifecycle` but does **not** package `governance_execution_lifecycle_verifier` yet.

Therefore authoritative projection and `claim_execution` are intentionally unavailable in the checked-in deployment state.

This is preferable to a permissive placeholder verifier.

The future verifier must supply three exact boundaries:

1. `resolve_execution_domain(proposal_id)`
   - reconstruct the domain from the current constitutional epoch, proposal authority, approved binding tally, exact action digest, threshold authority, executor designation, and effect-safety policy;
2. `verify_lifecycle_event(request)`
   - prove that the candidate actor had the exact institutional authority required for that event kind and domain; and
3. `verify_effect_safety_policy(request)`
   - prove that the exact bound safety mechanism is configured/enforceable for this attempt.

No legacy Phi/reputation/model-score fallback is permitted.

## Exact executor principal

The execution domain names one exact executor principal plus the exact authority that designated it.

`Claimed`, `Completed`, `Failed`, and `Uncertain` candidates must:

- be committed by that exact Holochain agent;
- name that exact DID as event actor; and
- survive external institutional verification.

Possession of the signing key alone is not sufficient institutional authority; the executor designation must remain valid.

## Strict source-chain claim fence

`claim_execution` must run only from a uniquely verified `Ready` lifecycle.

Its commit is deliberately separate from external effects.

For one bound Holochain executor agent, strict source-chain ordering provides a local serialization layer: concurrent writes compete against one chain head. This is not treated as a network-wide mutex and does not remove the need for deterministic attempt identity and external idempotency/fencing.

A claim must be durably committed before any future effect call is allowed to begin.

## Deterministic attempt identity

The claim event ID is publication provenance.

The attempt ID is derived from:

`exact execution-domain digest + exact ReadyAuthorized event ID`

Therefore same-Ready concurrent claims converge on the same downstream idempotency namespace even though competing verified claim events freeze lifecycle projection.

## Effect-safety policy

The execution domain commits an exact effect-safety policy digest/profile.

The runtime verifies that policy again before committing a claim.

No external-effect endpoint is enabled in v0.1. A future effect endpoint must reverify:

- exact current domain;
- exact claim event and attempt ID;
- exact executor principal/authority;
- exact effect-safety policy; and
- exact action index/idempotency key

immediately before each externally relevant effect.

If the target system cannot enforce the bound safety policy, automatic execution must deny.

## Crash windows

A source-chain commit and an external/network effect are not one atomic transaction.

Required interpretation:

- before claim commit: no attempt consumed;
- after claim commit, before effect: attempt consumed, no automatic re-claim;
- after effect when durable completion is unknown: reconcile to `Uncertain`, never automatic replay;
- after positively observed no-effect failure: `Failed(NoEffectObserved)` remains terminal in v0.1; retry requires a later explicit governed protocol.

## Projection completeness

The coordinator ignores candidates from stale/different execution domains rather than allowing them to contaminate current authority.

For the exact current domain:

- every candidate is independently verified or explicitly rejected;
- verified events are passed as a complete set to the pure projector;
- missing parents, multiple roots, competing children, causal-time regression, unreachable verified data, or terminal-transition violations all deny projection.

DHT arrival order never resolves ambiguity.

## Legacy execution state

The existing mutable `Timelock.status` remains compatibility state during migration.

It must not be interpreted as the future binding lifecycle authority. The migration is complete only when binding execution derives Ready/Claimed/terminal state from this verified append-only lifecycle and unauthorized direct timelock updates cannot influence execution authority.

## Legacy veto / advisory signals

Guardian veto / Phi-weighted override remains outside this new authority path.

A future review/cancellation event must have explicit rulebook/capability authority. Phi, consciousness metrics, reputation, stake, Symthaea outputs, or other advisory signals may trigger review requests but cannot directly mint or restore lifecycle authority.

## External-effects-disabled invariant

Until a qualified verifier and effect adapter exist:

`automatic_external_effects_enabled = false`

must remain true in the runtime status API (i.e. the reported boolean must remain `false`).

Adding an external-effect endpoint is a separate security tranche and must not be smuggled into lifecycle persistence/projection work.
