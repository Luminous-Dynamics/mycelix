# Governance Execution Lifecycle Verifier v0.1 — Implementation Invariants

Status: **normative implementation contract**

The lifecycle verifier is a composition boundary. It does not create governance policy or authority.

## Domain construction

`resolve_execution_domain(proposal_id)` MUST reconstruct the domain from independently verified current facts:

1. immutable execution-plan identity + exact action bytes;
2. fresh `governance_execution_preflight` result for those exact bytes;
3. fresh threshold execution authorization;
4. fresh executor designation; and
5. fresh effect-safety policy.

Caller-supplied domain data is never the source for this endpoint.

The v0.1 `ExecutionDomain.timelock_ref` field is a legacy field name. The verifier MUST populate it with the immutable execution-plan reference, never mutable `Timelock.status` or a blindly followed latest Timelock update. Renaming the field to `execution_plan_ref` is reserved for a protocol-version change.

## Stable Ready identity vs freshness

`ReadyAuthorized.preflight_ref` MUST identify stable authority semantics, not one verifier invocation.

The verifier therefore derives the preflight identity from the exact:

- proposal;
- proposal-authority action;
- constitutional statement digest;
- binding tally action;
- action digest; and
- action digest profile.

Verifier timestamps are deliberately excluded. Freshness is independently rechecked on every authoritative read.

## Opaque authority references

`threshold_authorization_ref` and `executor_authority_ref` are permitted to remain opaque in lifecycle v0.1 only under this invariant:

> Each ref identifies one immutable authorization record/content commitment whose meaning includes every authority-bearing field echoed by its verifier receipt.

For threshold authority that means at least:

- committee/institution;
- signing-policy digest/profile;
- key epoch;
- exact action digest/profile; and
- validity/revocation domain.

For executor authority that means at least:

- exact executor principal;
- granting institution;
- capability/scope;
- rulebook/policy reference;
- exact proposal/action authority domain; and
- validity/revocation domain.

A provider MUST NOT reuse the same authorization ref after changing any of those semantics. Doing so is a provider-contract violation and MUST deny qualification.

A later protocol version may replace opaque refs with explicit structured authority commitments in `ExecutionDomain`.

## Provider boundaries

The verifier depends on these independently authoritative providers:

- `governance_execution_plan_verifier`
- `governance_execution_preflight`
- `governance_threshold_authority_verifier`
- `governance_executor_authority_verifier`
- `governance_effect_safety_authority_verifier`
- `governance_execution_outcome_verifier`
- `governance_execution_review_authority_verifier`

Missing zome/function, failed call, decode failure, expired receipt, future verification time, malformed binding or exact-field mismatch is denial.

## Event qualification and projection

Per-candidate verification and lifecycle causality are deliberately separate responsibilities.

The verifier qualifies the authority/evidence local to each candidate. The pure lifecycle projector then enforces the complete-set causal rules, including:

- unique Registered root;
- parent presence;
- full reachability;
- unique child at each lifecycle step;
- Ready -> Claim ordering;
- exact claim/attempt binding;
- no transition after terminal state; and
- fail-closed fork semantics independent of DHT arrival order.

Therefore `Claimed` event qualification proves current executor + domain + effect-safety authority; the projector proves that the claim is the unique child of the verified Ready state. The event verifier MUST NOT recursively call the lifecycle projector to prove its own parent.

## Event authority classes

In v0.1:

- `Registered` is executor-authored and receives no authority merely from registration;
- `ReadyAuthorized` is executor-authored and must bind the stable current preflight identity;
- `Claimed` is executor-authored and requires current effect-safety qualification;
- `Completed`, `Failed`, and `Uncertain` are executor-authored and require independent outcome evidence;
- `Cancelled` requires an independent governed review/cancellation authority.

Phi, consciousness metrics, reputation, stake, model output, caller identity alone, Guardian status alone, DHT record existence or caller-provided `verified=true` values cannot qualify any binding lifecycle event.

## Effect safety

A requested safety policy is not proof that its mechanism exists.

The safety provider must positively verify that the exact committed policy is enforceable for the exact plan, executor and operation/effect class. `automatic_effects_allowed=false`, stale policy, unavailable fencing/idempotency or any mismatch denies domain resolution/claiming.

The lifecycle verifier itself exposes no effect executor.

## Packaging state

This tranche intentionally compiles the lifecycle verifier but does not package it into the binding governance DNA.

The provider zomes above are also intentionally not silently provisioned by this tranche. The result is a reviewable/compilable composition contract while authoritative lifecycle projection and claiming remain unavailable in the checked-in deployment state.
