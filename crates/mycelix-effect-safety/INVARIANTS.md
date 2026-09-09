# Effect Safety Invariants

This crate qualifies one exact effect-safety policy against independent proof domains and one exact adapter release. It does not execute effects.

## Five facts remain separate

```text
semantic EffectSafetyPolicy
        !=
immutable policy-record authenticity
        !=
institutional adoption of that exact policy
        !=
generation-bound currentness
        !=
current adapter enforcement evidence
```

Positive qualification requires all five facts to agree exactly.

## Exact action and adapter binding

The policy commits:

- exact action class;
- exact action digest under `mycelix-governance-execution-authority-v1-blake3-exact-json`;
- exact adapter profile;
- exact adapter release digest/profile;
- exact institution, jurisdiction and rulebook; and
- exact semantic mechanism requirements.

The adapter qualification must name the same profile/release and must assert that it enforces the exact authorized action digest.

## Mechanism truth is not policy text

A policy requesting idempotency, attempt fencing, precondition fencing or compensation does not prove those mechanisms exist. The exact adapter qualification must independently report support for every mechanism the policy requires.

Automatic external effects are rejected unless:

- the policy itself requires idempotency;
- the policy itself requires attempt fencing;
- the adapter currently supports both; and
- the adapter attestation says external effects are currently enabled.

## Currentness

`EffectSafetyPolicy::subject_ref()` creates exactly one generic freshness subject:

```text
kind       = EffectSafetyPolicy
namespace  = institution
subject_id = policy_id
identity   = canonical policy digest/profile
```

Qualification delegates current-state closure to `qualify_current_freshness` with exactly that one subject. Revoked, superseded, missing, unexpected, ambiguous or stale evidence denies.

The freshness effective time may not predate the policy validity epoch.

## Proof origin remains outside the pure theorem

Record, adoption, freshness and adapter receipts are deserializable evidence-shaped contracts. Structural success cannot prove they originated from their designated native/live verifiers.

The positive object therefore permanently reports all proof-origin methods as false. A later same-invocation runtime boundary must obtain those receipts directly from the designated local/native sources.

## Lease conservation

The final qualification horizon is exactly the minimum of:

- semantic policy validity;
- record-proof validity;
- adoption-proof validity;
- generation-currentness lease; and
- adapter-attestation validity.

Unlike legacy compatibility projections, this ABI carries the explicit resulting horizon, so short verifier leases do not require rejecting a longer-lived semantic policy. They simply narrow reuse.

Adapter attestation reuse is hard-capped at five seconds. This is a reuse ceiling, not an atomicity theorem: adapter/deployment state can still change immediately after observation.

## Stable current qualification vs dynamic evidence

The stable current qualification digest commits:

- canonical policy identity; and
- generation-bound freshness identity.

Refreshing record/adoption/adapter proof instances without changing policy or current generation changes dynamic evidence identity but does not mint a new semantic/current authority domain.

## Still not effect permission

`QualifiedEffectSafetyPolicy` proves policy/current/mechanism agreement only. It does not:

- verify proof-receipt runtime origin;
- create or claim an execution attempt;
- acquire an idempotency key or fence;
- lock a precondition;
- bind final authority/coordinator deployment;
- dispatch an adapter call; or
- grant execution authority.

The positive type is serializable for audit but not deserializable as a positive object.
