# Coverage Policy Freshness v0.1 — Normative Extension

Status: **additive subject-kind extension to `mycelix-authority-freshness`; no runtime provider and no effect authority**

This extension gives coverage-layer policy objects the same generation-bound revocation semantics already used by grants, signing policy, threshold authorization, executor designation, effect-safety policy, and delegation.

## Stable subject codes

`AuthoritySubjectKind` is part of canonical authority-freshness identity. Existing codes are frozen and MUST NOT be renumbered:

- `AuthorityGrant = 1`
- `SigningPolicy = 2`
- `ThresholdAuthorization = 3`
- `ExecutorDesignation = 4`
- `EffectSafetyPolicy = 5`
- `Delegation = 6`

This extension adds only:

- `AuthorityCoveragePolicy = 7`
- `CoverageTrustContextPolicy = 8`
- `WitnessTrustPolicy = 9`

Changing any existing code is a protocol-breaking change and is forbidden in v0.1.

## Why policy lifetime is insufficient

Semantic `valid_from` / `valid_until` windows do not model immediate revocation or supersession. A compromised source policy, context policy, or witness-trust policy may need to stop authorizing new challenges before its nominal expiry.

Current authority therefore requires both:

1. exact immutable policy identity; and
2. one exact current `AuthorityFreshnessSnapshot` for that policy subject.

A policy with a still-valid semantic lifetime but a current `Revoked` or `Superseded` freshness state MUST NOT authorize new live coverage.

## Exact policy identity

The freshness subject MUST use the exact canonical policy digest/profile produced by the owning policy kernel. A textual policy ID alone is never sufficient.

Therefore same-ID mutation, profile substitution, or different semantic bytes cannot preserve current authority.

## Generation semantics

Re-verification of unchanged current state may refresh proof metadata and lease horizons without changing the authority-generation identity.

An actual revocation, supersession, or later explicit reactivation changes generation/state and therefore changes the freshness identity.

Reactivation never erases the historical revoked generation.

## Intended live challenge set

Before a future `authority_state_context_policy_verifier` can return a positive challenge context, it must require current freshness for the exact policy objects involved:

- exact `AuthorityCoveragePolicy`;
- exact `CoverageTrustContextPolicy`; and
- when witness quorum is configured, exact `WitnessTrustPolicy`.

Direct-source mode MUST NOT invent a witness-trust freshness requirement when no witness-trust policy exists.

## Historical/live separation

Later policy revocation stops new live challenges and coverage qualification. It does not invalidate a historical proof that the policy was active at an earlier time.

Historical verification must use an as-of authority-state protocol; a historical snapshot may never be converted into current freshness.

## No authority by enum membership

Adding a subject kind does not itself create authority. Positive current authority still requires a verified source/projection and an `Active` generation through the freshness qualifier.

Code presence, policy existence, caller identity, DHT presence, timestamps, Phi, reputation, model output, Guardian labels, or non-empty proof strings cannot manufacture freshness.

## Runtime containment

This extension does not add or provision:

- an authority-state source;
- a coverage-policy registry;
- a context-policy verifier;
- a witness-trust verifier;
- a Holochain zome;
- a lifecycle claim; or
- any external effect path.

The follow-on runtime provider must consume the append-only covered state machinery from the authority-state source/coverage stack rather than invent a new mutable `current=true` flag.
