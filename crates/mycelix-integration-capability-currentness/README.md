# Mycelix Integration Capability Currentness

This crate closes generation-bound currentness for the institution-adopted operation-to-capability semantics introduced by `mycelix-integration-capability-policy`.

It reuses the shared authority freshness theorem rather than creating an integration-local revocation system.

## Shared subject identity

The shared `AuthoritySubjectKind` receives one additive code:

```text
1  AuthorityGrant
2  SigningPolicy
3  ThresholdAuthorization
4  ExecutorDesignation
5  EffectSafetyPolicy
6  Delegation
7  AuthorityCoveragePolicy
8  CoverageTrustContextPolicy
9  WitnessTrustPolicy
10 IntegrationCapabilityPolicy
```

Codes 1-9 are frozen. `IntegrationCapabilityPolicy` gets a distinct code so its current-state history cannot be confused with coverage, effect-safety, signing, or grant state.

The exact freshness subject is:

```text
kind       = IntegrationCapabilityPolicy
namespace  = policy.institution
subject_id = policy.policy_id
identity   = exact policy digest + POLICY_IDENTITY_PROFILE
```

## Currentness theorem

```text
QualifiedIntegrationCapabilityPolicy
+ exact VerifiedAuthorityFreshness
+ current time
    -> exact IntegrationCapabilityPolicy subject
    -> qualify_current_freshness([subject], [receipt], now)
    -> QualifiedCurrentIntegrationCapabilityPolicy
```

The current-state effective time may not predate the semantic policy's `valid_from_ms`.

Final reuse horizon is:

```text
min(
    adopted semantic/record/adoption qualification horizon,
    generation-currentness lease
)
```

A revoked, superseded, stale, wrong-identity, wrong-namespace, wrong-policy-ID, or ambiguous freshness subject therefore fails closed through the shared currentness machinery.

## Exact command mapping

`qualify_current_command_capability()` reuses the lower semantic command mapper and carries the generation-bound current policy identity into a new current mapping digest.

It proves only:

```text
this exact command
matches this exact operation semantic policy
whose exact generation is currently Active
```

It still does **not** prove that the current executor designation names the same `CapabilityId`, that the provider profile is current, that an attempt is reserved, or that an effect may start.

Therefore:

```text
CurrentCapabilityPolicy != CurrentExecutorCapabilityMatch
CurrentCapabilityMapping != QualifiedExecutionBinding
CurrentCapabilityMapping != AttemptAuthority
CurrentCapabilityMapping != ProviderPayload
CurrentCapabilityMapping != CurrentExecutionAuthority
```

The positive types are non-deserializable and permanently report `grants_execution_authority() == false`.
