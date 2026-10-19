# Mycelix Integration Capability Policy

This crate defines the semantic bridge from an external integration operation to one exact institutional `CapabilityId`.

It intentionally separates four facts:

```text
operation -> capability semantics
!= immutable policy-record authenticity
!= institutional adoption
!= generation-bound currentness
```

The current tranche closes the first three only.

## Policy identity

`IntegrationCapabilityPolicy` binds:

- institution and optional jurisdiction;
- exact rulebook identity/version/digest;
- external system;
- operation kind;
- semantic profile;
- side-effect class;
- optional target object type;
- exact required capability;
- semantic validity window;
- exact institutional adoption authority reference; and
- exact adoption-proof reference.

Connector instance is intentionally absent. Connector deployment/provider semantics belong to INT-04's provider-profile theorem, while this policy defines institution-level operation meaning.

## Independent proof domains

`VerifiedIntegrationCapabilityPolicyRecordProof` proves evidence about one exact immutable record identity.

`VerifiedIntegrationCapabilityPolicyAdoptionProof` separately proves evidence that the institution/rulebook authority adopted that exact policy identity and proof reference.

Neither proof can substitute for the other.

Positive qualification returns a non-deserializable `QualifiedIntegrationCapabilityPolicy`.

## Exact command mapping

`qualify_command_capability()` additionally requires an exact typed `IntegrationCommand` to match the adopted policy's:

```text
system
operation kind
semantic profile
side-effect class
optional target object type
```

It commits the command's canonical SHA-256 commitment together with the policy identity and capability.

A side-effect downgrade, semantic-profile substitution, operation substitution, target-type substitution, or cross-system target fails closed.

## Deliberate remaining blocker

The shared authority-freshness vocabulary currently has stable subject codes 1-9 and no integration-capability-policy subject kind.

This crate therefore does **not** fake currentness by reusing `AuthorityCoveragePolicy`, `EffectSafetyPolicy`, or another unrelated subject kind.

A child tranche must add a new stable shared subject code and then run the existing closed-set `qualify_current_freshness()` theorem over the exact policy identity.

Until then:

```text
QualifiedIntegrationCapabilityPolicy != CurrentIntegrationCapabilityPolicy
QualifiedIntegrationCapabilityMapping != CurrentExecutionAuthority
AdoptionEvidence != GenerationCurrentness
CapabilityLabel != ExecutionPermission
```

Both positive objects permanently report `generation_currentness_verified_here() == false` and `grants_execution_authority() == false`.
