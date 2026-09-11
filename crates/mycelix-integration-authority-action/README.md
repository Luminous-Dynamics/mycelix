# Mycelix Integration Authority Action

This crate translates one exact typed `IntegrationCommand` into the exact compact JSON bytes consumed by the existing `mycelix-execution-action-digest` governance authorization profile.

It is intentionally a **translation theorem, not an authority source**.

```text
ProposalId + exact IntegrationCommand
  -> local canonical command commitment
  -> deterministic integration governance-action JSON
  -> mycelix-governance-execution-authority-v1-blake3-exact-json digest
```

The JSON contains the exact command ID, SHA-256 canonical command commitment, connector instance, external system, operation kind, target identity, side-effect class, idempotency key and semantic profile. The command commitment transitively binds the command payload's `CanonicalEncodeV1` bytes.

## Why this exists

The generic governance authority stack already authorizes exact action bytes. Integrations should use that shared action identity rather than inventing a second authorization hash or treating provider routing metadata as permission.

This bridge also avoids a hidden payload translation oracle: callers do not supply arbitrary governance action JSON. The bytes are generated locally from the typed command and frozen by regression tests.

## Explicit non-claims

```text
ExactActionDigest != VerifiedAuthorityOrigin
ExactActionDigest != CurrentExecutorAuthority
ExactActionDigest != CapabilitySemanticCoverage
ExactActionDigest != QualifiedProviderProfile
ExactActionDigest != CurrentExecutionAuthority
ExactActionDigest != ProviderPayload
```

`QualifiedIntegrationAuthorityAction` is not deserializable as a positive object and reports false for authority origin verification, current-authority verification, capability closure, provider payload materialization and execution authority.

## Next composition

A later child should re-run the existing non-deserializable `qualify_current_executor_authority(...)` theorem from its original threshold/grant/designation/lineage/freshness evidence, require the exact `ACTIONS_DIGEST_PROFILE_V1`, and require this locally computed digest to equal the exact threshold/designation action digest.

Capability semantics remain an independent gate: the designation's capability scope must be shown to cover the integration operation under an institution-adopted mapping. Hash equality alone must never close that semantic question.

Only after that theorem and INT-04's current provider-profile binding are both present should a native critical-section composer consider provider payload materialization.