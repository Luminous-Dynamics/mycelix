# Mycelix Integration Current Action Authority

This crate composes the deterministic integration governance-action bridge with Mycelix's existing non-deserializable current executor theorem.

It does not accept a `CurrentExecutorAuthorityProjection`, an executor-provider receipt, a caller-provided `current=true`, or an already serialized positive authority object.

Instead, one call receives the original:

- threshold authorization;
- authority grant;
- executor designation;
- direct/delegated lineage evidence;
- current grant freshness;
- current threshold freshness;
- current executor-designation freshness; and
- current time.

It re-runs `qualify_current_executor_authority(...)` itself, then requires exact three-way equality across the locally generated integration action, threshold authorization and executor designation for:

- governance proposal identity;
- `mycelix-governance-execution-authority-v1-blake3-exact-json` profile; and
- exact action digest.

## What becomes true

A `QualifiedCurrentIntegrationAction` may report:

```text
exact_action_authorized_here == true
current_executor_requalified_here == true
wire_projection_used_here == false
```

It retains the actual non-deserializable `QualifiedCurrentExecutorAuthority` object returned by the same-call requalification, plus the exact integration command commitment and action digest.

## What remains false

```text
capability_semantics_closed_here == false
provider_profile_bound_here == false
attempt_bound_here == false
grants_execution_authority == false
```

Exact action bytes and current executor authority are necessary but still not sufficient. The designation's capability label must independently be shown to cover the integration operation kind under a current institution-adopted mapping. This crate intentionally refuses to infer that mapping from names such as `create-transfer`, `payments`, or `finance`.

Provider-profile currentness remains the independent INT-04 theorem in `mycelix-integration-execution-binding`. Attempt identity/lease remains the INT-03 runtime theorem.

## Next gate

Add a generic current **operation-to-capability policy** with immutable policy identity, institutional adoption, revocation/currentness, and exact mapping from integration `(system, operation_kind, semantic_profile)` to `CapabilityId`.

Only after that mapping qualifies should a child combine:

```text
QualifiedCurrentIntegrationAction
+ QualifiedIntegrationCapabilityCoverage
+ QualifiedExecutionBinding
```

Even that child should remain pre-materialization until the native critical-section/effect-start theorem is composed.