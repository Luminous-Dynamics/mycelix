# Mycelix Integration Current Capability Coverage

This crate closes one semantic gap and nothing more:

> does the exact current executor designation carry the exact capability that the exact current institution-adopted integration policy says this exact command requires?

## Same-call executor proof

The function does **not** accept a detached capability label or a serialized current-executor projection.

It re-runs `qualify_current_executor_authority(...)` from the original threshold, grant, designation, lineage and three freshness receipts, then requires the resulting current executor identity to equal the current executor already bound into the exact integration action.

This blocks a caller from taking capability text from designation B while using current action authority from designation A.

## Exact joins

Positive coverage requires all of the following to agree:

```text
current action command commitment == current capability mapping command commitment
same-call current executor identity == current action executor identity
policy institution == designation institution == current action institution
policy rulebook == designation rulebook == current action rulebook
policy jurisdiction == designation jurisdiction
policy required CapabilityId == designation required CapabilityId
```

The current capability mapping itself retains its exact generation-bound policy qualification, so a mapping from policy generation N cannot be silently paired with current generation N+1.

## Conservative lease

Final reuse is bounded by:

```text
min(
    current action lease,
    current capability mapping lease,
    same-call current executor lease
)
```

## Deliberate non-authority boundary

`MatchedCurrentIntegrationCapabilityCoverage` is a local, non-deserializable positive object.

It may report:

```text
exact_current_action_bound_here == true
current_capability_policy_bound_here == true
executor_capability_matched_here == true
current_executor_requalified_here == true
```

It permanently reports false for:

```text
provider_profile_bound_here
attempt_bound_here
provider_payload_materialized_here
grants_execution_authority
```

Therefore:

```text
CurrentExecutorCapabilityCoverage != QualifiedProviderProfile
CurrentExecutorCapabilityCoverage != DurableAttempt
CurrentExecutorCapabilityCoverage != ProviderPayload
CurrentExecutorCapabilityCoverage != DispatchStarted
CurrentExecutorCapabilityCoverage != CurrentExecutionAuthority
```

The next integration composition should join this result with #618's exact attempt/provider-profile `QualifiedExecutionBinding`. That composition must still remain pre-materialization and pre-dispatch.
