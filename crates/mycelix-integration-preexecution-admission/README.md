# Mycelix Integration Preexecution Admission

This crate composes the integration-plane facts that may safely exist **before** provider payload materialization or durable dispatch.

## Same-call composition

```text
MatchedCurrentIntegrationCapabilityCoverage
+ exact INT-03 ExecutionClaim
+ exact typed IntegrationCommand
+ QualifiedProviderExecutionProfile
+ current ProviderProfileTrustRoot
+ current time
    -> re-run qualify_execution_binding(...)
    -> exact command-commitment equality
    -> QualifiedIntegrationPreexecutionAdmission
```

A previously cached `QualifiedExecutionBinding` is deliberately **not** accepted as input. Provider profile generations and trust roots can rotate after an earlier qualification, so provider/claim compatibility is rerun in the admission invocation.

## What is established

The result joins:

- exact current institutional executor authority;
- exact current institution-adopted operation→capability semantics;
- exact capability equality with the current executor designation;
- exact INT-03 claim/attempt identity;
- exact typed command identity;
- exact current signed provider execution profile relative to the supplied current trust root;
- exact provider idempotency/reconciliation contract;
- exact materializer release commitment; and
- provider payload-size bound.

The command's canonical commitment is the join key between the authority/capability theorem and the same-call provider/attempt theorem.

## Three time boundaries remain distinct

The positive object records:

```text
authority_valid_until_ms           # exclusive
claim_lease_until_ms               # exclusive
provider_profile_valid_through_ms  # inclusive under INT-04 v1 semantics
```

These are not collapsed into a misleading single expiry because they do not have identical boundary semantics.

## Trust-root use-time rule

The supplied `ProviderProfileTrustRoot` proves provider-profile currentness only relative to that exact same-call configuration input.

The eventual materializer/effect-start boundary MUST recheck the then-current provider trust root. Therefore:

```text
provider_profile_requalified_here == true
provider_trust_root_origin_verified_here == false
reusable_without_provider_root_recheck_here == false
```

## Hard remaining boundary

This crate has no provider payload materializer, provider client, `DispatchStarted` call, coordinator/Admin mutation primitive, or external effect.

It permanently reports:

```text
provider_payload_materialized_here == false
dispatch_started_here == false
coordinator_update_excluded_here == false
grants_execution_authority == false
```

Therefore:

```text
PreexecutionAdmission != ProviderPayload
PreexecutionAdmission != DispatchStarted
PreexecutionAdmission != CoordinatorMutationExclusion
PreexecutionAdmission != ExternalEffect
PreexecutionAdmission != CurrentExecutionAuthority
```

The next native theorem must join this admission to the authority stack's durable attempt/exclusion boundary and recheck the provider trust root immediately before payload materialization/effect admission.
