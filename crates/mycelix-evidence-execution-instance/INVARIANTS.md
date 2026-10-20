# Provider Execution Instance V1 Invariants

This crate implements the identity-only waist of EVIDENCE-CI-004C / #1647.

## Identity theorem

A provider run identifier is not necessarily one execution. For GitHub Actions, reruns retain the run ID and increment `run_attempt`.

```text
provider profile
+ repository identity
+ provider run ID
+ known non-zero attempt
= ExecutionInstanceV1
```

Thus `(run 42, attempt 1) != (run 42, attempt 2)`.

## Identity only — no provenance relabeling API

This crate intentionally does **not** accept a bare workflow observation or qualification receipt and attach an attempt to it.

That would allow a caller to relabel the same attempt-unaware observation as attempt 1 or attempt 2 without proving where the attempt came from.

Provider-specific composition must instead establish attempt provenance from an authenticated provider observation, then compose that evidence with this qualified identity primitive.

```text
ExecutionInstanceV1
!= proof that arbitrary evidence came from that instance
```

## Unknown attempt

Unknown attempt identity cannot be normalized to attempt 1. Attempt 0 is invalid.

## Provider profile

Provider normalization/attestation profile identity is part of the execution identity. Same repository/run/attempt observed under a different provider profile is a different evidence context unless a separate equivalence theorem says otherwise.

## Independence from liveness semantics

Execution-instance identity does not depend on EVIDENCE-CI liveness/conjunction semantics. It is a standalone primitive.

A later composition layer may combine:

```text
qualified execution-instance identity
+ provider-authenticated attempt provenance
+ provider-neutral workflow observation
-> attempt-bound evidence
```

but this crate does not perform that composition itself.

## Authority boundary

No provider fetch, REST authentication, runner scheduling, theorem PASS/FAIL, cross-attempt composition, filesystem/process capability, or repository mutation authority exists here.

## Claim ceiling

A qualified V1 implementation may establish only:

`QualifiedProviderExecutionInstanceIdentityV1`

It does not establish that any particular observation/receipt came from that instance, authenticate provider truth independently, qualify theorem semantics, establish runner trust/availability, or authorize repository mutation.
