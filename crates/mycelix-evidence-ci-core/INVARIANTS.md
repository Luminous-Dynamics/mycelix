# EVIDENCE-CI Core V1 Invariants

This crate is the pure semantic waist for EVIDENCE-CI-004 / #1612 and the runner-independent closure rule from EVIDENCE-CI-005 / #1614.

## Authority boundary

The crate has no GitHub API client, network, filesystem, process, token, runner, repository mutation, or publication authority.

It consumes already-authenticated provider observations supplied by an outer adapter and derives only execution-liveness / conjunctive evidence classifications.

```text
provider observation
!= theorem subject
!= theorem verdict
!= runner authority
!= repository authority
```

## Exact observation binding

A required-job manifest binds all of:

```text
repository_id
workflow_path
qualification_head
required theorem jobs
```

An observation from another repository or another workflow at the same Git head cannot satisfy the manifest.

## One-run conjunction

`derive_conjunctive_receipt_v1` accepts exactly one `WorkflowRunObservationV1`.

There is no API for combining successful conjuncts from multiple workflow runs.

```text
partial run A + partial run B != PASS
```

Whole-theorem failover creates a new execution lineage that reruns the complete theorem.

## Required-job manifest

Only preregistered required theorem jobs determine closure.

Optional presentation, summary, annotation, or Boolean-only verdict jobs are irrelevant unless the theorem manifest explicitly makes them required.

This is how future qualification profiles avoid consuming a new runner solely to compute `A && B` after A and B already finished.

## Queue age

Queue age is retained as operational metadata only.

Changing queue age cannot change PASS/FAIL.

A separately registered failover policy may use the age of the blocked **required theorem job** to decide whether a new whole-theorem execution lineage is operationally eligible.

Queue age never supplies a missing theorem result.

## Semantic failure versus infrastructure failure

A semantic FAIL requires an executed registered theorem gate classified as `RegisteredTheoremGate`.

A runner/environment failure classified as `RunnerInfrastructureBeforeTheoremGate` must have no theorem steps executed.

Contradictory observations such as `success + RegisteredTheoremGate failure class` or `infrastructure-before-gate + executed theorem steps` are rejected rather than interpreted conveniently.

Missing, skipped, queued, cancelled, dependency-blocked, or unknown execution cannot be silently upgraded to PASS.

## Runner adapters and failover

Runner profiles describe execution context only.

A runner change is failover-eligible only if the registered theorem harness fingerprint is unchanged across:

- theorem subject;
- predicates;
- command set;
- canonical oracle;
- object-identity profile;
- claim boundary.

V1 additionally requires these runner properties to remain identical across failover:

- network policy;
- immutable action pins;
- required tool identities.

A registered policy may permit OS-family, architecture, or cache-policy changes, but those changes are explicit rather than inferred.

`WaitingOnRequiredDependency` is not runner-failover eligible in V1: changing runners does not discharge a missing/failed dependency.

The policy also binds admitted source/target runner profile IDs, a minimum no-start age, and a maximum concurrent-attempt count to prevent reflexive queue flooding.

The first V1 implementation deliberately uses opaque IDs for already-frozen theorem/profile identities. It does not invent a second cryptographic commitment system; CORE-COMMIT-001 / #1292 may later provide typed commitment adapters.

## Self-hosted runners

A self-hosted runner may improve liveness but is not trusted merely because it is operator-controlled.

Host trust, environment closure, correlated-failure accounting, and attestation remain separate evidence theorems.

## Claim ceiling

A qualified V1 implementation may establish only pure classification semantics equivalent to:

```text
QualifiedExecutionLivenessClassificationAndRunnerIndependentConjunctionV1
```

It does not:

- qualify an underlying product/scientific theorem;
- guarantee runner availability;
- authenticate raw GitHub API data;
- prove provider independence;
- authorize repository mutation;
- permit cross-run conjunctive stitching;
- make queue age a theorem result.
