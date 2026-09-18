# GitHub Actions Observation Adapter V1 Invariants

This crate implements the provider-specific normalization boundary from EVIDENCE-CI-004B / #1638.

## Authority boundary

The crate does not call GitHub, authenticate REST responses, schedule workflows, select runners, access credentials, mutate repositories, or decide theorem PASS/FAIL.

```text
authenticated raw GitHub observation
+ preregistered GitHub workflow normalization profile
-> provider-neutral EVIDENCE-CI observation
```

The downstream pure core remains the owner of liveness and conjunctive theorem closure.

## No runner-start inference from timestamps

GitHub can expose a queued job with all of:

```text
status = queued
started_at = created_at
runner_id = 0
runner_name = ""
steps = []
```

Therefore V1 never treats `started_at != null` as proof of theorem execution or runner assignment.

Registered theorem-step status is the semantic input.

## Preregistered semantics

The adapter does not infer theorem gates from names containing words such as `test`, `verify`, `qualify`, or `check`.

A profile freezes:

- repository ID;
- workflow path;
- exact qualification head;
- job keys;
- exact dependency topology;
- exact `(step number, step name)` theorem selectors;
- exact infrastructure selectors used for failure classification;
- normalization profile revision.

Changing the profile creates a different normalization theorem.

## Dependency state

GitHub job payloads do not themselves encode the `needs` graph, so dependency classification requires preregistered topology.

```text
all required dependencies completed success -> EligibleForRunner
required dependency pending/running         -> WaitingOnRequiredDependency
required dependency failed/cancelled/etc.   -> DependencyFailed
required dependency skipped                 -> DependencySkipped
ambiguous/missing dependency state           -> Unknown/refusal
```

Dependency blockage is not runner starvation.

## Gate execution

Only preregistered theorem steps count.

A skipped step is not executed merely because GitHub reports the step object as `completed`.

```text
no theorem step executed       -> NoTheoremStepExecuted
some theorem work executed     -> SomeTheoremStepsExecuted
all registered theorem steps executed/completed
                              -> AllRegisteredTheoremStepsExecuted
ambiguous provider shape       -> Unknown/refusal
```

A completed-success job missing a registered theorem step is rejected rather than promoted to PASS-compatible evidence.

## Failure classification

Job conclusion alone does not establish semantic RED.

- a negative registered theorem step -> `RegisteredTheoremGate`;
- exact admitted `startup_failure` before theorem execution, or a negative preregistered infrastructure step before theorem execution -> `RunnerInfrastructureBeforeTheoremGate`;
- negative terminal result after theorem work where no registered theorem step failed -> `Unknown`;
- skipped/neutral results remain non-semantic and cannot become theorem PASS by themselves.

## Queue age

Queue age is operational metadata only.

V1 never uses queued-job `started_at` as the age origin.

For a root queued job, a profile may admit the exact GitHub job creation time as the operational origin.

For a dependent queued job, the origin is no earlier than both:

- the latest successful prerequisite completion; and
- the target job creation time.

If exact prerequisite completion timestamps are unavailable, queue age stays unknown.

## One-run boundary

Normalization produces one `WorkflowRunObservationV1` for one exact GitHub run ID.

The adapter has no API for stitching job evidence from multiple runs.

## Extra provider jobs

The V1 profile names the jobs whose provider semantics are normalized. Extra raw jobs may exist in a GitHub run; they do not become theorem evidence unless preregistered in the normalization profile and, downstream, in the required-job theorem manifest.

## Claim ceiling

A qualified V1 implementation may establish only provider-specific normalization equivalent to:

`QualifiedGitHubActionsObservationNormalizationV1`

It does not establish GitHub REST truth independently, runner trust, theorem PASS/FAIL, runner availability, cross-run conjunction, repository mutation authority, or deployment readiness.
