# Qualification capsules (QCAP)

QCAP separates an immutable qualification theorem from the runner that executes it.

```text
qualification theorem != GitHub Actions workflow
qualification capsule = content-addressed theorem definition
runner adapter = execution mechanism
attempt receipt = one execution of the complete theorem
```

## Lineage

- **QCAP-001A**: runner-neutral capsule substrate, exact predecessor `4ab6084cbf6d11bb58ea7917b2006fb180a20908`.
- **QCAP-001A2**: execution-state and provenance hardening tracked by issue #1657.

A2 is a successor, not a reinterpretation of A1. Historical A1 schemas and canonical vectors remain unchanged.

## Capsule identity

Capsule identity remains v1 and is domain-separated:

```text
SHA-256("MYCELIX_QUALIFICATION_CAPSULE_V1\\0" || u64_be(len(canonical_json)) || canonical_json)
```

Canonical JSON forbids floats and non-string object keys, orders object keys by UTF-8 bytes, and requires semantic-set arrays to be sorted and unique where the manifest validator declares them sets. Gate scripts are referenced by SHA-256 and revalidated immediately before execution.

## A2 attempt state machine

Gate results are one of:

```text
GatePass
GateFail
RunnerInfrastructureFailure
GateNotRun
```

The receipt automaton is closed:

```text
all GatePass
    -> CompletedConjunctivePass

>=1 GateFail, no infrastructure failure
    -> CompletedConjunctiveFail

first RunnerInfrastructureFailure
+ only GateNotRun afterward
    -> RunnerInfrastructureFailure
```

`GateNotRun` is valid only as the suffix after the first runner failure. It is not theorem RED.

## Isolation

Every executed gate receives a fresh detached Git worktree at the exact `product_subject_sha`.

Tracked or non-ignored untracked mutation by an executed theorem gate converts that gate to `GateFail`. Mutated worktrees are discarded and never feed later evidence planes. The caller checkout is not moved.

## Timeout containment

On Linux, the runner enables child-subreaper semantics, starts each gate in a fresh process session/group, terminates the full group on timeout, reaps reparented descendants, cleans the isolated worktree, records `RunnerInfrastructureFailure`, and records later required gates as `GateNotRun`.

Other platforms must not claim Linux-equivalent descendant reaping unless separately implemented and qualified.

## Closed gate environment

The runner does not pass the ambient host environment through to theorem gates. Gates receive a small controlled environment (`PATH`, controlled `HOME`/`TMPDIR`, normalized locale/timezone, and QCAP variables). Unrelated host secrets/tokens are absent by default.

The resolved environment commitment remains a provenance claim bound into the attempt receipt; QCAP alone does not prove an untrusted runner told the truth about that commitment.

## Execution context v2

A2 uses execution-context format v2 and binds:

```text
runner_profile_ref
resolved_runner_commitment

toolchain_profile_ref
resolved_toolchain_commitment

environment_profile_ref
resolved_environment_commitment
```

The executing QCAP dispatcher verifies `resolved_runner_commitment` against its own bytes before theorem execution.

## Receipt v2

Attempt receipts use the domain:

```text
MYCELIX_QUALIFICATION_ATTEMPT_RECEIPT_V2\\0
```

and bind the exact capsule, repository identity field, product SHA, theorem identity/revision, attempt ID, execution context, ordered gate results, recomputed verdict, claim, and nonclaims.

Repository field equality and an observed configured GitHub origin are consistency checks, not independently authenticated repository provenance.

## Exit-code contract

```text
0   executed predicate PASS
10  executed predicate FAIL
20  runner/environment failure
other nonzero -> conservatively RunnerInfrastructureFailure
```

## Independent vectors

A1 vectors remain historical. A2 adds a separate v2 fixture and independent verifier under `qualification/vectors/`; the verifier deliberately does not import `qualification/runner/qcap.py`.

## Claim boundary

A2 is staged infrastructure only. It does **not** qualify FIN-ECO-002F or any other FIN-ECO theorem, does not permit cross-run stitching, does not establish GitHub runner availability, and does not establish independent cryptographic authenticity of runner, toolchain, environment, or repository origin.
