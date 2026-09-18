# mycelix-forge-hermetic-run-evidence

FORGE-004D3B1 defines the **same-run orchestration evidence contract** for M0 hermetic verification.

A qualified sandbox policy is not enough by itself: isolation evidence collected from one process must not be reusable to bless a different verifier execution. D3B1 closes that composition boundary before the concrete Linux launcher is implemented.

## Run plan

`HermeticRunPlan` binds:

- exact v2 execution-spec digest;
- exact Linux isolation-policy digest;
- exact Git object-validation policy digest;
- exact 32-byte run-challenge artifact digest.

The challenge is an ephemeral per-run correlation artifact. Its bytes are later mounted read-only through the same Linux isolation policy and carried as an exact execution input.

## Required phase order

A positive run must contain exactly this sequence:

1. `PreRuntimeClosure`
2. `GitObjectValidation`
3. `PolicyTrustQualification`
4. `IsolationProbe`
5. `VerifierExecution`
6. `IsolationQualification`
7. `PostRuntimeClosure`

The distinction between steps 4 and 6 is intentional. The inside/parent isolation observations can begin while the bubblewrap child is held, but `QualifiedIsolationEvidence` includes the child's final exit status and therefore cannot honestly exist until after verifier execution and child termination.

Missing, duplicated, or reordered phases fail closed.

The preflight and postflight runtime-closure evidence commitments must be identical. This turns tool/runtime mutation during the actual verification interval into a qualification failure.

## Process-instance binding

`HermeticRunObservation` also records:

- bubblewrap child PID for diagnostics;
- bubblewrap status commitment;
- whether a pidfd/stable process handle was acquired before releasing the child;
- whether the child remained blocked while parent-side process/namespace observations were captured;
- final child exit code.

The PID itself is **not** treated as a security identity. FORGE-004D3B2 must acquire a Linux pidfd while the bubblewrap child is held on its block FD and retain that stable handle through observation/release/wait.

## Positive type

`QualifiedHermeticRunEvidence` exposes the exact evidence commitment for every required phase plus the aggregate same-run evidence commitment.

It does not launch a process or independently prove the booleans reported by an arbitrary producer. D3B2 is the concrete qualified launcher implementation that must establish these observations using bubblewrap status/block FDs, pidfd process handles, the existing isolation collector, and exact tool/input artifacts.

## Claim boundary

D3B1 prevents receipt-order and cross-run ambiguity at the protocol layer. Final repository `OfflineEvidence` is still minted only by FORGE-004D3A after it cross-checks this run evidence against the portable replay, execution subject, Linux isolation evidence, runtime NAR closure, and local-key trust profile.

## Validation

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-features
```
