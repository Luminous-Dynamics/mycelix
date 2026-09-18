# mycelix-forge-execution

FORGE-004D1 defines a forge-independent contract for **evidence-bound constrained execution**. FORGE-004D1R refines its verification-time model before final M0 composition.

The contract exists because repository replay, build qualification, and release verification all have the same hidden-dependency problem: naming an input artifact is not enough if the verifier can silently depend on host tools, trust roots, environment, wall clock, home-directory state, or network access.

## Execution specification

`ExecutionSpec` binds:

- purpose and exact subject digest;
- exact tool artifacts, semantic versions, byte sizes, and optional derivation commitments;
- exact external trust material;
- exact input artifacts;
- canonical environment bindings;
- network policy;
- verification-time policy;
- filesystem policy.

Collections are canonicalized and reject duplicate semantic roles.

The execution-spec and execution-observation canonical domains are **v2**. This is intentional: the refined time semantics must not reinterpret an older v1 execution digest.

## Verification-time provenance

`VerificationTimePolicy` has four explicit modes:

- `NotUsed` — verification semantics do not depend on time;
- `FixedUnixSeconds(t)` — replay requires exactly the fixed instant `t`;
- `EvidenceDerived(commitment)` — time semantics are derived from separately authenticated evidence identified by the exact commitment;
- `HostRealtime` — ordinary ambient wall-clock execution.

The observation carries the corresponding typed `VerificationTimeObservation`. Strong qualification requires an exact match between policy and observation.

`HostRealtime` is representable for non-hermetic work, but it can never qualify as a hermetic candidate.

This matters for the Forge M0 profiles:

- the local SSH/GPG gittuf profile can use `NotUsed` instead of inventing a fake timestamp;
- a later Sigstore profile can use `EvidenceDerived(...)` to bind authenticated transparency/timestamp evidence;
- protocols that genuinely require a caller-selected instant can use `FixedUnixSeconds(...)`.

## Hermetic-candidate boundary

A structural hermetic candidate requires:

- `NetworkPolicy::Denied`;
- verification time other than `HostRealtime`;
- read-only inputs;
- ephemeral work directory;
- no host home directory.

This remains only a structural eligibility check. It does not prove that the kernel or executor enforced the requested policy.

## Observation boundary

`ExecutionObservation` records:

- executor identity;
- exact execution-spec digest;
- exact subject;
- outcome;
- output digest;
- executor-evidence commitment;
- typed verification-time observation.

`qualify_evidence_bound_execution` can produce `EvidenceBoundExecution` only when all exact bindings match and the specification is a hermetic candidate.

That positive type is deliberately **not** proof that the sandbox was actually enforced. FORGE-004D2 provides the separate isolation, runtime-closure, and verifier-trust evidence required before Forge may compose this result into `OfflineEvidence`.

## Claim boundary

FORGE-004D1R proves deterministic v2 subject formation and structural matching between an execution request and an executor observation, including time provenance. It does not prove sandbox enforcement, executable provenance, authenticity of evidence-derived time, trusted time, or network isolation by itself.

## Validation

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-features
```
