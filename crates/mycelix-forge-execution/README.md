# mycelix-forge-execution

FORGE-004D1 defines a forge-independent contract for **evidence-bound constrained execution**.

The contract exists because repository replay, build qualification, and release verification all have the same hidden-dependency problem: naming an input artifact is not enough if the verifier can silently depend on host tools, trust roots, environment, wall clock, home-directory state, or network access.

## Execution specification

`ExecutionSpec` binds:

- purpose and exact subject digest;
- exact tool artifacts, semantic versions, byte sizes, and optional derivation commitments;
- exact external trust material;
- exact input artifacts;
- canonical environment bindings;
- network policy;
- clock policy;
- filesystem policy.

Collections are canonicalized and reject duplicate semantic roles.

A hermetic-candidate specification currently requires:

- `NetworkPolicy::Denied`;
- `ClockPolicy::FixedUnixSeconds(...)`;
- read-only inputs;
- ephemeral work directory;
- no host home directory.

## Observation boundary

`ExecutionObservation` records an executor identity, exact execution-spec digest, exact subject, outcome, output digest, fixed observed clock, and an executor-evidence commitment.

`qualify_evidence_bound_execution` can produce `EvidenceBoundExecution` only when all exact bindings match and the specification is a hermetic candidate.

That positive type is deliberately **not** proof that the sandbox was actually enforced. The observation may come from an untrusted executor. FORGE-004D2 must provide a qualified Nix/Spore executor and concrete network/filesystem isolation evidence before Forge may use this contract to mint `OfflineEvidence`.

## Why clock is explicit

Cryptographic verification can depend on certificate or metadata validity windows. A verifier environment is therefore underspecified if it pins binaries and trust roots but silently inherits host wall-clock time.

A fixed clock makes replay semantics explicit. Authenticating or witnessing that time is a separate evidence claim.

## Claim boundary

FORGE-004D1 proves deterministic subject formation and structural matching between an execution request and an executor observation. It does not prove sandbox enforcement, executable provenance, trusted time, or network isolation by itself.

## Validation

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-features
```
