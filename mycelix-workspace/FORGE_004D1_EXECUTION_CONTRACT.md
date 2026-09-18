# Mycelix Forge FORGE-004D1 — Evidence-Bound Execution Contract

**Status:** implementation candidate  
**Depends on:** FORGE-004C portable repository closure  
**Role:** provider-neutral execution subject; no sandbox implementation

## Purpose

FORGE-004D1 separates the *requested verifier environment* from the *mechanism that enforces it*.

The same trust problem recurs beyond repository verification: future SLSA builders and release tooling also need exact tool, input, trust, environment, clock, filesystem, and network dependencies to be explicit. This tranche therefore introduces a reusable `mycelix-forge-execution` protocol crate instead of a gittuf-specific sandbox schema.

## Core theorem

An `EvidenceBoundExecution` means:

```text
one exact ExecutionSpec
        +
one exact successful ExecutionObservation
        +
matching subject/spec/clock/output
        +
an executor-evidence commitment
```

It does **not** mean the executor actually enforced the declared policy.

Observation is not authority and declared isolation is not proven isolation.

## ExecutionSpec

The v1 subject commits to:

- execution purpose;
- exact subject digest;
- exact tool artifacts;
- semantic tool versions;
- tool byte sizes;
- optional derivation commitments;
- exact external trust material;
- exact input artifacts;
- canonical environment bindings;
- network policy;
- clock policy;
- filesystem policy.

Tool, trust-material, and input collections use unique semantic roles. Environment keys are unique. All collections are canonicalized before hashing so input order cannot change the subject.

## Hermetic-candidate profile

A specification is merely a *hermetic candidate* when:

```text
network       = Denied
clock         = FixedUnixSeconds(T)
inputs        = read-only
workdir       = ephemeral
host $HOME    = hidden
```

This is a statement about requested policy, not proof of enforcement.

## Why time is a first-class dependency

Repository verification can involve certificate and metadata validity windows. A supposedly reproducible verifier that silently reads host realtime can change result without any source, binary, trust-root, or policy change.

FORGE-004D1 therefore makes clock semantics explicit. M0 hermetic verification uses a fixed Unix-second value. Whether that timestamp corresponds to a trustworthy external time source is a later evidence/witness claim.

## ExecutionObservation

The observation binds:

- executor name/version;
- exact `ExecutionSpec` digest;
- exact subject digest;
- success/failure outcome;
- output digest when successful;
- executor-evidence commitment;
- observed verification clock.

Deserialization re-runs constructor validation.

`qualify_evidence_bound_execution` rejects:

- non-hermetic-candidate specs;
- wrong spec digest;
- wrong subject;
- failed execution;
- missing output;
- missing executor evidence;
- fixed-clock mismatch.

## FORGE-004D2 handoff

A concrete Nix/Spore executor must later establish that the observation is trustworthy. At minimum it should bind:

- exact Nix derivation / store paths for Git and gittuf;
- exact repository bundle and manifest inputs;
- exact external trust-root artifacts;
- empty/controlled environment;
- read-only input mounts;
- isolated ephemeral work directory;
- hidden host home directory;
- denied network namespace;
- fixed clock mechanism or equivalent deterministic time injection;
- executor implementation identity;
- evidence that these controls were actually active.

Only that qualified-executor layer may combine an 004C `PortableRepositoryReplay` with 004D1 execution evidence to produce protocol-level `OfflineEvidence`.

## Claim boundary

FORGE-004D1 establishes deterministic execution subjects and evidence-bound structural qualification. It does not establish sandbox enforcement, executable provenance, trusted time, filesystem isolation, network isolation, or correctness of an executor implementation.
