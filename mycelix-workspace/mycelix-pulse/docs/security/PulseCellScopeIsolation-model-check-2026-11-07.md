# Pulse per-cell capability isolation — bounded model-check receipt

**Result:** PASS for the stated finite capability-scope model and exact model-check run  
**Workflow:** [Pulse Authority Lifecycle Model, run #356](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42401329288)  
**Job:** `128489512197` — Safe model and negative-control counterexamples  
**Artifact:** [TLC logs, counterexamples and receipt](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42401329288/artifacts/13585841413)  
**Artifact ID:** `13585841413`  
**Artifact ZIP SHA-256:** `ea8374e31015be1d5b7f51b99e9046f335d3ab4cc89fd4ce4346785d3d63ee01`  
**Exact branch subject SHA:** `d206c74670475e2e8fcd9f720e0f804af7469733`  
**TLC:** 1.8.0 / build `2026.08.11.125311`  
**TLC JAR SHA-256:** `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f`

## Results

| Configuration | Result | States generated | Distinct states | Outcome |
|---|---|---:|---:|---|
| `PulseCellScopeIsolation.cfg` | PASS | 66,674 | 12,687 | State queue exhausted; no invariant error |
| `PulseCellScopeIsolation-cross-cell-mutant.cfg` | Expected counterexample | 119 | 95 | `NoCrossCellGrantBorrow` violated |
| `PulseCellScopeIsolation-function-scope-mutant.cfg` | Expected counterexample | 120 | 96 | `NoFunctionScopeWidening` violated |
| `PulseCellScopeIsolation-collateral-revocation-mutant.cfg` | Expected counterexample | 107 | 84 | `NoCollateralRevocation` violated |

The overall workflow completed successfully. Each negative-control step succeeds only when TLC exits non-zero and the exact intended invariant name is present in the corresponding log. A generic parser/configuration failure does not count as mutation detection.

## Encoded contract

The model has two target cells and two independent host grants for the same assigned caller and same read function:

- Grant A is bound to Cell A, Secret A and `read_message`;
- Grant B is bound to Cell B, Secret B and `read_message`.

The safe model verifies that:
- a grant for Cell A cannot authorize a call targeting Cell B;
- a read-only function grant cannot authorize `write_message`;
- the exact grantee, secret and private claim must match the selected grant;
- revoking Grant A does not remove Grant B, even though both grants share the same principal and function name;
- a successful host decision is backed by the grant identity and active-state observation recorded at the decision point.

## Negative controls

1. **Cross-cell borrowing:** the mutant ignores the requested CellId during grant lookup. TLC found a call to Cell B authorized using Grant A from Cell A, violating `NoCrossCellGrantBorrow`.
2. **Function-scope widening:** the mutant ignores the grant's explicit function set. TLC found a write call authorized using a read-only grant, violating `NoFunctionScopeWidening`.
3. **Collateral revocation:** the mutant deletes grants by the shared `(principal, function)` tuple rather than exact CapGrant identity. TLC found that revoking Grant A also removed independent Grant B, violating `NoCollateralRevocation`.

## Model correction included in the checked subject

The first attempted run exposed a faulty invariant in the model itself: it compared a previous authorization decision to the grant's *current* host-active set. That would retroactively invalidate a call after a later revocation. The checked subject instead records the grant's activity at the call decision point and asserts:

- a non-authorized decision has no grant requirement;
- an authorized decision has a concrete grant identity and that grant was active at that decision.

This is the correct temporal contract: later revocation affects future calls, not the historical authorization result of an already-decided call.

## Boundaries

This is a finite abstract model with two cells, two grants, one assigned caller, one outsider, two functions and a fixed finite operation set. It does not simulate Holochain's actual conductor implementation, capability-grant storage, CellId serialization, real signed zome-call parameters, source-chain signatures, cross-DNA admin APIs, network partitions or production revocation latency. It does not prove that the implementation under PR #4891 or the selected Holochain 0.7.1 target is correctly wired to this model.

**Claim ceiling:** bounded model-check evidence for the exact TLA+ source/configurations on branch subject `d206c74670475e2e8fcd9f720e0f804af7469733` using the pinned TLC build. It is not a real conductor/Sweettest result or production qualification.
