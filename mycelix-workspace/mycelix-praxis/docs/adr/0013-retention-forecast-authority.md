# ADR-0013: Retention Forecast Authority

- Status: Proposed
- Subject: `PRAX-RETENTION-001A`
- Scope: DNA-neutral deterministic retention forecast

## Context

The legacy adaptive integrity module contains Ebbinghaus/FSRS-inspired floating-point helpers that accept caller-supplied values such as `mastery_permille`, successful review count, difficulty, elapsed time, interval, and confidence and then emit retention/stability/retrievability predictions and schedules.

Those functions are useful experiments, but the resulting values do not currently bind:

- the exact evidence-derived capability projection used as input;
- the exact temporal anchor event;
- estimator identity/version;
- parameter digest;
- forecast generation time;
- exact requested horizons;
- scientific calibration status.

## Decision

Praxis introduces an authority-safe retention forecast lineage stacked on deterministic BKT.

The core theorem is:

```text
BKT advisory estimate
+ deterministic decay model
!= measured retention
!= capability truth
!= mastery
!= credential eligibility
!= trust
!= authorization
```

### Source binding

The v1 retention forecast accepts an exact `BktProjectionReceipt` and requires:

- `praxis:bkt-projection` / `integer-v1` estimator identity;
- structurally valid BKT advisory estimate;
- exact equality between the BKT estimate's input event ordering and receipt ordering;
- exact recomputation of the BKT parameter digest from its public parameter/encoding fields;
- exactly one target capability dimension;
- an exact provenance-complete anchor attempt whose event ID is the final BKT event;
- matching learner, capability, and dimension;
- anchor time no later than the source BKT projection;
- forecast generation time no earlier than the BKT projection or anchor.

This prevents a caller from handing the retention model an unbound `mastery=0.8` scalar and treating it as a sourced capability state.

### V1 model

The first model is intentionally named:

```text
estimator_id      = praxis:retention-forecast
estimator_version = integer-hyperbolic-v1
```

It uses integer arithmetic only.

The retention curve is:

```text
R(t) = H / (H + t)
```

where `H` is an estimated half-life in minutes. Therefore `R(H)=0.5` by construction.

`H` is derived from:

- a configurable base half-life;
- the source BKT estimate weighted by an explicit permille parameter;
- an optional bounded per-source-event support bonus;
- an explicit maximum half-life.

All model parameters are included in a BLAKE3 parameter digest.

### Scientific boundary

`integer-hyperbolic-v1` is a transparent heuristic baseline, **not** a claim that Praxis has scientifically validated an FSRS, Ebbinghaus, or individualized memory model.

This separation is intentional. Future calibrated models can coexist under new estimator versions without rewriting historical forecast semantics.

The BKT `confidence_permille` field is carried forward only as `source_support_confidence_permille`. The retention model does not relabel it as statistical certainty about future memory.

### Forecast horizon binding

Forecast horizons are explicit positive integer minutes after `generated_at`. Input order is canonicalized; duplicates and zero horizons are rejected.

The receipt records:

- learner/capability/dimension;
- source BKT estimator ID/version/parameter digest;
- admission profile ID/version;
- exact source event sequence;
- source BKT estimate/support metadata;
- exact anchor event/time;
- retention model ID/version/parameter digest;
- estimated half-life;
- current model-relative retention estimate;
- sorted forecast points;
- generation time.

## Authority boundary

A retention forecast explicitly reports:

```text
is_measured_retention() == false
grants_credential_authority() == false
grants_trust_authority() == false
grants_authorization() == false
```

A recommendation or review scheduler may consume the forecast only under its own named/versioned policy while preserving this provenance.

## Legacy compatibility

The existing floating-point retention/retrievability/scheduling helpers are not removed by this tranche. They remain legacy advisory algorithms for compatibility and reproducibility.

Do not migrate their outputs into the new receipt by inventing BKT source receipts or anchor-event provenance.

## Follow-up research

Future work may add separately versioned models for:

- empirically calibrated FSRS-style scheduling;
- personalized stability estimation;
- uncertainty calibration;
- dimension-specific forgetting;
- assistance/context effects;
- model comparison against held-out retention probes.

Those models require benchmark/calibration evidence separate from deterministic implementation correctness.

## DNA boundary

This ADR changes no adaptive Holochain DNA. Persistence/materialization remains downstream of the repository-level isolated-workspace migration authority and installed-DNA qualification chain.
