# FSRE recovery-option qualification reference

A dependency-free Rust reference implementation that composes upstream owner-qualified dispositions for a declared scenario. It also emits a multidimensional, read-only summary of resilience-debt records. The v0.2 report includes the exact scenario digest, individual debt source references, predicate owner/schema/evidence/time references, and explicit `execution_authority=NONE` / `operational_actuation=NOT_AUTHORISED` fields.

## What it does

- Requires seven distinct predicates per recovery-option candidate: physical feasibility, spatial access, dependency independence, staffing, resources, authority and safety.
- Enforces the expected source owner for each predicate.
- Rejects future-dated evidence and invalid validity intervals.
- Treats evidence as stale when `valid_until_seconds <= evaluation_time_seconds`.
- Requires every predicate to be `QUALIFIED` and mobilisation to fit the declared scenario horizon before an option can be qualified.
- Preserves exact reason codes for unknown, not-independent, unavailable, expired, revoked, wrong-scope, out-of-domain, conflicting and missing evidence.
- Summarises debt items by dimension, status and currentness, with item and affected-service counts. It does not produce a scalar debt/resilience score.

## What it deliberately does not do

This crate does **not** calculate dependency closure or common-cause independence, inspect spatial graphs, simulate hydraulic/power/industrial processes, determine operator competence, authenticate upstream evidence, grant authority, execute actions, or prove service outcomes. It consumes already-qualified dispositions from Mycelix, Sol-Atlas, Symtropy and the operator.

The expected owner map is an integration contract for this synthetic profile:
- `physical_feasibility` → `symtropy`
- `spatial_access` → `sol-atlas`
- `dependency_independence` and `authority` → `mycelix`
- `staffing`, `resources` and `safety` → `operator`

Production integrations must bind to the canonical owner records, their schema versions, exact validity intervals and independently verifiable evidence lineage. The local string `evidence_ref` is an opaque locator, not authentication.

## Run

From the repository root:

```sh
cargo test --locked --offline --manifest-path tools/fsre-recovery-qualification/Cargo.toml
cargo run --locked --offline --manifest-path tools/fsre-recovery-qualification/Cargo.toml -- \
  tools/fsre-recovery-qualification/fixtures/scenario.psv \
  tools/fsre-recovery-qualification/fixtures/debt-items.psv \
  tools/fsre-recovery-qualification/fixtures/predicate-evidence.psv
```

## Input semantics

Inputs are strict pipe-delimited text (no quoting or embedded pipe characters). Each source begins with an exact header. The scenario file requires `scenario_id`, `scenario_digest`, `time_basis_id`, `evaluation_time_seconds`, `clock_uncertainty_seconds` and `horizon_seconds`. For the committed compound-water fixture, `scenario_digest` is SHA-256 over the documented newline-delimited canonical manifest fields (scenario ID, scenario kind, evaluation time, time basis, clock-uncertainty bound, horizon and ordered disruption tags). Each predicate record is bound to an exact service, scenario ID/digest and non-empty source schema version.

Debt rows retain source owner, opaque evidence reference, observation time and exclusive validity end. Predicate rows do the same for each option/predicate pair and include `service_id`, `scenario_id`, `scenario_digest` and `source_schema_version`. Every record carries a `time_basis_id` and bounded `clock_uncertainty_seconds`; these must be reconciled to the scenario's declared time basis by the canonical temporal owner before entering this reference evaluator. A time-basis mismatch, future/overlapping observation interval or invalid uncertainty bound is rejected. Freshness is conservative: the earliest possible `valid_until_seconds` must be later than the latest possible evaluation time. The validity end is exclusive. The evaluator does not perform clock-domain conversion or create a clock service; it consumes already-normalised assertions and their uncertainty bounds.

Missing predicate records are not silently defaulted; they generate `MISSING_PREDICATE` reasons. Duplicate option IDs, duplicate predicate rows, unknown status values, wrong owners and future timestamps are rejected as invalid inputs.

## Synthetic example

The fixture is entirely synthetic and represents no real utility, municipality, asset, route, workforce, supplier, authority grant or operational state. It contains:
- 3 service lanes in the related JSON research scenario;
- 14 debt records in 11 dimension/status/freshness groups;
- 7 candidate recovery options;
- one candidate that passes all seven predicates for the declared synthetic window;
- negative controls for common-cause dependence, missing staff, out-of-domain physical model, unknown resource/safety status, expired authority, stale spatial evidence and excessive mobilisation time.

The golden report is `fixtures/expected-report.txt`. It is checked by the path-specific workflow. The passing candidate demonstrates only that this deterministic evaluator accepts a synthetic record set meeting its declared rules. It is not a field-qualified recovery option.

## Qualification limits

The unit tests cover owner binding, wrong service/scenario/schema binding, unknown/expired/stale/missing dispositions, future evidence, time-basis mismatch, clock uncertainty, duplicate predicates, mobilisation limits and debt grouping. The CLI golden test checks deterministic end-to-end report output, including 14 debt records and all 49 predicate evidence references. None of these tests authenticates evidence, validates source systems, proves model adequacy or establishes physical safety.
