# FSRE interval flow-balance reference

A dependency-free Rust reference evaluator for a narrowly declared wastewater accounting boundary. It evaluates six terms over one service/geography/time interval:

`G + I = T + B + R + ΔS + U`

where generation (`G`) and other inflows (`I`) are reconciled against treated volume (`T`), bypass/overflow (`B`), reuse or export (`R`), storage change (`ΔS`) and an unaccounted residual (`U`).

## Safety and epistemic rules

- Every required term must be present exactly once.
- Every term binds to the same time basis, geography, start/end interval and unit.
- Known terms carry lower and upper bounds; a point value is represented by equal bounds.
- Unknown terms have no numeric bounds and block residual calculation. Unknown never means zero.
- A not-applicable term requires an explicit `[0,0]` interval and a boundary rationale. It is a declared structural zero, not missing data.
- Physical flow terms cannot have negative values. Storage change may be signed.
- Negative residuals are preserved as accounting mismatch; they are never clipped.
- A positive residual is not attributed to leakage, illegal discharge, pollution, theft or a specific asset/operator.
- The output has no authority or actuation path.

## Interval semantics

For interval inputs, the evaluator computes a conservative outer interval:

`U_low = G_low + I_low - T_high - B_high - R_high - ΔS_high`

`U_high = G_high + I_high - T_low - B_low - R_low - ΔS_low`

This interval is conservative without an independence assumption, but may be wide. The report emits the correlation-group labels for each source family and explicitly states that dependence was not modelled; it does not perform joint uncertainty propagation, probabilistic inference or covariance estimation. Use the owner-native model (Symtropy) for richer physical and joint-uncertainty analysis.

The sign classification is descriptive only:
- wholly positive: positive unaccounted residual under this declared boundary;
- wholly negative: negative accounting mismatch;
- exact zero: zero residual within the declared input intervals;
- straddles zero: sign or zero is not identifiable.

## Lineage and claim ceiling

Each term carries source owner, reference, schema version, admissibility label, method, interval and correlation group. The evaluator does not authenticate any source, hash or reconcile source snapshots, verify instrument calibration, or prove that the declared boundary matches a real sewer catchment. It is not a hydraulic solver and does not estimate wastewater generation.

The fixtures are synthetic. A successful result proves only deterministic reference behaviour on the named inputs. The report always states `source_authenticity=NOT_INDEPENDENTLY_AUTHENTICATED`, `physical_model_qualification=NOT_ASSESSED`, `intervention_authority=NONE` and `operational_actuation=NOT_AUTHORISED`.

## Run

```sh
cargo test --locked --offline --manifest-path tools/fsre-flow-balance/Cargo.toml
cargo run --locked --offline --manifest-path tools/fsre-flow-balance/Cargo.toml -- \
  tools/fsre-flow-balance/fixtures/synthetic-balanced.psv
```

The reference currently has 14 unit tests, including 64 endpoint combinations for one interval fixture. Tests cover unknown-vs-zero handling, signed residuals, sign ambiguity, scope mismatch, malformed admissibility, order invariance and structural-zero boundary rationale. The path-specific workflow runs formatting, tests, golden CLI comparisons and Clippy. A queued or skipped workflow is not a pass.
