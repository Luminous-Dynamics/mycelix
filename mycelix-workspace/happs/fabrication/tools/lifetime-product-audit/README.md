# Lifetime Product Audit (draft)

Offline Rust audit tools for the proposed LKI-01 Lifetime Product Record and its versioned synthetic benchmark suites.

These tools are **draft engineering aids**. Software passes establish only that checked artifacts match local schemas, semantics and frozen synthetic oracles. They do not establish physical safety, appliance conformity, supplier qualification, user accessibility, buildability, durability, or whole-life superiority.

## Run from the Mycelix repository root

```bash
# Dependencies are pinned by Cargo.lock; use --locked for repeatable dependency resolution.
cargo test --locked --manifest-path mycelix-workspace/happs/fabrication/tools/lifetime-product-audit/Cargo.toml --all-targets

# Frozen v0.3 record/schema baseline.
cargo run --locked --manifest-path mycelix-workspace/happs/fabrication/tools/lifetime-product-audit/Cargo.toml -- \
  --schema mycelix-workspace/happs/fabrication/docs/LIFETIME_PRODUCT_RECORD_SCHEMA_V1_DRAFT.json \
  --record mycelix-workspace/happs/fabrication/docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json \
  --benchmark mycelix-workspace/happs/fabrication/docs/LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_3_DRAFT.json \
  --as-of 2026-10-19

# Additive v0.4 repairability extension; it does not replace or mutate v0.3.
cargo run --locked --manifest-path mycelix-workspace/happs/fabrication/tools/lifetime-product-audit/Cargo.toml \
  --bin repairability_extension_audit -- \
  --corpus mycelix-workspace/happs/fabrication/docs/LKI_01_REPAIRABILITY_BENCHMARK_EXTENSION_V0_4_DRAFT.json
```

Pass an explicit `--as-of YYYY-MM-DD` to the v0.3 audit so supplier-evidence freshness checks are reproducible. Its `--freshness-days` defaults to `30` and adjusts warnings for stale records marked `availability_confirmed`. The frozen SUP-002 case retains its own 30-day negative control.

## Current checks

- v0.3: JSON Schema Draft 2020-12 meta-validation, example-record validation, component/interface ID and reference integrity, module-fit arithmetic, supplier evidence freshness, release-gate semantics, measured-metric evidence, and 17 frozen known-answer scenarios.
- v0.4 extension: pinned parent/version metadata, shape checks, duplicate/unknown scenario rejection, 11 deterministic known-answer outcomes, and five explicit software benchmark targets.
- Repairability semantics: observed human-facing opening and fault-identification tasks are not inferred from a calculated index; repair timing separates diagnosis, active labour, parts/tool wait, verification, rework and total downtime; a repair requires scoped post-repair verification; recurrence and parts-continuity claims remain bounded by evidence.
- Both tools retain the distinction between target, calculation, simulation, measurement, independent review, and physical safety/conformity.

The reports include SHA-256 digests of audited input bytes. Canonical reproducibility claims should also pin the source revision, tool version, committed lockfile, Rust toolchain, and complete execution environment.

## Deliberate limitations

- Neither benchmark file is a physical test suite.
- v0.3 remains a frozen baseline; v0.4 is an additive extension with its own evaluator. Unknown extension scenarios fail closed rather than being skipped.
- The tools are offline and do not fetch supplier pages or promote marketing claims into verified inventory.
- The product schema remains a draft and must be reconciled with Mycelix's existing product revision, manufactured-unit, qualification, BOM, and repair/restoration owners before becoming canonical.
- Physical service trials, user studies, current South African parts availability, appliance safety/conformity, 50-year life, and whole-life environmental superiority remain unestablished. Passing synthetic software cases is not evidence of any of those claims.
