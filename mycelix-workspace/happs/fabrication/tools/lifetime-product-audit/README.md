# Lifetime Product Audit (draft)

Offline Rust audit tools for the proposed LKI-01 Lifetime Product Record and its versioned synthetic benchmark suites.

These tools are **draft engineering aids**. Software passes establish only that checked artifacts match local schemas, semantics and frozen synthetic oracles. They do not establish physical safety, appliance conformity, supplier qualification, user accessibility, buildability, durability, or whole-life superiority.

## Run from the Mycelix repository root

```bash
# Dependencies are pinned by Cargo.lock; use --locked for repeatable dependency resolution.
cargo test --locked --manifest-path mycelix-workspace/happs/fabrication/tools/lifetime-product-audit/Cargo.toml --all-targets

# Frozen v0.3 record/schema baseline.
cargo run --locked --manifest-path mycelix-workspace/happs/fabrication/tools/lifetime-product-audit/Cargo.toml \
  --bin lifetime-product-audit -- \
  --schema mycelix-workspace/happs/fabrication/docs/LIFETIME_PRODUCT_RECORD_SCHEMA_V1_DRAFT.json \
  --record mycelix-workspace/happs/fabrication/docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json \
  --benchmark mycelix-workspace/happs/fabrication/docs/LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_3_DRAFT.json \
  --as-of 2026-10-19

# Additive v0.4 synthetic repairability extension.
cargo run --locked --manifest-path mycelix-workspace/happs/fabrication/tools/lifetime-product-audit/Cargo.toml \
  --bin repairability_extension_audit -- \
  --corpus mycelix-workspace/happs/fabrication/docs/LKI_01_REPAIRABILITY_BENCHMARK_EXTENSION_V0_4_DRAFT.json

# Draft physical-trial capture schema and planned-only sample record.
cargo run --locked --manifest-path mycelix-workspace/happs/fabrication/tools/lifetime-product-audit/Cargo.toml \
  --bin repair_trial_record_audit -- \
  --schema mycelix-workspace/happs/fabrication/docs/LKI_01_REPAIR_TRIAL_RECORD_SCHEMA_V0_1_DRAFT.json \
  --record mycelix-workspace/happs/fabrication/docs/LKI_01_REPAIR_TRIAL_RECORD_EXAMPLE_V0_1_DRAFT.json
```

Pass an explicit `--as-of YYYY-MM-DD` to the v0.3 audit so supplier-evidence freshness checks are reproducible. Its `--freshness-days` defaults to `30`.

## Current checks

- v0.3: JSON Schema Draft 2020-12 meta-validation, example-record validation, component/interface ID and reference integrity, module-fit arithmetic, supplier evidence freshness, release-gate semantics, measured-metric evidence, and 17 frozen known-answer scenarios.
- v0.4 extension: pinned parent/version metadata, shape checks, duplicate/unknown scenario rejection, 11 deterministic known-answer outcomes, and five explicit software benchmark targets.
- Repair-trial record: draft 2020-12 schema validation, explicit null/reason treatment for unmeasured phases, traceable evidence digests, exact revision identifiers, successful-outcome post-checks, damage disclosures, and fail-closed unpowered-surrogate constraints.
- The companion [unpowered fixture and trial protocol](../../docs/LKI_01_UNPOWERED_FIXTURE_AND_REPAIR_TRIAL_PROTOCOL_V0_1_DRAFT.md) specifies a staged, adjustable, candidate-independent mockup plan. No fixture is fabricated or dimensionally verified.

## Deliberate limitations

- Neither benchmark corpus is a physical test suite; the planned example record is not a performed trial.
- v0.3 remains a frozen baseline; v0.4 is an additive extension. Unknown extension scenarios fail closed rather than being skipped.
- The trial schema does not establish that recorded evidence is authentic or that a dimension, measurement, human trial or physical test is technically valid. Hashes bind bytes, not truth.
- The tools are offline and do not fetch supplier pages or promote marketing claims into verified inventory.
- The product schema remains a draft and must be reconciled with Mycelix's existing product revision, manufactured-unit, qualification, BOM, and repair/restoration owners before becoming canonical.
- Physical service trials, user studies, local South African parts availability, appliance safety/conformity, 50-year life, and whole-life environmental superiority remain unestablished. Passing synthetic software cases is not evidence of any of those claims.
