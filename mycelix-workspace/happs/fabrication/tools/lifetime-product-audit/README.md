# Lifetime Product Audit (draft)

A standalone, offline Rust validator for the proposed LKI-01 Lifetime Product Record, its illustrative example, and the frozen adversarial benchmark corpus.

This tool is a **draft engineering aid**. A passing report establishes only that these local draft artifacts satisfy this tool's checks; it does not establish physical safety, appliance conformity, supplier qualification, buildability, durability, or whole-life superiority.

## Run from the Mycelix repository root

```bash
# Dependencies are pinned by the committed Cargo.lock; use --locked for repeatable resolution.
cargo test --locked --manifest-path mycelix-workspace/happs/fabrication/tools/lifetime-product-audit/Cargo.toml --all-targets

cargo run --locked --manifest-path mycelix-workspace/happs/fabrication/tools/lifetime-product-audit/Cargo.toml -- \
  --schema mycelix-workspace/happs/fabrication/docs/LIFETIME_PRODUCT_RECORD_SCHEMA_V1_DRAFT.json \
  --record mycelix-workspace/happs/fabrication/docs/LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json \
  --benchmark mycelix-workspace/happs/fabrication/docs/LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_3_DRAFT.json \
  --as-of 2026-10-19
```

Pass an explicit `--as-of YYYY-MM-DD` so time-sensitive supplier checks are reproducible. `--freshness-days` defaults to `30` and adjusts warnings for stale records marked `availability_confirmed`. The frozen SUP-002 known-answer case intentionally retains its own 30-day scenario rule and expired-quote negative control.

The tool writes a JSON report to stdout and exits:
- `0` when there are no errors (warnings may still be present);
- `1` when schema, record semantics, or benchmark checks fail;
- `2` for command-line, file, or JSON parsing failures.

The report includes SHA-256 digests of the exact schema, record, and benchmark bytes. Canonical reproducibility claims should also pin the tool version, dependency lock, Rust toolchain, and complete execution environment.

## Current checks

- Require module envelope + minimum fit allowance to fit within the actual bay clear envelope on width, depth and height; this is a basic dimensional check, not full CAD interference or service-path analysis.

- Validate the schema itself against JSON Schema Draft 2020-12 meta-schema.
- Validate the product record against that schema with format assertions enabled.
- Reject duplicate/empty component or interface IDs and unresolved interface references.
- Reject a component marked qualified without a qualification/sample-inspection record.
- Disclose future-dated supplier checks, stale availability confirmation, and expired quotes.
- Reject passing release gates without evidence; reject "not applicable" gates without a rationale; prevent records with unsatisfied gates from being marked released.
- Require evidence references for measured/bench-tested/independently verified metrics.
- Check the 17 frozen v0.3 known-answer scenario oracles against explicit deterministic rules.
- Retain the distinction between simulation, measurement, qualification, and physical product safety.

## Deliberate limitations

- The benchmark corpus is not itself a physical test suite.
- The evaluator covers the named frozen v0.3 scenarios; unknown scenarios fail closed instead of being skipped.
- The tool is not a purchasing system and never fetches supplier pages or promotes a marketing claim into verified inventory.
- The current schema is a draft and must be reconciled with existing Mycelix product revision, manufactured-unit, qualification, BOM, and repair/restoration owners before becoming canonical.
- Hosted Rust 1.96.0 run [39210363080](https://github.com/Luminous-Dynamics/mycelix/actions/runs/39210363080) compiled the prior formatted revision, passed all 16 tests, and returned PASS with 17 v0.3 scenarios and zero errors/warnings. Subsequent workflow/documentation changes must be validated on the exact current head. Do not treat a prior-head pass as evidence for a newer SHA. The lockfile is now committed; the exact-head workflow must pass with --locked before claiming lockfile-pinned reproducibility.
