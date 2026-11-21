# Event-aware seasonal GHG coverage auditor

A standalone Rust auditor for synthetic campaign schedules and event-aware GHG flux coverage. The input envelope is specified by `research/terra-preta/schemas/seasonal-ghg-campaign.schema.json`; the output report is specified by `research/terra-preta/schemas/seasonal-ghg-coverage-report.schema.json`.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/seasonal-ghg-coverage/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/seasonal-ghg-coverage/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/seasonal-ghg-coverage/Cargo.toml --all-targets -- -D warnings
cargo run --release --manifest-path research/terra-preta/tools/seasonal-ghg-coverage/Cargo.toml -- \
  research/terra-preta/tools/seasonal-ghg-coverage/fixtures/synthetic-seasonal-ghg-campaign.json \
  /tmp/seasonal-ghg-coverage-report.json
```

## v0.1 behavior

- Validates a predeclared UTC integration window, expected schedule slots, maximum gap and boundary tolerance.
- Validates predeclared event windows and explicit before/after observation roles.
- Binds observations to schedule slot IDs and exact times; rejects duplicate or mismatched observations.
- Rejects mixed gas species / mass basis / units.
- Treats left-censored non-detects as censored observations, not zero.
- Treats flagged/rejected observations as unusable for integration.
- Integrates only between adjacent expected schedule slots when both observations are usable and their gap is within the maximum.
- Missing/invalid scheduled slots break integration segments; the algorithm never bridges over a missing schedule slot.
- Reports partial integral, covered support, uncovered duration, schedule/event coverage, gaps and unqualified reasons.
- Output is create-new; existing report files are never overwritten.

## Qualification boundary

Even if schedule and event coverage are complete, v0.1 returns `coverage_complete_uncertainty_unqualified`. It deliberately does not propagate correlated measurement, sampling, event or model uncertainty. It therefore never emits a qualified seasonal/annual GHG total.

It does not infer missing flux, apply a censoring likelihood, extrapolate leading/trailing coverage, estimate treatment effects, provide net carbon accounting, or qualify physical field measurements. A missing high-risk event window yields `incomplete_event_coverage`; the partial integral covers only adjacent observed schedule segments.

The synthetic fixture deliberately omits a post-rainfall slot and campaign endpoint. It is not empirical evidence.
