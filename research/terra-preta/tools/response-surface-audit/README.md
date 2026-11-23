# Terra Preta response-surface audit

Standalone Rust utility for descriptive factorial dose-response auditing.

Run:

```sh
cargo test --manifest-path research/terra-preta/tools/response-surface-audit/Cargo.toml
cargo run --manifest-path research/terra-preta/tools/response-surface-audit/Cargo.toml -- observations.csv report.json 3 0.0
```

The CSV requires study, site, assignment-unit, block, soil source, material batch, treatment, dose and unit, root regime, wet-dry cycle count, antecedent state, forcing, endpoint, time point, allocation label, observation ID and outcome value.

Repeated sensor readings, cores, images, tracer samples and time points must share the true treatment-assignment unit. They are averaged within assignment unit and response cell before dose summaries. The report separates observation rows from independent assignment units, reports shared block and batch support, describes non-monotonic response reversals and compares root/cycle-conditioned dose slopes on common dose support.

The slope contrasts are descriptive, not causal or statistically inferential. Synthetic tests establish software behaviour only. This tool does not authorize experiments, recommend application rates or establish soil efficacy.
