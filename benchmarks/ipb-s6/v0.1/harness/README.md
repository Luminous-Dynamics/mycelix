# IPB-S6 submission evaluator

Standalone Rust reference harness for comparing system submissions against the separate v0.1 oracle. It does not import Mycelix domain code and does not execute actions against assets or external systems.

Run from the repository root:

    cargo run --manifest-path benchmarks/ipb-s6/v0.1/harness/Cargo.toml -- \
      benchmarks/ipb-s6/v0.1/cases.json \
      benchmarks/ipb-s6/v0.1/oracle.json \
      path/to/submission.json

The submission schema is in submission-schema.json. Each submission must identify the system, build, configuration and adjudicator. The harness compares action outcomes and claim dispositions, then reports false-clears, false-blocks, prohibited inferences, unauthorized mutations, blind retries after unknown outcomes and bitemporal history rewrites separately.

Extra claim dispositions are reported as unscored; the harness does not silently turn them into correct or incorrect results. The evaluator fails closed on missing side-effect observations and duplicate claim-dimension rows. It reports malformed claim and side-effect records instead of silently defaulting them to zero or overwriting them. Two focused Rust unit tests exercise the false-clear and unknown-outcome retry controls; additional tests cover duplicate claim rows and missing side-effect telemetry. The evaluator is a prototype and requires compilation, tests, schema validation and independent review before benchmark qualification. It is not a legal, safety or title evaluator. No sample system results are included; no performance claims are made.
