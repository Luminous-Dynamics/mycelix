# IPB-FINALITY Submission Evaluator

Standalone Rust evaluator for candidate submissions to IPB-FINALITY-001.

## Inputs

1. **Input-only manifest:** `docs/research/IPB_FINALITY_SCENARIOS.json`. This file is safe to provide to systems under test; it contains no oracle outcomes.
2. **Independent oracle:** `docs/research/IPB_FINALITY_ORACLE.json`. Do not provide this to the candidate system.
3. **Candidate submission:** validate against `docs/research/IPB_FINALITY_SUBMISSION.schema.json`; include one result per every scenario ID, including unsupported cases. A candidate cannot omit cases it does not support.

## Run

```sh
cargo fmt --manifest-path lise-benchmarks/ipb-finality-evaluator/Cargo.toml --check
cargo test --manifest-path lise-benchmarks/ipb-finality-evaluator/Cargo.toml
cargo run --manifest-path lise-benchmarks/ipb-finality-evaluator/Cargo.toml -- \
  docs/research/IPB_FINALITY_SCENARIOS.json \
  docs/research/IPB_FINALITY_ORACLE.json \
  submission.json
```

## Reporting

The evaluator reports case conformance, disposition and outcome-class accuracy, required-assertion precision/recall, high-severity false passes, positive-control false denials, unexpected provider dispatch, mutations under no-mutation controls, blind retries, missing effect/independent observations, operation substitution, stale authority, constitution/resource mismatches, delegation amplification, excess trajectory privilege, route bypass and missing clarification.

It emits SHA-256 receipts for the exact input manifest, oracle, submission and evaluator binary, plus the candidate's build/configuration/trace-bundle identities.

## Limits

The evaluator compares declared outputs and assertion references; it does not authenticate raw evidence or inspect a trace bundle automatically. A digest binds bytes, not truth. Independent reviewers must inspect the referenced traces, provider contracts, owner-native authority records and protected-effect observations.

The public 34-case development oracle is not a hidden holdout. Do not claim SOTA performance or production qualification from this evaluator alone.


## Reproducible comparison requirements

For each evaluation arm, preserve:
- system/build and configuration digests;
- model provider/name/version and reasoning settings when applicable;
- prompt and tool-schema digests;
- harness version and random seed;
- raw per-case trace references and SHA-256 digests;
- provider contract references where an external effect is possible.

Run at least three pinned comparator arms (for example, a fixed checklist/rule baseline, a configured policy/approval workflow, and an agent-based system) on the identical input panel and reviewer budget. Use a separately held, access-controlled confirmatory set before any SOTA or superiority claim.

The public development oracle is intentionally separate from the input manifest but is not a secret holdout. Evaluators must not supply it to candidate systems.
