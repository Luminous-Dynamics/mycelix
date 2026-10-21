# NRVCA Validator

Research-only Rust validator and adversarial mutation runner for the NRVCA v0.1 report contract.

## Run from the repository root

    cargo test --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml
    cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- --mode all

The default paths are repository-root-relative. Override them with --fixture, --corpus and --schema if running from another directory.

Modes:
- all: check both report and benchmark-schema contracts, validate the golden report and the design-only benchmark preregistration, then execute every adversarial mutation.
- validate: schema-contract checks plus semantic validation of the golden report and benchmark preregistration.
- corpus: execute the adversarial mutation corpus against the golden report.

Optional paths: `--benchmark PATH` and `--benchmark-schema PATH` override the default preregistration record/schema.

## What this checks

- report status and period/jurisdiction essentials;
- source identity uniqueness and source-reference resolution;
- SNA-style account basis consistency;
- separation of financing, revaluation, secondary-market asset transfers and current production;
- unknown versus zero;
- unit consistency, canonical JSON-Pointer bindings, and independently reconstructed reconciliation identities;
- canonical stock-flow account closure and financial-event linkage;
- the v0.1 ban on adding incompatible units into a universal scalar;
- typed claim classification and the prohibition on promoting synthetic claims to observed or causal evidence;
- benchmark preregistration consistency, source-manifest/source-ID matching, source digest and licence gates, baseline/candidate arms, unique metrics, primary metric presence and retention of failed/unfavourable cases;
- evidence-tier / synthetic-to-real overclaim controls;
- every mutation is rejected in the expected structural or semantic diagnostic class.

## Limits

This is not a full JSON Schema Draft 2020-12 validator and does not prove SNA/SEEA conformity, source truth, data completeness, causal impact, or policy quality. It checks a documented subset of the contract. It has no Holochain, network, database, or runtime-service dependency. The golden fixture is synthetic.

Adversarial tests are qualified only when the Rust runner executes them successfully. A specification entry, a parsed JSON file, or an equivalent check in another language is not an executed Rust test. The claim checker uses typed claim fields; it does not attempt to infer truth from natural-language disclaimers.


The reconciliation contract binds every operand via `value_pointer` and each LHS via `lhs_pointer`; stock-flow accounts carry signed changes and reconcile independently. The validator is still a limited semantic checker, not a full JSON Schema or national-account standard validator.

A `BENCHMARK PREREGISTRATION PASS` only means the design record is internally consistent; the current South African run remains `blocked_missing_sources` until exact source bytes are pinned and rights are reviewed. It is not a benchmark result.
