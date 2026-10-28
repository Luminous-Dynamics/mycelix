# CECB Housing Fixture Checker

Standalone Rust-first validator for the synthetic known-truth fixture used by the Capability-Economy Comparative Benchmark (CECB).

## Status and claim ceiling

This tool checks the *fixture's internal consistency* and generates a candidate-visible observation view. It does not execute economic policies, rank institutional profiles, validate the real world, or prove causal impact.

Its evaluator contains ground-truth expectations. Candidate policy code must consume the policy-view export, never the original fixture or validation report.

## Run

From this directory in a Rust toolchain environment:

```sh
cargo test
cargo run
cargo run -- --policy-view-out /tmp/cecb-policy-view-v0-2.json
cargo run -- --paired-plan-out /tmp/cecb-housing-paired-plan-v0-1.json
cargo run -- --fixture /path/to/CECB_HOUSING_KNOWN_TRUTH_V0_1.json
```

The default fixture is embedded at compile time using `include_str!`, so it is version-bound to the source tree. `--fixture` is intended for explicit mutation testing and alternate copies. A passing report is emitted as JSON to stdout. The candidate-view export uses schema `cecb.housing.policy_view.v0.2`, removes semantic case IDs and purpose prose, and emits only a `case_token` plus candidate-visible `observation`. Exit codes: `0` fixture passes, `1` consistency validation fails, `2` input or invocation error.

## Checks

- Frozen schema version, exact required case-ID set, and unique IDs.
- Required case sections and non-empty invariant lists.
- Per-case arithmetic/physical constraints for the ten canonical fixtures.
- Grantor/recipient transfer closure and refinancing decomposition.
- Housing suitability and maintenance capacity changes.
- Candidate observation fields are exact-allowlisted per case, including nested mirrored-source claim objects; new unreviewed fields fail closed.
- Candidate-observation separation also includes a recursive intersection check for field names shared with evaluator facts.
- Mutation tests deliberately break each case's known truth, inject oracle-only data, or add unregistered top-level/nested observation fields.

## Paired allocation-only design manifest

The `--paired-plan-out` command materializes `cecb.housing.paired_design.v0.1`. It expands six declared stress dimensions (supply, construction delay, income shock, observation quality, accessibility mix, maintenance backlog) into 216 configurations and crosses each with 30 deterministic paired development seeds and five allocation-rule arms (price, queue, lottery, need-priority, hybrid): 32,400 explicit run slots.

All arms in a configuration/replicate block receive the same seed identifier. This is necessary for pairing but is not, by itself, sufficient to guarantee common exogenous shocks if policy arms consume random draws differently. The Civilization Twin adapter must use event-keyed exogenous draws or separate named RNG streams for world shocks and policy-internal randomness, then test shock-schedule invariance across arms.

Each row begins with `status=not_executed` and `disposition=not_started`. The arms are marked `adapter_pending`. This artifact is an experiment schedule, not a simulator, score, result, or policy ranking. CI validates its schema, exact matrix size, unique run identifiers, one seed and one occurrence of each arm per paired block, and absence of accidental execution claims. The CI receipt schema is `cecb.ci_run_receipt.v0.2` and binds the paired-plan output digest. The factor levels are semantic labels only: `numeric_scenario_value_mapping=unresolved_labels_only`, Civilization Twin/profile bindings are pending, and `execution_authorized=false`. These statuses must change only with a reviewed adapter contract, not merely because the schedule can be generated.

The manifest includes a typed twelve-coordinate outcome contract with explicit units/aggregation and `unknown_not_zero` missing-data rules. It does not yet execute a run-result verifier, estimate effects, or define any aggregate welfare score.

The seed schedule is public and deterministic for development/replay. It is not cryptographic and must not be used to claim evaluator blindness. Any later confirmatory holdout requires evaluator-owned seed/token mappings and isolation.

## Known limitations

- The checker validates fixed v0.1 fixture expectations; it is not a general-purpose economic simulator or a generic JSON Schema engine.
- It has not been run against the Rust toolchain as part of authoring because runner/toolchain access was unavailable. A skipped GitHub Actions run must not be represented as a pass.
- The fixture is synthetic. No result from this checker may be cited as evidence that a particular financing, ownership, or allocation regime works better in a real community.
- The exported `case_###` tokens are currently assigned in fixture order; they reduce semantic-label leakage but are not cryptographic blinding. A research candidate with repository access can recover the fixture and token order. A competitive evaluation needs an isolated candidate process, a hidden evaluation set, and a per-run randomized token mapping held by the evaluator.
- The evaluator code is not a security boundary if candidate code can read the original fixture or share process memory. Keep evaluator and candidate execution separated in the future runner, and pass only the generated policy-view artifact across that boundary.

## Exact-run evidence receipt

The path-scoped GitHub Actions job creates a `cecb.ci_run_receipt.v0.2` JSON receipt and uploads it with the test logs, validation report, candidate policy view, and resolved `Cargo.lock` when available. It records the exact source commit, run ID/attempt, Rust/Cargo versions, SHA-256 hashes of the fixture and checker inputs, output hashes, and each qualification step's outcome. The artifact is designed to remain useful when a check fails; a missing artifact or a skipped job is not a passing result.

The receipt's evidence class is explicitly E0 synthetic-fixture contract validation. It must not be cited as a policy-performance result or as real-world impact.
