# CivOS benchmark corpus validator

This is a Rust validation tool for the frozen, synthetic-only CivOS Multi-Agent
Correctness and Resilience corpus. It is intentionally not a scenario runner.

## Run

From the repository root:

```sh
cargo test --manifest-path tools/civos-benchmark-validator/Cargo.toml
cargo run --manifest-path tools/civos-benchmark-validator/Cargo.toml -- \
  docs/research/civos-multi-agent-scenarios-v0.json \
  docs/research/civos-multi-agent-scenarios-v0.schema.json \
  --receipt /tmp/civos-corpus-validation-receipt.json \
  --coverage docs/research/civos-multi-agent-coverage-v0.json \
  --coverage-schema docs/research/civos-multi-agent-coverage-v0.schema.json
```

For a comparison against a previously reviewed/frozen corpus, also pass
`--expected-corpus-sha256 <64-lowercase-hex-digest>`. A mismatch fails closed.

## Scenario-to-test traceability

The optional `--coverage` / `--coverage-schema` pair validates
`docs/research/civos-multi-agent-coverage-v0.json`, a separate 21-row manifest
bound to the exact corpus SHA-256. It records the owning domain, intended
execution profile, source-linked test candidates and remaining gaps.

- `unmapped`: no qualifying test reference is currently mapped.
- `candidate_partial`: cited tests cover a narrow invariant, not the whole
  scenario.
- `implemented_unqualified`: implementation/test references exist, but no
  exact-head execution receipt has qualified the scenario.

There is deliberately **no** `qualified` status in coverage v0.1.0.
Every mapping has explicit gaps; unmapped rows cannot contain test refs, and
mapped rows must cite a 40-character source commit and symbol. The validator
requires all 21 scenario IDs to appear exactly once and in corpus order, and it
fails if the manifest's corpus digest differs from the checked-in corpus.

Current manifest status is planning-only: 18 scenarios are unmapped and 3
have candidate-partial test mappings. These are traceability observations, not
pass rates. A later execution receipt must bind the scenario, exact source
head, test command, runtime tuple, deterministic seed, negative-control result
and state digests before a scenario can be called qualified.

## What is checked

- The schema itself validates as JSON Schema Draft 2020-12.
- The corpus conforms to the checked-in schema and to a closed Rust data model.
- The benchmark stays `proposed-unmeasured` until separate execution receipts
  justify another status; it is not possible to mark it `qualified` just by
  changing the JSON status string.
- IDs are unique, correctly shaped, and targets/expected actions/negative
  controls/observables are non-empty; hard gates require negative controls.
- Unknown applicability is not a pass, fixtures are synthetic-only, the clock
  is fixed/injected, and randomness is deterministic.
- The receipt binds the exact corpus bytes, schema bytes, validator source,
  manifest and dependency lockfile using SHA-256 and includes stable coverage
  counts by class and target.

The receipt always says `scenario_execution: NOT_EXECUTED` and leaves
`benchmark_result` null. This proves only that the *corpus contract* passed
validation; it does not prove the Mycelix implementation passes any of the 21
scenarios. Real comparisons require exact-head multi-agent execution, negative
controls, a pinned runtime tuple, reproducible seeds and reviewed receipts.

## Versioning

The validator deliberately freezes corpus version `0.1.0` and the current
21-scenario count. Extend or edit the corpus only with a reviewed contract
version change and a new digest; never overwrite a benchmark baseline silently.
