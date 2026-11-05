# CIV-IMPACT Census Verifier (0.1)

This is a **small standalone Rust research tool** for the Mycelix CIV-IMPACT-001 benchmark protocol.

It is intentionally not a simulator, router, optimizer, economic model, or production adapter. It creates a deterministic paired-run schedule and validates the completeness and basic internal consistency of a result census produced by a separate evaluator.

## Commands

Generate the proposed initial synthetic schedule (100 scenarios × 30 seeds × 7 comparison arms = 21,000 scheduled runs):

```sh
cargo run --manifest-path tools/civ-impact-census/Cargo.toml -- generate /tmp/civ-impact-run 100 30
```

This writes `scenarios.json` and `schedule.json`. Both the generated scenarios and schedule are labeled synthetic-only. The command does not execute any of the scheduled arms.

Verify a results file produced by an external harness:

```sh
cargo run --manifest-path tools/civ-impact-census/Cargo.toml -- verify /tmp/civ-impact-run/schedule.json /tmp/civ-impact-run/results.json
cargo run --manifest-path tools/civ-impact-census/Cargo.toml -- analyze /tmp/civ-impact-run/schedule.json /tmp/civ-impact-run/results.json
```

The results file must use manifest version `0.1`, match the benchmark ID and scenario-set digest, bind to the SHA-256 of the exact pretty-serialized `schedule.json` bytes via `schedule_artifact_sha256`, and include one result per scheduled `run_id`. A result file generated for a schedule cannot be replayed against a changed schedule—even if the scenario-set digest stayed the same and only arms or implementation references changed. A complete example row has this shape:

```json
{
  "run_id": "syn-transport-0000:seed-0001:incumbent_baseline",
  "scenario_id": "syn-transport-0000",
  "seed": 1,
  "arm_id": "incumbent_baseline",
  "disposition": "completed",
  "metrics": {
    "observed_demand_units": 3000,
    "served_units": 2900,
    "compute_cost_microcredits": 10,
    "operational_cost_microcredits": 8,
    "capital_amortization_cost_microcredits": 12,
    "data_cost_microcredits": 5,
    "integration_cost_microcredits": 20,
    "human_review_cost_microcredits": 15,
    "coordination_cost_microcredits": 5,
    "failure_recovery_cost_microcredits": 0,
    "total_cost_microcredits": 75,
    "completion_latency_ms": 120
  },
  "failure_reason": null
}
```

The total is the sum of compute + operational + capital amortization + data + integration + human review + coordination + failure/recovery costs. The figures above are a format illustration only, **not benchmark data or a proposed result**. The `microcredits` labels are synthetic accounting units and must not be interpreted as dollars, tokens issued by Mycelix, or monetary value. An empirical profile needs a registered currency/unit and a defensible mapping of each cost category to measured or modeled data.

A non-completed run uses one of `invalid_input`, `timeout`, `non_converged`, `missing_output`, or `evaluator_error`, includes a non-empty `failure_reason`, and does not advertise complete metrics. Failed and incomplete runs must remain visible in the census.

## Paired descriptive analyzer

The `analyze` command validates the census first, then pairs the incumbent and full-stack runs by the exact scenario ID and seed. It reports mean per-run cost per demand unit and service rate, plus both the unweighted mean of per-run cost-per-served-unit ratios and the **pooled cost-per-served-unit ratio** (total cost divided by total units served). Cost-per-served metrics are withheld unless every expected pair is complete and neither arm has a zero-service pair.

If any registered incumbent/full-stack pair is incomplete, the analyzer reports the pair counts but withholds headline means and deltas instead of treating a complete-case subset as representative of the full schedule.

The command also computes a deterministic, regime-stratified scenario-cluster percentile resampling interval from 10,000 replicates, **only** when every incumbent/full-stack pair is complete, neither has zero service, and every baseline scenario cluster has positive cost. It resamples whole scenarios within each registered regime and keeps all seeds and paired arms in the selected scenario together; this avoids treating repeated seeds inside one scenario as independent sites.

These are **descriptive resampling intervals for the supplied synthetic scenario corpus**, not population confidence intervals, a preregistered significance test, a causal estimate, or an economic-benefit qualification. The interval seed is derived from the exact schedule and result bytes, so identical artifacts produce reproducible intervals. Incomplete pairs remain visible in `incomplete_pairs`; zero-service outcomes prevent cost-per-served-unit estimates and bootstrap intervals. The output contract is documented in [CIV_IMPACT_COMPARISON_REPORT_V0_1.schema.json](https://github.com/Luminous-Dynamics/mycelix/blob/research/civ-impact-v0-1-review/docs/research/schema/CIV_IMPACT_COMPARISON_REPORT_V0_1.schema.json).

## Comparison arms

The generated schedule includes:
- `incumbent_baseline`: placeholder for a real baseline; replace and pin the implementation before any confirmatory run.
- `symtropy_only`: single-component comparison.
- `full_stack`: Symthaea + Symtropy + Sol-Atlas + Mycelix.
- `without_symthaea`, `without_symtropy`, `without_sol_atlas`, `without_mycelix`: leave-one-out ablations.

Implementation references in this generated schedule are explicitly placeholders. They are not qualified implementation identities, and the schedule is not preregistered until those references, objective, budgets, metric, holdout, rights manifest, evaluator, guardrails and stopping rule are frozen through the protocol process.

## Determinism and integrity

- Scenario IDs, scenario parameter generation, run IDs, and ordering are deterministic for the same arguments and tool version.
- Scenario generation uses an explicitly named XorShift64 variant for reproducibility, not cryptographic randomness.
- The scenario-set SHA-256 hashes the exact compact JSON serialization produced by this implementation. Independent implementations must duplicate the serialization contract or compare the provided scenario file's exact bytes instead.
- The results manifest binds the exact schedule artifact SHA-256; verification and analysis recompute it from the exact schedule file bytes. The report also hashes the exact schedule and result file bytes.
- The verifier rejects missing/duplicate/unexpected run identities, mismatched schedule identities, missing failure dispositions/reasons, served demand above registered demand, absent completed-run metrics, and total-cost arithmetic that does not equal the sum of registered cost components.

These are integrity controls, not authentication. A party can rewrite an artifact and recompute its hash; immutable storage, signed or independently anchored preregistration, evaluator separation and access controls remain external requirements. The verifier also does not prove that an evaluator truthfully measured its metrics.

## How the synthetic stress parameters should be interpreted

Five balanced families are generated: nominal, demand shock, capacity loss, stale spatial data, and coordinator outage. The records are generic aggregate fixtures, not road geometries, route costs, observed capacities, real disruption probabilities or operational data. Scenario parameters are deterministic synthetic inputs, not forecasts of any actual network.

The tool does **not**:
- implement vehicle routing or call Symtropy;
- read or redistribute CVRPLIB, OSM, or other third-party datasets;
- model real service operations;
- decide that a measured difference is statistically reliable or establishes economic benefit; calculate confidence intervals, social welfare or ROI;
- authorize dispatch, reserve resources, issue payments or change system state;
- turn a valid JSON file, completed CI job, or hash into evidence of real-world truth.

## Existing work to compose, not duplicate

- [CIV-IMPACT-001 protocol and issue #5046](https://github.com/Luminous-Dynamics/mycelix/issues/5046)
- [Mycelix CIV-ECON-006 Civilization Twin](https://github.com/Luminous-Dynamics/mycelix/issues/4816)
- [Mycelix CIV-CORE-003 independent evaluator boundary](https://github.com/Luminous-Dynamics/mycelix/issues/4835)
- [Mycelix logistics proposal seam #5044](https://github.com/Luminous-Dynamics/mycelix/issues/5044)
- [Symtropy global transport benchmark PR #1662](https://github.com/Luminous-Dynamics/symtropy/pull/1662)
- [Symtropy resilience envelope #1660](https://github.com/Luminous-Dynamics/symtropy/issues/1660)
- [CIV-IMPACT paired comparison report schema](https://github.com/Luminous-Dynamics/mycelix/blob/research/civ-impact-v0-1-review/docs/research/schema/CIV_IMPACT_COMPARISON_REPORT_V0_1.schema.json)
- [CVRPLIB best-known-solution challenge](https://galgos.inf.puc-rio.br/cvrplib/index.php/en/bks_challenge/overview)

Do not benchmark against third-party instances until source identity, exact objective convention, license/usage terms, transformation lineage, and reference-solution semantics have been audited and pinned.

## Reproducibility status

- `rust-toolchain.toml` pins Rust `1.99.1`.
- Direct crate dependencies are version-pinned in `Cargo.toml`, including `jsonschema = 0.58.6` for Draft 2020-12 meta-schema and instance validation.
- A generated and committed `Cargo.lock` is still required to pin transitive dependency resolution before treating builds as fully reproducible or using the harness for confirmatory research. The current checkout cannot generate that lock locally because no Rust toolchain is installed in this authoring environment.

## Tests

The crate defines 30 unit tests covering the default 21,000-run schedule; balanced stress regimes; deterministic identities; complete paired coverage; missing/duplicate results; scenario and schedule digest tampering; cost reconciliation and demand bounds; paired descriptive metrics; reproducible scenario-cluster resampling; incomplete and zero-service outcomes; JSON Schema meta-validation; serialized result/report instance validation; Rust/schema field-set agreement; and bounded, modulo-bias-rejected random range sampling.

**Execution status:** tests have been authored but are not reported as run here. An exact-head `cargo test --manifest-path tools/civ-impact-census/Cargo.toml` result is required before claiming code-level qualification. Queued, skipped, cancelled, or absent workflow runs are not passes.
