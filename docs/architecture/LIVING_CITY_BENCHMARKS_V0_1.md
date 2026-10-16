# Living-City benchmark contract v0.1

Status: **target specification, not measured results**. Benchmark output must identify exact commit, fixture generator/version and seed, dependencies, operating system/CPU/RAM, run count, raw observations, and normalized report digest. Queued CI is not a pass.

## Release correctness gates

1. Every mutation/parser control rejects for its intended reason and every positive control passes.
2. Every cross-reference used by a decision-grade record resolves; blocked/not-assessed remains distinct from numeric zero.
3. No descriptive interaction is labeled a causal effect by implication.
4. Every causal-effect claim has an eligible experimental/quasi-experimental design, resolved source evidence, an evidence-reviewed review receipt, a negative control, a distinct robustness/falsification check, and all checks consistent with declared assumptions.
5. A confirmed critical rights/safety/service-floor breach is never offset by a gain in another metric or cohort.
6. The same exact inputs and pinned dependency environment produce byte-identical normalized validation reports in repeated clean runs.

## Proposed engineering performance targets

Measure schema + semantic validation separately from the full mutation-control harness. Use deterministic generated synthetic cases with 1,000, 10,000, and 100,000 flows; include linked balances, evidence references, interventions, and cohort impacts. On a pinned 4-vCPU/8-GiB reference runner, target p95 below 2 seconds at 1,000 flows, below 10 seconds at 10,000 flows, and below 60 seconds at 100,000 flows, with peak RSS below 1 GiB. Run each size at least five times after a recorded warm-up policy. Save raw durations and peak-RSS observations; report median/p95. These are goals awaiting a benchmark harness and baseline, not claims about current performance.

For payload boundary tests, preserve the current 16 MiB ceiling and exercise sizes immediately below, at, and above the limit. Hash and parse the same byte buffer. Continue rejecting duplicate keys, non-standard numeric constants, path traversal, absolute paths, symlink escape and envelope/payload disagreement.

## Scientific and decision benchmarks

- **Forecast calibration:** for each metric with sufficient held-out data (target at least 100 independent site/time cases), measure empirical coverage of nominal 80% and 95% predictive intervals and report binomial uncertainty; target coverage within ±5 percentage points of nominal when the sample permits meaningful assessment. Compare against a declared seasonal/persistence baseline; otherwise report calibration inconclusive.
- **Counterfactual credibility:** predeclare the estimand, treatment unit/assignment, comparison construction, primary outcome, time window, confounder strategy, interference/spillover assumptions, falsification checks, and analysis choices before examining outcomes. Design-specific review—not one universal sample size or p-value—determines adequacy.
- **Distributional coverage:** publish the population frame, included pseudonymous cohort references, known omissions, group-identification method, intersectional-review status, evidence and limitations. This is a transparency benchmark, not proof every affected population has been found.
- **Decision safety:** name critical rights/safety/service floors before ranking; show floor breaches and blocked evidence; retain a Pareto set instead of hiding trade-offs in one composite score.
- **Scale decisions:** simulated benefit alone is insufficient. Require field measurement, a credible counterfactual where feasible, cost/maintenance accounting, distributional impacts, unintended consequences and an accountable operating owner.

## Interpretation

A digest match proves byte identity relative to a declared digest, not producer authenticity or scientific truth. Schema validity proves contract conformance, not causal validity. Difference-in-differences arithmetic remains a descriptive interaction until a reviewable identification design supports a stronger claim. When evidence, calibration, cohort coverage, or falsification is inadequate, report **inconclusive/blocked**, not a fabricated numeric score.


## Independent review separation

For contribution/causal-effect claims, the review receipt must resolve and must not reuse an outcome evidence reference. The causal-effect path also declares an independent-review role; contribution claims need independent or community review. This is a referential-integrity guard, not identity proof: deployments need authenticated reviewer credentials and accountable review procedures if reviewer independence itself must be verified.


## Executable benchmark harness

Run the harness using the same pinned environment as the schema workflow:

```sh
python scripts/living-city/benchmark_metabolism.py --sizes 1000,10000,100000 --repeats 5 --warmups 1 --runner-label "4 vCPU / 8 GiB pinned reference runner" --output /tmp/living-city-benchmark.json
```

Each sample runs in a fresh subprocess, first builds a larger synthetic-only fixture, then separately times schema validation and semantic validation. Fixture expansion and catalogue loading are excluded from the validation timing. Output includes raw sample measurements, median/p95 by size, process peak RSS, Python/jsonschema version, environment label, exact workflow commit when `GITHUB_SHA` is set, and a report digest. The label is user-declared rather than hardware-probed; record actual runner CPU and memory externally. A 1,000-flow smoke run is wired into the Living-City workflow. Full 1k/10k/100k benchmark remains an explicit command because it is more expensive than a correctness gate.

The harness validates only synthetic records and never emits predictions about Johannesburg. Timing comparisons are meaningful only on the same pinned reference runner; the public CI smoke result is not a performance qualification.


## CI benchmark evidence retention

The schema workflow writes the 1,000-flow smoke measurement to a JSON report, publishes the measured values and report digest in the Actions job summary, and uploads the report as a 30-day workflow artifact. The report is keyed to the pull-request head SHA, so timings should not be attributed to a different commit. A successful smoke step proves that the harness ran and emitted a report; it does not establish the 10k/100k performance targets or scientific calibration.


The scale fixture cycles through all existing synthetic flow shapes rather than multiplying one simple metric, and adds a synthetic cohort-impact/coverage record plus a descriptive-only causal-evaluation record. Each result reports the number of distinct metrics, flows with uncertainty, cohort impact/coverage records, causal-evaluation records, evidence references and conservation checks. It still makes no real city or cohort claim. Evidence references on synthetic flow rows intentionally remain empty; the benchmark is not a test of high-volume evidence-catalogue joins.

The Actions job summary exposes workload-composition counts beside timing and memory: distinct metric types, flows with uncertainty, distributional impact/coverage records, causal-evaluation records, and conservation checks. This prevents a fast measurement from being misread without knowing which validator paths the fixture exercised.
