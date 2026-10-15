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

Measure schema + semantic validation separately from the full mutation-control harness. Use seeded synthetic cases with 1,000, 10,000, and 100,000 flows; include linked balances, evidence references, interventions, and cohort impacts. On a pinned 4-vCPU/8-GiB reference runner, target p95 below 2 seconds at 1,000 flows, below 10 seconds at 10,000 flows, and below 60 seconds at 100,000 flows, with peak RSS below 1 GiB. Run each size at least five times after a recorded warm-up policy. Save raw durations and peak-RSS observations; report median/p95. These are goals awaiting a benchmark harness and baseline, not claims about current performance.

For payload boundary tests, preserve the current 16 MiB ceiling and exercise sizes immediately below, at, and above the limit. Hash and parse the same byte buffer. Continue rejecting duplicate keys, non-standard numeric constants, path traversal, absolute paths, symlink escape and envelope/payload disagreement.

## Scientific and decision benchmarks

- **Forecast calibration:** for each metric with sufficient held-out data (target at least 100 independent site/time cases), measure empirical coverage of nominal 80% and 95% predictive intervals and report binomial uncertainty; target coverage within ±5 percentage points of nominal when the sample permits meaningful assessment. Compare against a declared seasonal/persistence baseline; otherwise report calibration inconclusive.
- **Counterfactual credibility:** predeclare the estimand, treatment unit/assignment, comparison construction, primary outcome, time window, confounder strategy, interference/spillover assumptions, falsification checks, and analysis choices before examining outcomes. Design-specific review—not one universal sample size or p-value—determines adequacy.
- **Distributional coverage:** publish the population frame, included pseudonymous cohort references, known omissions, group-identification method, intersectional-review status, evidence and limitations. This is a transparency benchmark, not proof every affected population has been found.
- **Decision safety:** name critical rights/safety/service floors before ranking; show floor breaches and blocked evidence; retain a Pareto set instead of hiding trade-offs in one composite score.
- **Scale decisions:** simulated benefit alone is insufficient. Require field measurement, a credible counterfactual where feasible, cost/maintenance accounting, distributional impacts, unintended consequences and an accountable operating owner.

## Interpretation

A digest match proves byte identity relative to a declared digest, not producer authenticity or scientific truth. Schema validity proves contract conformance, not causal validity. Difference-in-differences arithmetic remains a descriptive interaction until a reviewable identification design supports a stronger claim. When evidence, calibration, cohort coverage, or falsification is inadequate, report **inconclusive/blocked**, not a fabricated numeric score.
