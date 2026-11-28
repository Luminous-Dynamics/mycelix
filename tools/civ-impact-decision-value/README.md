# CIV-IMPACT-002 Decision-Value Harness

**Status:** first executable synthetic prototype; exact-head tests must pass before any qualification claim.  
**Parent:** [CIV-IMPACT-002](https://github.com/Luminous-Dynamics/mycelix/issues/5053)  
**Protocol:** [CIV_IMPACT_DECISION_VALUE_V0_1.md](../../docs/research/CIV_IMPACT_DECISION_VALUE_V0_1.md)

## Run

```sh
cargo test --manifest-path tools/civ-impact-decision-value/Cargo.toml
cargo run --manifest-path tools/civ-impact-decision-value/Cargo.toml -- evaluate /tmp/civ-impact-decision-value-report.json
```

The evaluator writes a JSON report containing individual scoring receipts, policy aggregates, regime-stratified aggregates, exact observation/truth digests, costs, regret, evidence requests and guardrail outcomes. The aggregate report schema is separate from the per-run receipt schema; the Rust tests validate the report, each policy/truth fixture and every emitted receipt.

## What's in this prototype

- 12 deterministic stress regimes × 8 seeds = 96 synthetic episodes.
- Seven policy arms: observation-rule baseline, forced-action negative control, always-defer negative control, query-all control, fixed-query control, value-of-information policy, and an unauthorized-requester negative control.
- 672 scheduled policy/episode runs with paired same-episode comparison to the observation-rule baseline.
- Four machine-validated Draft 2020-12 contracts for the policy-visible view, evaluator-only truth, per-run scoring receipt, and aggregate report.
- SHA-256 binding of canonical serialized policy input, evaluator truth, returned observation sets, and source code used by this single-file prototype.
- Regime-specific results, so distribution shift, stale/missing evidence, query-budget pressure and correlated sources cannot disappear inside one mean.
- Invalid evidence receipts are not added to the policy-visible observation set. Attempts at unauthorized acquisition are blocked and recorded.
- Twenty-two authored Rust tests cover schema validation, truth separation, budget-aware evidence-value gating, cost and paired-loss reconciliation, negative controls, distribution-shift hazards, and complete regime reporting.

## Separation rule

`choose_action(policy, &PolicyView, &queried_options)` has no evaluator-truth parameter, oracle input, outcome callback, or global truth state. The evaluation loop can use evaluator truth only to synthesize the response to a registered evidence request and then score the final decision. The scorer's truth digest is carried in a separate receipt field; it is never attached to the policy-visible view.

This is a useful software boundary, not proof against arbitrary malicious code or process-memory introspection. A production benchmark should run policy and scorer in separate processes/containers with distinct inputs and access controls, validate artifacts externally, and preserve an independent audit trail.

## Important interpretation limits

- The `value_of_information` arm is a deliberately simple reference heuristic, not the production Symthaea VoI planner. It queries a high-declared-reliability independent source when the view lacks fresh trusted evidence, the declared expected decision-loss reduction exceeds acquisition plus delay cost, the request fits budget, and source/rights constraints are satisfied. It does not yet estimate calibrated EVSI or optimize a calibrated decision-loss model. A negative control includes high-reliability evidence with too little declared decision value to pay for itself.
- The declared source-reliability values are synthetic fixture metadata, not empirically calibrated probabilities.
- Eight seeds per regime is a fast contract/smoke corpus, not an inferential sample-size claim. The 96 episode-level fixture cases are not independent real-world observations; this prototype publishes descriptive per-regime summaries only and computes no population confidence intervals or causal estimates.
- The network/bridge world is a tiny toy problem whose loss values, source accuracy and costs are synthetic fixtures. The result is not evidence of real-world logistics performance.
- The distribution-shift fixture intentionally returns a fresh-looking but incorrect status against a lower declared reliability expectation. A fixed-query control can trigger an unsafe action; the reference VoI policy rejects the request because its expected value/reliability gates fail. This illustrates a designed synthetic failure mode, not an estimate of real source error.
- The source digest identifies the exact `main.rs` source file used by this prototype, not a compiled executable or a dependency lockfile. Until a committed `Cargo.lock`, build metadata and binary digest are added, environment-level replay is incomplete.
- The paired end-to-end loss delta is descriptive. The `realized_end_to_end_loss_reduction_vs_comparator_microcredits` field compares the candidate with the observation-rule baseline on the same episode; it is not a causal estimate of value-of-information and does not justify generalization. The comparator digest binds a canonical baseline comparison core, not a complete baseline scoring receipt.
- The scorer's "oracle" is the minimum registered decision loss under synthetic ground truth, not an achievable policy and not a real-world baseline.
- No live evidence is fetched; no request can contact people, purchase data, reserve resources, dispatch transport or create external side effects.
- A schema-valid receipt and a reproducible synthetic win do not qualify economic benefit. Hosted CI status for the exact commit is a separate gate.

## Initial engineering hypotheses

These are targets to preregister for a later confirmatory profile, not standards or results from this toy harness:

- zero hard authority, rights, privacy or budget violations for the candidate policy on registered adversarial cases;
- all scheduled runs get a disposition; no deleting timeouts, invalid receipts or deferrals from the denominator;
- aspire to ≥10% lower mean decision regret than the strongest fair non-VoI baseline on held-out synthetic worlds, where the ratio is meaningful;
- no more than 2% relative regression in preregistered high-priority stress regimes where the metric has a valid denominator;
- compare selective risk only with coverage curves and end-to-end loss including query, wait, fallback and recovery costs;
- report each regime and tail outcome instead of choosing a winner from one composite score.

The candidate must not optimize these numbers post hoc. Register endpoint, loss units, cost model, source model, seeds/holdout, budget, uncertainty method and guardrails before confirmatory scoring.

## Methodological anchors

- [NIST TEVV-Athlon Framework for Evaluating AI Systems (2026 draft)](https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems) supports use-case-specific evaluation of intended outcomes and negative impacts.
- [Holistic Agent Leaderboard (HAL), Princeton / ICLR 2026](https://hal.cs.princeton.edu/) emphasizes cost-aware, third-party evaluation and focuses current work on agent reliability.
- [Conformal selective prediction with cost-aware deferral under distribution shift (Scientific Reports, 2026)](https://www.nature.com/articles/s41598-026-40637-w) is a relevant research anchor for jointly considering calibration, selective risk and deferral cost. Its medical results do not transfer to this logistics toy world.
