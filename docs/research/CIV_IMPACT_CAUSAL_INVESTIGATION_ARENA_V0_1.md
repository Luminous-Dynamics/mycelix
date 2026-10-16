# CIV-IMPACT causal investigation arena v0.1

- Status: deterministic synthetic prototype; no model-agent or field evaluation claimed
- Owner: Mycelix CIV-IMPACT research
- Parent: https://github.com/Luminous-Dynamics/mycelix/issues/5080
- Composition: CIV-IMPACT-004 trial artifact verifier, decision-value PR #5054, adaptive challenge PR #5067

## Research thesis

A credible causal investigator must do more than produce a plausible explanation. It must choose interventions that distinguish hypotheses, recognize when confounding can reverse a conclusion, account for measurement cost, preserve null outcomes, and abstain when a valid estimate is unavailable.

This prototype defines a small, deterministic, known-truth structural causal model (SCM) arena with an independent scoring path. Five synthetic regimes exercise clean positive effect, hidden confounding, sign reversal, a null effect, and treatment-effect heterogeneity. Six strategies include randomized assignment, naive observation, measured-confounder adjustment, stratified randomization, a noisy-cue-guided design choice, and always-abstain.

## Paradigm shifts

1. **Prompt correctness -> investigation quality.** Score the experiment/design selected, estimation error, interval behavior, resource use, and whether the system makes an unsupported directional claim.
2. **Correlation leaderboard -> known causal truth.** Each regime has a specified data-generating mechanism. The evaluator calculates the cohort-average treatment effect from latent potential outcomes, independently of the estimate.
3. **Single mean -> error surface.** Publish every regime/strategy cell and aggregate error, sign recovery, 95% interval coverage, null-world directional claims, cost, identifiability, and abstention.
4. **Unlimited evidence -> costed measurement.** Randomization, confounder measurement, and passive observation have different synthetic costs. Report units are deliberately abstract cost units, not money or energy.
5. **Answer everything -> calibrated abstention.** Always-abstain is a negative control: low false-claim rate cannot count as success when estimability is zero.
6. **Published test -> diagnostic only.** The public source reveals the generator; results are useful for contract testing, not confirmatory performance claims. A future hidden lane requires evaluator-held seeds, a rotation policy, and independently anchored run receipts.

## Strategies

- randomized_naive: randomized treatment assignment; unadjusted difference-in-means estimator.
- observational_naive: assignment depends on a covariate; unadjusted estimator.
- observational_adjusted: same observational allocation mechanism, but the covariate is measured and the estimator is standardized over the enrolled cohort's strata.
- stratified_randomized: treatment randomized within measured covariate strata; standardized estimator.
- cue_guided: uses a noisy pre-registered confounding-risk cue; chooses stratified randomization on a high signal and cheap observation on a low signal.
- always_abstain: no causal estimate and a nonzero administrative cost; reports abstention explicitly.

The cue_guided signal is noisy and is not the hidden SCM parameter. The policy does not receive the true average effect or the evaluator's confounding coefficient.

## Metrics

- Cohort-average true effect vs estimate; signed and absolute error, with Monte Carlo standard errors across replications.
- Sign recovery for non-null effects, with Wilson 95% intervals over replications.
- Nominal 95% Wald interval coverage for the estimator's declared design, with Wilson 95% intervals to prevent small repetition counts from looking more precise than they are.
- Rate of a directional interval-excludes-zero claim in the null-effect regime, with Wilson 95% intervals.
- Cost units and nominal inverse-variance precision per cost unit; precision alone is never treated as causal quality.
- Estimability and abstention rates; every strategy/regime/replication row is retained.

Coverage is a diagnostic for these small, synthetic linear models. Wald intervals may be inadequate in small or badly imbalanced strata. The benchmark must not present their observed coverage as universal or substitute it for a design-specific inferential justification.

## Proposed targets (not achieved results)

- Exact deterministic replay of generated results for identical source/configuration/seed.
- Zero silent loss of any planned regime × strategy × seed cell.
- The sign-reversal regime must demonstrate how a naive observational estimate can point in the wrong direction, while randomized assignment recovers the declared cohort effect within a preregistered tolerance across sufficiently large repetitions.
- Measured-confounder adjustment should reduce observational bias under the SCM's stated conditional exchangeability/positivity assumptions.
- Null-world false directional claims should be measured and reported with Monte Carlo uncertainty; the zero-false-claim target applies only to hard invalid claims, not as an assumed finite-sample rate for valid 95% intervals.
- Always-abstain must never score well on utility without an explicit abstention penalty or estimability term.
- Any public point estimate is diagnostic. Confirmatory claims require fresh evaluator-held seeds, independent review, appropriate uncertainty analysis and exact-head CI.

## Commands

    cargo test --manifest-path tools/civ-impact-causal-arena/Cargo.toml
    cargo run --release --manifest-path tools/civ-impact-causal-arena/Cargo.toml -- run --replications 128 --units 256 --seed 20261016 > arena-report.json

The report binds the exact source digest, includes each cell's treatment/control counts and every regime/strategy/seed row, reports the complete/missing census, MC standard errors for error summaries, and Wilson 95% intervals for empirical rates; it follows the closed Draft 2020-12 schema. It contains no network requests or live interventions.

## Research basis

- Chen et al., CausalGame: Benchmarking Causal Thinking of LLM Agents in Games, ICML 2026: https://proceedings.mlr.press/v306/chen26l.html
- Chen et al., ACED-Bench: Evaluating How LLM Agents Acquire and Act on Causal Evidence, posted 9 October 2026: https://arxiv.org/abs/2610.14381
- NIST, TEVV-Athlon Framework, public draft announced 7 August 2026: https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems

These works motivate design principles, not validation of this implementation.

## Non-goals and limitations

- No LLM integration, external data, user data, participant recruitment, or field experimentation.
- No proof of general causal competence or real-world transportability.
- No claim that the naive intervals are valid for all designs.
- No use of a public synthetic score as a deployment or funding impact claim.
- Randomization in the simulator is generated by its deterministic pseudorandom function, not a cryptographically secure or auditable allocation system.
