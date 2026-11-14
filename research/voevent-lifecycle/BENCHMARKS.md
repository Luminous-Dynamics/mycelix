# Scientific observability benchmark protocol v0.1

Status: proposed protocol. No system leaderboard or real-science performance claim is made by this document.

## 1. Benchmark the whole decision system

The unit under test is not just a language model or anomaly detector. Record the complete configuration:

- model and model version;
- reasoning agent and policy;
- parsers, source adapters and evidence stores;
- benchmark harness and tool versions;
- system prompts/configuration and safety/authority policy;
- compute, wall-clock, data, reviewer and instrument budgets.

A run is not reproducible if any of these are omitted. Freeze the task inputs, split definitions, budget, seeds, expected evidence graph and success criteria before comparing systems.

## 2. Evaluation ladder

Keep the same task API and evidence semantics while increasing physical realism.

| Tier | Evaluation environment | What it establishes | What it does not establish |
|---|---|---|---|
| T0 | Synthetic, adversarial, known-answer corpus | Parser/replay semantics, temporal isolation, exact graph preservation, deterministic behavior | Real-world scientific validity |
| T1 | Retrospective, rights-cleared public observations | Reproducibility against historical data; forecast and investigation quality | Prospective operational value |
| T2 | Cross-domain and out-of-distribution holdouts | Transfer across observatories, sensors, platforms, geography, season, processing and annotation | Robustness to all unseen environments |
| T3 | Digital twin with latency, noise, failures and common-cause faults | Closed-loop decisions under controlled physical-model uncertainty | Real instrument performance |
| T4 | Authorized live instrument or field campaign | Prospective scientific and operational value | Universal generalization beyond the campaign |

Do not move to T4 on the basis of a benchmark score alone. It requires separate rights, safety, operator and instrument authorization.

## 3. Baseline tournament

Every task should compare the full system against the following baselines under identical initial evidence and resource budgets:

1. No follow-up.
2. Fixed-schedule follow-up.
3. Existing expert or broker workflow.
4. Threshold/rule-based alerting.
5. Anomaly-score greedy selection.
6. Entropy or information-gain greedy selection.
7. Decision-theoretic EVSI/ENBS selection.
8. Model-only and agent/tool-enabled variants.
9. Symthaea as the complete configured system.
10. Hidden-truth oracle on synthetic data only, as an upper bound rather than a deployable competitor.

For decision policies, match follow-up count, instrument time, exposure, latency and monetary/compute budget. For model/agent systems, record the exact model, agent, harness, prompt/configuration and tool set.

## 4. Scorecard

Never collapse all dimensions into a single accuracy number. Publish a per-domain scorecard.

| Dimension | Primary measures |
|---|---|
| Prediction | Brier score, log loss, reliability/calibration, precision-recall, interval coverage |
| Event detection | False alarms per unit exposure/time, missed-event rate, detection latency |
| Scientific value | Realized decision loss/regret, accepted discoveries, hypothesis discrimination, time-to-resolution |
| Follow-up utility | EVSI/ENBS, information gained that changes a decision, follow-up cost |
| Evidence integrity | Temporal leakage, packet/digest binding, lost citations, unresolved-reference handling, replay determinism |
| Generalization | Leave-one-source/domain/region/sensor/platform/season/processing/annotation-out results; worst-stratum performance |
| Human work | Reviewer minutes, escalations, reversals, evidence reconstruction time |
| Resource efficiency | Compute, wall time, data transfer, instrument time and cost per accepted resolution |
| Governance | Rights violations, unauthorized actions, missing authority, provenance bypasses |

Hard gates are not averaged into the score. Any temporal look-ahead, silent history deletion, digest mismatch accepted as valid, rights bypass or unauthorized instrument action is a gate failure.

## 5. Required adversarial cases

At minimum, freeze known-answer cases for:

- late-arriving evidence with earlier source publication time;
- source event time that is not UTC or is not directly comparable;
- duplicate, missing, malformed and reused identifiers;
- follow-up, supersession and retraction by different publishers;
- unresolved references that become resolvable later;
- contradictory and cyclic citation graphs;
- packet alteration, digest mismatch and incomplete receipt records;
- shared source, gateway, operator, calibration or trust-root common causes;
- clock inversion, stale calibration, telemetry-only evidence and source conflict;
- domain shift, missing modalities, sensor failure and annotation disagreement;
- a genuinely unknown or misspecified physical model, where OTHER/abstain is correct.

A negative control must fail closed. A system must not receive credit for a correct answer reached by reading data that would not have existed at the decision cutoff.

## 6. Statistical protocol

- Pre-register task versions, data rights, splits, metrics, baselines, budgets and exclusion rules.
- Use event/mission/time-blocked splits; never randomly split correlated frames or packets across train and test.
- Hide final test labels and outcomes where feasible.
- Use paired comparisons on the same tasks and seeds.
- Report distributions and confidence intervals, not only means or leaderboard ranks.
- Use cluster-aware bootstrap or a justified hierarchical model when observations share an event, source, site or campaign.
- Determine trial counts by power analysis. Three trials per task are a pilot, not a robust ranking.
- Publish per-domain and worst-stratum results. A large domain must not hide failure in a smaller one.
- Include failed, abstained and unresolved trials in accounting; do not silently drop them.
- Publish immutable task manifests, exact input digests, source licenses/rights, evaluator version, run receipts and full configuration.

## 7. Proposed acceptance targets

These are goals for future evaluations, not results achieved by v0.1.

### Integrity gates

- Zero temporal look-ahead across all known-answer and adversarial cases.
- Zero silently lost source packets or lifecycle citations.
- 100% expected edge/effect fidelity on the frozen synthetic corpus.
- 100% deterministic replay for identical inputs and cutoff.
- Zero silent coercion of conflicting lifecycle assertions into a single canonical truth.
- Zero rights, authority or instrument-tasking bypasses.

### Scientific-value goals

- At least 20% lower held-out investigation regret than the strongest non-oracle baseline at the same follow-up and compute budget is a stretch target.
- No material degradation in the worst held-out domain while improving pooled performance.
- Calibrated uncertainty and explicit abstention/OTHER when the evidence cannot distinguish hypotheses.
- Lower reviewer and follow-up burden at equal or better scientific decision quality.

The 20% regret target is not a claim of current superiority. It must be tested with paired, held-out tasks and confidence intervals.

### Scale gates

Measure replay at 10^3, 10^4, 10^5 and 10^6 packet records, reporting throughput, memory, p50/p95 latency and index/build costs. Keep correctness tests enabled at each scale. Do not optimize away provenance or temporal semantics to improve throughput.

## 8. Position relative to current benchmarks and standards

Autonomous Science Bench 0.1 is a useful methodological reference for full-configuration evaluation, task-specific discovery and learning outcomes, equal-budget baselines, and the replay-oracle → digital-twin → live-lab progression. Its initial public release has six pilot tasks, three trials per task, and explicitly warns that small leaderboard gaps are not meaningful. Its percentages are not comparable to an astronomy event-reasoning benchmark.

CrossCD-Eval (Scientific Reports, 3 October 2026) evaluates remote-sensing change detection across in-domain, cross-domain and few-shot settings using CLCD, HRSCD and LEVIR-CD. Its findings reinforce that strong in-domain results do not guarantee cross-domain transfer. Our satellite evaluation should therefore hold out geography, sensor/platform, season, resolution, processing and annotation regimes independently.

IVOA VOEvent 2.1 already standardizes event packets and lifecycle citations. This work should interoperate with that standard, not invent a competing astronomy packet format. The additional contribution is an auditable, receipt-time-aware evidence and decision layer around source-native packets. VOEvent itself does not define the transport layer, publisher policy or intellectual-property/rights policy; those remain explicit system responsibilities.

## 9. Release rule

A public benchmark release must include the versioned corpus manifest, per-case rights and provenance, frozen split and cutoff definitions, expected outputs, evaluator source, baseline configurations, raw run receipts and a limitations report. Keep the hidden evaluation set private until a run is submitted; then publish enough evidence to reproduce the result without leaking future test labels.
