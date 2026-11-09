# EMSR controlled-intervention benchmark 0.1

**Status:** initial synthetic reference evaluator; not real-world causal discovery  
**Issue:** [EMSR-CAUSAL-001 / Mycelix #5981](https://github.com/Luminous-Dynamics/mycelix/issues/5981)

## Why a separate track?

The temporal dependency hypergraph evaluates common dependencies in a declared inventory. It cannot discover an upstream dependency that was omitted from that inventory. This benchmark asks a different question: can controlled interventions, sham trials, endpoint observations and timing evidence expose a hidden common cause?

It is deliberately not a probabilistic causal-discovery engine. Version 0.1 starts with a deterministic intervention contract and a small reference comparison so that the assumptions are explicit and falsifiable.

## Two trust domains

The candidate process receives only a public scenario:
- intervention target, domain and latency envelope;
- randomized active and sham trial schedule;
- expected observation endpoints;
- event-time and receive-time observations;
- per-observation clock-mapping state;
- declared response thresholds.

The evaluator process receives the hidden truth only after candidate output is locked. The two command-line entry points are separate:
- `causal-candidate`: reads one public scenario and emits a candidate report.
- `causal-evaluator`: reads the public scenario, hidden truth and candidate report, then emits a score.

The truth commitment is a SHA-256 commitment to the serialized hidden-truth structure with its commitment field cleared. This binds the commitment to the hidden topology bytes under the defined JSON serialization; it is not a signature, custody receipt or proof of independent evaluator control.

## v0.1 decision rule

For each declared endpoint, compare the degraded-response rate in active trials against the degraded-response rate in sham trials. An endpoint is responsive only when its active response rate meets the intervention threshold and its sham response rate remains below the nuisance ceiling.

A common cause is supported within scope when at least two endpoints respond. A high sham response is treated as confounding/nuisance and forces abstention. Every trial must also include a declared negative-control observation channel. An active response in that channel is treated as evidence of a shared observation/collection artifact and forces abstention rather than being counted as an independent endpoint. Missing observations, unknown time mapping or insufficient coverage also force abstention. Invalid time mappings, duplicate observations, out-of-window events, non-increasing endpoint sequence and malformed schedules are invalid.

The four candidate dispositions are:
- `COMMON_CAUSE_SUPPORTED_WITHIN_INTERVENTION_SCOPE`
- `NO_COMMON_CAUSE_DETECTED_WITHIN_TESTED_ENVELOPE`
- `INDETERMINATE_INTERVENTION_COVERAGE`
- `OBSERVATION_OR_CLOCK_MAPPING_INVALID`

The non-detection disposition is not evidence of physical or statistical independence.

## Public controls and scoring

The public suite has 13 named software conformance cases. The Rust tests cover active/sham contrast, positive and negative outcomes, sham nuisance, shared-collector negative-control response, incomplete observation coverage, unknown/invalid clock mapping, delayed ingestion, false non-detection, false-positive synchronous unrelated faults, exact-byte input binding, hidden topology commitment mutation and duplicate observation rejection.

The evaluator records:
- true positive, false positive, true negative and false negative;
- abstention;
- false-independence/non-detection on a ground-truth common-cause scenario;
- hidden common-cause endpoint IDs;
- exact public input and candidate report digests;
- hidden truth commitment.

This is a deterministic synthetic software benchmark. It is not blind merely because the scoring API accepts a separate hidden-truth file. A genuine blind challenge needs evaluator-held truth outside the candidate repository, access isolation, run-order control, locked candidate outputs, commitment/reveal receipts and independent custody.

## Proposed benchmark targets

These are internal target hypotheses, not standards or field-safety guarantees:
- zero false non-detection/false-independence claims on the public negative controls;
- zero false non-detection on the evaluator-sealed common-cause set before any field-readiness claim;
- at least 95% recall on in-scope injected common causes, reported by domain;
- false common-cause rate no more than 1% on synchronized-independent controls, with confidence intervals;
- 100% abstention when observation coverage or time mapping is insufficient;
- 100% binding of reports to exact public input bytes and truth commitments;
- p50/p95/p99 detection latency reported by fault domain and scenario size.

Zero observed failures must always be reported with sample counts and confidence bounds. Zero false claims on a small synthetic set is not a low-risk guarantee.

## Claim ceiling

Version 0.1 tests a synthetic intervention/observation model and hidden-topology scoring semantics. It does not discover arbitrary real-world causal structure, establish failure probabilities, prove physical independence, authenticate the hidden-truth custodian, qualify telecom/PNT equipment, or replace domain-specific hardware-in-the-loop fault injection and safety review.


## Comparison with current research and industry practice

The nearest comparators solve related but different tasks:

| Comparator | Established contribution | What it does not directly test in this track |
|---|---|---|
| [TCD-Arena (ICLR 2026)](https://tcd-arena.github.io/) | Robustness evaluation of time-series causal-discovery algorithms under 33 assumption violations; its reported study describes roughly 30 million discovery attempts | Operational intervention attribution in instrumented infrastructure, with explicit clock/collector artifacts and false-independence scoring |
| [CausalBench](https://www.nature.com/articles/s42003-025-07764-y) | Network inference on real interventional single-cell datasets; its initial benchmark includes two perturbation experiments and more than 200,000 intervention datapoints | Telecom/PNT/industrial failure topologies, event-time/receive-time semantics, and telemetry/measurement common-cause controls |
| [CSuite](https://github.com/SCAI-Lab/csuite-causal-benchmark) | Synthetic SCMs with known causal graphs, observational train/test data and interventional test environments | Operational fault-injection workflows and instrumentation integrity |
| [CauseMe](https://causeme.uv.es/) | Time-series causal-discovery datasets with known or high-confidence structures and varied real-world challenges | A controlled infrastructure common-cause challenge with explicit interventions, sham trials and safety-scoped abstention |
| [Anduril (SOSP 2024)](https://doi.org/10.1145/3694715.3695979) | Feedback-driven fault injection to reproduce 22 real distributed-system failures; the paper reports a median reproduction time of eight minutes | A standardized hidden-topology benchmark for false common-cause and false-independence decisions across PNT, timing, RF and industrial endpoints |

The relevant conclusion is not that this v0.1 implementation is already better than those systems. It is not. The research opportunity is to define a different benchmark task at the intersection of interventional causal discovery, fault injection and measurement assurance.

### Required baselines

A credible comparison should include:
- **Declared-graph baseline:** Mycelix #5929 closure over the available inventory. It must not be scored as discovering an omitted edge.
- **Active/sham rate baseline:** the current deterministic rule, with explicit nuisance and coverage handling.
- **Time-series causal discovery baselines:** PC/GES/GIES and appropriate time-series methods from TCD-Arena, only on data for which their assumptions and input semantics are satisfied.
- **Interventional network-inference baselines:** applicable CausalBench/CSuite methods where the generated structural model matches their assumptions.
- **Fault-injection baseline:** feedback-driven injection/reproduction strategy in the style of Anduril, measuring whether a fault can be reproduced and isolated—not treating reproduction alone as proof of a shared upstream cause.

Every baseline must see the same public observations, intervention assignments and timing frontiers. Hidden truth and held-out topology remain evaluator-owned.

### Metrics that distinguish this benchmark

Do not rank systems on structural F1 alone. Report separately:
1. Hidden common-cause recall, causal-path precision/recall and structural Hamming distance where a graph is produced.
2. False common-cause rate on synchronized-independent controls.
3. False non-detection/false-independence rate on true shared-cause cases (the primary safety metric).
4. Abstention and selective risk as observation coverage, clock mapping or inventory completeness are degraded.
5. Detection latency and intervention count to distinguish the competing hypotheses.
6. Cost and risk of the selected interventions; unsafe action selection must be a hard gate, not averaged into accuracy.
7. Calibration only when a candidate emits probabilities, with reliability plots and proper scoring rules.
8. p50/p95/p99 runtime and memory by endpoint count, trial count, topology size and path depth.

Stratify results by clock, GNSS, oscillator, power, network, correction provider, firmware, collector and antenna-site causes. Include leave-one-site, leave-one-firmware, leave-one-clock-architecture and leave-one-intervention-family holdouts.

## Next paradigm: assurance as active experiment design

A fixed intervention schedule is only the first step. A stronger system should maintain a set of competing causal hypotheses and select the **least disruptive safe experiment that best separates them**. For v0.2, prefer set-valued/minimax discrimination over uncalibrated probability scores:
- enumerate safe, permitted interventions and their predicted observable signatures under each live hypothesis;
- remove actions outside the approved sandbox, maintenance or hardware-in-the-loop envelope;
- choose the action that minimizes the worst-case remaining hypothesis set, subject to a hard intervention-risk budget;
- return abstention when no safe action separates the remaining hypotheses;
- bind the recommendation to the system boundary, abort conditions, evidence frontier and exact hypothesis set.

This turns assurance from passive telemetry review into a closed-loop scientific diagnostic process. Do not deploy active fault injection against live critical infrastructure by default. Production use needs separate engineering authorization, safe containment and explicit abort controls.
