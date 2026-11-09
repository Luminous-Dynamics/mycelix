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

A common cause is supported within scope when at least two endpoints respond. A high sham response is treated as confounding/nuisance and forces abstention. Missing observations, unknown time mapping or insufficient coverage also force abstention. Invalid time mappings, duplicate observations, out-of-window events, non-increasing endpoint sequence and malformed schedules are invalid.

The four candidate dispositions are:
- `COMMON_CAUSE_SUPPORTED_WITHIN_INTERVENTION_SCOPE`
- `NO_COMMON_CAUSE_DETECTED_WITHIN_TESTED_ENVELOPE`
- `INDETERMINATE_INTERVENTION_COVERAGE`
- `OBSERVATION_OR_CLOCK_MAPPING_INVALID`

The non-detection disposition is not evidence of physical or statistical independence.

## Public controls and scoring

The public suite has 12 named software conformance cases. The Rust tests cover active/sham contrast, positive and negative outcomes, sham nuisance, incomplete observation coverage, unknown/invalid clock mapping, delayed ingestion, false non-detection, false-positive synchronous unrelated faults, exact-byte input binding, hidden topology commitment mutation and duplicate observation rejection.

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
