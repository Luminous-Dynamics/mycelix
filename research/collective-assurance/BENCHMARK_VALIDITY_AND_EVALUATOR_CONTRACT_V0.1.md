# Benchmark Validity and Evaluator Independence Contract v0.1

**Status:** qualification protocol and reference validator. No benchmark, model or system result is claimed by this document.

## Why qualify the benchmark itself?

An agent leaderboard is only as trustworthy as its tasks, hidden split, ground truth, scorer, evaluator independence, execution environment and cost accounting. A score can be precisely calculated and still answer the wrong question because the task set leaked, the scorer is biased, the environment differs, the evaluator sees the answer key, or cost is omitted.

Recent field reviews expose this gap:
- A 2026 review of 15 prominent agent benchmarks reports that none integrated safety/security or cost efficiency into primary evaluation, and that 13/15 relied exclusively on binary success measures. This is a review of a selected benchmark sample, not a census of every benchmark. [From benchmarks to deployment](https://doi.org/10.1007/s10462-026-11571-0).
- A separate 2026 survey of 17 prominent benchmarks reports no complete joint control of data contamination, nondeterminism and execution cost, and no complete standardized run-cost record. [Large language model agent evaluation and benchmarking](https://link.springer.com/article/10.1007/s10462-026-11678-4).
- The 2026 ACL survey describes a shift toward dynamic, more realistic and continuously updated agent evaluations, while highlighting cost-efficiency, safety and robustness as gaps. [A Survey on Evaluation of LLM-based Agents](https://aclanthology.org/2026.findings-acl.1330/).

The opportunity is not to replace AgentDojo, AgentDyn, GroupMemBench, COLLABMEM, GroupAssistBench, ControlArena or Inspect. It is to qualify the evidence package that makes a result from those systems decision-worthy.

## Paradigm shift: evaluation claims become evidence objects

Move from:

`benchmark name + score + leaderboard rank`

to:

`claim scope → frozen task/data/split → independent oracle/scorer → leakage and holdout audit → reproducible execution → cost and safety evidence → independent review → bounded result`

A benchmark itself becomes a system-under-test. Its owners, evaluator, hidden split, oracle, scoring implementation, execution environment, data rights and reporting pipeline are all part of the assurance boundary.

The unit of comparison is not a score alone. It is a score plus its reproducibility, evaluator independence, task validity, cost, uncertainty, safety and coverage evidence.

## Three qualification profiles

### Synthetic contract

For deterministic schemas, state machines, allocation algorithms and security invariants. Requires:
- canonical task catalog, data and split commitments;
- independent oracle and scorer validation;
- property-test/exhaustive-case coverage evidence;
- deterministic replay evidence;
- leakage and sealed-holdout evidence;
- repeated runs and full cost telemetry.

A passing synthetic contract demonstrates conformance within its declared model and fixture set. It does not prove host isolation, human behavior or real-world safety.

### Agent system

For agents operating tools, external content, persistent memory or environment side effects. In addition to core evidence, requires:
- pinned environment image and harness digest;
- declared threat/attack-surface manifest;
- effect-sink coverage evidence, including successful action vs attempted action vs verified postcondition;
- separated development, adaptive-red-team and sealed-confirmation sets;
- explicit attack utility and false-positive/over-defense reporting.

Do not award a security pass to an agent that merely disables every useful tool. Report safety and authorized-task utility together.

### Human/team

For human-subject or collective-outcome benchmarks. In addition to core evidence, requires:
- preregistration and primary estimand;
- independent ethics approval;
- accessible consent and accommodations;
- withdrawal and data-minimization plan;
- cluster-aware analysis plan when teams are the assignment unit.

A synthetic validator cannot approve recruitment or establish human capability change. Human research gates remain external and study-specific.

## Core qualification requirements

1. **Task/data provenance:** pin benchmark identity, version, task catalog, dataset, split and data-rights/license evidence.
2. **Hidden-test integrity:** commit the hidden split before candidate evaluation; preserve access logs and an independent leakage audit. Unknown leakage status is not a pass.
3. **Ground-truth independence:** keep answer keys/oracles outside candidate and benchmark-owner control during evaluation. A valid signature or hash proves integrity of bytes, not factual truth.
4. **Scorer validity:** pin the scoring specification and implementation. Validate the scorer against known-answer and adversarial cases.
5. **Judge calibration:** LLM-judge and hybrid systems require a calibration report and an independently human-adjudicated sample. Human scoring requires inter-rater reliability evidence. A judge cannot certify itself.
6. **Reproducibility:** require at least three repeated runs for the baseline qualification floor. Stochastic evaluations require at least three distinct seeds. Deterministic evaluations bind an explicit replay record. These are minimum entry gates, not sufficient statistical power for every superiority claim.
7. **Complete cost telemetry:** record model/tokenizer/version, input and output tokens, tool/API calls, wall-clock and active compute, memory, storage, network, energy where measurable, and human setup/adjudication time. If a quantity is not measurable, report it as unknown, not zero.
8. **Claim-specific scope:** synthetic, agent-system and human/team claims have separate additional evidence. Passing one profile does not qualify another.
9. **Independent evaluation:** benchmark owner and independent evaluator identities must differ, and an independently retained evaluator receipt must bind the evaluation plan and outputs.
10. **Non-compensatory disposition:** invalid rights, oracle, leakage, holdout, scorer or evidence integrity fails qualification. Missing evidence remains `INDETERMINATE`. A high task score cannot compensate.

## Reference validator

`src/benchmark_quality.rs` implements the structural qualification contract:
- three claim profiles;
- deterministic oracle, independent human adjudication, LLM judge and hybrid scoring modes;
- evidence digest validation and mandatory gate outcomes;
- benchmark-owner/evaluator separation;
- run/seed minimums;
- profile-specific evidence requirements;
- `PASS`, `FAIL` and `INDETERMINATE` semantics.

The `qualify_benchmark_result_comparison` entry point combines benchmark-definition qualification with the actual baseline/candidate manifests. It verifies that the certified benchmark identity and version match both runs, and separately returns matched-claim versus contextual-claim eligibility. A cross-model result can never become a matched claim merely because its benchmark certificate is valid.

The module validates declared evidence references and states. It does not fetch evidence, authenticate signers, prove that independent parties are truly independent, inspect a hidden dataset for leakage, calculate inter-rater reliability, or measure compute itself. Those are upstream evidence producers; production qualification requires authenticated, independently retained receipts.

## Required result card

Every reported result should carry:
- benchmark and version, task catalog/data/split digests, data license/rights;
- model, tokenizer, prompt, tools, environment and scorer digests;
- task inclusion/exclusion and hidden-split commitment;
- scoring method, oracle independence and judge calibration/human adjudication evidence;
- all seeds, repetitions, failures, timeouts, abstentions, missingness and exclusions;
- confidence intervals, cluster unit, multiplicity and preregistered estimand;
- cost telemetry across AI, tool, infrastructure and human review;
- attack coverage, unauthorized actions/disclosures, false positives and false negatives;
- exact qualification disposition and its claim ceiling.

## How to compete with current industry

Use existing benchmarks as adapters and comparators:
- AgentDojo and AgentDyn for prompt-injection/tool-use security;
- MASpi for multi-agent indirect-injection behavior where its official protocol is reproducible;
- ControlArena and Inspect for controlled agent evaluations;
- GroupMemBench for multi-party memory;
- COLLABMEM for role-aware memory operations and provenance;
- GroupAssistBench for open-ended group assistance;
- HiddenBench for distributed-information reasoning;
- Common Ground for joint action and shared understanding;
- SympoeticBench / Mycelix #6597 for delayed human/team capability.

Run the official benchmark first, then add Luminous extensions in separate tracks. Compare same-backbone, same-task, same-scorer and equal-resource conditions; provide a native-resource Pareto track separately. Keep cross-model results contextual. Never manufacture a new metric to avoid reproducing the published baseline.

## Target ladder

Prospective targets, not achieved results:
- 100% of benchmark definitions used for a superiority claim have a completed qualification manifest;
- zero candidate-visible answer-key/holdout leakage accepted by the gate;
- zero benchmark-owner self-evaluation accepted as independent evaluation;
- 100% of LLM-judge claims include calibration and independent human adjudication;
- 100% of reports include run-cost telemetry or an explicit unknown field;
- all stochastic comparisons report at least three distinct seeds for initial qualification and use stronger pilot-informed sample sizes for confirmatory claims;
- 100% reporting of refusals, abstentions, timeouts, exclusions and missingness;
- 100% profile-specific claim limits stated with results.

These are process-conformance goals. They do not imply high reliability, strong statistical power or scientific validity by themselves.

## Ownership and composition

- Mycelix #6619 owns the matched run comparison validator; this module qualifies the benchmark/evaluator evidence and binds its identity/version to the actual run manifests before a result claim is eligible.
- Mycelix #6656 clarifies GroupAssistBench parity and matched baselines.
- Mycelix #6676 owns dynamic complementarity and the six-stage collective capability outcome panel.
- Mycelix #6677 owns paired leave-one-source-origin-out replay qualification.
- Symthaea SAT #13574 owns system-level authority, adaptive control, auditor/evaluator and evidence-integrity tournament composition.
- Benchmark owners retain their own dataset, licensing, scorer and official reproduction responsibilities.

## Claim ceiling

A structural PASS qualifies only the declared benchmark definition and evidence references for the selected profile. It does not prove that a candidate system is capable, safe, aligned, superior, or suitable for deployment. Those require exact benchmark results, statistical evidence and independent replication.
