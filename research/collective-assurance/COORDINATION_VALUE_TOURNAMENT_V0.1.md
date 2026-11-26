# Coordination Value Qualification v0.1

**Status:** reference validator and tournament protocol. No universal agent-count threshold or measured Luminous coordination result is claimed.

## Research trigger: more agents is not a scaling law

Kim et al., *Capable language models can outgrow the benefits of collaboration*, Nature Machine Intelligence 8, 1157–1172 (published 24 July 2026), report a controlled comparison of single-agent and multi-agent architectures across 260 configurations, six agentic benchmarks, five coordination architectures and three LLM families. Task prompts, tools and total compute budgets were held constant while coordination structure and model capability varied.

The paper reports that:
- Single-agent baseline performance was the most robust predictor of whether coordination helped or hurt.
- A separately fitted capability-saturation decision rule using an approximately 45% baseline threshold predicted the sign of multi-agent gains in 94% of 16 validation configurations on SWE-bench Verified and Terminal-Bench. The authors present this as a practical within-domain selection rule, not a universal scaling law; the validation subset is small.
- Cross-validated architecture selection accuracy was 87% in held-out within-domain configurations; cross-validated R² was 0.373 (0.413 with a task-grounded capability metric).
- Baseline-scaled error amplification survived cluster-robust inference (P_robust=0.030).
- Multi-agent outcomes were domain-dependent: reported results ranged from +80.8% on structured financial analysis to −70.0% on sequential planning. Tested multi-agent variants slightly degraded relative to single-agent baselines on SWE-bench Verified.
- Coordination overhead and message/token growth made larger teams less efficient under the tested token-based communication protocol.

Source: [Kim et al., Nature Machine Intelligence (2026)](https://www.nature.com/articles/s42256-026-01268-y). These are paper-specific results under its tested models, tasks, architectures and budgets—not universal statements about every multi-agent design.

## A second signal: information disclosure is model- and task-dependent

Erdogan et al., *Who Should Know What? Empirical Information Design for Multi-Agent LLM Economies* (arXiv:2611.07667, submitted 5 November 2026), compare disclosure granularity across ten frontier models in trade, bargaining and double auctions while holding interaction rules and models fixed. More detailed information sometimes improved collective outcomes and sometimes reduced welfare or caused market breakdown.

Source: [arXiv:2611.07667](https://arxiv.org/abs/2611.07667). This is a preprint and its economic-market tasks should not be generalized to all team settings.

Together these findings motivate a joint design problem: **when should the system coordinate, and what information should each participant receive?** More agents and more disclosure are interventions to test—not defaults to assume.

## A third signal: role separation does not make verification correct

[TeamBench](https://teambench.github.io/) evaluates 851 software-engineering, data-engineering and incident-response task templates (931 seeded instances, 19 categories) under five conditions: Solo, Restricted, No Plan, No Eval and Full. Planner, Executor and Verifier run in isolated containers with distinct information and write permissions; the verifier is read-only.

Its results expose three important failure modes:
- Team benefit is conditional. The lowest Solo-score quintile gains +15.7 percentage points; the top quintile loses to coordination overhead. Mean Team-vs-Solo uplift over its 155-task reference pool is +0.5 points (p=0.20).
- In its 27-configuration role-mixing pool, LLM Verifiers approved 49.4% of submissions that failed the deterministic grader (Wilson 95% CI [45.9, 52.9], n=1,083). Removing the Verifier improved mean partial score in the reference ablation.
- Prompt-only and OS-enforced teams reached statistically indistinguishable pass rates, but prompt-only runs produced 3.6× more cases where the Verifier attempted to rewrite the Executor's code.

TeamBench also defines a Teamwork Necessity Index:

```text
TNI = (S_team - S_restricted) / max(ε, S_oracle - S_restricted)
```

Its leaderboard reports a TeamBench-90 full-team pass rate of 37.8 for Claude Opus 4.7 versus 35.6 solo, while GPT-5.4 Mini reports 28.9 full versus 33.3 solo. These are model/task/harness-specific results, not a general agent-count ranking.

Source: [TeamBench benchmark and results](https://teambench.github.io/); [paper](https://arxiv.org/abs/2605.07073).

**Implication for Mycelix:** a verifier is another fallible participant, not an oracle. Role isolation, evidence delivery, certification authority and independent grading must be separate. A role that can see the full task and attest completion must not automatically be able to modify the implementation; more importantly, its attestation must be compared with an independent outcome oracle. Measure false-accept, false-reject, attempted role-boundary violations and corrective value separately.

## Paradigm shift

Move from:

`more agents → more intelligence → better results`

to:

`single-agent reference → task-structure diagnosis → matched coordination experiment → measured marginal gain and coordination tax → evidence-qualified architecture choice`

The selection question is not "how many agents can we deploy?" It is:

> Does this coordination architecture add enough verified task value, information complementarity or error correction to justify its communication, latency, token, cost and failure-propagation overhead under the same total budget?

The single-agent baseline remains mandatory. Agent count is an experimental treatment variable, not free additional compute.

## Experimental design

### Architectures
Compare at minimum:
1. Single agent.
2. Independent parallel agents with no shared intermediate state before final aggregation.
3. Centralized orchestrator with workers.
4. Decentralized peer coordination.
5. Hybrid coordination.
6. **Structurally role-separated teams** (Planner / Executor / read-only Verifier), with prompt-only and OS/container-enforced isolation as separate treatments.

Use the same model snapshot for the first matched architecture study. Heterogeneous model teams are a separate treatment and must be compared with a matched homogeneous baseline.

### Task structure
Every task family receives preregistered features:
- decomposability score;
- sequential-interdependence score;
- partial-observability indicator;
- need for environment/tool feedback;
- expected subtask overlap and source diversity.

Do not infer task features from candidate success after the fact. Use independent task characterization and report inter-rater reliability for subjective labels.

### Matched resource envelope
Hold constant benchmark/task/split, model/tokenizer, prompt/decoding, tools/environment, scorer/oracle, context/output limits, total token/compute/wall-clock/memory budgets and paired seed policy. Agent count and coordination structure are the treatments.

Run two tracks:
- **Equal-envelope:** fixed total compute/token/time budget across architectures.
- **Native-envelope:** each architecture uses its natural operating point; report quality/cost/latency Pareto curves. Do not call this a strict matched win.

### Outcomes
Record paired task-quality gain over the single-agent baseline, unique-information preservation, actual cost/token/latency ratios, inter-agent messages and message density, error amplification, redundant work, failed coordination, tool calls, timeouts and retries. For role-separated conditions also measure per-role information access, attempted boundary violations, unauthorized writes/reads, verifier false-accept/false-reject against the deterministic grader, attestation-to-outcome disagreement and whether removing a role improves or worsens partial score.

The reference validator requires execution, outcome and cost evidence digests. It verifies matched task/model/tool/evaluator/seed/budget controls while intentionally allowing agent count and coordination configuration to differ.

## Qualification and decision semantics

`qualify_coordination_trial` validates one paired trial:
- baseline is a one-agent architecture;
- treatment is a named multi-agent architecture with at least two agents;
- task instance, task family, benchmark, model, tokenizer, prompt, tools, evaluator, scorer, decoding, budgets and seed match;
- run IDs and system variants differ;
- task structure and outcome measures are valid;
- the paired gain interval is simultaneous at the declared confidence level and bound to a paired-unit manifest and inference plan;
- the gain point estimate equals coordinated quality minus single-agent quality;
- actual cost, token and latency ratios are reported independently;
- relative error amplification is reported against a study-defined ceiling; the single-agent reference is fixed at exactly 1.0.

Disposition:
- `CoordinationSupported`: paired gain lower bound is above zero and all efficiency/error gates are within their preregistered ceilings.
- `PositiveGainButEfficiencyGateExceeded`: positive gain is supported, but at least one cost/token/latency/error limit is exceeded.
- `SingleAgentPreferred`: paired gain upper bound is at or below zero.
- `Inconclusive`: the interval crosses zero.
- `NotEvaluable`: matched evidence, sample, task or execution qualification fails or is incomplete.

A positive score delta alone is not a deployment recommendation. A configuration that gains quality by violating an authority, privacy, evidence or execution gate is ineligible under the separate assurance/Pareto contract.

## Capability saturation: estimate it, do not hard-code it

The approximate 45% threshold reported by Kim et al. is a useful replication hypothesis, not a constant to bake into production logic. Estimate architecture-selection boundaries separately by model family/snapshot, benchmark/task family, task decomposability and sequential interdependence, interaction horizon and tool dependence, and fixed total budget.

Use held-out task families and model configurations. Report calibration, uncertainty, false-selection costs and baseline selection as a valid outcome. Prefer the single agent whenever evidence for coordination benefit is absent or inconclusive.

## Statistical requirements

- Use paired tasks/seeds and identical independent-unit manifests.
- Treat team/task clusters as independent units, not messages or turns.
- Use cluster-aware bootstrap or randomization inference.
- Report full distributions of paired gains and ratios, not only mean percentage improvement.
- Preserve negative outcomes, timeouts, abstentions and failed runs.
- Separate development, adaptive tuning, validation and sealed confirmation sets; prevent task-family/model-family leakage.
- Report sign prediction, calibration, architecture-selection accuracy and regret relative to the best tested architecture on held-out configurations.
- Correct for multiple comparisons across architectures and task families.

## Proposed competitive targets

Prospective research targets, not achieved results:
- Reproduce the published matched single-agent vs multi-agent protocol on at least three benchmark families before extending it.
- Evaluate all five architecture families with equal total budget and pinned model/task/scorer.
- At least 95% held-out prediction accuracy for the sign of coordination gain across at least 100 independent held-out configurations, with the 95% confidence-interval lower bound above 90% and family-stratified results.
- At least 90% held-out architecture-selection accuracy, including the single-agent option, with the 95% confidence-interval lower bound above 85% under a preregistered regret metric.
- Demonstrate positive, confidence-bounded quality gain without exceeding cost/token/latency/error limits on at least two structurally distinct task families.
- Zero authority, privacy, evidence or execution-boundary gate failures in the qualified comparison set.
- Publish task-structure, coordination-overhead, message-density and error-amplification traces for every configuration.
- On deterministic role-boundary cases, zero unauthorized cross-role reads/writes and zero false verifier PASS. On stochastic held-out TeamBench-compatible cases, target verifier false-accept <5% against the deterministic grader, with false-reject, coverage and abstention reported separately.

The paper reports 94% sign prediction on 16 validation configurations and 87% architecture selection on its held-out within-domain configurations. Our proposed 95%/90% targets are intentionally stronger and require at least 100 independent held-out configurations plus uncertainty bounds. These targets are not directly comparable until we reproduce the paper's task/model split and selection protocol; do not claim superiority from the target definition.

## Integration

- Mycelix #6676 owns dynamic human–AI complementarity and the six-stage collective capability panel. This module adds the matched single-agent/multi-agent coordination decision.
- Mycelix #6616 owns information-order and anchoring experiments.
- Mycelix #6089 owns distributed information discovery, disclosure rights and provenance.
- Mycelix #6677 owns source-origin counterfactual replay.
- Mycelix #6678 owns benchmark/evaluator validity.
- Mycelix #6685 owns the quality/safety/cost Pareto frontier.
- Symthaea SAT #13574 owns the wider system-level adaptive-control and sovereignty tournament.

Information disclosure granularity remains a separate experimental factor. Do not quietly add it to the existing four-arm #6616 design.

## Claim ceiling

A `CoordinationSupported` result qualifies one matched task/model/configuration and its declared efficiency limits. It does not establish a universal capability threshold, prove that multi-agent systems are generally superior, or justify increasing agent count beyond the tested envelope.
