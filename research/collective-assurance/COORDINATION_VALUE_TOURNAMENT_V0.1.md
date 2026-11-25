# Coordination Value Qualification v0.1

**Status:** reference validator and tournament protocol. No universal agent-count threshold or measured Luminous coordination result is claimed.

## Research trigger: more agents is not a scaling law

Kim et al., *Capable language models can outgrow the benefits of collaboration*, Nature Machine Intelligence 8, 1157–1172 (published 24 July 2026), report a controlled comparison of single-agent and multi-agent architectures across 260 configurations, six agentic benchmarks, five coordination architectures and three LLM families. Task prompts, tools and total compute budgets were held constant while coordination structure and model capability varied.

The paper reports:
- Single-agent baseline performance was the most robust predictor of whether coordination helped or hurt.
- A separately fitted capability-saturation decision rule using an approximately 45% baseline threshold predicted the sign of multi-agent gains in 94% of 16 validation configurations on SWE-bench Verified and Terminal-Bench. The authors present this as a practical within-domain selection rule, not a universal scaling law; the validation subset is small.
- Cross-validated architecture selection accuracy was 87% in held-out within-domain configurations; the model's cross-validated (R^2) was 0.373 (0.413 with a task-grounded capability metric).
- Baseline-scaled error amplification survived cluster-robust inference ((P_{robust}=0.030)); coordination structures without centralized verification propagated errors more as baseline difficulty increased.
- Multi-agent outcomes were strongly domain-dependent: reported results ranged from +80.8% on structured financial analysis to −70.0% on sequential planning. On SWE-bench Verified, tested multi-agent variants slightly degraded relative to single-agent baselines.
- Coordination overhead and message/token growth made larger teams less efficient under the tested token-based communication protocol.

Source: [Kim et al., Nature Machine Intelligence (2026)](https://www.nature.com/articles/s42256-026-01268-y). These are paper-specific results under its selected models, tasks, architectures and budgets—not universal statements about every multi-agent design.

## A second signal: information disclosure is model- and task-dependent

Erdogan et al., *Who Should Know What? Empirical Information Design for Multi-Agent LLM Economies* (arXiv:2611.07667, submitted 5 November 2026), compare disclosure granularity across ten frontier models in multi-resource trade, bargaining and double auctions while holding interaction rules and models fixed. More detailed information sometimes improved collective outcomes and sometimes reduced welfare or caused breakdown. For example, the paper reports that full disclosure reduced welfare for several models in double auctions, while other model/task conditions benefited from more detailed disclosure.

Source: [arXiv:2611.07667](https://arxiv.org/abs/2611.07667). This is a preprint and its economic-market tasks should not be generalized to all team settings.

Together these findings motivate a joint design problem: **when should the system coordinate, and what information should each participant receive?** More agents and more disclosure are both interventions to test—not defaults to assume.

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

Use the same model snapshot for a first matched architecture study. Heterogeneous model teams are a separate treatment and must be compared against a matched homogeneous baseline, with model-family and capability mix pinned.

### Task structure

Every task family receives preregistered features:
- decomposability score;
- sequential-interdependence score;
- partial-observability indicator;
- need for environment/tool feedback;
- expected subtask overlap and source diversity.

Do not infer task features from the candidate's success after the fact. Use independent task characterization and publish inter-rater reliability for subjective labels.

### Matched resource envelope

Hold constant:
- benchmark, task instance and split;
- model snapshot, tokenizer, prompt and decoding;
- tools and environment image;
- scorer/evaluator and oracle;
- context and output token limits;
- total token, compute, wall-clock and memory budgets;
- seed policy and paired task/team cohort.

The number of agents and coordination structure are the treatments. Do not give an MAS system more total compute and then attribute its advantage to coordination.

Run two tracks:
- **Equal-envelope:** fixed total compute/token/time budget across architectures.
- **Native-envelope:** each architecture uses its natural operating point; report quality/cost/latency Pareto curves. Do not call this a strict matched win.

### Outcomes

Record:
- paired task-quality gain over the single-agent baseline;
- unique-information preservation and coverage;
- actual cost, tokens and latency ratios;
- inter-agent messages and message density;
- error amplification relative to the single-agent reference;
- redundant work and failed coordination events;
- tool calls, timeouts, retries and resource waste.

The reference validator requires execution, outcome and cost evidence digests. It verifies matched task/model/tool/evaluator/seed/budget controls while intentionally allowing agent count and coordination configuration to differ.

## Qualification and decision semantics

`qualify_coordination_trial` validates one paired trial:
- baseline must be a one-agent architecture;
- treatment must be a named multi-agent architecture with at least two agents;
- task instance, task family, benchmark, model, tokenizer, prompt, tools, evaluator, scorer, decoding, budgets and seed must match;
- run IDs and system variants must differ;
- task structure and outcome measures must be valid;
- the paired gain interval must be simultaneous at the declared confidence level and bound to a paired-unit manifest and inference plan;
- the point estimate must equal coordinated quality minus single-agent quality;
- actual cost, token and latency ratios are reported independently;
- relative error amplification is reported against a study-defined ceiling.

Disposition:
- `CoordinationSupported`: paired gain lower bound is above zero and every efficiency/error gate is within its preregistered ceiling.
- `PositiveGainButEfficiencyGateExceeded`: positive gain is supported, but at least one cost/token/latency/error-amplification limit is exceeded.
- `SingleAgentPreferred`: paired gain upper bound is at or below zero.
- `Inconclusive`: the interval crosses zero.
- `NotEvaluable`: matched evidence, sample, task, or execution qualification fails or is incomplete.

A positive score delta alone is not a deployment recommendation. A configuration that gains quality by violating an authority, privacy, evidence or execution gate is ineligible under the separate assurance/Pareto contract.

## Capability saturation: estimate it, do not hard-code it

The approximate 45% threshold reported by Kim et al. is a useful replication hypothesis, not a constant to bake into production logic. Estimate architecture-selection boundaries separately by:
- model family and model snapshot;
- benchmark/task family;
- task decomposability and sequential interdependence;
- interaction horizon and tool dependence;
- fixed total budget.

Use hierarchical or within-domain models, held-out task families and held-out model configurations. Report calibration, uncertainty, false-selection costs and baseline selection as a valid outcome. Prefer the single agent whenever evidence for coordination benefit is absent or inconclusive.

## Statistical requirements

- Use paired tasks/seeds and identical independent-unit manifests.
- Treat team/task clusters as independent units, not messages or turns.
- Use cluster-aware bootstrap or randomization inference.
- Report the full distribution of paired gains and ratios; do not report only mean percentage improvement.
- Preserve negative outcomes, timeouts, abstentions and failed runs.
- For a learned selection rule, separate training/development, validation and sealed confirmation sets; prevent task-family/model-family leakage.
- Report sign prediction, calibration, architecture-selection accuracy and regret relative to the best tested architecture on held-out configurations.
- Correct for multiple comparisons across architectures and task families.

## Proposed competitive targets

Prospective research targets, not achieved results:
- Reproduce the published matched single-agent vs multi-agent protocol on at least three of its benchmark families before extending it.
- Evaluate all five architecture families with equal total budget and pinned model/task/scorer.
- At least 90% held-out prediction accuracy for the sign of coordination gain across at least 100 independent held-out configurations, with confidence intervals and family-stratified results.
- At least 85% held-out architecture-selection accuracy, including the single-agent option, under a preregistered regret metric.
- Demonstrate a positive, confidence-bounded quality gain without exceeding cost/token/latency/error limits on at least two structurally distinct task families.
- Zero authority, privacy, evidence or execution-boundary gate failures in the qualified comparison set.
- Publish task-structure, coordination-overhead, message-density and error-amplification traces for every configuration.

The paper's 94% sign-prediction result is a replication comparator, not a target that can be claimed without matching its task/model/configuration conditions. Our proposed 100-configuration target broadens the held-out confirmation base.

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
