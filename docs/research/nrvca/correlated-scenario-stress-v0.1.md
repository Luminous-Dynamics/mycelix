# NRVCA-005: correlated scenario stress testing

Status: synthetic method fixture; no calibrated physical model, real utility assessment or causal evaluation. The example outcomes are deliberately hand-authored to test an evaluation contract, not simulated from South African observations.

## Why the next paradigm shift is not “predict better”

The existing literature on Decision Making under Deep Uncertainty (DMDU) and Robust Decision Making (RDM) shifts the focus from selecting one forecast to finding plans that meet declared goals across a broad set of plausible conditions. The World Bank describes RDM as scenario-neutral vulnerability analysis and recommends combining methods when appropriate; a 2026 guide for water utilities discusses DMDU methods and adaptive pathways. These are established fields, not NRVCA inventions:

- World Bank, [Investment Decision Making Under Deep Uncertainty](https://ppp.worldbank.org/library/investment-decision-making-under-deep-uncertainty).
- Water Utility Climate Alliance / Water Institute, [A Beginner's Guide to Decision Making Under Deep Uncertainty for Water Utilities](https://www.cakex.org/documents/beginners-guide-decision-making-under-deep-uncertainty-water-utilities).
- Herman et al., [How Should Robustness Be Defined for Water Systems Planning under Change?](https://doi.org/10.1061/%28ASCE%29WR.1943-5452.0000509).

NRVCA's potential contribution to test is an evidence-bound interface linking source vintages and multidimensional accounts to strategy performance, explicit guardrails, vulnerability reports and contingent actions. That contribution must be tested against a dashboard/ordinary RDM workflow; do not claim novelty just for including equity, uncertainty or scenarios.

## Scenario-neutral synthetic stress set

The fixture defines five coherent named cases: nominal operations; drought with increased heat demand; grid disruption affecting powered pressure equipment; a compound drought/grid/crew-delay case; and a prolonged repeated disruption after reserve depletion. These are jointly described futures. They do not have probability weights.

Three policies are evaluated over every scenario:
1. output-only static allocation;
2. equity-constrained static allocation;
3. adaptive staged policy with explicit storage, backup-power and repair-crew trigger rules.

Every strategy-scenario outcome has service lower bound, underserved-service lower bound, reliability lower bound, emissions upper bound, scenario cost and triggered-action IDs. All dimensions retain their units. The results are hand-authored synthetic assumptions to qualify the evaluator only; they are not outputs of a calibrated water model or estimates for Johannesburg.

## Metrics

- **Robust success fraction:** count of scenarios where every hard guardrail passes, divided by the declared scenario count, unweighted unless defensible probabilities are separately preregistered. This is a stress-set statistic, not a probability of success.
- **Worst-case shortfalls:** service, underserved coverage and reliability floor deficits; max emissions/cost cap excess.
- **Vulnerability map:** scenario IDs and stressor combinations where each strategy fails.
- **Trigger and cost audit:** action IDs follow explicit stressor conditions; every strategy's base cost must fit the primary budget; every scenario's incremental cost must fit the contingency reserve, and total cost must fit the overall cap.
- **Coverage:** every strategy must contain exactly one outcome for every declared scenario; no silent omission, duplicate cell or favorable scenario selection.
- **Regret and adaptive value:** future extension should compare feasible strategies to per-scenario oracle choices and assess whether triggered options improve worst-case outcomes at acceptable reserve cost.

## Known-answer results

With the frozen fictional floors (service lower bound >= 800; underserved lower bound >= 800; reliability >= 0.95; emissions upper bound <= 12; total scenario cost <= 14):

| Strategy | Scenarios passing | Robust pass fraction | Worst service shortfall | Worst underserved shortfall | Worst reliability shortfall |
|---|---:|---:|---:|---:|---:|
| Output-only static | 0/5 | 0.00 | 550 | 720 | 0.57 |
| Equity-constrained static | 1/5 | 0.20 | 500 | 600 | 0.37 |
| Adaptive staged | 4/5 | 0.80 | 10 | 40 | 0.02 |

The adaptive strategy still fails the prolonged/repeated-disruption case. This is important: the benchmark should expose a vulnerable scenario, not quietly drop it to make a policy look robust. A high pass fraction is not automatically adequate when any one critical constraint failure can be unacceptable. Thresholds and trade-offs are explicitly normative toy choices.

## Acceptance gates (targets, not real results)

- 100% matrix coverage and no duplicate strategy-scenario cells.
- 100% deterministic reproduction of robust-feasibility counts and worst-case diagnostics.
- Zero scenario probability inferred from scenario counts; weighted metrics only when probability basis is separately justified.
- Adding a new failing scenario cannot improve the robust pass fraction; added pass scenarios may increase it, so always publish the scenario denominator and full set.
- Changing a descriptive label or attaching probability metadata cannot change scenario-neutral metrics.
- Missing results, failed evaluations and unresolved trigger rules fail closed. Base-budget violations, reserve overruns and total-budget overruns are rejected independently.
- No unsubstantiated calibration, water-security, national-economy or causal claim.

## Next tier

Replace hand-authored outcomes with a transparent scenario generator whose parameters are grounded in authorized and source-pinned hydrology, supply availability, demand, grid uptime, crew capacity, repair lead times and intervention response. Define physically admissible correlations and dependencies; do not combine individually plausible bounds into an impossible joint future. Compare:
- incumbent strategy;
- static constrained allocation;
- a standard RDM/DMDU vulnerability workflow;
- an adaptive staged policy.

Use independent review and a shadow deployment before a prospective field test. Report stability, worst-case/percentile shortfalls where distributions are defensible, scenario discovery conditions, regret, reserve cost, human-review burden and outcomes for underserved groups. No synthetic pass fraction constitutes evidence of real-world improvement.
