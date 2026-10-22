# NRVCA Scenario-Set Sensitivity v0.1

**Status:** exhaustive synthetic subset diagnostic  
**Expected answers:** [scenario-subset fixture](../../../fixtures/nrvca-scenario-selection-sensitivity-expected-v0.1.json)  
**Input:** [NRVCA-005 stress fixture](../../../fixtures/nrvca-correlated-stress-synthetic-v0.1.json)  
**Tracking issue:** [NRVCA-008 / #5421](https://github.com/Luminous-Dynamics/mycelix/issues/5421)  
**Stacked implementation:** [PR #5422](https://github.com/Luminous-Dynamics/mycelix/pull/5422)

## Why scenario selection is itself a benchmark

A robustness fraction is conditional on a scenario set. The numerator and denominator are choices: which futures are represented, which are omitted, how the set was generated, and whether it gives adequate coverage of failure modes. A high fraction over a favorable or incomplete set is not proof of broad robustness.

This issue is recognized in the water-resources literature:

- McPhail et al. (2020), *Impact of Scenario Selection on Robustness*, quantitatively investigates how choosing scenarios can affect robustness measures and alternative rankings: https://doi.org/10.1029/2019WR026515
- Herman et al. (2015), *How Should Robustness Be Defined for Water Systems Planning under Change?*, compares robustness metrics and calls for explicit definitions and sensitivity analysis: https://doi.org/10.1061/(ASCE)WR.1943-5452.0000509
- Matrosov et al. (2013), *Robust Decision Making and Info-Gap Decision Theory for water resource system planning*, describes exploratory scenario analysis and vulnerability discovery: https://doi.org/10.1016/j.jhydrol.2013.03.006

NRVCA should compose with these methods, not claim the general idea is novel.

## Exhaustive small-case experiment

The current NRVCA-005 fixture has five scenarios and three policies. For the five named scenarios, the test evaluates every non-empty subset:

\[
2^5 - 1 = 31
\]

For each subset it retains the included and excluded scenario IDs, exact denominator, each strategy's hard-constraint pass count/fraction, and the set of top-ranked policies including ties. The score is recalculated from the underlying constraints; the test does not trust the frozen pass-fraction summary as input.

This is a combinatorial audit of one tiny synthetic fixture—not a probability distribution, not 31 equally likely worlds, and not a substitute for a domain-grounded method of generating physically plausible futures.

## What the fixture demonstrates

On the complete five-scenario set, frozen NRVCA-005 scores remain:

| Strategy | Passes | Robust pass fraction |
|---|---:|---:|
| Output-only static | 0/5 | 0.00 |
| Equity-constrained static | 1/5 | 0.20 |
| Adaptive staged | 4/5 | 0.80 |

Exhaustive subset scoring reveals how strongly the fraction depends on which scenarios are included:

| Strategy | Minimum fraction across non-empty subsets | Maximum fraction | Distinct fractions observed |
|---|---:|---:|---:|
| Output-only static | 0.00 | 0.00 | 1 |
| Equity-constrained static | 0.00 | 1.00 | 6 |
| Adaptive staged | 0.00 | 1.00 | 6 |

These ranges are not performance uncertainty intervals. For the equity and adaptive strategies, the range reaches 0 when the subset includes only a scenario each fails, and reaches 1 when a subset contains only scenarios each passes. The unchanged underlying outcomes have not improved; only the denominator and included cases changed.

### Leave-one-out results

| Omitted scenario | Output-only | Equity static | Adaptive staged |
|---|---:|---:|---:|
| Compound drought + grid + crew delay | 0/4 | 1/4 | 3/4 |
| Grid disruption | 0/4 | 1/4 | 3/4 |
| Nominal operations | 0/4 | 0/4 | 3/4 |
| Prolonged/repeated disruption | 0/4 | 1/4 | 4/4 |
| Seasonal drought | 0/4 | 1/4 | 3/4 |

Removing the prolonged/repeated case changes the adaptive policy's fraction from 4/5 (0.80) to 4/4 (1.00). That does not change its poor performance in the omitted case. The omitted identifier is therefore an essential part of the score, not ancillary metadata.

For this fixture, the ordering by pass fraction is adaptive staged ≥ equity static ≥ output-only for all 31 subsets. This is a property of the hand-authored pass sets—the adaptive policy passes the nominal case too and passes more of the other cases—not a general theorem about adaptive strategies. Ties must remain visible: deleting nominal operations makes equity static and output-only tie at 0/4; restricted subsets can tie other policies as well.

## Correct scoring rules

- Always publish the scenario-set identifier, included IDs, excluded IDs, denominator and policy score.
- No scenario count is treated as a probability or weight.
- A changed scenario set is a new diagnostic result, not silently comparable as if the denominator were unchanged.
- Adding a passing case can increase, leave unchanged, or decrease a fraction depending on the prior score; adding a failing case can also change the denominator. No universal monotonicity claim is valid.
- Do not remove a difficult case from a headline set after seeing results. Report scenario-generation rules and preserve held-out cases.
- Keep scenario plausibility/coverage separate from exhaustive subset arithmetic. A subset can be mathematically valid while representing a poor or physically incoherent assessment set.

## Tests and acceptance targets

The Rust integration test checks:

- all 31 non-empty subsets appear exactly once, with exact scenario membership and exclusions;
- every pass count is independently recalculated from budget, reserve, total-cost, service, underserved-service, reliability and emissions constraints;
- each fraction equals pass count divided by the explicit included-scenario denominator;
- full-set and all five leave-one-out outputs match known answers;
- scenario order, strategy order, row order and descriptive labels do not affect subset scores;
- no probabilities or weights are inferred from subset enumeration;
- the original full-set 0/5, 1/5 and 4/5 results remain unchanged.

These are deterministic software checks on synthetic data. They do not prove empirical robustness, calibrate the stressors, or establish causal effects.

## How to beat industry practice in a defensible way

A stronger comparison protocol should combine this sensitivity diagnostic with:

1. A documented and reviewed scenario-generation method, including joint physical dependencies and repeated-event history.
2. Scenario-discovery analyses that identify which combinations produce constraint breaches.
3. A held-out scenario family that is not used while designing/adapting the policy.
4. Performance comparisons with the incumbent and credible RDM/MORDM baselines under the same model, candidate set, constraints and evaluation budget.
5. Preregistered outcomes: robust feasibility, worst-case service/equity/reliability shortfall, recovery time, cumulative service loss, reserve depletion, and lifecycle cost where supported by the model.
6. Sensitivity to thresholds, source vintages, scenario-selection procedure and model family, with results independently reproduced.

The field is not beaten by enumerating every subset of five examples. This slice makes a common source of overclaim measurable and testable, so later larger experiments cannot silently hide how a favorable scenario set inflates a headline robustness fraction.
