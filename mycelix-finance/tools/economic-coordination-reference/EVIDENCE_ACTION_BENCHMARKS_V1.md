# Decision-Sensitive Procurement Evidence Actions v1 — benchmark contract

**Status:** deterministic decision-support prototype; authored tests and targets are not qualification results.  
**Tracking:** [PROC-INFO-001 #5163](https://github.com/Luminous-Dynamics/mycelix/issues/5163).  
**Optimizer dependency:** [bounded exact joint allocator PR #5162](https://github.com/Luminous-Dynamics/mycelix/pull/5162).

## Paradigm shift

Do not ask for the most data or the cheapest fact by default. Ask which evidence action can change the set of feasible procurement decisions or the explicit per-request shortage/cost frontier before the decision deadline.

The v1 selector accepts finite, declared outcomes and their resulting frontiers. It does not assign probabilities, calculate expected value of sample information (EVSI), or assert expected regret. Without justified probabilities, it uses deterministic sensitivity and a structural frontier-difference measure. That is a heuristic for action triage, not a universal utility function or a theorem that the highest-ranked action is globally optimal.

## Input contract

Each action must identify:
- stable action ID and claim/version key;
- authority/source class and the procurement subjects the observation covers;
- acquisition cost in a declared acquisition currency;
- latest completion time and the evidence validity horizon;
- a nonempty finite set of outcome IDs and the resulting Pareto frontier for each outcome;
- a frozen procurement manifest, baseline frontier, objective currency, evaluation time and decision deadline.

The same claim/version is an alternative-acquisition group: the selector recommends at most one method for that fact. An inconclusive/unavailable observation must be represented as an explicit branch; it is not automatically negative evidence. Actions whose latest completion is after the decision cutoff, or whose returned evidence would already be stale, are ineligible.

## Decision-sensitivity metric

For each action, canonicalize and deduplicate the branch frontiers for structural counts while preserving each named branch in the report. Reject an action whose latest completion precedes the evaluation time. Report:
- number of distinct resulting frontiers (not raw duplicated outcome labels);
- number of distinct frontiers that differ from the baseline;
- maximum symmetric-difference size between the baseline and an outcome frontier;
- separate acquisition cost and completion time;
- disposition (recommended, dominated, duplicate claim alternative, no decision value, after deadline, or stale by completion).

Pareto-filter eligible actions across structural impact, cost and completion time, then apply a stable lexicographic tie-break. A larger frontier difference is a deliberately simple structural signal, not a calibrated utility delta: losing a feasible option and adding an attractive option both contribute to the set difference. Consumers must inspect branch-level consequences before acting.

## Acceptance gates

| Measure | Target |
|---|---:|
| Exact agreement with a separate exhaustive action-value oracle on toy tables | 100% |
| Invented probabilities / EVSI claims absent calibrated probability evidence | 0 |
| Recommended action completing after the decision deadline | 0 |
| Recommended action whose evidence is stale by completion | 0 |
| Duplicate recommendations for the same claim/version | 0 |
| Duplicated outcome branches falsely inflate distinct-frontier count | 0 |
| Outcome branch uses inconsistent request IDs, units, or procurement currency | 0 accepted |
| Identical manifest with permuted outcomes/subject IDs/frontier entries changes output | 0 |
| Zero-decision-value actions recommended solely because they reduce raw uncertainty | 0 |
| Hand-authored stable fixtures | At least 40 |
| Deterministic generated micro-instances, after oracle qualification | At least 10,000 |

## Baselines

On identical frozen action/outcome manifests, compare:
1. random feasible action;
2. lowest acquisition cost first;
3. shortest completion time first;
4. largest raw uncertainty/outcome-count first;
5. highest decision-frontier sensitivity;
6. sensitivity with cost/deadline Pareto filtering.

Measure top-action agreement with the exhaustive oracle, missed high-impact actions, infeasible/late/stale recommendation rate, duplicated claim rate, branch frontier changes, outcome catalogue size and query costs. Do not call the structural symmetric-difference metric expected monetary value or EVSI.

## Research and non-novelty boundary

Targeted evidence collection for supplier selection, value-of-information analysis for supplier quality, and information-aware resilient procurement are existing research areas. The contribution sought here is a reproducible and auditable selector tied directly to the Mycelix joint-allocation frontier, explicit evidence freshness/deadlines, shared claims and a no-invented-probabilities contract—not invention of active information acquisition.

- Abel et al. (2020), targeted evidence collection for uncertain supplier selection: https://doi.org/10.1016/j.eswa.2020.113583
- Quigley et al. (2018), value of information for supplier-quality decisions: https://doi.org/10.1016/j.ejor.2017.05.044
- Procurement portfolios, information value and risk aversion (2024): https://doi.org/10.1007/s10479-024-05918-5
- Open Contracting Data Standard lifecycle linkage: https://standard.open-contracting.org/latest/en/primer/how/

## Safety boundary

No external supplier contact, auditing, scraping, credential probing, paid data request, inventory mutation, reservation, purchase order, contract, dispatch or payment is executed. The module recommends a candidate evidence action only. Real acquisition requires a separately authorized adapter and a policy check of the cited action and its source.
