# Relation-Edge Counterfactual Qualification v0.1

**Status:** Rust reference validator and synthetic paired-trial fixture. No benchmark run, causal population estimate, source-truth claim, or SOTA improvement is asserted.

## Research question

RER (#6688) expands retrieved evidence by traversing verified relations. Source-origin counterfactuals (#6677) remove one source and its derivation closure. A complementary question remains: holding every evidence record constant, what is the contribution of an individual relation edge?

A relation can be useful, redundant, misleading, stale, or harmful. The experiment must be able to detect all five outcomes rather than reward graph expansion by construction.

## Experimental intervention

For one preregistered edge, compare a full-graph baseline against exactly one counterfactual:

- **Add:** introduce one edge that was absent from baseline.
- **Remove:** remove one existing edge.
- **Reverse:** swap endpoints of one directed edge, preserving its relation type and stable edge ID. Symmetric relations cannot be reversed.
- **Relabel:** preserve edge ID and endpoints while changing the relation type.

Reversed or relabelled edges require a new relation receipt digest. All other edges, including their receipts, must remain byte-for-byte identical.

## Strict paired-run controls

Both runs must match on benchmark ID/version and qualified benchmark-certificate digest; split, task, and source-group cluster; candidate-pool and retrieval-seed digests and exact evidence-ID lists; model, tokenizer, prompt, tools, decoding, scorer and evaluator; preregistered inference plan, seed policy and random seed; and context/output token, compute-cost, wall-clock, memory and agent-count budgets.

The baseline and counterfactual must use different run IDs. Candidate evidence records and original retrieval seeds remain unchanged. Only the relation graph may change, and its edge-set difference must equal the declared intervention exactly.

A source-group ID is retained for cluster-aware analysis. Multiple tasks or relation interventions from the same group are not independent observations.

## Integrity and completeness

The validator computes a domain-separated SHA-256 digest of the sorted relation-edge set. Each edge digest commits to its stable edge ID, source evidence ID, target evidence ID, relation tag and relation-receipt digest.

Each run must provide receipts for candidate-pool completeness, relation-index completeness, execution, generation-input exposure, RER bundle, response constraints, independent response audit and evaluator outcome. Missing receipts or incomplete candidate/index coverage produce INDETERMINATE; malformed receipt digests and structural mismatches produce FAIL.

The validator checks that graph endpoints belong to the frozen candidate pool, that edge IDs are unique, and that evidence and seed IDs are unique and properly nested.

## Outcomes

The report keeps outcomes separate: decision changed or unchanged; selected evidence set changed or unchanged; independent decision-quality delta; unauthorized-exposure delta; latency delta; total-cost delta; baseline and counterfactual ACB/constraint-gate outcomes; baseline and counterfactual independent response-audit outcomes.

A gate failure is a valid observed result, not a reason to discard the paired trial. A structurally qualified trial does not establish that the intervention improved quality.

## Required adversarial tests

The confirmatory suite should cover false support edges, missing edges, stale updates, incorrect direction, conflicting supersession, cycles, same-origin duplicate holders, relation poisoning, unauthorized endpoints, unknown endpoints, duplicate edge IDs, fan-out exhaustion, incomplete indexes, missing receipts and altered model/task/budget/evaluator manifests.

Negative controls must demonstrate that an unrelated second edge change is rejected, and that a source-pool change cannot masquerade as a relation intervention.

## Statistical protocol

- Use source group as the primary independent cluster.
- Compare paired within-task outcomes; do not count repeated tasks or edge variants from the same group as independent.
- Preregister the confirmatory relation families and family-wise correction across edge types.
- Report effect sizes, cluster-aware confidence intervals, evaluator uncertainty, missingness and INDETERMINATE rates.
- Separate exploratory edge-type interactions from confirmatory analyses.
- Seal held-out source groups and adversarial relation-poisoning patterns.
- Replicate across model families and include a non-LLM retrieval/graph baseline.

One paired trial is a mechanistic observation, not a population-level causal estimate. Aggregate estimates require the preregistered cluster-aware study.

## Proposed targets (not achieved results)

1. Reject 100% of deterministic single-edge integrity mutations in the declared synthetic fault corpus.
2. Never accept an extra edge change, candidate-pool change, seed change, or model/evaluator/budget mismatch as the declared one-edge intervention.
3. Produce a reproducible graph digest for every accepted relation graph.
4. Preserve ACB failures and audit failures as observed outcomes without converting them into trial-integrity failures.
5. Demonstrate on sealed held-out data at least one beneficial, one redundant and one harmful relation intervention, if supported by the data.
6. Identify relation types that improve decision quality without violating authority, privacy, evidence, or evaluator gates, and report their total cost.

## Integration ownership

- #6089: authorized discovery, inquiry, delivery, rights and source provenance.
- #6619: evidence ledger, constraint binding and independent response audit.
- #6677: source-origin removal and derivative-closure counterfactuals.
- #6678: benchmark, evaluator, leakage and result qualification.
- #6688: bounded RER traversal and evidence-use diagnostics.
- #6690/#6696: signed generation-input exposure.
- #6691: research question and study protocol for relation-edge attribution.
- This module: paired relation-edge intervention validation and outcome-delta reporting.

It does not create a memory store, relation-truth oracle, authorization engine, experiment runner, or statistical inference implementation.

## References

- GroupAssistBench: https://arxiv.org/abs/2611.04867
- COLLABMEM: https://aclanthology.org/2026.emnlp-main.1194/
- HiddenBench: https://proceedings.mlr.press/v306/li26ej.html
- Mycelix #6677, #6678, #6688, #6690/#6696, #6691.
