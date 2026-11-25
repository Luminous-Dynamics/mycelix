# GroupAssistBench Comparison Parity Addendum v0.1

**Status:** superseding clarification to the GroupAssistBench target wording in `BENCHMARK_TOURNAMENT_V0.1.md`. This addendum does not change any reported paper result or claim a Luminous benchmark run.

## Why this clarification is required

The November 2026 GroupAssistBench paper reports different model backbones and different context-token footprints. In its full-context rows:
- GPT-5.6-terra reports 0.431 average L2 assistance with 52,435 context tokens.
- DeepSeek-v4-flash reports 0.469 average L2 assistance with 52,435 context tokens.

The reported retrieval baselines use different context footprints (for example, BM25 10,769 and dense retrieval 9,539 tokens). The paper's external-memory systems also have their own context footprints. The values are valid published benchmark references, but they are not automatically strict matched comparisons for a new system.

Two distinctions must be preserved:

1. **Different backbone:** the 0.469 DeepSeek-v4-flash result is a cross-model reference. It cannot be the matched baseline or confidence-bound threshold for a GPT-5.6-terra experiment.
2. **Different resource envelope:** using the same backbone does not by itself make a comparison strict-matched. Context tokens, total token budget, output budget, compute/time budget, task split, scorer, tools, memory budget, agent count and seed policy must also be controlled or explicitly reported as different.

The published 0.431 GPT-5.6-terra result is therefore a same-backbone reference point, not a claim of strict resource parity with retrieval systems. The 0.469 DeepSeek-v4-flash result is a contextual cross-model frontier reference, not a matched GPT baseline.

Source: [One Assistant, Many Memories: Benchmarking Real-World Group Assistance When Recall Is Not Enough, arXiv:2611.04867](https://arxiv.org/abs/2611.04867). The paper describes 80,299 messages from 50 groups and 2,395 evaluation instances (1,374 L1 recall and 1,021 L2 memory-use instances). Its Table 3 reports the model and retrieval results above.

## Benchmark-definition qualification

Before interpreting a reproduced baseline or declaring a superiority result, qualify the benchmark release, evaluator and scoring evidence with [Mycelix #6678](https://github.com/Luminous-Dynamics/mycelix/pull/6678). Its manifest binds task/data/split provenance, hidden-split commitment, oracle/scorer evidence, independent evaluator, leakage audit, judge calibration and human adjudication where applicable, reproducibility records and cost telemetry. A benchmark-definition PASS does not qualify a candidate system; a matched-run PASS does not repair an unqualified benchmark. These are separate gates.

## Required comparison tiers

### Tier 0 — reproduce the published reference

Reproduce the official GroupAssistBench task version, split, prompt, scoring rubric and baseline configurations. Report deviations and reproduce L1 and L2 separately, including M1/M2/M3. Do not tune on the held-out test split.

### Tier 1 — strict matched-system experiment

Choose one pinned answerer model and hold constant:
- model snapshot and tokenizer;
- system/developer prompt, decoding and toolset;
- benchmark version, dataset/split digest, task family and evaluator/scorer;
- context-token cap, max output, total token budget, time budget and compute budget;
- agent count, memory budget, seed policy, and paired seeds.

The system variant (for example full-context, BM25, dense retrieval, graph memory, provenance ledger, constraint binding, or their combination) is the intended treatment difference. If any other field changes, the comparison validator must reject a strict leaderboard claim or classify the result as context-only.

Where full-context and retrieval systems naturally consume different amounts of context, run both:
- an **equal-envelope track**, with a common context and compute/token budget; and
- a **native-envelope track**, retaining each method's natural budget and reporting quality-cost Pareto curves.

Do not call the native-envelope result a strict matched win. This separation makes the resource trade-off visible instead of hiding it.

### Tier 2 — cross-model contextual frontier

Run separate tournaments for each model family under Tier 1 conditions. The DeepSeek-v4-flash 0.469 result may motivate a DeepSeek-specific reproduction, but it cannot be compared as a matched win against a GPT-5.6-terra candidate. Cross-model rows remain contextual until each model has a same-backbone, same-budget candidate/baseline pair.

### Tier 3 — capability and rights frontier

Report L2 assistance alongside source attribution, authorized visibility, conflict preservation, stale-evidence rejection, abstention quality, response grounding, agency and burden. A gain in mean L2 cannot compensate for an unauthorized disclosure, a fabricated source, or silent loss of a group's minority-held evidence.

## Revised target interpretation

The earlier proposed values remain useful as aspirations, but their comparator semantics are now:

| Target | Correct interpretation |
|---|---|
| L2 mean ≥0.45 | Initial research target. Compare against a reproduced, same-backbone baseline under the same resource envelope. The published GPT-5.6-terra 0.431 is an orientation point, not proof of parity. |
| L2 mean ≥0.50 | Stretch point target. Its lower 95% confidence bound must exceed the **reproduced strict-matched baseline from the same tournament**, not automatically the cross-model 0.469 result. |
| M3 ≥0.35 | Conflicting-memory target. Compare primarily against a reproduced, same-backbone, same-budget M3 baseline. The published GPT-5.6-terra 0.303 is a context anchor, not an automatic matched comparator. |
| Claim of surpassing 0.469 | Requires a separate DeepSeek-v4-flash tournament with a reproduced DeepSeek baseline, identical task/scorer/resource envelope, and uncertainty on the paired difference. |

These are target definitions, not achieved scores. Do not report a leaderboard rank or SOTA superiority until the official evaluation has been reproduced and the strict comparison manifest has passed.

## Statistical unit and uncertainty

GroupAssistBench contains many evaluation instances but only 50 source groups. Repeated questions from the same group are not independent observations. Use paired, group-clustered bootstrap resampling (with M1/M2/M3 strata preserved) or another preregistered cluster-aware estimator. Report:
- paired score differences and 95% intervals;
- per-group and per-dependency-class results;
- M1, M2 and M3 separately;
- missingness, judge failures and scorer disagreement;
- worst-decile group performance and source-grounding failures.

Do not bootstrap individual questions as if all instances were independent. If using an LLM judge, pin its version and rubric, blindly score conditions where feasible, and adjudicate a sampled set with independent human raters.

## Required result card

Every published GroupAssistBench result must include:
1. benchmark release/version and dataset/split digests;
2. answerer, model snapshot, tokenizer, prompts, tools and decoding digests;
3. scorer/judge version and rubric digest;
4. context, total-token, output, compute, time, memory and agent-count budgets;
5. M1/M2/M3 counts and scores;
6. paired seed policy and group-clustered uncertainty;
7. source-attribution, conflict, unauthorized-disclosure and abstention outcomes;
8. comparison classification from the Mycelix manifest validator: `MatchedSystem`, `ContextOnly`, or `NotComparable`.

The comparison layer must bind to the actual run manifests and evaluator receipts. A written declaration that two settings were matched is not evidence of parity.

## Relationship to the tournament

This addendum supersedes only the interpretation of the GroupAssistBench comparator and confidence-bound targets in `BENCHMARK_TOURNAMENT_V0.1.md`. It does not alter HiddenBench, GroupMemBench, COLLABMEM, Common Ground, SympoeticBench, or the human-study safeguards.
