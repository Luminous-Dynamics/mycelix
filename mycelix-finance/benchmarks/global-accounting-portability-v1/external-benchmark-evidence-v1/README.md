# External Accounting Benchmark Source Evidence

This directory records public APEX-Accounting and FINBALANCE source observations. It is a source-evidence register, not a benchmark runner, task corpus, run receipt, dataset mirror or performance claim.

It composes with [FIN-ACC-EVAL-001 / #5799](https://github.com/Luminous-Dynamics/mycelix/issues/5799), which owns candidate/oracle isolation, canonical run manifests, run receipts and matched experiment arms. It does not create a competing run-manifest schema.

## Snapshot history

- **v1.0.0** is retained as an earlier 2026-11-05 source observation without a capture time.
- **v1.1.0** is a time-labeled capture at **2026-11-05 19:39 SAST**. It separated APEX methodology, paper and live leaderboard records.
- **v1.2.0** is a later capture at **2026-11-05 22:12 SAST**. Its schema and fixture are `benchmark_source_evidence_v1.2.0.schema.json` and `fixtures/accounting_benchmark_source_snapshot_v1.2.0.json`. It adds the official publisher article as a fifth source and expands the register to 46 observations across five source records.

Neither snapshot archived raw page bytes. Each source therefore has `source_bytes_archived=false` and `source_bytes_sha256=null`. A SHA-256 over the JSON fixture identifies the fixture only, never the source website's bytes.

## APEX: the 100-task live page and 160-task paper/blog cohort remain distinct

The v1.2.0 source set separates four official APEX references:

1. **Current methodology card:** 100 tasks, k=4, single-turn task format and a single-shot canonical harness.
2. **Published technical paper (29 July 2026):** a 160-task private held-out cohort across ten worlds. It reports 56.4% Mean Criteria@3 for the leading evaluated model, best Pass@8 of 21.5%, no model above 2.6% Pass^8 and 58% of tasks never fully solved by any model attempt. It excludes human-in-the-loop clarification tasks and tax, audit, consolidation, multi-entity, multi-currency and external reporting.
3. **Official Mercor publisher article (31 July 2026):** independently describes the original 160-task/ten-world cohort. It reports more than 40 accounting professionals, median 11 years' experience, 13.7 rubric criteria per task and 97% judge/expert agreement. The public sample is one world and ten tasks; the full held-out set remains private. It says the canonical leaderboard uses a Loop Harness and describes a separate purpose-built Ramp Harness ablation that improves Mean Criteria@3 by 1.2 percentage points on average. The article explicitly says this is not an evaluation of the full Ramp Stack product.
4. **Live leaderboard at 22:12 SAST:** 100 tasks, 1,475 criteria and ten held-out worlds. Its highest displayed score is 71.5% ±4.8. It describes a Loop Harness; eight runs for Pass@1/Pass@8/Pass^8; canonical Mean Criteria k=4; five-million-token and 500-step caps; and task-level 95% bootstrap intervals over 10,000 resamples.

**Do not compare 71.5% with 56.4% as if a model improved on one unchanged benchmark.** The live page denominator is 100; paper/blog sources describe 160. The methodology card describes single-turn/single-shot, while the publisher article and live page describe a Loop Harness. The paper's Mean Criteria@3 and live page's Mean Criteria k=4 must keep their attempt semantics. The official sources reviewed here do not provide an explicit cohort, harness, rubric or version mapping. Preserve these observations separately and leave the relationship unresolved.

The publisher article also reports that roughly seven in ten failures among the top three models were reasoning failures rather than information-gathering failures. It describes agents finding discrepancies but later omitting or contradicting them in their final journal entries. This supports a Mycelix benchmark dimension for **evidence-to-decision continuity**: preserve a discovered discrepancy and its resolution through the final projection, not just retrieve a supporting document.

Links:
- Live leaderboard: https://www.mercor.com/apex/apex-accounting-leaderboard/
- Methodology card: https://www.mercor.com/apex/methodology/
- Published paper: https://arxiv.org/abs/2607.27189
- Official publisher article: https://www.mercor.com/blog/introducing-the-ai-productivity-index-for-accounting/

## FINBALANCE: a separate task family

The FINBALANCE paper (Findings of EMNLP 2026) reports a 710-record evaluation split across eight industries, three period types and five difficulty levels, with 23 inconsistency-code labels. The best exact final balance-sheet accuracy among six evaluated LLMs is 46%. Reconstructing statements from journal entries improved four models by 26–41 percentage points.

This supports a distinct Mycelix hypothesis: **make source-bound economic events and journal entries an explicit intermediate representation, then independently reconstruct statements from the ledger.** It does not prove that the gain transfers to another corpus. FINBALANCE exact-statement accuracy is not comparable with APEX Mean Criteria.

Paper: https://aclanthology.org/2026.findings-emnlp.675/

## Matched evaluation protocol

Use the same permitted cases, source bytes, tools, permissions, retry policy, model budget and evaluator version. Freeze split identity/digest (or custodian-held commitment for hidden cases), candidate code commit, prompts, model/version, tool versions, rubric, evaluator, seeds, attempt count, timeouts, raw candidate outputs and tool traces. Candidate and oracle artifacts must remain physically separate; any oracle leakage invalidates a run.

Compare:
1. deterministic rules-only;
2. model-only;
3. model + conventional tools;
4. model + tools + ledger reconstruction;
5. typed source-bound economic-event graph;
6. typed graph + independent deterministic verifier;
7. Symthaea candidate analyst + Mycelix verifier;
8. qualified accounting professional on a blinded, stratified subset.

Report metric families separately:
- **APEX-aligned:** Mean Criteria, Pass@1, Pass@8, Pass^8, attempts, failure taxonomy, runtime and cost. Use an authorized, cohort-matched harness; do not extrapolate between the 100-task live page and the 160-task paper/blog cohort.
- **FINBALANCE-aligned:** exact final balance-sheet accuracy, account/line accuracy, source-to-journal linkage, inconsistency detection by label and journal-reconstruction ablation. Preserve the record-level denominator.
- **Mycelix evidence-qualified:** source-binding completeness per journal line, exact ledger balance, period/entity/currency correctness, clarification precision, question-source precision, abstention quality, evidence-to-decision continuity, historical replay, false qualification, reviewer time and cost.

## Proposed Mycelix target gates (not achieved)

Create an independent expert-adjudicated corpus rather than claiming benchmark equivalence:
- S0: 20 synthetic accounting cases and 8 synthetic event-graph cases with public oracles and deterministic negative controls.
- S0.1: >=50 synthetic adversarial cases per benchmark track.
- S1: >=100 professionally reviewed cases, dual annotation and adjudication.
- S2: >=250 cases plus a hidden holdout covering multiple entities, currencies, accounting frameworks, policy versions, periods and source systems.

Hard gates remain outside aggregate scores: 100% material source-locator binding; zero unbalanced journals; zero silent entity/currency/period coercion; 100% clarification/abstention for seeded decision-critical unknowns; zero stale/future source leakage; 100% exact replay of frozen inputs; zero false transfer qualifications or live ledger side effects.

After S1 is frozen, proposed quality goals are >90% exact statement accuracy on S1 and >95% on S2, >=95% decision-relevant clarification questions, <=5% redundant questions, >=90% of solvable ambiguity resolved with the minimal adjudicated question set and >=30% reviewer-time reduction without increased material errors. These are goals, not reported performance.

Use confidence intervals and per-stratum results. Report false positives/negatives, abstentions, selective risk/coverage, p50/p95 latency, fully loaded cost and reviewer time. Never average away a hard integrity failure.

## Rights and claim ceiling

APEX's full scored set is private. Its public ten-task sample does not grant permission to copy or redistribute hidden cases. FINBALANCE artifact rights are not established by its bibliographic page; check exact terms before import. No dataset was copied in these snapshots.

This register establishes only what the public sources displayed or reported at capture time. It does not establish benchmark reproduction, source-byte integrity, dataset access, licensing, accounting correctness or superiority over SOTA.