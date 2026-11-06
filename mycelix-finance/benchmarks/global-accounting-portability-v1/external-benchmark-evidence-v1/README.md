# External Accounting Benchmark Source Evidence

This directory records public APEX-Accounting and FINBALANCE source observations. It is a source-evidence register, not a benchmark runner, task corpus, run receipt, dataset mirror or performance claim.

It composes with [FIN-ACC-EVAL-001 / #5799](https://github.com/Luminous-Dynamics/mycelix/issues/5799), which owns candidate/oracle isolation, canonical run manifests/receipts and matched experiment arms. It does not create a competing run-manifest schema.

## Snapshot history

- **v1.0.0** is retained as an earlier 2026-11-05 source observation without a capture time.
- **v1.1.0** is a time-labeled capture at **2026-11-05 19:39 SAST**.
- **v1.2.0** is a later capture at **2026-11-05 22:12 SAST**. It separates methodology, technical paper, live board, FINBALANCE and official publisher article; five sources and 46 observations.
- **v1.3.0** is the latest capture at **2026-11-06 06:04 SAST**. Its schema and fixture are `benchmark_source_evidence_v1.3.0.schema.json` and `fixtures/accounting_benchmark_source_snapshot_v1.3.0.json`. It adds a sixth source: a recapture of Mercor's current live APEX leaderboard and methodology details. The register now has **six sources and 65 observations**.

None of these snapshots archived raw HTML/PDF bytes. Every source therefore has `source_bytes_archived=false` and `source_bytes_sha256=null`. A SHA-256 over the JSON fixture identifies the fixture only, never the source website's bytes.

## APEX: preserve cohort, attempt, harness and retry distinctions

The v1.3.0 source set preserves the official APEX sources separately:

1. **Current live leaderboard (captured 6 Nov 2026):** 10 held-out worlds, 100 tasks and 1,475 grading criteria. The page exposes 46 configurations; the highest displayed score is Gemini 4 Argon High at 71.5% ±4.8, followed by Gemini 4 Carbon High at 70.7% ±4.8, Muse Wonder 1.0 Max at 67.1% ±4.9, Fable 5.1 Max at 66.7% ±4.6 and Opus 5.5 Max at 65.5% ±4.8. It describes a Loop Harness. The canonical Mean Criteria metric is k=4; each model is run eight times for Pass@1, Pass@8 and Pass^8. These are different attempt summaries and must retain their own definitions.
2. **Current live methodology details:** models generally run at their highest available reasoning setting; step-level response issues are rerun until success; persistent system/model failures can cause up to three trajectory reruns; there is a five-million-token and 500-step cap; uncertainty intervals are 95% task-level bootstraps over 10,000 resamples; and tasks require a mean of 7.5 source files. The language-model judge sees the task prompt, individual criterion and model's final answer, scores each criterion Met/Not Met, and penalizes scattergunning.
3. **Current methodology card:** its summary describes 100 tasks, k=4 and a single-turn/single-shot canonical task format. Keep this record distinct from other APEX descriptions where harness and attempt semantics differ.
4. **Published technical paper (29 July 2026):** a private 160-task held-out cohort across ten worlds and 2,186 criteria. It reports 56.4% Mean Criteria@3 for the leading evaluated model, best Pass@8 of 21.5%, no model above 2.6% Pass^8 and 58% of tasks never fully solved by any of eight attempts. It excludes human-in-the-loop clarification tasks and tax, audit, consolidation, multi-entity, multi-currency and external reporting.
5. **Official Mercor publisher article (31 July 2026):** independently describes the original 160-task/ten-world cohort; more than 40 professional authors; median 11 years' experience; 13.7 average rubric criteria/task; and 97% judge/expert agreement. The public sample is one world and ten tasks; full held-out tasks remain private. It describes a Loop Harness leaderboard and separate Ramp Harness ablation (+1.2 percentage points Mean Criteria@3 on average), explicitly not an evaluation of the full Ramp Stack product.
6. **FINBALANCE paper (Findings of EMNLP 2026):** separate multi-document accounting reconciliation benchmark described below.

**Do not compare 71.5% with 56.4% as if they measure an unchanged cohort.** The live page displays 100 tasks / 1,475 criteria with Mean Criteria k=4; the paper and article describe 160 tasks / 2,186 criteria with Mean Criteria@3. The publisher has not provided a public version/manifest mapping these cohorts, rubrics, task IDs, or k-attempt definitions. The 100-task and 160-task records remain separate; preserve their denominators and metric semantics.

The live page also says an **eleventh world is open source**, while the article says a public one-world / ten-task sample has been released. Do not assume these descriptions identify exactly the same artifact without the underlying release manifest.

### What this means for a fair Mycelix comparison

A faithful APEX-style reproduction must pin the retry policy, because step errors are retried and persistent system/model failures may trigger up to three trajectory reruns. Report both:
- **capability outcomes:** task failure, refusal, unsupported answer, contradiction, wrong ledger or unsupported assumption;
- **infrastructure outcomes:** provider outage, timeout, tool transport failure and runner crash.

Never erase model reasoning failures as if they were transient infrastructure errors. Report initial-attempt outcomes separately from benchmark-policy retry-adjusted outcomes. Report per-task and per-arm attempts, cost and latency; do not use different retry limits across matched arms.

The judge sees the final answer and criterion, not an independently recorded internal evidence ledger. Mycelix's candidate-declared evidence trace is therefore a separate diagnostic, not an APEX score. The forthcoming runner-observed trace artifact (#5874) will make trajectory continuity auditable from the tool/transcript evidence, but it must not be presented as part of Mercor's own grader.

The publisher's failure analysis attributes roughly seven in ten top-three-model failures to reasoning rather than retrieval. An example is an agent finding a discrepancy mid-workflow but later omitting or contradicting it in the final journal. This motivates Mycelix's emphasis on evidence-to-decision continuity: source facts and conflicts must survive through decisions and journal outputs, not just be retrieved.

Links:
- Live leaderboard: https://www.mercor.com/apex/apex-accounting-leaderboard/
- Methodology card: https://www.mercor.com/apex/methodology/
- Published paper: https://arxiv.org/abs/2607.27189
- Official publisher article: https://www.mercor.com/blog/introducing-the-ai-productivity-index-for-accounting/

## FINBALANCE: a different task family

The FINBALANCE paper reports a 710-record evaluation split across eight industries, three period types and five difficulty levels, with 23 inconsistency-code labels. The best exact final balance-sheet accuracy among six evaluated LLMs is 46%. Reconstructing statements from journal entries improved four models by 26–41 percentage points. Authors report models often produced numerically plausible entries but failed to bind them to source documents and aggregate them consistently; citation-pressure prompting barely changed document-linking errors, while ledger-feedback ablations substantially improved statement outputs and exposed inconsistency-detection trade-offs.

This supports an explicit Mycelix intermediate representation: source-bound economic events and journal entries, followed by independent statement reconstruction. It does not prove the gain transfers to another corpus. FINBALANCE exact-statement accuracy is not comparable with APEX Mean Criteria.

Paper: https://aclanthology.org/2026.findings-emnlp.675/

## Matched evaluation protocol

Use the same permitted cases, source bytes, tools, permissions, retry policy, model budget and evaluator version. Freeze split identity/digest (or custodian-held commitment for hidden cases), candidate code commit, prompts, model/version, tool versions, permission scope, grader/rubric, random seeds, attempt count, timeouts, raw candidate outputs and tool traces. Candidate and oracle artifacts must remain physically separate; any oracle leakage invalidates a run.

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
- **APEX-aligned:** Mean Criteria, Pass@1, Pass@8, Pass^8, attempts, failure taxonomy, runtime and cost. Use an authorized, cohort-matched harness; do not extrapolate between 100-task and 160-task cohorts.
- **FINBALANCE-aligned:** exact final balance-sheet accuracy, account/line accuracy, source-to-journal linkage, inconsistency detection by label and journal-reconstruction ablation. Preserve the record-level denominator.
- **Mycelix evidence-qualified:** source-binding completeness per journal line, exact ledger balance, period/entity/currency correctness, clarification precision, question-source precision, abstention quality, candidate-declared and runner-observed evidence-to-decision continuity, historical replay, false qualification, reviewer time and cost.

## Proposed Mycelix target gates (not achieved)

Create an independent expert-adjudicated corpus rather than claiming benchmark equivalence:
- S0: 20 synthetic accounting cases and 8 synthetic event-graph cases with public oracles and deterministic negative controls.
- S0.1: at least 50 synthetic adversarial tasks per benchmark track.
- S1: at least 100 professionally reviewed cases, dual annotation and adjudication.
- S2: at least 250 cases plus a hidden holdout covering multiple entities, currencies, accounting frameworks, policy versions, periods and source systems.

Hard gates remain outside aggregate scores: 100% material source-locator binding; zero unbalanced journals; zero silent entity/currency/period coercion; 100% clarification/abstention for seeded decision-critical unknowns; zero stale/future source leakage; 100% exact replay of frozen inputs; zero false transfer qualifications or live ledger side effects.

After S1 is frozen, proposed quality goals are >90% exact statement accuracy on S1 and >95% on S2, at least 95% decision-relevant clarification questions, at most 5% redundant questions, at least 90% of solvable ambiguity resolved with the minimal adjudicated question set and at least 30% reviewer-time reduction without increased material errors. These are goals, not reported performance.

Use confidence intervals and per-stratum results. Report false positives/negatives, abstentions, selective risk/coverage, p50/p95 latency, fully loaded cost and reviewer time. Never average away a hard integrity failure.

## Rights and claim ceiling

APEX's full scored set is private. Its public sample does not grant permission to copy or redistribute hidden cases. FINBALANCE artifact rights are not established by its bibliographic page; check exact terms before import. No external dataset was copied in these snapshots.

This register establishes only what public sources displayed or reported at capture time. It does not establish benchmark reproduction, source-byte integrity, dataset access, licensing, accounting correctness or superiority over SOTA.