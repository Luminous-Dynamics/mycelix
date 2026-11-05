# External Accounting Benchmark Source Evidence

This directory records public APEX-Accounting and FINBALANCE source observations. It is a source-evidence register, not a benchmark runner, task corpus, run receipt, dataset mirror or performance claim.

It composes with [FIN-ACC-EVAL-001 / #5799](https://github.com/Luminous-Dynamics/mycelix/issues/5799), which owns candidate/oracle isolation, canonical run manifests, run receipts and matched experiment arms. It does not create a competing run-manifest schema.

## Snapshot history

- **v1.0.0** records an earlier 2026-11-05 source observation without a capture time. Its APEX live-page row recorded a different set of displayed score values. It is retained as historical evidence; it is not silently overwritten.
- **v1.1.0** is a later, time-labeled capture at **2026-11-05 19:39 SAST**. Its schema and fixture are `benchmark_source_evidence_v1.1.0.schema.json` and `fixtures/accounting_benchmark_source_snapshot_v1.1.0.json`.

Neither snapshot archived raw page bytes. Each source therefore has `source_bytes_archived=false` and `source_bytes_sha256=null`. The SHA-256 of a JSON fixture identifies that fixture only, never the source website's bytes.

## APEX: paper and live leaderboard are distinct cohorts

The timestamped v1.1.0 snapshot separates three source records:

1. **Methodology page:** the current methodology listing describes APEX-Accounting as 100 tasks and k=4.
2. **Published paper (29 July 2026):** describes a private 160-task held-out cohort across ten worlds. It reports 56.4% Mean Criteria@3 for the best evaluated model, 21.5% best Pass@8, no model above 2.6% Pass^8, and 58% of tasks never fully solved by any model attempt. The paper excludes human-in-the-loop tasks and explicitly identifies asking clarifying questions rather than assuming as a missing professional capability. It also excludes tax, audit, consolidation, multi-entity, multi-currency and external reporting.
3. **Live leaderboard at 19:39 SAST:** describes 100 tasks across ten worlds and 1,475 criteria. It shows Gemini 4 Argon High at 71.5% ±4.8 and the next displayed model at 70.7% ±4.8. It reports eight runs for Pass@1 / Pass@8 / Pass^8, while Mean Criteria is computed at k=4; its run cap is five million tokens and 500 steps, with 95% task-level bootstrap intervals over 10,000 resamples.

**Do not compare 71.5% with 56.4% as if one model improved over the other.** The live page uses a 100-task denominator; the paper describes 160 tasks. The sources do not explain whether the live 100-task view is a refreshed cohort, subset, different rubric cut or benchmark-version change. There is also a harness-description mismatch: the methodology card says single-turn/single-shot, while the paper and live leaderboard describe a Loop Harness. The paper's Mean Criteria@3 and live page's Mean Criteria (k=4) must retain their attempt semantics. Until Mercor publishes an explicit cohort, harness and rubric version mapping, preserve these source states separately and leave the relationships unresolved.

Links:
- Live leaderboard: https://www.mercor.com/apex/apex-accounting-leaderboard/
- Methodology: https://www.mercor.com/apex/methodology/
- Published paper: https://arxiv.org/abs/2607.27189

## FINBALANCE: separate task family

The FINBALANCE paper (Findings of EMNLP 2026) reports a 710-record evaluation split across eight industries, three period types and five difficulty levels, with 23 inconsistency labels. The best exact final balance-sheet accuracy among six evaluated LLMs is 46%. Reconstructing statements from journal entries improved four models by 26–41 percentage points.

This supports a distinct Mycelix hypothesis: **make source-bound economic events and journal entries an explicit intermediate representation, then independently reconstruct statements from the ledger.** It does not prove that the gain transfers to another corpus. FINBALANCE exact-statement accuracy is not comparable with APEX Mean Criteria.

Paper: https://aclanthology.org/2026.findings-emnlp.675/

## Matched evaluation protocol

Use the same permitted cases, source bytes, tools, permissions, retry policy, model budget and evaluator version. Freeze the split identity/digest (or a custodian-held commitment for hidden cases), candidate code commit, prompts, model/version, tool versions, rubric, evaluator, seeds, attempt count, timeouts, raw candidate outputs and tool traces. Candidate and oracle artifacts must remain physically separate; any oracle leakage invalidates a run.

Compare:
1. deterministic rules-only;
2. model-only;
3. model + conventional tools;
4. model + tools + ledger reconstruction;
5. typed source-bound economic-event graph;
6. typed graph + independent deterministic verifier;
7. Symthaea candidate analyst + Mycelix verifier;
8. qualified accounting professional on a blinded subset.

Report task and metric families separately:
- **APEX-aligned:** Mean Criteria, Pass@1, Pass@8, Pass^8, attempts, failures, runtime and cost. Use an authorized, cohort-matched harness; do not extrapolate from the paper to the current live denominator or vice versa.
- **FINBALANCE-aligned:** exact final balance-sheet accuracy, account/line accuracy, source-to-journal linkage, inconsistency detection by label and journal-reconstruction ablation. Preserve the record-level denominator.
- **Mycelix evidence-qualified:** source-binding completeness, exact ledger balance, period/entity/currency correctness, clarification precision, abstention quality, historical replay, false qualification, reviewer time and cost.

## Proposed Mycelix target gates (not achieved)

Create an independent expert-adjudicated corpus rather than claiming benchmark equivalence:
- S0: 50 synthetic tasks with public oracle and deterministic negative controls.
- S1: >=100 professionally reviewed cases, dual annotation and adjudication.
- S2: >=250 cases plus a hidden holdout, multi-entity/currency/accounting-policy strata.
- S3: authorized external evaluation only when access and terms permit.

Hard gates remain outside aggregate scores: 100% material source-locator binding; zero unbalanced journals; zero silent entity/currency/period coercion; 100% clarification/abstention for seeded decision-critical unknowns; zero stale/future source leakage; 100% exact replay of frozen inputs; zero false transfer qualifications or live ledger side effects.

After S1 is frozen, proposed quality goals are >90% exact statement accuracy on S1 and >95% on S2, >=95% decision-relevant clarification questions, <=5% redundant questions, >=90% of solvable ambiguity resolved with the minimal adjudicated question set and >=30% reviewer-time reduction without increased material errors. These are goals, not reported performance.

Use confidence intervals and per-stratum results. Report false positives/negatives, abstentions, selective risk/coverage, p50/p95 latency, fully loaded cost and reviewer time. Never average away a hard integrity failure.

## Rights and claim ceiling

APEX's full scored set is private; public metadata does not grant permission to copy or redistribute it. FINBALANCE artifact rights are not established by its bibliographic page; check exact terms before import. No dataset was copied in these snapshots.

This register establishes only what the public sources displayed or reported at capture time. It does not establish benchmark reproduction, source-byte integrity, dataset access, licensing, accounting correctness or superiority over SOTA.
