# External Finance Benchmark and Industry Evidence Register

This directory records public benchmark-source observations and selected vendor product announcements. It is not a benchmark runner, task corpus, run receipt, dataset mirror or performance claim.

It composes with [FIN-ACC-EVAL-001 / #5799](https://github.com/Luminous-Dynamics/mycelix/issues/5799), which owns candidate/oracle isolation, canonical run manifests/receipts and matched experiment arms. No competing run-manifest schema is created here.

## Snapshot history

- **v1.0.0** — earlier 5 Nov 2026 source observation without a capture time.
- **v1.1.0** — 5 Nov 2026, 19:39 SAST.
- **v1.2.0** — 5 Nov 2026, 22:12 SAST; added the official APEX publisher article.
- **v1.3.0** — 6 Nov 2026, 13:59 SAST; rechecked the APEX live leaderboard and separately represented alternate page rendering.
- **v1.4.0** — **7 Nov 2026, 10:34 SAST**; schema `benchmark_source_evidence_v1.4.0.schema.json`, fixture `fixtures/accounting_benchmark_source_snapshot_v1.4.0.json`; 13 source records and 96 observations across eight benchmark/product identities.

None of the snapshots archives raw HTML/PDF bytes. Every source therefore has `source_bytes_archived=false` and `source_bytes_sha256=null`. The JSON fixture itself can be fingerprinted, but that hash identifies the fixture, not the source website bytes. Public metadata access does not grant redistribution rights for benchmark tasks or products.

## APEX-Accounting: preserve cohort and harness distinctions

The v1.4.0 records preserve the methodology card, technical paper, publisher article, live leaderboard and alternate live-page rendering separately.

The official live leaderboard, rechecked 7 Nov 2026, displays 10 worlds, 100 held-out tasks, 1,475 grading criteria and a highest listed score of **71.5% ±4.8**. It notes that first-attempt full completion remains low, consistency can decline across repeated attempts, higher cost does not guarantee higher scores, and models have different task blind spots.

The July 2026 paper and publisher article describe an earlier **160-task / 10-world** held-out cohort. The paper reports a leading 56.4% Mean Criteria@3, best Pass@8 of 21.5%, no model above 2.6% Pass^8 and 58% of tasks never fully solved in any attempt. The official article describes the one-world/ten-task open sample, 40+ professional authors, median 11 years' experience, 13.7 rubric criteria/task and 97% judge/expert agreement. It also notes that clarification tasks are out of scope.

**Do not compare 71.5% and 56.4% as progress on one unchanged benchmark.** The denominators, metric attempt semantics and harness descriptions differ. The method card's single-turn/single-shot text also differs from the later paper/live Loop Harness description. No official mapping between all cohort and harness versions is established here.

The official article's failure taxonomy says roughly seven in ten top-three-model failures were reasoning failures rather than retrieval failures; one example is an agent discovering a discrepancy then dropping or contradicting it in the final journal. This supports the Mycelix evidence-to-decision continuity and trajectory-evaluation tracks.

Sources:
- Live leaderboard: https://www.mercor.com/apex/apex-accounting-leaderboard/
- Methodology: https://www.mercor.com/apex/methodology/
- Paper: https://arxiv.org/abs/2607.27189
- Publisher article: https://www.mercor.com/blog/introducing-the-ai-productivity-index-for-accounting/

## FINBALANCE: source-to-journal and ledger-reconstruction evidence

FINBALANCE is a different benchmark family: 710 records, eight industries, three period types, five difficulty levels and 23 inconsistency labels. The paper reports maximum exact final balance-sheet accuracy of 46% across six evaluated models. For four models, reconstructing statements from generated journal entries improves the result by 26–41 percentage points. It also reports that citation-pressure prompting barely changed document-linking errors while ledger feedback substantially improved reported statement consistency.

This directly motivates the source-bound event graph and journal-replay verifier. The result is not an APEX score and cannot be averaged with APEX Mean Criteria.

Paper: https://aclanthology.org/2026.findings-emnlp.675/

## New benchmark coverage (v1.4.0)

| Benchmark | What it tests | Reported result / scale | Mycelix implication |
|---|---|---|---|
| ProFinR | Professional finance reasoning across four task areas; 53 tools / 13 categories; DAG planner and case-based memory | 528 expert-designed problems; authors report +49.81% vs. their baselines and 47.1% lower latency | Compare sequential vs DAG/evidence-graph workflows; pin each paper's baseline and budget |
| ReimburseBench | Full-workflow multimodal reimbursement controls, firm policy, receipts and approval evidence | 151 workflows; 12 audit logic points; best 72.5% overall, none above 47.7% on hard cases | Add visual evidence, policy admissibility, quantitative precision and risk-sensitive leniency metrics |
| ForensicBench | Coordinated journal-entry fraud schemes rather than isolated anomalies | Five scheme types; 12 open-weight models; best Entry-F1 34.7%, Type-F1 lower | Measure scheme-level fraud instance and scheme classification separately |
| TraceProbe | Execution-based safety: non-adversarial failures, internal authority compromise and external attacks | 155 scenarios; 12 models; mean attack success rate 0.347 | Score latent intermediate violations even if the endpoint looks safe |
| AgentRx | First critical failure localization from long-horizon trajectories | Published paper: 170 annotated trajectories across 11 settings; reports 75% average step-localization improvement over prior work | Preserve first-unrecoverable-step, propagation and containment labels in finance-agent traces |

These metrics belong to their own task definitions and experimental protocols. ProFinR's relative improvement is not a universal financial-agent score; ReimburseBench's process rubric is not APEX; ForensicBench F1 is not accounting accuracy; TraceProbe ASR is not task completion; AgentRx localization improvement is not end-to-end success. Do not combine them into one SOTA percentage.

Sources:
- ProFinR (ICML 2026): https://proceedings.mlr.press/v306/huang26ap.html
- ReimburseBench (Findings EMNLP 2026): https://aclanthology.org/2026.findings-emnlp.821/
- ForensicBench (EMNLP Industry 2026): https://aclanthology.org/2026.emnlp-industry.147/
- TraceProbe (EMNLP 2026): https://aclanthology.org/2026.emnlp-main.1233/
- AgentRx (Findings EMNLP 2026): https://aclanthology.org/2026.findings-emnlp.598/

## Vendor product evidence: BlackLine Verity Prepare

BlackLine announced general availability of Verity Prepare on **27 July 2026**. The vendor describes a multi-agent reconciliation-preparation workflow that analyzes support documents, matches transactions, identifies reconciling items and assembles an audit-ready package for human sign-off. These are vendor-stated capabilities, not independently measured benchmark results.

This changes our positioning. Do not attempt to rebuild BlackLine's financial-close orchestration. Mycelix should provide a portable, independent evidence/provenance and trajectory-verification plane that can qualify work produced by BlackLine and other platforms, replay the evidence-to-decision path, and detect policy/authority failures across tools.

Vendor announcement: https://investors.blackline.com/news-releases/news-release-details/blackline-advances-governed-ai-finance-general-availability

## Mycelix benchmark programme and matched comparison

Current public synthetic work:
- [#5858 evidence-qualified accounting S0](https://github.com/Luminous-Dynamics/mycelix/pull/5858): candidate/oracle separation, source-to-journal and financial-statement reconstruction.
- [#5867 event graph](https://github.com/Luminous-Dynamics/mycelix/pull/5867): bitemporal event semantics and policy-projection binding.
- [#5908 trajectory S0](https://github.com/Luminous-Dynamics/mycelix/pull/5908): first critical failure, latent violations and containment.
- [#5799 benchmark orchestration](https://github.com/Luminous-Dynamics/mycelix/issues/5799): canonical run receipts and matched arms.

Use identical rights-cleared inputs, source versions, model/tool versions, permissions, retries, prompts, budgets, seeds and evaluator versions. Compare rules-only, model-only, model+tools, model+ledger feedback, DAG planning, event-graph+verifier, Symthaea+Mycelix, and professional reviewers. Preserve candidate/oracle separation and raw trajectories.

Proposed progression:
- **S0**: synthetic smoke cases with deterministic negative controls.
- **S0.1**: >=50 adversarial synthetic cases per track, including randomized step insertion and hidden latent violations.
- **S1**: >=100 dual-reviewed and adjudicated professional cases per core track.
- **S2**: >=250 cases plus hidden holdout across currencies, entities, frameworks, time frontiers, source systems and industry sectors.

Proposed hard gates (targets, not results): zero false-safe critical control outcomes; 100% critical source-binding completeness; zero unbalanced journals; zero future-evidence leakage; zero false cross-tenant merges; 100% exact replay of frozen evidence/policy inputs; zero unauthorized successful side effects. Report accuracy, safe coverage, abstention, false-safe rate, latency, reviewer time and cost separately. Do not average away a critical integrity failure.

## Rights and claim ceiling

Public bibliographic records are not permission to mirror datasets. APEX's full scored set is private; its public sample does not grant rights to hidden tasks. ForensicBench and AgentRx advertise public releases, but exact code/data licenses must still be checked before import. ReimburseBench and FINBALANCE artifact terms must be reviewed separately. BlackLine product claims remain vendor claims.

This register establishes only what public sources report or vendors announce. It does not establish benchmark reproduction, raw-source byte integrity, dataset licensing, production product performance, accounting correctness, audit opinions or superiority over SOTA.