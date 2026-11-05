# External Accounting Benchmark Source Evidence v1.0.0

This directory records public APEX Accounting and FINBALANCE source metadata observed on **2026-11-05**. It is not a benchmark runner, task corpus, run receipt, dataset mirror or performance claim.

It composes with [FIN-ACC-EVAL-001 / #5799](https://github.com/Luminous-Dynamics/mycelix/issues/5799), which owns oracle isolation, matched experiment arms and canonical run manifests. This package deliberately does not create a competing run-manifest schema.

## APEX: three different snapshots

The source fixture keeps three APEX records separate:

1. **Methodology page:** the page still describes an earlier 100-task, k=4 design. It is marked historical and must not be used as the current paper cohort.
2. **Research paper:** reports 160 held-out tasks across ten synthetic company worlds, plus a public development set of ten tasks from one world. The paper's best reported Mean Criteria@3 is 56.4%. It also reports no evaluated model above 2.6% Pass^8 and 58% of tasks never fully solved in eight attempts. These are distinct metrics.
3. **Live leaderboard:** on the observation date, it described ten worlds, 160 tasks and 2,186 criteria, while displaying 61.0% in the headline, 61.8% in a model row and 56.4% in a cost table. The fixture preserves all three values and marks the page internally inconsistent. **No canonical current leaderboard score is selected.**

The paper covers U.S. GAAP accrual work in synthetic company worlds. It excludes tax, audit, consolidation, multi-entity, multi-currency and external reporting. It removed 24 incomplete-information/clarification tasks before evaluation, so it does not directly measure professional clarification behaviour. APEX is useful for close-cycle agent capability, not a comprehensive test of global accounting, tax or legal applicability.

## FINBALANCE: complementary, not comparable by headline

The paper reports a 710-record evaluation split, eight industries, three period types, five difficulty levels and 23 inconsistency labels. The best exact final balance-sheet accuracy among six evaluated LLMs is 46%. Reconstructing the balance sheet from journal entries improved four models by 26–41 percentage points.

This motivates a falsifiable Mycelix experiment: reconstruct an explicit journal/economic-event intermediate representation, then independently rebuild statements from it. Do not assume the gain transfers. A 46% exact balance-sheet score is not directly comparable with APEX Mean Criteria@3.

## Snapshot integrity and rights

Every source record sets `source_bytes_archived=false` and `source_bytes_sha256=null`. The URLs and observations were inspected; raw HTML/PDF bytes were **not** archived or hashed in this commit. Hashing this JSON fixture would identify this local record only, not the referenced web pages.

Before freezing an external baseline, archive permitted source bytes or stable versioned paper artifacts, hash the exact bytes, record paper revision/leaderboard observation and verify dataset terms. Keep source changes as new snapshot versions.

APEX's ten-task public development subset is a smoke/reproduction set only; the 160-task scored cohort requires authorized access. FINBALANCE dataset artifact rights are not established by this metadata snapshot; do not import or redistribute records until exact terms are reviewed.

## Matched evaluation protocol (compose with #5799)

Use the same permitted cases, evidence, tools, permissions, model budget, retries and evaluator. Freeze the dataset split/digest, candidate code commit, model/version, prompt/tool versions, rubric/evaluator, seeds, attempt count, timeout rules and raw traces. Keep candidate and evaluator artifacts physically separate. Any oracle leakage invalidates a run. Count abstentions, unsupported cases, crashes and timeouts in the denominator.

Compare deterministic rules-only, model-only, model + conventional tools, source-bound evidence graph, evidence graph + independent verifier, Symthaea candidate analyst + Mycelix verifier, and a qualified professional baseline on a blinded subset. Attribute gains to the right component; do not credit a model for deterministic correction or a larger budget.

- **APEX-aligned:** report Mean Criteria, Pass@1, Pass@8, Pass^8, attempt count, task-level failures, time and cost separately. Never present the 10-task public dev subset as the 160-task score.
- **FINBALANCE-aligned:** report exact final balance-sheet accuracy, source-to-journal linkage, inconsistency detection by label and the reconstruction-from-journal ablation. Preserve the record denominator.

## Mycelix-specific targets (proposed, not achieved)

Build an independent expert-adjudicated corpus rather than claiming equivalence to APEX or FINBALANCE.

- S0: ten synthetic data-flow fixtures, deterministic oracle conformance and negative controls.
- S1: at least 100 professionally reviewed cases stratified across controller/processor roles, support access, backup/DR, onward transfers, purposes, regime versions, retention and legal holds.
- S2: at least 250 cases with a pre-frozen hidden holdout, multi-reviewer adjudication, per-stratum results and confidence intervals.

Hard integrity gates remain outside weighted averages: zero false transfer qualifications/authorisations; zero accounting profile used as transfer authority; zero unknown location silently treated as local; zero missed seeded foreign support/backup/onward facets; zero deletion under an active hold; zero frozen replay mismatch; zero oracle leakage. After S1 is frozen, target at least 95% recall for seeded applicability axes, 100% critical source/profile-version binding and at least 30% reviewer-time reduction without increased material false qualification. These are target gates, not results.

No benchmark dataset was copied, no external benchmark was run, and no superiority over SOTA is claimed by this source-evidence bundle.
