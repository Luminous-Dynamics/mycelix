# Research baseline: side-effect-aware and time-complete institutional decisions

**Date:** 2026-11-14  
**Status:** literature-informed benchmark design; no vendor/model comparison has been run.

## 1. New agent evaluation baseline: SABench

Huang et al., *Task Success Is Not Enough: Side-effect-Aware Evaluation of Tool-Using Language Model Agents* (arXiv, 25 October 2026) introduce SABench, a 201-task / 47-template benchmark across email, calendar and ticket tools.

The benchmark crosses four state structures (independent, dependent, externally reachable, cross-tool) with four trap families (ambiguity, mis-targeting, cascade harm, overreach). Its deterministic trace oracle separately evaluates goal satisfaction and whether every realized write stayed inside an explicit authorization boundary.

The study reports:
- 15.2% collateral-damage rate across 2,211 model-task runs;
- 12.8% unauthorized-write rate even among goal-satisfied runs;
- 38.8% violations on ambiguity tasks;
- about twice the collateral-damage rate in dependent and cross-tool state structures relative to independent state structures.

These are study-specific findings, not a universal model failure rate. The important research implication is that final-state task success is insufficient. Permission must be checked for every state-changing operation.

Reference: https://arxiv.org/abs/2610.34167  
Artifact: https://github.com/hjj194/sabench-artifact  
Dataset: https://huggingface.co/datasets/Phaedrus/sabench

### IAE adaptation

Our vertical slice now records a normalized possible-write trace and an explicit authorization boundary:
- action;
- purpose;
- actor;
- target asset;
- modified fields;
- authorization reference;
- whether the operation may affect an external party.

The evaluator checks every submitted write-intent record against this boundary and reports unauthorized side effects separately from decision correctness. It also checks that the write-trace event maps to the same operation/idempotency identity as the timeout record.

The synthetic lease submission is a *possible* external side effect with unknown external visibility, not a confirmed completed lease. It must be reconciled before any further mutation.

This is intentionally stricter than checking only the final execution state. The next benchmark expansion should add 4-way and cross-tool variants: dependent records, externally reachable counterparties, ambiguous asset identity, cascading downstream notifications, wrong-asset targeting, extra-field writes, and unrelated-system updates.

## 2. Temporal retrieval: TEMPO and decision-time evidence coverage

TEMPO (EMNLP 2026) contains 1,730 complex temporal retrieval queries, 3,976 decomposed retrieval steps, 13 domains and an evaluation of 16 retrieval systems. Its best reported result is 32.0 NDCG@10 and 71.4% Temporal Coverage@10.

Reference: https://aclanthology.org/2026.emnlp-main.291/

TEMPO measures retrieval relevance and coverage across required time periods. Our DTEC proposal is not directly comparable: it measures whether every critical source/version/authority reference required for a particular decision-time slice is recovered and correctly bound to effective time, knowledge time, source competence and authority.

The vertical slice's Q-HISTORICAL and Q-CURRENT-REPLAY queries deliberately hold effective time constant while changing knowledge time. The v5 land-use correction is effective 2026-01-15 but observed 2026-02-05. The historical query must retain v4; the later knowledge view must select v5.

## 3. Requirements quality and human judgment

Shefa et al., *Two Truths and A Lie? Benchmarking Off-the-Shelf LLMs for Requirements Quality Assessment* (3 September 2026) report that the best evaluated model detected a median 47% of expert-identified issues and false-flagged 11% on their particular corpus and protocol. Necessity and correctness judgments were especially difficult.

Reference: https://arxiv.org/abs/2609.03230

This supports our decision to separate modality/source fidelity, semantic adjudication, claim competence, authority and human acceptance. An LLM-generated requirement remains a candidate until a person with appropriate institutional authority accepts it.

## 4. Empirical reproducibility guidance

Baltes et al., *Guidelines for empirical studies in software engineering involving large language models* (published 3 November 2026) recommend reporting model roles and versions, configurations/customizations, system/prompt design, interaction/session traces, suitable baselines and metrics, open-model baselines, human validation, and limitations.

Reference: https://link.springer.com/article/10.1007/s10664-026-10922-3

Our benchmark submission protocol should therefore capture:
- evaluation arm and system version/build;
- model/provider/version when a model is used;
- prompt/system-instruction digest and tool-schema digest;
- temperature, sampling and step limits;
- source corpus, oracle, submission and evaluator digests;
- raw interaction/tool-call trace and normalized write trace;
- adjudicator protocol and disagreements.

A deterministic rule-based reference, expert process, incumbent configuration, direct LLM and integrated Luminous system should all be included. Matching input evidence and reviewer budget is essential.

## 5. Critical-infrastructure trustworthiness

NIST's AI RMF Profile on Trustworthy AI in Critical Infrastructure is an ongoing profile-development project, initiated April 2026 and updated July 17, 2026. NIST describes its aim as actionable AI risk-management guidance for critical-infrastructure operators, developers and supply-chain stakeholders.

Reference: https://www.nist.gov/programs-projects/concept-note-ai-rmf-profile-trustworthy-ai-critical-infrastructure

This is a useful design reference for lifecycle, safety, security, reliability and supply-chain assurance. It is an ongoing profile project, not a finalized certification or evidence that IAE conforms to it.

## 6. Updated performance claim

The near-term competitive claim should be narrow:

> On a matched, independently adjudicated workload, Luminous will reduce unsupported claim promotion, unauthorized side effects, temporal reconstruction errors and unnecessary evidence requests while reducing expert review/reconstruction effort.

The claim must be measured against configured incumbent systems, traditional rules/checklists, expert reviewers, open and closed LLM baselines, and a direct LLM workflow. No SOTA superiority or benchmark performance is claimed at this development stage.

## 7. Metrics to report separately

- Goal/disposition correctness.
- Unauthorized side effects and externally visible unauthorized effects.
- False clears on hard constraints.
- Authority correctness at action time.
- Decision-time evidence coverage and temporal projection accuracy.
- Claim-dimension/competence violations.
- Unknown-outcome retries and duplicate mutations.
- Modality, condition and source-span fidelity.
- Unnecessary evidence requests and disclosure burden.
- Human review and reconstruction time.
- Abstention quality and unresolved-question recall.

Do not create a composite score that allows better task completion or speed to compensate for unauthorized writes, rights violations, privacy exposure or safety false clears.
