# Craft Claim Support V1 — Benchmark Protocol

- **Work item:** CRAFT-CLAIM-SUPPORT-001 (#6752)
- **Status:** Proposed qualification protocol; no measurements yet
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## 1. Research question

Can a source-grounded claim-support pipeline reduce unsupported claims entering ratified community projects by separating provenance from semantic entailment, retaining qualifiers and contradictions, and requiring independent domain review—without creating unacceptable abstention burden?

Do not benchmark fluency, rationale length or LLM agreement as a proxy for correctness.

## 2. Matched baselines

Use the same claim, exact source packet, locator, time, jurisdiction and review budget for:
1. Citation-existence/digest validation only.
2. Generic single-model LLM-as-judge.
3. Retrieval-grounded LLM entailment classification.
4. Multi-agent claim and counterclaim review.
5. NLI/semantic textual similarity models with appropriate calibration.
6. Human domain-expert assessment.
7. Independent human source-authority review.
8. Symthaea-assisted assessment with mandatory human confirmation.

Report model/provider versions, retrieval indices, prompt/method digests, source versions, compute/time budget and correlation/common-cause assumptions. A baseline that ignores units, negation or jurisdiction is not semantically equivalent.

## 3. Frozen corpus

Build three tiers:
- **Synthetic conformance:** controlled claim/source pairs with gold qualifier states and mutation seeds.
- **Public corpus:** consented/de-identified source passages, technical reports, standards extracts and claim variants with adjudicated annotations.
- **Restricted field corpus:** community- and source-owner-governed material; publish aggregate metrics only.

Each episode includes the exact claim, source version, relevant excerpt, locator, source context, domain/jurisdiction/time metadata, qualifier annotations, independent reviewer roles and adjudication history. Use at least two independent domain annotators for high-consequence data; retain disagreements and confidence intervals rather than forcing a false consensus.

Split by source document, system, institution and time so that near-duplicate reports and shared source text cannot leak across train/test. Keep a prospective holdout.

## 4. Adversarial families

- Negation reversal: “not safe” → “safe”.
- Numeric scale: 10 L/min → 10 L/day.
- Unit conversion, dimension mismatch and rounded thresholds.
- Temporal staleness: historical measurement presented as current.
- Population/scope generalization.
- Jurisdictional overgeneralization.
- Modal strengthening: “may” → “will”.
- Correlation → causal claim.
- Omitted uncertainty, confidence interval or qualifying condition.
- Cited document contains relevant vocabulary but wrong component/system.
- Passage-level source truncation or context omission.
- Summary cites an underlying source incorrectly.
- Conflicting primary and secondary sources.
- Contradictory source hidden behind positive evidence selection.
- Source revision or correction after assessment.
- OCR/transcription corruption of decimal, negation, unit, speaker or date.
- Translation/code-switching semantic drift.
- Out-of-domain or unqualified assessor.
- Multiple LLMs share the same source/retrieval error.
- Independent review expired, withdrawn or bound to old assessment digest.
- Claim changed after assessment but old support reused.

## 5. Metrics

| Measure | Definition | Proposed target |
| --- | --- | --- |
| Direct-support precision | Claims marked DirectSupport that are genuinely supported as worded | ≥ 0.99 |
| False-positive support | Unsupported claims marked DirectSupport | 0 in frozen adversarial corpus |
| Contradiction recall | Material contradictions detected and escalated | ≥ 0.99 |
| Partial-support calibration | Partial/stronger-claim mismatch not promoted to direct support | 0 false promotions |
| Qualifier preservation | Correct retention of required semantic dimensions | 100% frozen corpus |
| Source revision invalidation | Reuse after source/claim digest change | 0 |
| Review independence | Assessor approves their own assessment | 0 |
| Role coverage | Supported result without every required reviewer role | 0 |
| Abstention calibration | Correct NotAssessable/ContextRequired/ReviewRequired decisions | report per class and confidence |
| Cross-implementation replay | Independent evaluator reproduces frozen disposition | 100% |
| Review burden | Human minutes per supported claim and review queue age | measured, not optimized alone |

These are proposed qualification gates, not achieved results. Report confusion matrices, precision/recall, class imbalance, confidence intervals and domain-specific slices. Never hide a dangerous false acceptance inside an aggregate.

## 6. Statistical and independence controls

- Separate source truth/authority annotation from semantic support annotation.
- Use blinded reviewers and independent adjudication.
- Record model/provider family, shared retrieval corpora and common-source dependencies.
- Cluster confidence intervals by source document and project rather than treating every claim variant as independent.
- Evaluate high-consequence, multilingual and low-resource strata separately.
- Include explicit no-ground-truth and conflicting-source examples.
- A model rationale is not an independent evidence source.
- Do not count multiple model calls from one model/provider as independent human confirmation.

## 7. Better-than-SOTA strategy

Compete on *false acceptance under real-world constraints*, not just aggregate NLI accuracy:
1. Zero dangerous false acceptance in frozen high-consequence tests.
2. Higher direct-support precision while preserving reasonable recall.
3. Better qualifier and context retention across paraphrase, transcription and translation.
4. Stronger contradiction and stale-source handling.
5. Calibrated abstention with manageable expert-review burden.
6. Robustness against common-cause model/source failures.
7. Deterministic, digest-bound independent replay.

The target is an evidence assurance boundary around model-assisted claims. It is not a claim that Symthaea has more general knowledge or inherently reasons better than commercial models.

## 8. Standards and industry baselines

- W3C PROV-DM for provenance entities, activities, agents and derivation.
- W3C VC Data Model 2.0 and Bitstring Status List 1.0 for compatible credentials/status.
- ISO/IEC/IEEE 29148:2018 and INCOSE requirement-writing guidance for statement quality.
- NIST AI RMF for validity/reliability, safety, robustness, transparency, privacy and fairness.
- ISO/IEC 42001 for organizational AI management system context.

Commercial LLM evaluation, document intelligence and requirements-management vendors already provide citations, source-grounded drafting, review workflows and traceability features. Compare their actual configured outputs; do not assert that citations or human review are unique to Craft. The test is whether evidence qualifiers, source revisions, semantic contradictions, independent role approvals and exact digest invalidation survive end-to-end.

## References

- Dąbrowski et al. (2026), *Large Language Models for Requirements Engineering: A Cross-Task Empirical Evaluation*: https://arxiv.org/abs/2608.21531
- Shefa et al. (2026), *Two Truths and A Lie? Benchmarking Off-the-Shelf LLMs for Requirements Quality Assessment*: https://arxiv.org/abs/2609.03230
- Sabatucci et al. (2026), *A Methodology for the Systematic Evaluation of LLMs in Requirement Engineering Tasks with No Ground Truth*: https://doi.org/10.3390/systems14060598
- W3C PROV-DM: https://www.w3.org/TR/prov-dm/
- W3C Verifiable Credentials Data Model 2.0: https://www.w3.org/TR/vc-data-model/
- W3C Bitstring Status List 1.0: https://www.w3.org/TR/vc-bitstring-status-list/
- ISO/IEC/IEEE 29148:2018: https://www.iso.org/standard/72089.html
- INCOSE Guide to Writing Requirements V3.1: https://www.incose.org/publications/products/se-handbook/guide-to-writing-requirements
- NIST AI Risk Management Framework: https://www.nist.gov/itl/ai-risk-management-framework
- ISO/IEC 42001:2023: https://www.iso.org/standard/42001
