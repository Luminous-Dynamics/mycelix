# Craft Decision Closure V1 — Benchmark Protocol

- **Work item:** CRAFT-DECISION-CLOSURE-001 (#6756)
- **Status:** Proposed benchmark protocol; no results measured
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## 1. Research question

Can a conclusion-relative material-element closure check detect when a set of individually supported claims still fails to justify the overall decision, while preserving exceptions, scope conditions, authority, dependencies and alternate expert mappings?

The core research reference is Gupta (2026), published 20 November 2026, DOI https://doi.org/10.1007/s44163-026-02305-w. It distinguishes source existence, atomic claim support and conclusion-relative closure. Its framework is theoretical and explicitly calls for empirical validation of material-element mappings, inter-rater agreement, audit time and field use. The Craft benchmark should address those gaps rather than claim the paper's framework has already been validated.

## 2. Matched baselines

Apply the same decision conclusion, authorized corpus, claims, source set and domain standard to:
1. Citation existence only.
2. Atomic claim support only (including Claim Support V1).
3. Retrieval answer-faithfulness / sufficient-context evaluation.
4. Manual domain checklist.
5. Domain-expert audit using the nine-step reviewer protocol in the paper.
6. Decision Closure V1 with one approved mapping.
7. Decision Closure V1 with multiple independently reviewed mappings and dependency checks.

A baseline that does not represent material elements, exceptions or dependencies must be described as non-equivalent, not as a failing implementation of a capability it never claimed.

## 3. Frozen dataset design

Create three tiers:
- **Synthetic conformance:** exact ground-truth material elements, claims, support sets, exceptions and dependencies. Publish generator, seeds and immutable vectors.
- **Public, consented and de-identified corpus:** real but publication-permitted domain decisions, standards, source evidence and independently annotated mapping alternatives.
- **Restricted field corpus:** held under the participating authority/community's access and retention rules; publish aggregate results only.

Start with distinct domains:
- regulatory/compliance control evidence;
- safety and engineering assurance;
- clinical guideline recommendation;
- public-service/administrative eligibility;
- infrastructure and asset commissioning.

Split by domain corpus, institution, conclusion family and time. Keep a prospective holdout. Avoid treating repeated claims from one source corpus as independent observations.

## 4. Material-element annotation

For each conclusion:
1. Define the conclusion at the consequence level a decision-maker will rely on.
2. Decompose generated output into atomic claims and conclusion-bearing claims.
3. Verify source existence, exact locator and authorized corpus status.
4. Obtain claim-support judgments for each claim.
5. Construct the material-element mapping from the domain instrument, authority guidance or accepted professional practice.
6. Identify exceptions, scope, population, jurisdiction, effective dates and conditional dependencies.
7. Record the evidence set supporting each element.
8. Record element dependencies and not-applicable decisions.
9. Obtain independent mapping reviews and preserve alternative defensible mappings.

Use at least two independent domain reviewers per mapping role in the field qualification protocol. Preserve mapping disagreements; do not force a single gold decomposition where multiple mappings are defensible.

## 5. Metrics

| Measure | Definition | Proposed target |
| --- | --- | --- |
| False closure | Decision marked Closed with any required element unsupported | 0 frozen adversarial cases |
| Required-element recall | Expert-identified material elements represented in an approved mapping | ≥ 0.99 |
| Exception recall | Applicable exceptions/scope conditions represented | ≥ 0.99 |
| Dependency integrity | Dependent elements counted while prerequisites remain unsatisfied | 0 |
| Cycle detection | Four-element directed graph classified incorrectly against independent reachability oracle | 0 across all 4,096 edge configurations |
| Cross-scope evidence acceptance | Result from wrong domain/jurisdiction/corpus accepted | 0 |
| Cross-proposal evidence acceptance | Support result from an older or unrelated proposal revision accepted | 0 |
| Stale support acceptance | Expired, invalidated or older-than-policy result counted | 0 |
| Not-applicable integrity | Element excluded without authority decision | 0 |
| Mapping reviewer integrity | Approval accepted for changed mapping digest or missing quorum | 0 |
| Mapping agreement | Independent reviewers agree on element existence, scope, applicability and dependency | report per domain |
| Verdict sensitivity | Closure verdict flips across defensible alternate mappings | measure and report |
| Replay | Independent implementation reproduces all frozen closure vectors | 100% |

Targets are proposed qualification requirements, not measured results. The zero-false-closure target applies to the frozen conformance/adversarial set, not to universal real-world risk.

## 6. Adversarial families

- all atomic claims are supported, but one decision material element is missing;
- rule is supported, implementation is not;
- implementation is supported, but timing or actor authority is missing;
- general rule is cited but an exception applies;
- evidence concerns a different population or jurisdiction;
- monitoring or documentation is missing;
- one mapping merges elements that another splits;
- one reviewer omits an element and another identifies it;
- mapping revision changes after approval;
- not-applicable status is inferred from silence;
- all elements have claim support but a prerequisite ordering is unsatisfied;
- a clinical recommendation has risk stratification but omits contraindication or bleeding-risk assessment;
- multiple positive sources are present alongside one contradictory source;
- support result is stale, withdrawn, older than its maximum-age policy or tied to an older claim/source/standard/proposal revision;
- alternative mappings yield different closure verdicts.

## 7. Inter-rater reliability and mapping sensitivity

This is the benchmark's most important research challenge. Measure separately:
- agreement on element inclusion/omission;
- agreement on element split/merge;
- agreement on applicability and exception interpretation;
- agreement on dependency structure;
- agreement on which support results cover each element;
- closure verdict agreement;
- sensitivity of the verdict when each defensible mapping is applied to the same evidence set.

Report raw agreement and suitable chance-corrected statistics only where their assumptions fit the annotation structure. Report adjudication effort and domain-specific variance. A high agreement score does not prove that reviewers used a complete mapping; compare with enumerated standards where authoritative element structures exist.

## 8. Better-than-SOTA strategy

Do not claim that the closure concept is wholly novel: the 20 November 2026 paper formalizes it. Compete through operational validation and robust execution:
1. A reference implementation that fails closed on missing material elements.
2. Dependency-aware closure rather than flat element checklists.
3. Versioned, locally authorized mapping alternatives.
4. Measured inter-rater agreement and mapping-verdict sensitivity across domains.
5. Exact digest-bound evidence and reviewer replay.
6. Clear, element-level explanation of why a decision is open.
7. Field measurement of reviewer effort and decision quality against manual domain audits.

The decisive metric is false closure under a frozen adversarial corpus, not model fluency or an aggregate citation score.

## 9. Prospective pilot

A pilot should choose a bounded conclusion such as commissioning acceptance of a community water-pump system. Candidate material elements may include:
- approved design and applicable standard;
- electrical and mechanical inspection;
- authorized assessor and current competence;
- safety and exception handling;
- pressure/flow and water-quality test;
- record of corrections and retest;
- owner acceptance and documentation.

The element set must be authored and approved by the applicable domain/safety/resource owners, not invented by an LLM. Test whether supported source claims fail to close the whole commissioning conclusion when one element is deliberately withheld. Measure actual commissioning outcomes independently.

## References

- Gupta, H. (2026), “Regulatory evidentiary closure as a formal framework for citation faithfulness in large language models,” *Discover Artificial Intelligence* 6, article 1919, published 20 November 2026: https://doi.org/10.1007/s44163-026-02305-w
- W3C PROV-DM: https://www.w3.org/TR/prov-dm/
- NIST AI Risk Management Framework: https://www.nist.gov/itl/ai-risk-management-framework
- ISO/IEC/IEEE 29148:2018: https://www.iso.org/standard/72089.html
