# PRAXIS-NRES-003: Anchored rubric for civic-resilience reasoning

**Status:** draft instrument for reviewer calibration. These anchors are proposed measurement rules, not an externally validated rubric or a political-policy answer key.

**Applies to:** PRAXIS-NRES-002-FIXTURE-001 and the capstone workbook. The rubric scores the quality of reasoning and evidence, never a learner's conclusion for or against compulsory service.

## Scoring procedure

Score each of the eight dimensions independently from 0 to 4. Use the dimension's observable evidence, not fluency, confidence, political views, identity, or agreement with the assessor.

- **0 — absent or contradicted:** required evidence is missing, materially false, or contradicted by the submitted work.
- **1 — weak:** some relevant work appears, but a major omission or unsupported leap makes the criterion unreliable.
- **2 — partial:** the core idea is present, but there are meaningful gaps, incomplete checks, or unresolved contradictions.
- **3 — strong:** the requirement is substantially met; remaining gaps are minor, explicit, and do not invalidate the claim.
- **4 — complete and auditable:** the requirement is met with traceable evidence, explicit checks, relevant limits, and a reproducible explanation.
- **Not assessable:** use only when an assessor cannot judge the criterion because the supplied task/material is defective or inaccessible. Record the reason; do not silently convert it to zero. Study scoring and missingness rules must be preregistered.

Scores 1 and 3 interpolate between the adjacent anchor descriptions. Assessors must cite a concrete span, calculation, or artifact for every score. Do not award extra points for length or jargon.

## R1 — Evidence discipline

**0:** Presents assumptions, estimates, or unknowns as observed facts; invents sources or treats a signature, popularity, or model output as proof that a claim is true.

**2:** Labels some uncertainty and supplies some evidence, but leaves material claims uncategorized or does not explain how important claims could be checked.

**4:** Separates fixture facts, assumptions, estimates, unknowns, and claims requiring qualified review; preserves conflicting sources; specifies independent verification for decision-sensitive claims.

**Evidence to cite:** claim ledger, citations/provenance, conflict treatment, verification owner and method.

## R2 — Legal authority and accountability

**0:** Invents authority, treats a course/AI output as a lawful decision, or recommends coercive policy while omitting basic legal process and safeguards.

**2:** Names relevant institutions and rights but leaves decision ownership, applicable process, review, or appeal materially underspecified.

**4:** Separates proposal from authorization; identifies the legitimate decision owner and required qualified review; records rights, accountability, oversight, review and appeal constraints without pretending the course itself determines the law.

**Evidence to cite:** authority map, decision boundary, review/appeal path, cited primary legal source where a legal claim is made.

## R3 — Resource and service feasibility

**0:** Exceeds stated budget/responder-hours without identifying infeasibility, or claims a service floor is met without a check.

**2:** Provides a plausible allocation but leaves arithmetic, units, dependencies, or one or more service-floor checks incomplete.

**4:** Reconciles all budget and responder-hour totals, directly checks each defined service floor, states units and constraints, and marks any infeasible trade-off explicitly.

**Evidence to cite:** allocation table, arithmetic, service-floor checks and infeasibility statements.

## R4 — Resilience and partition tolerance

**0:** Assumes central coordination or unavailable information remains available, with no contingency for the stated outage.

**2:** Identifies the coordination failure and offers local action, but does not define safe boundaries, stale-data handling, or reconciliation after communication returns.

**4:** Distinguishes local continuity from central coordination, defines authorized local scope, makes uncertainty/staleness visible, and specifies safe reconciliation and correction when coordination resumes.

**Evidence to cite:** dependency map, outage mutation response, local authority boundary and reconciliation rule.

## R5 — Equity, accessibility and rights

**0:** Ignores clear access barriers or proposes discrimination, political profiling, or coercion without rights review.

**2:** Names equity and accessibility but omits a concrete accommodation, care/work constraint, due-process safeguard, or evidence of whose needs were considered.

**4:** Identifies foreseeable access and distributional barriers; offers a concrete mitigation or a justified evidence request; preserves privacy, due process, nondiscrimination, and relevant alternative-service considerations.

**Evidence to cite:** safeguards, access/consultation plan and explicit constraints. Do not require learners to disclose personal disability, medical status, political beliefs, or service preference.

## R6 — Cost and opportunity-cost accounting

**0:** Treats duration or headcount as a cost/capability proxy, or presents unsupported cost/readiness numbers as established evidence.

**2:** Notes some direct costs but omits major lifecycle costs, training/instructor throughput, retention, support, or foregone work/study.

**4:** Defines a comparable cost boundary across options, identifies direct and indirect lifecycle costs and opportunity costs, distinguishes measured values from unknowns, and avoids fabricated precision.

**Evidence to cite:** cost boundary, input provenance, capacity assumptions and sensitivity analysis.

## R7 — Benchmark quality

**0:** Uses an unfair baseline, chooses outcomes after seeing results, lacks any falsification condition, or exposes purportedly held-out answers before evaluation.

**2:** Defines a reasonable outcome and comparison but leaves baseline comparability, blinded scoring, uncertainty, holdout integrity, or missingness underspecified.

**4:** Preregisters the primary outcome and analysis, justifies the baseline, uses blinded assessment, reports uncertainty and missingness, protects genuinely held-out cases, and states what evidence could refute the claim.

**Evidence to cite:** evaluation plan, preregistration fields, split policy, analysis plan and falsification condition.

## R8 — Uncertainty and revisability

**0:** States certainty beyond available evidence or refuses to revisit a conclusion after a decision-relevant assumption changes.

**2:** Acknowledges uncertainty but cannot say which new evidence would change the recommendation or does not respond coherently to a material scenario mutation.

**4:** Gives a conditional conclusion, names decision-changing evidence, updates the recommendation when constraints/objectives change, and explains what remains unresolved.

**Evidence to cite:** uncertainty ledger, variant response, change trigger and confidence reflection.

## Non-compensable review flags

These flags are recorded separately from the 0–32 score. They are not hidden by summation:

1. Fabricated material evidence or invented authority.
2. Unflagged violation of a stated resource constraint.
3. Claim that a service floor is satisfied when the provided allocation does not satisfy it.
4. Use of the course/AI/rubric as an official legal, operational, or practical certification authority.
5. Rights, due-process, or accessibility omission that materially changes a proposed coercive policy.
6. Political-affiliation, enlistment-propensity, identity, or loyalty scoring.

A flag requires a recorded explanation and second review before any program-level acceptance decision. During formative teaching, report the flag as feedback; do not convert it into a judgment about a learner's political beliefs or personal suitability for service.

## Assessor calibration and reliability

1. Train assessors on examples that are not in the public benchmark fixture.
2. Have at least two assessors independently score a calibration set before study scoring.
3. Discuss disagreements using cited evidence, then freeze the rubric before confirmatory assessments.
4. Double-score the preregistered sample; report dimension-level agreement and an ordinal agreement statistic with uncertainty where appropriate.
5. Treat weighted kappa of 0.75 as a provisional target hypothesis only, not a universal standard. If distribution/prevalence makes it unstable, preregister an alternative before confirmatory analysis.
6. Do not change anchors after viewing confirmatory results. Any necessary change creates a new rubric version and a new confirmatory cohort.

## Interpretation boundary

The score is a criterion-referenced learning feedback instrument. It is not a ranking of political beliefs, citizenship worth, loyalty, readiness to enlist, or individual suitability for compulsory service. Multiple opposing policy conclusions can earn equally strong scores when evidence handling, lawful process, feasibility, rights analysis and evaluation quality meet the same anchors.
