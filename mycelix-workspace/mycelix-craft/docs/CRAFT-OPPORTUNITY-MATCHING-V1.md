# Craft Opportunity Matching V1

- **Work item:** CRAFT-MATCH-001
- **Status:** Research and reference implementation; live opportunity retrieval and production evidence verification remain pending
- **Version:** 1.0-draft
- **Reference date:** 2026-11-26

## 1. The problem with conventional job matching

A typical matcher turns profile attributes into one score and ranks people or jobs. That is useful for search triage, but it can collapse several distinct questions:

- Did the person actually perform the skill?
- Is the evidence authentic and current?
- Was the skill demonstrated in the required context?
- Is the credential legally or professionally required?
- Does the person consent to disclose that evidence?
- Is the requirement truly mandatory, or can an equivalent pathway satisfy it?

A self-declared skill tag is not an independently verified competence claim. A credential signature proves that an issuer made a signed claim; it does not by itself prove safe performance in a real work setting. A missing record means unknown, not necessarily incapable.

Craft should therefore move from **person-score ranking** toward **requirement-level evidence matching**.

## 2. The paradigm shift: from scores to requirement evidence graphs

Each opportunity is a versioned set of requirements. Each requirement points to one or more acceptable evidence classes and a domain-specific assessment policy.

A minimum future model:

| Record | Required meaning |
| --- | --- |
| Opportunity requirement | Stable requirement ID, skill/competency reference, task context, level, jurisdiction, whether mandatory, recency window, and allowed alternative pathways |
| Evidence reference | Subject, issuer/observer, source system, claim scope, artifact digest, observation time, status reference, disclosure rule and evidence type |
| Assessment | Met / Not met / Unknown / Human review, with exact evidence references, policy version, rationale and expiry |
| Candidate disclosure | Exact claims/evidence the candidate chose to reveal for this opportunity, with purpose and expiry |
| Match explanation | Which requirements are met, missing or unknown, why the opportunity was surfaced, and what evidence or learning could close a gap |

The result is not a universal score of a person. It is a transparent explanation of how the disclosed evidence relates to this particular opportunity.

### Tri-state is the default

- **Met:** permitted evidence supports the exact requirement under the named policy.
- **Not met:** evidence was assessed and does not satisfy the requirement.
- **Unknown:** evidence is missing, inaccessible, stale or insufficient to assess.

Unknown must not be converted into Not met by default. For non-regulated and negotiable requirements, a candidate should remain discoverable and be able to present equivalent experience or request review. Where law, safety or licensing requires a mandatory qualification, the relevant authority's policy still applies; Craft must not invent an exception.

## 3. What the current matcher actually does

The current Leptos matcher operates on a local subset and uses string overlap between supplied skills and job requirements. Its score weights required skills, preferred skills and an education-label match. It is a lightweight heuristic, not a learned model and not a verified competence assessment.

The source audit identified these defects, addressed in the accompanying implementation slice:

1. An absent education requirement awarded an education bonus.
2. A free-text PhD label acted as a universal substitute for other education labels.
3. A missing or below-threshold aggregate vitality value removed jobs from the results.
4. A card presented declared education as a “Credential match”.
5. Demo job postings were loaded regardless of live/mock mode.
6. The live posting button sent a generic placeholder rather than a completed posting form.
7. The UI has no working indexed read API for live job discovery.

The immediate implementation removes the education bonus when no requirement exists, requires exact declared-label matching, treats the legacy aggregate signal as an annotation rather than a hard gate, labels the heuristic as advisory, shows demo jobs only in mock mode, and disables placeholder posting/application writes. Live retrieval and real posting forms remain open work.

## 4. Evidence and standards interoperability

Craft should not become another credential issuer or duplicate Identity/Praxis data.

- **Identity / Lawful Identity:** issuer and holder identity, credential proof, status, issuer authority and revocation.
- **Praxis:** learner-controlled course progress, practice and learning opportunities. Progress is evidence of participation/completion under its policy, not automatic occupational competence.
- **Craft:** opportunity requirements, candidate-selected disclosure, match explanation, work scope, acceptance evidence and work-agreement references.
- **Work Evidence V1:** portable work-claim and artifact references with a claim ceiling; do not reinterpret a signed statement as physical ground truth.
- **Finance / local issuers / resource owners:** instrument profiles, issuance, balances, budget authority, resource access, payment and settlement. Craft only references their owner-scoped obligations.
- **Symthaea:** advisory analysis only (see below).

Prefer versioned adapters and shared wire contracts. Do not duplicate credential signatures, learner progress or financial state in Craft. Never assume a credential is valid because a UI field says it is.

## 5. Symthaea integration: useful without becoming the authority

Symthaea can improve the experience if it is a **reasoning and learning assistant**, not an authority oracle.

Permitted advisory functions:
- extract candidate requirements from job descriptions with cited source spans;
- propose skill-taxonomy mappings and explain uncertainty;
- identify gaps between disclosed evidence and a specific requirement;
- recommend Praxis learning/practice pathways;
- draft questions for an employer or assessor where evidence is ambiguous;
- explain why a job was surfaced and what additional evidence could change the assessment;
- abstain when a required source, ontology or status cannot be checked.

Hard boundaries:
- no Symthaea-generated skill claim becomes verified evidence;
- no inferred semantic similarity silently converts into requirement satisfaction;
- no consciousness, vitality, reputation or global person score authorizes work;
- no hire, license, payment, guild assignment or credential issuance is executed by a recommendation;
- model output must include model/version, input references, evidence citations, uncertainty, policy version and human review state;
- poisoned job descriptions and profile text are untrusted data, never executable instructions.

Use Symthaea to expand a person's path to capability, not to close an opportunity because their profile is unfamiliar.

## 6. Economic and governance composition

An opportunity can link to one or more independently typed economic components: wages, cooperative participation, mutual credit, time exchange, grants, commons access, in-kind resources or voluntary contribution.

The matcher must not optimize for compensation token balance, MYCEL reputation, or a single economic instrument. Candidates should be able to compare the actual terms and obligations. Local guilds choose their governance policy; their honorifics and reputational systems do not become a global employment rank.

## 7. Industry baseline and how to compete

The individual ingredients are not novel:

- Work marketplaces already support contracts, milestones and payment/escrow processes.
- Enterprise talent-intelligence products already offer skills-based candidate calibration and internal project recommendations.
- W3C Verifiable Credentials and Open Badges already provide portable, verifiable credential models.
- Professional and apprenticeship frameworks already specify competency, supervised learning, trainers, assessment and monitoring.

Craft should compare itself fairly against those capabilities. The differentiated hypothesis is the **composition**: evidence provenance and claim ceilings; user-controlled disclosure and correction; requirement-specific, explainable matching; locally governed guilds; portable work history; and mixed economic agreements across independently governed communities.

### Benchmark suite

| Metric | Measurement |
| --- | --- |
| Precision@k / Recall@k | Human-reviewed relevance against a frozen, consented job-candidate dataset |
| nDCG@k | Ranking quality over graded, requirement-specific relevance |
| Evidence coverage | Percentage of displayed requirement assessments with traceable source references |
| Unknown-state quality | Whether the matcher correctly abstains when evidence is missing or unusable |
| False exclusion | Candidates excluded despite an accepted alternative evidence pathway or missing-only evidence |
| Explanation faithfulness | Whether each explanation is supported by the evidence and policy actually used |
| Calibration | Reliability of any stated confidence; do not call a heuristic ranking score a probability |
| Candidate agency | Disclosure comprehension, correction rate, appeal time and successful correction |
| Privacy | Unnecessary disclosure, data retention and cross-context linkability |
| Learning-to-work | Time to paid work, competency gain, learner pay, safety and retention—not placement count alone |

### Baseline comparisons

Use the same frozen task set and acceptance policy to compare:
1. lexical overlap baseline;
2. BM25/TF-IDF retrieval;
3. canonical skill-taxonomy matching;
4. evidence-aware requirement matching;
5. an available commercial baseline where licensing and lawful access permit.

Publish precision@k, recall@k and nDCG@k alongside false exclusion, abstention, explanation and privacy results. Do not use vendor marketing claims as measured baselines. No single benchmark should decide all hiring contexts.

### Initial safety gates

- 0 education bonus when the posting has no education requirement.
- 0 ungrounded equivalency inferred from free-text qualification labels.
- 0 opportunity exclusion solely because an evidence field is unknown.
- 100% distinction between self-declared and independently verified evidence.
- 100% of displayed requirement assessments traceable to evidence and a policy version.
- 0 demo postings displayed in live mode.
- 0 model-generated claims promoted to verified evidence without an independent issuer/assessor.
- 0 hiring/payment/credential authority inferred from a match score.

Targets are qualification criteria, not achieved results.

## 8. Implementation sequence

1. **MATCH-001:** repair the advisory matching semantics and disclose current limitations.
2. **MATCH-002:** versioned job requirement and evidence-assessment contracts.
3. **MATCH-003:** live, paginated job-posting read API with status and expiry validation.
4. **MATCH-004:** real job-posting editor with wire-contract golden vectors.
5. **MATCH-005:** Identity/Praxis/work-evidence adapters with issuer and status verification.
6. **MATCH-006:** Symthaea advisory matching and learning-path explanations with abstention and prompt-injection tests.
7. **MATCH-007:** frozen benchmark corpus, independent review and consented pilot.

The immediate UI deliberately does not pretend that live retrieval, job posting or applications are complete. Re-enable those controls only with real forms, versioned contracts, real indexed reads and tests.

## 9. Research references

- Upwork contract types, milestones and payment workflows: https://support.upwork.com/hc/en-us/articles/211063418-How-hourly-and-fixed-price-contracts-are-different-on-Upwork
- Eightfold matching and calibration: https://docs.eightfold.ai/en_US/talent-intelligence-platform/tipmatching-and-calibration
- Eightfold project recommendations: https://docs.eightfold.ai/en_US/learning-and-professional-growth/project-recommendations
- W3C Verifiable Credentials Data Model 2.0: https://www.w3.org/TR/vc-data-model/
- 1EdTech Open Badges 3.0 Certification Guide: https://www.imsglobal.org/spec/ob/v3p0/cert/
- ILO Recommendation 208, Quality Apprenticeships: https://www.ilo.org/resource/other/r208-quality-apprenticeships-recommendation-2023
- OECD, Platform Cooperatives and Employment: https://www.oecd.org/content/dam/oecd/en/publications/reports/2023/09/platform-cooperatives-and-employment_8e8a1d61/3eab339f-en.pdf

## Claim boundary

This document does not establish that any match is correct, a candidate is qualified, a job is legally compliant, or a hiring decision is fair. The current implementation is a deterministic advisory heuristic. Real evidence verification, indexed live discovery, production assessments, independent matching benchmarks and human-outcome pilots remain pending.
