# Craft Need Intake V1 — Benchmark Protocol

- **Work item:** CRAFT-NEED-INTAKE-001 (#6741)
- **Status:** Proposed qualification protocol; no results measured
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## 1. Research question

Can AI-assisted need decomposition reduce the time and effort required to translate heterogeneous community input into a reviewable project proposal, while preserving provenance, uncertainty, minority/dissenting priorities, safety constraints and local decision authority better than existing manual and AI-assisted approaches?

The benchmark must test the full pipeline from raw need material to ratified proposal, not merely whether generated sentences look like well-formed requirements.

## 2. Current state of the art and fair claims

Relevant 2026 evidence shows promise but also substantial limitations:

- Dąbrowski et al. (21 August 2026), *Large Language Models for Requirements Engineering: A Cross-Task Empirical Evaluation*, evaluates feedback classification, specification generation, traceability identification and explanation. Performance varies substantially by task; no one model wins consistently. It provides replication material and supports task-specific rather than one-model-for-everything deployment.
- Shefa et al. (3 September 2026), *Two Truths and A Lie? Benchmarking Off-the-Shelf LLMs for Requirements Quality Assessment*, evaluates ten models over 100 independent runs and reports asymmetric errors: the best model's median issue detection is 47% with 11% false flags; necessity/correctness issues are particularly difficult. Newer models are not monotonically better. The authors conclude that off-the-shelf LLMs are not trustworthy autonomous requirement evaluators.
- Sabatucci et al. (23 May 2026), *A Methodology for the Systematic Evaluation of LLMs in Requirement Engineering Tasks with No Ground Truth*, addresses the absence of gold labels for free-form RE outputs through literature-derived reference standards, decomposed closed-form questions and controlled synthetic cases.
- The 2025 ACM Computing Surveys expert-team-formation review identifies a lack of standardized benchmarking and calls for fair, time-sensitive, multi-factorial and open-source methods.

Craft must not claim that structured requirements, traceability, LLM extraction, RAG, multi-agent decomposition or human review are novel. Its potential contribution is the **ratifiable evidence contract**: provenance-bounded claims, explicit assumption/inference types, source visibility constraints, contradiction and dissent preservation, exact-digest review bindings and a separation between machine proposal and local authority.

## 3. Matched baseline suite

All approaches receive the same source packet, privacy permissions, need statement and time budget:

1. **Unstructured human facilitation:** current community process without a formal schema.
2. **Structured human intake:** facilitator uses a fixed template, evidence register and review checklist.
3. **Generic LLM prompting:** one model produces outcomes, requirements and packages.
4. **Retrieval-grounded LLM:** source-grounded extraction with quoted passages and reference links.
5. **Multi-agent LLM pipeline:** separate elicitation, requirement drafting, safety critique and traceability critique agents.
6. **Symthaea-assisted decomposition:** same source packet and token/compute budget, with explicit abstention and traceability output.
7. **Human expert synthesis:** independent domain and community experts prepare the reference proposal.

Use identical information access and disclose when a method cannot represent a specific construct. A baseline that omits dissent or safety cannot be called semantically equivalent.

## 4. Dataset construction

Create a public, de-identified benchmark plus a private consent-controlled field corpus.

Each episode contains:
- original need statements in natural language;
- source documents, interviews, measurements and existing records;
- source-level visibility and permitted-use metadata;
- stakeholder groups with differing priorities;
- standards and jurisdictional context;
- expert-annotated outcomes, requirements, source spans and claim-basis classes;
- assumptions, inferences, unknowns and contradictions;
- safety constraints and non-negotiable rights;
- change requests and revisions;
- review decisions and the authority roles required.

Gold labels must allow multiple acceptable decompositions. Use at least two independent domain reviewers and community review for the underlying need; adjudicate disagreements and preserve legitimate dissent rather than forcing consensus.

Split by community/project and time, not random statements, so near-duplicate projects or shared source documents cannot leak across train/test. Maintain a prospective holdout for a new community and project category.

## 5. Core metrics

| Measure | Definition | Target |
| --- | --- | --- |
| Claim-source precision | Proportion of source-backed claims actually supported by cited source spans | ≥ 0.98 |
| Claim-source recall | Relevant source spans represented in linked claims | ≥ 0.95 |
| Unsupported promotion rate | Unsupported/inferred statements presented as source-backed facts or ratified requirements | 0 |
| Fabricated citation rate | Links to nonexistent, inaccessible or digest-mismatched sources | 0 |
| Assumption label integrity | Explicit assumptions incorrectly described as facts | 0 |
| Inference disclosure | Inferences missing method/source disclosure | 0 |
| Contradiction retention | Material source/stakeholder conflicts preserved in the proposal | ≥ 0.98 |
| Dissent preservation | Minority priorities not silently erased or averaged away | ≥ 0.98 |
| Safety constraint recall | Expert-identified mandatory safety constraints represented or escalated | ≥ 0.99 |
| Dangerous omission | Critical safety issue not surfaced for review | 0 in frozen adversarial set |
| Review-binding integrity | Approvals accepted for a changed/non-current digest | 0 |
| Review-role conflict | Multiple conflicting decisions accepted for one role and exact digest | 0 |
| Empty project structure | Ratification of a proposal without an outcome or work package | 0 |
| Baseline integrity | Blank declared baseline treated as an unknown baseline | 0 |
| Conflict-resolution authority | Resolved contradiction accepted without decision digest and authority ref | 0 |
| Request-change masking | RequestChanges hidden by another approval on the same digest | 0 |
| Unauthorized ratification | Proposal ratified without all required role and local-authority approvals | 0 |
| Unselected alternative handoff | Downstream handoff emitted without a local selection of the exact alternative | 0 |
| Model-selected alternative | AI/decomposer role selects the adopted option | 0 |
| Handoff field loss | Ratified proposal is converted with missing vocabulary/policy/scope/context/freshness data | 0 |
| Consent-term conflation | Proposal digest is reused as participant acceptance terms without a separate consent contract | 0 |
| Unratified handoff | Project handoff emitted while proposal is Draft/ReviewRequired/Rejected | 0 |
| Unmapped claim | Outcome or capability requirement exists outside the proposed work-package graph | 0 |
| Safety review omission | SafetyCritical capability requirement exists without a mandatory safety review path | 0 |
| Alternative traceability | Candidate decomposition or individual trade-off lacks source/assumption/inference support | 0 |
| Duplicate stakeholder identity | A single stakeholder group is duplicated to simulate plural positions | 0 |
| Dissent erasure | Stakeholder conflict reduced to one unattributed consensus claim | 0 |
| Independent replay | Second implementation reproduces every frozen classification | 100% |

These are proposed qualification thresholds, not measured results. Report confidence intervals and every false positive/negative. The zero targets apply to the frozen conformance and adversarial corpus, not a claim of universal zero risk.

## 6. Human and community outcomes

Measure separately from automated metrics:
- number of materially distinct options presented and reviewed;
- whether stakeholders understand and can contest each option's trade-offs;
- which option the community/local authority selected and why;
- minutes of facilitator effort;
- minutes of domain-expert review;
- number of substantive edits needed per proposal;
- time to identify missing or contradictory information;
- stakeholder comprehension of the final proposal;
- perceived control over priorities and disclosure;
- whether participants can contest or withdraw input;
- whether minority and low-power voices are retained;
- reviewer disagreement and adjudication burden;
- correction rate after a proposal is ratified;
- number and impact of late safety/design changes.

Do not optimize for fewer review steps if that reduces meaningful challenge or informed consent.

## 7. Adversarial benchmark families

- fabricated citation, wrong passage, mismatched digest or hidden source truncation;
- cited source supports only a weaker or narrower claim;
- omitted negation, uncertainty, unit, context or temporal qualifier;
- source conflict, measurement conflict and contradictory stakeholder preferences;
- one organizer falsely claims to represent an institution;
- model invents a consensus or silently averages a minority position;
- alternative decomposition is selected by the model without a human decision record;
- alternative trade-offs are omitted, selectively framed or unsupported;
- one stakeholder group is duplicated to impersonate multiple opposing perspectives;
- preferred capability becomes required without authority;
- prestige, university, degree or credential label becomes a proxy for capability;
- safety-critical work or legal acceptance criterion omitted;
- standard from one jurisdiction applied to another;
- invented budget, baseline, unit conversion or asset availability;
- hidden assumption treated as observed fact;
- private participant information disclosed outside the allowed profile;
- approval bound to an old proposal digest;
- empty proposal structure or blank baseline;
- contradiction "resolved" using only an unauthenticated decision identifier;
- duplicated approvals and RequestChanges decisions for one role/digest;
- reviewer identity authentic but authority expired/revoked;
- safety review replaced by domain approval or vice versa;
- ratification occurs after material change or withdrawal.

For adversarial sweeps use property-based generation and mutation tests. Do not equate number of generated cases with independent proof.

## 8. Better-than-SOTA strategy

The right objective is not to claim a higher ROUGE/BLEU score or a more fluent requirements writer. The target is a better **decision-support system** under matched constraints:

1. Higher grounded-claim precision without unacceptable recall loss.
2. Higher safety recall with zero dangerous false acceptance in the frozen set.
3. Better uncertainty and contradiction preservation.
4. Lower human correction effort without reducing community agency.
5. Better cold-start performance where no prior project or institutional history exists.
6. Robustness to multilingual and low-resource input, dialect, code-switching and oral transcription.
7. Exact proposal/review replay across models and implementations.
8. Explicit abstention when information or authority is missing.

Use task-specific model selection, calibrated review routing and deterministic post-generation validation. Do not assume model generations improve monotonically or that a multi-agent ensemble automatically reduces errors.

## 9. Standards and industry comparison

- **W3C PROV-DM** already standardizes the general concepts of source entities, activities, agents, derivations and provenance bundles. Craft should map to PROV rather than invent a universal provenance model.
- **ISO/IEC/IEEE 29148:2018** remains the published requirements-engineering standard as of 26 November 2026. A third edition (DIS 29148) entered the ISO enquiry stage in September 2026. Benchmark requirement statement quality against the published 2018 edition; track the draft separately and do not treat it as normative until publication.
- **INCOSE Guide to Writing Requirements V3.1 (2022)** and the applicable Needs and Requirements Manual provide established systems-engineering authoring guidance. Benchmark against these rubrics rather than asserting Craft invents quality requirements.
- **NIST AI RMF** already frames trustworthiness around validity/reliability, safety, security/resilience, accountability/transparency, explainability, privacy and fairness. Need Intake should operationalize these in one proposal workflow, not claim to replace AI governance.
- **ISO/IEC 42001:2023** specifies requirements for an AI management system. Organizational adoption and audit remain distinct from a single software library's conformance; Need Intake should provide evidence inputs for an AIMS, not claim system certification.
- **Jama Connect** documents AI-assisted requirements authoring, templates, electronic reviews/approvals, version control, compliance workflows, end-to-end traceability and live change impact. Its 9.41.x release notes (2 November 2026) describe sequential approval reviews, review checklists, baseline creation at review finalization, live compliance and permission controls. This is a mature engineering lifecycle baseline, not a strawman.
- **IBM Engineering Requirements Management DOORS Next** supports requirement reviews from baselines, participant approval/disapproval, revision, trace links to development/test artifacts, baseline comparisons and suspect-link/change-impact analysis. Use the configured DOORS workflow as a baseline where available.
- **Decidim** already provides participatory spaces and proposal/debate/survey/meeting/budget/vote/accountability components, including staged processes from diagnosis and proposals through prioritization, decision and results monitoring.
- **CONSUL DEMOCRACY** already provides citizen proposals, debates, voting polls, participatory budgeting and collaborative legislation. The EU Interoperable Europe catalogue described roughly 250 cities, regional governments and partners at its October 2026 entry.
- **LLM/RAG products** already summarize source corpora and propose structured requirements. The distinction to test is whether each claim's source, assertion type, uncertainty, contradiction, stakeholder position, alternative trade-off and exact approval state is preserved across the handoff from participation to engineering and project formation.

## 10. Prospective community pilot

Pilot a community water-service need:
- source packet: resident reports, service measurements, existing asset records and jurisdiction-specific technical standards;
- community process: multiple groups set priorities and define acceptable trade-offs;
- proposal: measurable service outcome, work packages, safety requirements, assumptions and unresolved questions;
- ratification: community, domain, provenance and safety review followed by local authority adoption;
- downstream project formation: only the ratified digest is handed to Project Formation V1.

Measure water-service continuity and quality using independent instruments and inspectors. A successful intake or signed project record is not evidence that service improved. Pre-register baselines, outcome thresholds, field measurement methods and adverse-event reporting.

## References

- Dąbrowski et al. (2026), *Large Language Models for Requirements Engineering: A Cross-Task Empirical Evaluation*: https://arxiv.org/abs/2608.21531
- Shefa et al. (2026), *Two Truths and A Lie? Benchmarking Off-the-Shelf LLMs for Requirements Quality Assessment*: https://arxiv.org/abs/2609.03230
- Sabatucci et al. (2026), *A Methodology for the Systematic Evaluation of LLMs in Requirement Engineering Tasks with No Ground Truth*: https://doi.org/10.3390/systems14060598
- Saeedi et al. (2025), *A Survey of Subgraph Optimization for Expert Team Formation*: https://doi.org/10.1145/3737455
- ISO/IEC/IEEE 29148:2018: https://www.iso.org/standard/72089.html
- ISO/IEC/IEEE DIS 29148 (draft under enquiry): https://www.iso.org/standard/94091.html
- INCOSE Guide to Writing Requirements V3.1: https://www.incose.org/publications/products/se-handbook/guide-to-writing-requirements
- W3C PROV-DM: https://www.w3.org/TR/prov-dm/
- NIST AI Risk Management Framework: https://www.nist.gov/itl/ai-risk-management-framework
- ISO/IEC 42001:2023: https://www.iso.org/standard/42001
