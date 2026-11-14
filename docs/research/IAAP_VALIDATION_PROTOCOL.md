# IAAP validation protocol v1

**Status:** research protocol draft  
**Owner:** [LISE-IAAP-VALIDATION-001 / #6198](https://github.com/Luminous-Dynamics/mycelix/issues/6198)  
**Related:** [IAAP research profile](./INSTITUTIONAL_AUTHORIZATION_ASSURANCE_PROFILE.md), [independent evaluation boundary #4835](https://github.com/Luminous-Dynamics/mycelix/issues/4835), [IPB-FINALITY #6136](https://github.com/Luminous-Dynamics/mycelix/issues/6136)

This protocol evaluates whether reviewers can apply the Institutional Authorization Assurance Profile (IAAP) consistently and detect deliberately introduced claim/evidence mismatches. It does **not** evaluate whether a product is secure, issue a certification, or produce a universal trust score.

The motivation is Elmasry, “Authorization assurance for tool-using LLM agents: a structured review of enforcement semantics and effect-level security evidence,” published 10 November 2026 (https://doi.org/10.1007/s10207-026-01343-w). That review describes MAAP as preliminary and specifically calls for independent testing of inter-rater applicability, sensitivity to known mismatches, usability, completeness, and its relationship to reproduction/deployment-relevant gap detection.

## 1. Unit of analysis

The unit is one **claim record**, not an entire paper, product, institution, or agent.

A claim record states one bounded property and includes enough source material for reviewers to identify:
- claimed property and strength;
- principal/actor/delegation context;
- protected resource and named effect;
- declared system boundary and threat assumptions;
- policy decision point (PDP), policy enforcement point (PEP), executor and resource-side control, where disclosed;
- enforcement time/location;
- evidence endpoint and what it directly observes;
- residual assumptions and unsupported dimensions.

A source may contribute several records when it makes materially different claims with different boundaries. Do not label a whole paper “secure” or “insecure” from one claim record.

## 2. Appraisal vocabulary

Each claim record receives one top-level appraisal:

| Value | Meaning |
|---|---|
| `SUPPORTED_AT_SCOPE` | The cited evidence supports the stated property within the declared boundary and assumptions. |
| `NARROWER_SUPPORT_ONLY` | Evidence supports a narrower proposition than the stated claim; record the narrower wording. |
| `UNSUPPORTED_EVIDENCE` | The claim is made, but the supplied evidence does not establish its required obligations. This is not evidence of insecurity. |
| `CONTRADICTED_BY_OBSERVATION` | A disclosed observation conflicts with the claim under the declared scope. |
| `INDETERMINATE` | Missing, conflicting, incomplete or ambiguous information prevents appraisal. |
| `NOT_CLAIMED` | The source does not make this property claim. Do not penalize omission of a property it does not claim. |

Reviewers must separately code the evidence endpoint, enforcement locus, assumptions, confidence/rationale and any scope limitation. The top-level value must not erase field-level disagreements.

## 3. Core coding dimensions

Use a versioned codebook. At minimum, code:

1. **Property:** authorization correctness, complete mediation, non-bypassability, least privilege, revocation, delegation containment, fail-safe behavior, effect accounting, traceability, or other explicitly named property.
2. **Claim strength:** descriptive, tested, implementation-demonstrated, formally established within a model, externally replicated, or unspecified.
3. **Principal chain:** human/owner, operator/deployer, orchestrator, sub-agent, tool identity and credential subject; mark unavailable rather than infer.
4. **Protected effect:** disclosure, write, transfer, message, physical actuation, credential change, or named resource transition.
5. **Boundary:** declared components, interfaces, resources, alternative paths, trusted components and exclusions.
6. **Enforcement locus:** provisioning, task-start, planning, tool-admission, execution, protected-resource acceptance, or post-hoc observation.
7. **PEP correspondence:** exact PEP identity, whether invocation is evidenced, and whether the evidence covers every in-scope path.
8. **Observation endpoint:** plan, selection, attempted call, admitted call, executor result, provider acknowledgement, protected state, independent observation or deployment effect.
9. **Evidence type:** formal, implementation, synthetic test, stateful benchmark, production observation, independent replication, documentation or assertion.
10. **Residual assumptions:** route visibility, credentials, metadata integrity, environment fidelity, policy freshness, executor behavior, provider guarantees, human trust and implementation correspondence.
11. **Completeness/independence:** expected denominator, known omissions, source provenance, shared dependencies and independence qualification.
12. **Claim ceiling:** the narrowest statement the evidence actually supports.

Property-specific obligations are conditional on the claim. A study that only claims a policy parser property need not test revocation; a study claiming revocation must test post-revocation use at the relevant downstream boundary.

## 4. Paired synthetic controls

Before literature coding, create at least 24 synthetic packets with a separate known-answer oracle. Include minimal pairs in which one fact changes while the rest remains fixed:

- tool-call trace versus protected-resource observation;
- policy decision recorded versus exact PEP invocation evidenced;
- one registered route versus a disclosed reachable alternate path;
- complete route inventory versus incomplete inventory;
- successful final state with confirmation versus identical final state with required confirmation omitted;
- authority revoked before versus after the action's relevant execution point;
- delegation child within versus beyond one parent scope dimension;
- provider acknowledgement versus qualified independent observation;
- high-integrity but provider-originated receipt versus independent source;
- property not claimed versus property claimed without supporting evidence;
- model/formal result within a declared model versus implementation correspondence evidence;
- complete observed action prefix versus missing/unknown suffix.

The oracle records the justified claim ceiling, not a binary system-security label. Oracle authors must derive expected values from frozen definitions and preserve the rationale before reviewer exposure.

## 5. Literature sample

Pre-register the retrieval date, databases/search engines, exact search strings, inclusion/exclusion rules and source-version policy. The literature sample should include at least 30 claim records and should deliberately span different system boundaries and observation endpoints.

A record must retain:
- stable record ID;
- source URL and bibliographic identifier;
- exact source version/date;
- page/section/table locator;
- short paraphrase of the claim;
- a concise description of evidence used;
- reviewer-accessible source packet;
- retrieval and exclusion metadata.

Use lawful access and brief paraphrases. Do not republish full papers, long source passages or third-party supplementary datasets. Report the sampling frame and every exclusion. This is not a field-prevalence survey unless the sampling design supports that inference.

## 6. Reviewer procedure

1. Publish and version the codebook and reviewer instructions.
2. Train reviewers on a calibration subset.
3. Discuss calibration disagreements and revise the codebook.
4. Freeze the codebook, synthetic oracle and holdout packet digests.
5. Give each main-sample packet to at least two reviewers independently.
6. Blind reviewers to one another's codes and, where feasible, author/institution identity.
7. Lock independent coding before adjudication.
8. Use a third reviewer to adjudicate disagreements; retain both original codes and rationale.
9. Preserve unresolved disagreement instead of forcing consensus.
10. Evaluate the held-out mutations only after the primary coding is frozen.

Record reviewer expertise, training, conflicts, shared authorship, shared source-generation lineage and any access limitations. Do not describe reviewers as independent where material dependencies exist.

## 7. Metrics

Report metrics per coding dimension, never only in aggregate:

- raw exact agreement;
- Krippendorff's alpha using nominal or ordinal distance as appropriate, with uncertainty intervals;
- Gwet's AC1 as a supplementary prevalence-sensitive agreement statistic;
- per-field confusion matrices and disagreement taxonomy;
- sensitivity and specificity against synthetic claim-ceiling controls;
- false promotion of call-level evidence to effect-level support;
- false conversion of missing evidence into a finding of insecurity;
- confusion among `NOT_CLAIMED`, `UNSUPPORTED_EVIDENCE`, `CONTRADICTED_BY_OBSERVATION` and `INDETERMINATE`;
- adjudication and unresolved-disagreement rates;
- reviewer time, clarification burden and unclassifiable packet rate;
- performance on held-out mutations after codebook freeze.

Agreement is not truth. Synthetic sensitivity is not field completeness. Adjudication is not an oracle for contested interpretation. Publish raw denominators, missingness and uncertainty.

## 8. Prospective thresholds

The following are preregistered research targets, not achieved results or certification thresholds:

- Krippendorff's alpha at least 0.80 for core fields after calibration, with intervals reported; any field below target remains visibly unreliable.
- At least 90% exact agreement on frozen synthetic claim-ceiling controls.
- Zero false promotion from call-level evidence to effect-level support on the synthetic panel.
- Zero false conversion of insufficient evidence into an insecurity verdict.
- 100% preservation of unresolved disagreements and alternative interpretations in the released data.
- No material deterioration on the held-out mutation panel after codebook freeze.

Do not omit low-agreement fields, tune the holdout after exposure, or combine field results into a weighted score.

## 9. Validity safeguards

- **Sampling:** no field-wide prevalence claims from a convenience sample.
- **Construct:** assess each property independently; do not conflate authorization with overall safety, privacy, institutional legitimacy or general trust.
- **Rater dependence:** publish reviewer background and dependency disclosures.
- **Oracle circularity:** synthetic labels must not be derived from reviewer consensus or candidate outputs.
- **Prevalence:** provide raw agreement alongside chance-corrected statistics.
- **Overfitting:** version every codebook change and use a fresh holdout after material revisions.
- **Inference:** distinguish an unobserved effect from a demonstrated no-effect state.
- **Scope:** an appraisal qualifies only the named claim, source version, system boundary and evidence packet.

## 10. Deliverables and claim ceiling

Deliver:
- versioned codebook and claim-record schema;
- calibration, frozen and held-out synthetic packets;
- retrieval/sampling manifest;
- blinded reviewer protocol;
- anonymized field-level coding and rationales;
- preserved disagreement/adjudication log;
- analysis with intervals, limitations and codebook revisions;
- machine-readable claim appraisal with exact source locators and justified claim ceiling.

A successful study supports only the repeatability, sensitivity and stated usability of the tested IAAP protocol for the disclosed records and reviewers. It does not establish source truth, system security, field-wide prevalence, deployment effectiveness, institutional legitimacy or legal compliance.
