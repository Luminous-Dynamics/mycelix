# LISE Frontier Research and Benchmark Roadmap

**Status:** Research hypotheses and proposed work programme, 2026-11-13  
**Evidence boundary:** This document does not claim that the proposed capabilities have been demonstrated or that LISE outperforms existing institutional frameworks.

## Executive direction

LISE should become an engineering discipline for the design, operation, assurance and evolution of institutions. Its unit of analysis is not the organization label. It is the institutional capability exercised through a real workflow, under a defined constitution, legal context, resource constraint and threat model.

The central research loop is:

**Specify → authorize → execute → observe → independently evaluate → contest → adapt.**

The engineering objective is to make this loop inspectable and reproducible without transferring sovereignty to software or reducing rights and legitimacy to numerical scores.

## 1. Paradigm shifts to investigate

### 1.1 Institutions as capability graphs

Model an institution as a graph of capabilities, authorities, obligations and dependencies rather than one indivisible organization record.

A capability contract should state:

- mandate and permitted operations;
- authority issuer, subject, scope and delegation limits;
- obligations, service levels and liability allocation;
- required evidence and verification procedure;
- data purpose, permitted disclosure and retention;
- conflict, dispute and appeal procedure;
- expiry, revocation, replacement and exit conditions.

This allows a cooperative to retain its constitution and ownership while using an independent auditor, shared procurement service or community-owned infrastructure operator. The graph also exposes hidden single points of failure and conflicts between provider and authorizer.

**Research question:** does capability composition lower coordination cost or improve resilience without creating unacceptable dependency, legal ambiguity or accountability gaps?

### 1.2 Trust as a contextual, expiring claim

Replace institution-wide trust scores with claims scoped to a specific actor, capability, context, time and relying party.

A credential can establish who issued a claim and whether its integrity can be verified. It does not, by itself, establish that the claim is true or sufficient for a particular decision. The relying institution must evaluate the issuer, evidence, freshness, subject, purpose and consequences under its own policy.

Measure claim freshness, successful verification, revocation propagation, appeal rates, false acceptance and false rejection. Do not use a trust score to determine fundamental rights.

### 1.3 Constitutions as versioned policy lifecycles

Do not jump from prose directly to smart contracts. Define a traceable compilation pipeline:

1. authoritative source clause and jurisdiction;
2. human-approved interpretation and unresolved ambiguities;
3. formal policy statement and scope;
4. implementation mapping;
5. positive and negative tests;
6. deployment and policy-version binding;
7. audit, appeal and amendment history.

A machine-checkable rule is only as legitimate as its authorized interpretation. Ambiguous provisions must remain visibly ambiguous instead of being silently resolved by developers.

**New test category:** constitutional semantic drift. Change the source clause, amendment status or policy implementation and verify that reviewers are shown exactly which tests, workflows and deployed versions are affected.

### 1.4 Institutional operations as observable control systems

An institution should expose enough privacy-preserving telemetry to understand the difference between policy intent and actual behavior.

Track events such as proposal creation, standing determination, conflict disclosure, deliberation, authorization, execution, exception, appeal, remedy and outcome. Each event should reference the applicable policy version and authority evidence.

This enables operational questions that maturity scores cannot answer alone:

- Where does a process repeatedly stall?
- Which groups bear the cost of delay?
- Which authority is invoked most often?
- Which appeals overturn decisions, and why?
- Which safeguards create false positives?
- Does an intervention improve outcomes or simply move work elsewhere?

Do not turn observability into universal surveillance. Use purpose limitation, minimization, access control, retention limits and participant rights.

### 1.5 Institutional resilience as lifecycle engineering

Treat succession, provider replacement, recovery, constitutional continuity, federation exit and graceful dissolution as first-class capabilities.

A resilience benchmark should remove the founder, the signing authority, the infrastructure provider, the identity issuer and the primary knowledge custodian in separate controlled scenarios. It should also test whether a member can leave without arbitrary retaliation, confiscation or loss of portable records.

Institutional success is not identical to institutional survival. An institution that should dissolve must be able to do so safely and accountably.

### 1.6 From ideal-agent simulations to calibrated adversarial institutions

An institutional digital twin should combine:

- agent-based models for heterogeneous participants and strategic behavior;
- discrete-event models for queues, deadlines, appeals and service capacity;
- dependency graphs for providers, resources and authority;
- policy state machines for constitutional transitions;
- empirical observations for calibration and validation;
- adversarial agents with explicit budgets, information and capabilities.

Do not assume benevolent, perfectly informed, homogeneous or utility-maximizing participants. Model bounded rationality, power asymmetry, incomplete information, coalition formation, fatigue, non-participation and strategic manipulation.

A model is useful only if it is calibrated on declared data and predicts held-out observations better than a simpler baseline. Simulation output remains a hypothesis until field behavior validates it.

### 1.7 Rights-constrained, multiobjective institutional performance

Do not collapse institutional performance into one scalar.

Use a layered evaluation:
- **Hard constraints:** rights, due process, safety and legal obligations cannot be compensated by efficiency.
- **Operational outcomes:** service quality, reliability, time and cost.
- **Distribution:** who benefits, who bears burdens and who is excluded.
- **Agency:** meaningful choice, participation, contestability and exit.
- **Ecological effects:** relevant resource use and externalities.
- **Uncertainty:** confidence, missing data and contested evidence.

Publish Pareto profiles and disaggregated impacts. Let communities and affected parties define context-specific outcome priorities within applicable rights constraints.

### 1.8 Interoperability as an institutional protocol stack

The interoperability unit should be a negotiated institutional relationship, not merely an API call.

The minimal protocol needs:
1. organization and representative identity;
2. authority and delegation scope;
3. capability or service description;
4. evidence and provenance exchange;
5. data-use and privacy agreement;
6. liability and incident allocation;
7. dispute, appeal and remedy;
8. expiry, revocation and key rotation;
9. succession, portability and exit.

Test mutual recognition between organizations with different constitutions and legal structures. Each organization must be able to refuse a request, preserve local rights and leave the relationship.

## 2. What existing frameworks already contribute

| Reference | Established contribution | LISE research contribution to test |
|---|---|---|
| ISO 37000 | Governance principles and guidance | Trace operational actions and evidence to selected governance commitments |
| ISO 37004 | Governance maturity model | Add bounded behavioral, adversarial and lifecycle experiments |
| OECD Citizen Participation Guidelines | Participation process design, inclusion, accountability and evaluation | Test whether participant input actually affects decisions and receives reasoned disposition |
| World Bank GTMI 2025 | Comparative GovTech maturity across economies | Do not turn maturity indicators into organization-level performance rankings; test task-level interoperability |
| NIST AI RMF | Govern, Map, Measure and Manage AI risk | Apply risk controls to AI-assisted institutional workflows while retaining accountable human authority |
| W3C VC Data Model 2.0 and 2.1 draft | Interoperable credential representation and issuer/holder/verifier roles | Add contextual authority, claim sufficiency, evidence quality, revocation, contestability and recourse; track draft changes without prematurely adopting them |
| Ostrom IAD / SES research | Analysis of actors, action situations, rules-in-use, context and outcomes | Connect institutional analysis to executable scenarios and evidence-backed field evaluation |

These are foundations, not straw-man competitors. LISE should preserve the distinction between principle guidance, maturity assessment, technical conformance, behavioral robustness and observed outcomes.

## 3. The potential differentiator

The proposed differentiation is an integrated, reproducible assurance loop:

**Constitutional clause → policy model → authority graph → executable scenario → event trace → independent replay → appeal/remedy → measured outcome.**

A result should identify:
- the exact institutional rule and its authoritative status;
- the policy and software versions used;
- the actors and delegated authorities;
- the complete scenario and controlled inputs;
- permitted and prohibited outcomes;
- the evidence and uncertainty;
- test environment and evaluator;
- failure, appeal, remedy and residual harm.

This should be released as an open benchmark and reference implementation. It should not require adoption of the Mycelix platform: third parties should be able to implement the schema and submit comparable results.

## 4. Benchmark family

| Benchmark | Core capability | Required stressors |
|---|---|---|
| IPB-GOV | Governance and constitutional integrity | Authority mismatch, amendment, quorum, appeal and procedural manipulation |
| IPB-CAP | Capture resistance | Sybil activity, collusion, wealth concentration, coercion and insider compromise |
| IPB-INT | Institutional interoperability | Different constitutions, liability conflicts, partial trust and member exit |
| IPB-RES | Resilience and succession | Founder removal, provider outage, key compromise, recovery and dissolution |
| IPB-ECO | Economic coordination | Budget allocation, shared ownership, unequal contribution and externalities |
| IPB-PAR | Participation quality | Under-representation, accessibility barriers, performative consultation and participant fatigue |
| IPB-EVO | Institutional evolution | Amendment, version migration, constitutional drift and rights regression |
| IPB-FLO | Human flourishing | Agency, service access, distribution, dignity, ecological burden and non-member effects |
| IPB-EVID | Evidence and auditability | Provenance, disputed claims, missing data, replay and independent reconstruction |
| IPB-AI | AI-assisted institutional decisions | Automation bias, hidden influence, calibration, dissent loss and authority escalation |

Do not release a single universal institutional score. Each family should publish task-specific profiles, confidence intervals, rights violations, harms and evidence maturity.

## 5. Initial quantitative gates

All thresholds are proposed for experiments and must be pre-registered with participants and independent evaluators.

| Measure | Initial target |
|---|---|
| Unauthorized consequential action | Zero in the bounded, locked negative-control suite |
| Valid action availability | Positive controls must succeed; report false-positive rate |
| Evidence reconstruction | 100% of sampled benchmark actions replayable |
| Seeded attack success | Zero successful attacks in the declared initial red-team suite |
| Succession | 100% of defined critical-role removal scenarios preserve required essential functions |
| Cross-organization coordination | At least three independent organizations, with two or more distinct internal governance models |
| Participation disposition | 100% of submitted inputs receive a traceable acceptance, modification or reasoned rejection |
| ICC | Pilot hypothesis: at least 10% reduction against a matched baseline without rights or outcome regression |
| Digital-twin validity | Held-out predictive performance and calibration reported against a simpler baseline |

A zero-failure result must include the sample count and sampling assumptions. Under independent representative trials, the approximate one-sided 95% upper bound is 3/n. This does not cover untested attack classes or prove universal safety.

For the ICC target, define costs and outcome-quality criteria before the pilot. Report administration, negotiation, verification, dispute resolution, participant burden, technology, compliance and externalized costs separately.

## 6. How LISE should compare with the state of the art

Use one comparison protocol per scenario:

1. Select a real task with a defined legal and institutional context.
2. Document a conventional workflow and at least one relevant alternative.
3. Match information, participant resources, staffing, time budget and threat model.
4. Pre-register primary and secondary measures and non-inferiority margins.
5. Run held-out positive and adversarial cases.
6. Have independent evaluators score anonymized traces where feasible.
7. Publish confidence intervals, failures, participant burden, rights impacts and replication material.

A claimed improvement is credible only when it exceeds a meaningful baseline difference and does not worsen protected dimensions beyond pre-registered margins. Do not claim “better than SOTA” from architecture diagrams, feature counts, simulated agent counts or a self-authored benchmark score.

## 7. Public-repository work allocation

| Repository | Concrete responsibility |
|---|---|
| Mycelix | LISE specifications, IPB schemas, policy and authority fixtures, institutional protocol and evidence receipts |
| Symtropy | Discrete-event and agent-based institutional twin, fault injection, counterfactual replay and calibration |
| Symthaea | Evidence synthesis, uncertainty analysis, deliberation support, adversarial scenario generation and AI-risk evaluation |
| Sol-Atlas | Spatial commons, service access, community assets, resource dependencies and ecological distribution |

Keep one benchmark schema and one scenario registry. Repositories should implement distinct adapters and capabilities rather than duplicate policy engines.

## 8. Near-term execution sequence

### P0 — Reconcile the governance assurance case
- Align the 0.67 fixed runtime behavior with comments, tests and design documentation.
- Keep the v0.26 adaptive threshold schedule draft-only pending constitutional adoption and quorum semantics.
- Resolve chamber-specific tallying and the eligible-voter denominator.
- Add an automated cross-document consistency test.

### P1 — Expand the assurance crate
- Add delegation-chain and authority-scope fixtures.
- Add separation-of-duties and quorum tests.
- Add consume-once authorization and unknown-outcome reconciliation.
- Add appeal and remedy lifecycle tests.
- Add a deterministic JSON scenario schema and receipt verifier.

### P2 — Build the first institutional twin
- Model a small cooperative procurement workflow.
- Add budget limits, members, conflicts, proposals, independent reviewer and vendor dependency.
- Inject conflicted approver, malicious vendor, member exit and provider outage.
- Compare simple majority, delegated approval and independent review workflows.

### P3 — Establish external evaluation
- Publish the locked S2/S3 scenario suite.
- Invite cooperative, public administration, legal, security and standards reviewers.
- Run one tabletop exercise with a real organization.
- Select one low-risk field pilot with participant consent, data protection and independent evaluation.

## 9. Research foundations and references

- OECD, Digital Government Outlook 2026: https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en.html
- World Bank, GovTech Maturity Index 2025: https://www.worldbank.org/en/programs/govtech/gtmi-2025-update
- ISO 37000:2021: https://www.iso.org/standard/65036.html
- ISO 37004:2023: https://www.iso.org/standard/65037.html
- OECD Guidelines for Citizen Participation Processes (2022): https://www.oecd.org/en/publications/oecd-guidelines-for-citizen-participation-processes_f765caf6-en.html
- NIST AI Risk Management Framework 1.0: https://nvlpubs.nist.gov/nistpubs/ai/NIST.AI.100-1.pdf
- W3C Verifiable Credentials Data Model v2.0: https://www.w3.org/TR/vc-data-model/
- W3C Securing Verifiable Credentials using JOSE and COSE Recommendation (13 October 2026): https://www.w3.org/TR/2026/REC-vc-jose-cose-20261013/
- W3C Verifiable Credentials Data Model v2.1 Working Draft (22 October 2026): https://www.w3.org/TR/2026/WD-vc-data-model-2.1-20261022/
- W3C Verifiable Credential Confidence Methods v1.0 Working Draft (22 October 2026): https://www.w3.org/TR/2026/WD-vc-confidence-method-20261022/
- Ostrom, Beyond Markets and States: Polycentric Governance of Complex Economic Systems (2010): https://doi.org/10.1126/science.1191665
- Ostrom, Understanding Institutional Diversity (2005): https://press.princeton.edu/books/paperback/9780691120475/understanding-institutional-diversity

## 10. Initial implementation status

The LISE research branch contains a Rust institutional-assurance crate with fifteen deterministic S2 conflict-of-interest cases, a JSON runner, evidence receipts and integration tests. It is a synthetic reference fixture and is not yet connected to production Holochain authorization. Its test sources have been committed; execution and CI evidence remain pending.

The current governance-threshold discrepancy has been documented separately. No production threshold behavior was changed by the LISE prototype.


## 11. Digital-twin prior art and LISE's novelty boundary

Institutional and organizational digital twins are already being researched. LISE must not claim to invent the general concept:

- Edrisi et al. published a Digital Twin of the Organization architecture in IEEE Access in 2024.
- A 2025 Data & Policy study, using a German public-administration case, describes limited governance adoption and barriers including institutional readiness, privacy and fragmented institutional responsibilities.
- Choudhury, Du and Lawrence posted Learning-by-Twinning: Organizational Digital Twins and the Future of Search as a working paper on 1 October 2026. It combines firm-specific digital traces, AI-based behavioural simulation and governance rules for strategic search.

The potential LISE distinction is therefore not "an organization has a digital replica." It is a **rights-aware, constitution-linked, plural and independently evaluated institutional twin** for institutions that must coordinate across legal and governance boundaries.

Research features to test:

1. A capability and authority graph tied to source clauses and versioned policies.
2. A process model that distinguishes formal rules from observed rules-in-use.
3. Explicit dissent, uncertainty and disagreement between stakeholder-specific models.
4. Cross-organization dependencies, negotiated contracts, liability and exit.
5. Privacy and consent boundaries for every observation stream.
6. Held-out predictive validation and comparison against simpler baselines.
7. Counterfactual interventions with no automated authority to execute the proposed intervention.

### Plural institutional twin ensemble

Do not build one canonical simulation that claims to represent the whole institution. Investigate an ensemble of models with different scopes and accountable assumptions:

- operational twin: queues, approvals, resource and service flows;
- authority twin: roles, grants, delegation, policies and constitutional constraints;
- participant twin: heterogeneous needs, access barriers, participation costs and bounded rationality;
- economic twin: budgets, distribution, incentives and externalities;
- ecological/spatial twin: place-based resource and service dependencies;
- adversarial twin: capture, coercion, collusion, insider and provider-failure strategies.

The ensemble should preserve disagreement between models, identify assumptions that cause divergence and show sensitivity to missing data. A decision-maker should see a range of plausible consequences, not a single synthetic answer presented as objective truth.

A key research measure is **counterfactual calibration**: when the model predicts an outcome distribution, compare it against held-out real outcomes and report calibration, interval coverage, subgroup error and drift. Also measure how often different models disagree materially and whether that disagreement changes a proposed intervention.

## 12. Formal rules, rules-in-use and institutional reality

LISE builds on the Institutional Analysis and Development (IAD) tradition rather than replacing it. The IAD framework examines actors, action situations, rules, incentives, interactions and outcomes. Scholarship on IAD distinguishes formal legal rules from working rules: in practice, the two may align, interact with social norms or diverge.

This creates a necessary LISE evidence layer:

- declared rule: authoritative text, source, jurisdiction and version;
- interpreted rule: authorized interpretation and unresolved ambiguity;
- implemented rule: policy code and workflow;
- observed rule: what participants and operators actually do;
- enforced rule: what is consistently rewarded, blocked or sanctioned;
- experienced rule: what affected participants believe the rule to be, measured through suitable qualitative and quantitative methods.

A mismatch is not automatically proof of misconduct. It can indicate lawful discretion, local adaptation, tacit coordination, an outdated constitution, selective enforcement or an illegitimate bypass. Benchmark it with a human-reviewed taxonomy and inter-rater reliability rather than treating every deviation as the same kind of failure.

A proposed metric is the **Formal-to-Operational Rule Gap (FORG)**: the share of sampled action situations where declared, implemented and observed rules materially diverge, reported by rule type and divergence category. The metric must not score institutions negatively merely for lawful local variation; it is a diagnostic and investigation trigger.

Potential data sources include process logs, case-file sampling, participant interviews, observation and independent review. Use consent, purpose limitation, data minimization and protections against retaliation. Do not turn institutional observability into surveillance of community members.

## 13. Current standards and research status to track

- W3C Verifiable Credentials Data Model 2.0 is a Recommendation. Version 2.1's latest published Working Draft in the current research snapshot is dated 22 October 2026; do not describe the draft as a finalized standard. W3C published Securing Verifiable Credentials using JOSE and COSE as a Recommendation on 13 October 2026, and Verifiable Credential Confidence Methods v1.0 as a Working Draft on 22 October 2026. Cryptographic verification, presenter-to-subject confidence and substantive claim validation are distinct questions.
- NIST states that AI RMF 1.0 is being revised and is developing a Trustworthy AI in Critical Infrastructure profile. Its concept note highlights deterministic behavior, graceful degradation and fail-safe operation. Track the revision and profile rather than freezing LISE's AI governance mapping at version 1.0.
- ISO 37000:2021 was reviewed and confirmed current in August 2026. ISO 37004:2023 is the current published governance maturity guidance.
- The World Bank's 2025 GTMI covers 197 economies and 48 indicators and explicitly is not a ranking or an assessment of GovTech performance.
- OECD participation guidance requires more than attendance or comment volume: processes should have room for actual influence, resources, a clear response to contributions and evaluation.

These status notes are dated 13 November 2026 and should be revisited before publication or standards-conformance claims.

### Added research references

- Edrisi et al., Developing and Evolving a Digital Twin of the Organization (IEEE Access, 2024): https://doi.org/10.1109/ACCESS.2024.3381778
- Lost in translation: why digital twins thrive in research but falter in politics and public administration (Data & Policy, 2025): https://doi.org/10.1017/dap.2025.10027
- Choudhury, Du and Lawrence, Learning-by-Twinning (working paper, 1 October 2026): https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7547723
- Cole, Laws, norms, and the Institutional Analysis and Development framework (2017): https://doi.org/10.1017/S1744137416000486
- W3C VC Data Model 2.1 Working Draft (22 October 2026): https://www.w3.org/TR/2026/WD-vc-data-model-2.1-20261022/
- W3C Securing Verifiable Credentials using JOSE and COSE Recommendation (13 October 2026): https://www.w3.org/TR/2026/REC-vc-jose-cose-20261013/
- W3C Verifiable Credential Confidence Methods v1.0 Working Draft (22 October 2026): https://www.w3.org/TR/2026/WD-vc-confidence-method-20261022/
- NIST AI RMF current status: https://www.nist.gov/itl/ai-risk-management-framework
