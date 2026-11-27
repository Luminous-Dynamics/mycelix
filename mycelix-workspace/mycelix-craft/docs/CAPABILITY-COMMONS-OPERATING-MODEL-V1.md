# Mycelix Craft — Federated Capability Commons Operating Model V1

- **Work item:** CRAFT-CAPABILITY-COMMONS-001 (#6715)
- **Status:** Draft architecture and qualification proposal; not runtime behavior
- **Version:** 1.0-draft
- **Reference date:** 2026-11-26

## 1. Thesis: organize capability formation, not just candidate discovery

A conventional job marketplace starts with a vacancy and searches for a person. A capability commons can start with a community need and help participants create the project, capabilities, team, resource access and agreements required to address it.

Craft should therefore be the coordination layer for a network of autonomous productive communities. A project may be initiated by a neighbourhood, cooperative, guild, public-benefit organization, employer, research group or individual. It need not begin as a job posting. Work may be paid, cooperative, grant-funded, reciprocal, commons-based, in-kind, voluntary or hybrid.

The proposed differentiator is not that Craft has profiles, credentials, contracts, skills matching, cooperative governance or payment protection. Those are already present in established systems and standards. The differentiator to prove is that independent communities can form useful work, develop missing capability, compose teams and resources, govern collaboration and preserve evidence and economic sovereignty across institutional boundaries.

**Core design rule:** Craft coordinates commitments and evidence references. It does not become the universal identity issuer, professional regulator, currency, payment rail, resource registry, physical-world oracle or owner of participating guilds.

## 2. Principles and invariants

1. **Community-initiated work:** a valid project can begin with a need or desired outcome, not only an employer-authored vacancy.
2. **Contextual capability:** capability is a claim about a defined activity, conditions, equipment, method, jurisdiction and time—not a universal property or score attached to a person.
3. **Evidence and uncertainty:** represent reported, artifact-bound, issuer-verified, independently attested, demonstrated, disputed, revoked, conflicting and unknown states distinctly.
4. **Development before exclusion:** an evidence gap should normally produce a learning, mentoring, supervised-practice or human-review path. A mandatory safety or legal qualification remains a hard authorization gate for the hazardous task; it must not be disguised as a universal person-ranking penalty.
5. **Plural institutions:** each guild chooses its own lawful charter, decision rules, membership policy and economic arrangements.
6. **No authority inheritance:** membership, founder provenance, honorifics, seniority, reputation, wealth, professional assessment authority and governance office are separate.
7. **No universal value unit:** unlike currencies, credits, hours, grants, access rights, reputation and in-kind resources are not added together or assumed convertible.
8. **Owner-specific effects:** Finance, issuers, cooperatives and resource owners authorize their own ledger, issuance, reservation, payment and finality operations.
9. **Portable evidence and exit:** people can inspect, selectively disclose, challenge, correct and carry their own evidence without surrendering their professional identity to a guild or platform.
10. **Physical reality remains external:** a plan, signed agreement, work log, sensor message or credential is not by itself proof that a physical outcome occurred or was safe.
11. **Local autonomy survives federation:** federation edges are explicit agreements; no transitive administrative, professional or treasury authority is implied.
12. **Human outcomes over platform activity:** optimize access to meaningful, safe and fairly compensated productive participation—not profile count, engagement, token circulation or transaction volume.

## 3. The capability-commons lifecycle

The lifecycle is a reference composition, not one global state machine. Every stage is independently versioned and owned.

### Stage 1 — Identify a need or opportunity

A person or community records:
- the need, driver or desired change;
- the affected people and communities;
- location and operating context, where material;
- desired outcomes and unacceptable harms;
- urgency, dependencies and known constraints;
- who is authorized to propose or sponsor the work.

A need record is a proposal, not an approved project, procurement authority or funding commitment. Communities must be able to record dissent, alternative proposals and the decision process that adopted, deferred or rejected a need.

### Stage 2 — Constitute a project

A project charter identifies:
- purpose, scope, expected outcomes and non-goals;
- sponsor and affected-party representation;
- decision domains and locally selected governance profile;
- project-level risk, safeguarding, safety and accessibility requirements;
- budget or resource envelope references, if any;
- amendment, pause, dispute, appeal and exit procedures;
- evidence required for acceptance and learning.

Charter approval belongs to the authority selected by the project's constitution. Craft records its evidence and scope; it does not decide which constitution is legitimate for every community.

### Stage 3 — Decompose work into packages

A project is divided into bounded work packages. Each package declares:
- outcome and deliverable;
- dependencies and sequence constraints;
- capability requirements and whether each is mandatory, conditional, preferred or a learning goal;
- safety controls, supervision and legal/jurisdictional conditions;
- expected work evidence and acceptance method;
- resource/equipment requirements;
- accountable owner and affected parties;
- schedule and explicit unknowns.

A package may be decomposed further. Work-package changes must create a new revision and preserve the prior scope and commitments.

### Stage 4 — Discover capability and design a team

Matching should be requirement-level and explainable. For every requirement, return one of:

- **Met:** current, relevant evidence satisfies this requirement under the declared policy.
- **NotMet:** available evidence demonstrates that the requirement is not met.
- **Unknown:** evidence is absent, stale, inaccessible or not yet verified.
- **Conflicting:** evidence sources materially disagree.
- **HumanReview:** the requirement or evidence needs an accountable human decision.

The system should disclose the exact evidence, issuer, scope, time, conditions, limitations and policy revision used for each disposition. Self-declaration can be useful evidence of interest or experience, but it is not silently promoted to independent verification.

Team formation is a constrained coverage problem, not a global person-ranking contest. A team proposal should expose:
- requirement coverage and the evidence supporting it;
- uncovered requirements and their safety consequence;
- complementary capabilities and useful redundancy;
- availability, location, language, accessibility and stated preferences;
- training/supervision needs and possible mentor capacity;
- conflicts of interest, dependencies and uncertainty;
- the reason a proposed role is suitable and the conditions on which that conclusion depends.

No overall human-worth, employability or guild-quality score is produced. Any optimization objective (cost, delivery time, continuity or other criterion) must be declared by the project authority, bounded by hard safety and rights constraints, and reported separately from evidence confidence.

**Important distinction:** missing evidence must not silently erase a person from discovery. If a task legally or safely requires a current qualification, the person may be ineligible to perform that task until the requirement is verified, while remaining eligible for a supervised, non-hazardous or training role where policy permits.

### Stage 5 — Build missing capability

A capability gap can produce a development path containing:
- learning resources or formal training;
- a named and consenting mentor;
- supervised work or practice;
- a bounded work-sample assessment;
- access to tools, equipment or a lab;
- feedback, reflection and re-assessment;
- portable evidence of the learning and demonstrated activity.

Learning records remain learner-controlled in Praxis. Mentor appointment, compensation and workload are explicit. Training completion, credential issuance and demonstrated competence are separate events. Apprenticeship design should include written roles and obligations, adequate remuneration or other compensation, occupational safety, data protection, dispute resolution and labour-market mobility safeguards consistent with the applicable ILO guidance and local law.

### Stage 6 — Agree people, work, resources and economics

Before work begins, the relevant parties accept version-pinned agreements for:
- work scope and responsibilities;
- employer, worker, mentor, learner, assessor and organization roles;
- milestones and acceptance conditions;
- safety, supervision, insurance and escalation where applicable;
- resource access, custody, use and return;
- data disclosure and evidence permissions;
- one or more typed economic components.

A hybrid example may combine a conventional wage, a community mutual-credit obligation, a training grant and time-limited access to shared equipment. Each component keeps its own issuer, profile, unit, owner, obligation, acceptance evidence and settlement state. No component is converted or substituted without an explicit, versioned quote and informed agreement by the required parties.

Craft's `CraftEconomicAgreementV1` is an agreement envelope and reference validator, not a currency or ledger. It must not create balances, journal entries, issuance, FX execution, resource reservations, payments or settlement finality. Finance or the respective community, cooperative, grantor or resource owner performs those effects.

### Stage 7 — Perform work and collect evidence

The project links, but does not conflate:
- planned work and commitments;
- performed activities and observations;
- artifacts and their digests;
- material and resource consumption;
- inspections and capability assessments;
- acceptance decisions;
- economic obligations and owner-side effects;
- physical condition and measured outcomes.

A submitted work event is not automatically accepted. An artifact digest identifies bytes; it does not prove correctness. A signed attestation proves that an identified authority made a claim; it does not prove the physical work was safe. Sensor observations require source, calibration, timing, environment and uncertainty context. Conflicting or missing evidence remains visible.

Compose with Manufacturing for work orders, material use and repair/restoration evidence; with Symtropy for qualified physical observations and model-to-world comparisons; and with the Work Evidence verifier for cryptographic and scoped attestation checks. Do not create duplicate authoritative copies of their state.

### Stage 8 — Accept, settle, learn and renew

Closeout produces distinct records for:
- work-package acceptance or reasoned rejection;
- outstanding corrections, disputes and defects;
- owner-authorized economic effect and its independently reported finality;
- resource return, condition and stewardship;
- capability gained and evidence the learner chooses to retain or disclose;
- project outcome, distribution of benefits and burdens;
- recurrence, maintenance and next needs.

A project is not “complete” merely because its agreement exists, a milestone was submitted or payment was recorded. Completion policy must define the required evidence and unresolved exceptions. Lessons should feed new development paths and project proposals without silently rewriting prior records.

## 4. Proposed logical records and ownership

These names are conceptual envelopes for design and review, not a claim that the records already exist in code.

| Record | Core content | Authoritative owner / boundary |
| --- | --- | --- |
| `CommunityNeedV1` | Driver, outcome, affected parties, constraints, proposal provenance | Originating person/community |
| `ProjectCharterV1` | Purpose, scope, governance profile, decision domains, risk and appeals | Project's adopting authority |
| `WorkPackageV1` | Deliverable, dependencies, capability requirements, safety and acceptance policy | Project/work-package authority |
| `CapabilityRequirementV1` | Taxonomy/profile reference, method, context, jurisdiction, requirement class | Work-package owner |
| `CapabilityEvidenceRefV1` | Claim/evidence references, issuer, scope, time, status and limitations | Source issuer / evidence owner |
| `DevelopmentPathV1` | Learning, mentor, supervised practice, resources and assessment plan | Learner and learning provider; Praxis owns learning records |
| `TeamPlanV1` | Requirement coverage, proposed roles, gaps, redundancy, constraints and rationale | Project authority; recommendation remains advisory |
| `ResourceAccessAgreementRefV1` | Resource ID, owner, access terms, capacity, custody, validity and return evidence | Resource owner |
| `WorkEventRefV1` | Activity, artifact, time, actor, process and provenance | Work execution / Manufacturing owner |
| `AcceptanceReceiptV1` | Exact package revision, policy, evaluator, evidence references and decision | Contractually designated acceptance authority |
| `CraftEconomicAgreementV1` | Participant consent and typed obligation references | Craft stores terms; Finance/issuers/resource owners own effects |
| `OutcomeObservationV1` | Measured outcome, baseline, method, uncertainty, distribution and recurrence | Relevant independent measurement/source owner |

Do not create one `ProjectStatus` enum that attempts to encode governance, learning, evidence, resource, payment and physical-world states. They have different owners, valid-time semantics and failure modes. Use linked, revisioned projections with explicit unknown, disputed, suspended, revoked and not-applicable states.

## 5. Standards and ecosystem interoperability

Craft should map to existing standards with version-pinned profiles, not replace their owners.

| Domain | Existing ecosystem | Intended Craft use | Boundary |
| --- | --- | --- | --- |
| Skills and occupations | ESCO; O*NET; national occupational classifications such as South Africa's OFO | Optional taxonomy crosswalks for capabilities, occupations and training discovery | No taxonomy is a universal truth or proof of individual competence |
| Credentials | W3C Verifiable Credentials Data Model 2.0; 1EdTech Open Badges 3.0 | Portable issuer/holder/verifier claims, achievement and evidence references | Credential signature and issuer claim do not prove a physical outcome or legal recognition everywhere |
| Cooperative identity | International Cooperative Alliance identity and principles; OECD platform-cooperative research | Support member ownership, democratic control, autonomy and cooperative economic participation | Craft is not the cooperative's owner, board or registrar |
| Governance | Ostrom's commons design principles; Sociocracy 3.0 patterns; locally chosen constitutions | Configurable circles, delegated domains, collective choice, monitoring, appeals and nested federation | No mandatory constitution; no single governance pattern is universally optimal |
| Apprenticeships | ILO Recommendation 208 and jurisdiction-specific rules | Agreement, learning balance, compensation, safe work, mentor support, evidence and outcome measures | Local labour law and competent authorities remain authoritative |
| Economic process | Valueflows vocabulary and existing Mycelix Economic OS profiles | Interoperate with intents, commitments, agreements and economic events where semantics match | No duplicate ledger, unit conversion or implicit settlement |
| Physical supply-chain evidence | GS1 EPCIS 2.0.1 and CBV profiles | Share interoperable observations about objects, processes, location and custody | An EPCIS event is not by itself a safety certification or proof of final acceptance |

All mappings must pin the exact standard/profile revision, URI or artifact digest, issuer/maintainer, effective date and mapping version. Crosswalk outputs need one of: `EquivalentForDeclaredPurpose`, `Narrower`, `Broader`, `Partial`, `Unsupported`, or `HumanReview`. Do not silently map distinct concepts to one another.

## 6. Where Craft can outperform existing platforms

The comparison must be capability-specific and fair.

| Baseline | What it already does well | What Craft should prove beyond it |
| --- | --- | --- |
| Eightfold-style talent intelligence | Calibrated role requirements, semantic skills, matching and ranked internal candidate discovery | Requirement-level evidence lineage, explicit uncertainty, development routes, no universal person score, and community-controlled opportunity governance |
| Upwork-style work marketplace | Hourly/fixed-price contracts, funded milestones, work review, escrow/payment protection and dispute workflows | Cross-community project formation, non-market and hybrid economic arrangements, shared resource access, and portable work evidence without taking over settlement |
| W3C VC / Open Badges | Portable credential claim structure and issuer/holder/verifier interoperability | Context-rich work evidence, authority and claim ceilings, correction/dispute links, and integration into a productive work lifecycle |
| Platform cooperatives | Member ownership, democratic control and worker/user participation | Reusable technical infrastructure across independent cooperatives while leaving ownership and governance local |
| Sociocracy / commons institutions | Practical patterns for roles, circles, consent, collective choice, monitoring and conflict resolution | Machine-checkable scoped delegations, evidence, appeals and federated interoperability without mandating the method |
| Occupational taxonomies | Shared skill/occupation vocabulary, multilingual terms and classification relationships | Local and sector-specific extensions without reducing people to taxonomy labels or treating taxonomy matches as demonstrated capability |

Craft should not try to win by reproducing every feature of every vendor. It should demonstrate a coherent cross-institution workflow that current tools often leave to manual coordination: need → project → team → development → resource access → agreement → work evidence → independent acceptance → owner-specific settlement → measurable outcomes.

## 7. Benchmark and pilot programme

All targets are proposals. Report each result with exact source SHA, command, toolchain, hardware, frozen dataset/fixture digest, policy version, threat model, test output and evidence artifact. A queued or skipped job is not a pass. Synthetic conformance is not a human outcome.

### CRAFT-COMMONS-1 — Need-to-project and project formation

Measure:
- elapsed time from validated community need to scoped and authorized project;
- project proposal acceptance, deferral and dissent rates;
- time to define work packages and acceptance criteria;
- affected-party participation and representation;
- cost and administrative effort per project;
- number of projects initiated without a pre-existing vacancy.

Compare with manual community coordination, employer-led project initiation and marketplace vacancy workflows. Report distributions, not just averages.

### CRAFT-TEAM-1 — Explainable team composition

Compare:
- lexical / keyword matching;
- BM25 or TF-IDF;
- taxonomy-only mapping;
- conventional skills-based matching;
- work-sample / verified evidence matching;
- Craft's requirement-level evidence-aware team coverage.

Measure precision@k, recall@k, nDCG@k where ranking is appropriate, requirement coverage, false exclusions, calibration, abstention quality, explanation faithfulness, team redundancy, completion outcomes and human override rates. Separate self-declared, artifact-bound and independently verified signals. Include counterfactual tests where evidence is missing or inaccessible, credentials are nontraditional, or candidates have adjacent transferable skills.

**Safety target:** zero hazardous task authorization from unknown, expired or out-of-scope mandatory evidence. **Inclusion target:** zero silent exclusion solely because optional evidence is missing; the system must present a clear development or review path.

### CRAFT-LEARN-1 — Capability development and apprenticeship

Measure:
- pre/post competency change using a declared rubric;
- transfer to unseen tasks, delayed retention and independent practice;
- time to first fairly compensated contribution;
- completion, non-completion and 6/12-month retention;
- learner and mentor compensation, unpaid hours and workload;
- safety incidents, complaints, dispute resolution and time to remedy;
- portable evidence retention and subject-controlled disclosure.

Use ILO Recommendation 208 as a baseline and report local-law review separately. Never maximize apprenticeship placements at the expense of compensation, safety, learning quality or worker agency.

### CRAFT-GOV-1 — Institutional quality and anti-capture

Run matched scenarios across multiple charter profiles: one-member-one-vote, consent-based circles, delegated/representative, board/association, multi-stakeholder and federated hybrid.

Hard security targets:
- zero unauthorized actions in the frozen adversarial suite;
- zero expired, revoked or out-of-domain delegation accepted;
- zero governance-to-professional-authority inheritance;
- zero federation authority inherited transitively without agreement;
- 100% historical record preservation in migration fixtures.

Measure separately: proposal access, participation, role/treasury concentration, repeated tenure, decision latency, appeals, reversals, conflict handling, member exit, local autonomy and dispute cost. Do not rank all constitutions on one score.

### CRAFT-RESOURCE-1 — Shared productive capacity

Measure:
- time to discover and access suitable resources;
- utilization, downtime, maintenance and idle capacity;
- access distribution across members and newcomers;
- resource damage/loss and recovery;
- custodial burden and contribution;
- local availability under connectivity interruption;
- reservation conflicts and owner-side reconciliation.

A discovered inventory record is not proof the equipment exists or is currently available. Only the resource owner confirms reservation, custody, condition and return.

### CRAFT-ECON-1 — Plural economic fit

Compare the same project under fiat, cooperative participation, mutual credit, time exchange, grant, commons allocation, in-kind, voluntary contribution and hybrid terms. Include a no-currency control.

Measure separately:
- time/cost to reach informed agreement;
- obligations fulfilled, delayed, disputed and unknown;
- fees, spreads, defaults, losses and recovery;
- worker income and distribution of surplus;
- liquidity and access for newcomers/unbanked participants;
- participant comprehension of units and conversion risk;
- external-rail dependence, issuer/treasury concentration and privacy;
- administrative burden and cross-system semantic loss.

Hard targets: zero implicit parity, zero unlike-unit arithmetic, zero unconsented substitutions, zero duplicate owner effects and zero Craft-originated ledger mutations. Do not use currency creation or transaction volume as a welfare proxy.

### CRAFT-OUTCOME-1 — Productive and community outcomes

Measure:
- delivered outcome against a pre-registered baseline;
- quality, rework, defect recurrence and maintenance reliability;
- time/cost per accepted outcome;
- benefit and burden distribution;
- environmental/resource impacts where relevant;
- community-defined satisfaction, agency and trust;
- new capability formed and retained after project completion.

Field pilots require participant consent, privacy and safeguarding review, clear ownership of data, independent evaluation where appropriate, and publication of failures and confidence intervals. Never infer causal improvement from a small uncontrolled demonstration.

## 8. First prospective pilot: community repair and renewable-energy capability

A useful first pilot is a small community project around a repairable productive asset (for example, a water-pump control system, solar/battery subsystem or workshop tool). Select a real asset and community only after owner permission, site safety review and local legal/professional qualification checks.

Possible work packages:
1. Intake, problem statement and asset identity.
2. Safety isolation, inspection and diagnostic plan by appropriately qualified people.
3. Fault diagnosis and repair plan.
4. Parts/material procurement and custody.
5. Supervised repair and documented work events.
6. Independent test/inspection under the applicable standard and jurisdiction.
7. Handover, maintenance plan and recurrence observation.
8. Learner reflection, competency assessment and portable evidence, with learner consent.

A project might combine a grant-funded parts budget, paid specialist hours, a cooperative contribution and shared workshop/tool access. Each remains separately typed and owner-controlled. No assumption is made that a community credit, hour, grant amount or asset-use right can be exchanged at par with another.

Compose:
- Craft: need, project, work packages, team plan, charter references and economic agreement.
- Praxis: learner-owned learning journey.
- Mycelix Manufacturing: work order, parts consumed, restoration and recurrence.
- Symtropy: independently qualified physical observations where suitable.
- Work Evidence: scoped signed attestation and replay-safe evidence verification.
- Finance / instrument issuers / resource owners: their own obligations and settlement or access effects.

Pilot success is not “the platform was used.” It is an independently observable useful outcome, safe work, fairly treated participants, capability gained, resource condition preserved and auditable owner-specific economic effects.

## 9. Implementation sequence and release gates

1. **G0 — Contract design:** publish record ownership, semantic definitions, threat model, data minimization and failure-state matrix.
2. **G1 — Reference cores:** qualify the governance and economic agreement crates; separate professional assessor authority from governance actions; add frozen fixtures.
3. **G2 — Evidence interoperability:** Work Evidence verifier, issuer/assessor trust inputs, correction/dispute/supersession lifecycle, cross-language golden vectors and independent implementation.
4. **G3 — Versioned Holochain records:** V2 membership, charter, work-package, authority and agreement references; fail-closed production behavior; preserve legacy records and enum bytes.
5. **G4 — Project and team planning:** implement need/project/work-package and explainable requirement-coverage records; keep recommendations advisory.
6. **G5 — Apprenticeship and resource paths:** employer-authored offer, apprentice-accepted agreement, mentor appointment, compensation and explicit resource-owner access agreement.
7. **G6 — Owner adapters:** integrate Praxis, Manufacturing, Symtropy and Finance only when each owner's contract and replay semantics are qualified.
8. **G7 — Prospective pilot:** independent safety and community review, matched baselines, pre-registered outcomes and a published evidence report.

Each gate reports: implemented, locally tested, exact-head CI tested, structurally inspected, independently reproduced, pilot measured and legally reviewed as separate statuses.

## 10. Non-goals

This proposal does not create:
- a universal guild constitution or central federation administrator;
- a global professional licensing or assessor authority;
- a universal reputation, employability, capability or welfare score;
- a Craft currency, universal unit of account or FX engine;
- duplicate Finance, cooperative, grantor or community ledgers;
- a new identity, credential, taxonomy, asset registry or physical-world source of truth;
- autonomous hiring, professional promotion, payment or hazardous-work authorization by Symthaea or another model.

## 11. References

- Eightfold, *Matching and Calibration* (updated 2026-11-13): https://docs.eightfold.ai/en_US/talent-intelligence-platform/tipmatching-and-calibration
- Upwork, *How hourly and fixed-price contracts are different*: https://support.upwork.com/hc/en-us/articles/211063418-How-hourly-and-fixed-price-contracts-are-different-on-Upwork
- Upwork, *How Upwork protects your payments*: https://support.upwork.com/hc/en-us/articles/211062568-How-Upwork-protects-your-payments
- W3C, *Verifiable Credentials Data Model v2.0* (Recommendation, 2025-05-15): https://www.w3.org/TR/vc-data-model/
- 1EdTech, *Open Badges 3.0 Certification Guide*: https://www.imsglobal.org/spec/ob/v3p0/cert/
- International Cooperative Alliance, *Cooperative identity, values & principles*: https://icett.ica.coop/en/cooperatives/cooperative-identity
- OECD, *Platform cooperatives and employment* (2023-09-30): https://www.oecd.org/en/publications/platform-cooperatives-and-employment_3eab339f-en.html
- Sociocracy 3.0, *Practical Guide* (latest available version 2026-01-26): https://sociocracy30.org/guide/
- Elinor Ostrom, *Prize Lecture*: https://www.nobelprize.org/uploads/2018/06/ostrom_lecture.pdf
- ILO Recommendation No. 208, *Quality Apprenticeships Recommendation* (2023): https://normlex.ilo.org/dyn/nrmlx_en/f?p=NORMLEXPUB:12100:0::NO::P12100_ILO_CODE:R208
- European Commission, *ESCO Classification* (v1.2.1; release v6.3.0): https://esco.ec.europa.eu/en/classification
- South African DHET, *Organising Framework for Occupations* overview in Critical Skills List technical report: https://www.dhet.gov.za/Planning%20Monitoring%20and%20Evaluation%20Coordination/2022%20Technical%20Report%20Finalisation%20of%20the%20Critical%20Skills%20List.pdf
- GS1, *EPCIS Standard Archive* (v2.0.1 published 2025-07-01): https://ref.gs1.org/standards/epcis/archive
- Valueflows ontology and project: https://valueflo.ws/ and https://github.com/valueflows/valueflows

## Claim boundary

This document is a design proposal. It does not establish a production project lifecycle, safe team optimizer, legal compliance, competent professional status, improved employment outcomes, resource reservation, physical restoration or monetary solvency. Those require separate implementation, verification, independent assessment and real-world evaluation.
