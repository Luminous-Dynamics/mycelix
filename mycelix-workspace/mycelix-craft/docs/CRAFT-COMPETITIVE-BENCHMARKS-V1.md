# Mycelix Craft Competitive Benchmarks V1

- **Work item:** CRAFT-BENCH-001
- **Status:** Benchmark protocol and proposed targets; no measured performance or outcome claims
- **Version:** 1.0-draft
- **Reference date:** 2026-11-26
- **Owner:** Mycelix Craft

## 1. Purpose and claim boundary

Craft should not claim superiority because it combines profiles, job posts, badges, guilds, or payments. Established products and standards already cover those individual capabilities.

The qualification question is narrower and more demanding:

> Can independent communities create, govern, verify, and fulfill useful work agreements across different skill systems and economic institutions, while preserving member agency, explicit authority, evidence provenance, and owner-specific settlement boundaries?

Every benchmark result must name its dataset, baseline, threat model, jurisdiction, build SHA, test command, hardware/software environment, and evidence artifact. Targets below are proposals, not measured results. A skipped workflow, a queued workflow, or a test that was never executed is not a pass.

## 2. Industry and standards baseline

| Area | Existing baseline | Craft response |
| --- | --- | --- |
| Work marketplaces | Upwork already supports hourly and fixed-price contracts, milestones, funded milestones, review/approval and escrow/payment workflows. | Do not claim contracts or protected payments as novel. Measure portable evidence, cross-community work history, member-controlled disclosure and plural economic terms. |
| Enterprise talent intelligence | Eightfold documents calibrated, stack-ranked candidate matching and skills-based internal project recommendations. | Do not claim skills matching as novel. Compete on explainable evidence lineage, user-controlled disclosure, scope-specific competence, abstention, correction and no universal person score. |
| Verifiable credentials | W3C VC Data Model 2.0 is a Recommendation; Open Badges 3.0 is a current credential specification with conformance guidance. | Interoperate with credential ecosystems. Add work-event evidence, independent assessment context, issuer/authority provenance and explicit claim ceilings. |
| Credential status | W3C Bitstring Status List 1.0 is a Recommendation; version 1.1 was still a Working Draft on the reference date. | Implement the stable profile first; track drafts separately and version-gate adoption. Never silently switch verification semantics. |
| Platform cooperatives | OECD describes platform cooperatives as digital interfaces with democratic member ownership and control. | Support cooperative ownership and constitutional autonomy without pretending that democratic governance itself is new. |
| Quality apprenticeships | ILO Recommendation 208 covers competency needs, on/off-job learning balance, qualified trainers, assessment/certification, apprentice consent in multi-enterprise pathways, and monitoring/evaluation. | Treat apprenticeship quality and worker outcomes as core measures, not merely placement counts. |
| Commons governance | Ostrom's design principles include locally congruent rules, collective choice, accountable monitoring, graduated sanctions, accessible dispute resolution, recognition of self-organization and nested institutions. | Offer configurable governance profiles and measure whether local members can actually shape rules, contest decisions and resolve disputes. |
| Consent-based governance patterns | Sociocracy 3.0 documents consent, empiricism, equivalence, transparency and accountability, with explicit domain and role patterns. | Use patterns as optional composable methods, not a mandated constitution for every guild. |

### Baseline references

- Upwork, fixed-price and hourly contract comparison: https://support.upwork.com/hc/en-us/articles/211063418-How-hourly-and-fixed-price-contracts-are-different-on-Upwork
- Upwork legal/payment terms: https://www.upwork.com/legal
- Eightfold, Matching and Calibration (updated 2026-11-13): https://docs.eightfold.ai/en_US/talent-intelligence-platform/tipmatching-and-calibration
- Eightfold, Project Recommendations (updated 2026-11-13): https://docs.eightfold.ai/en_US/learning-and-professional-growth/project-recommendations
- W3C Verifiable Credentials Data Model 2.0: https://www.w3.org/TR/vc-data-model/
- W3C Verifiable Credentials Data Model 2.1 Working Draft: https://www.w3.org/TR/2026/WD-vc-data-model-2.1-20260905/
- 1EdTech Open Badges 3.0 Certification Guide: https://www.imsglobal.org/spec/ob/v3p0/cert/
- W3C Bitstring Status List 1.0: https://www.w3.org/TR/2025/REC-vc-bitstring-status-list-20250515/
- W3C Bitstring Status List 1.1 Working Draft: https://www.w3.org/TR/2026/WD-vc-bitstring-status-list-1.1-20261021/
- OECD, Platform Cooperatives and Employment: https://www.oecd.org/content/dam/oecd/en/publications/reports/2023/09/platform-cooperatives-and-employment_8e8a1d61/3eab339f-en.pdf
- ILO Recommendation 208, Quality Apprenticeships: https://www.ilo.org/resource/other/r208-quality-apprenticeships-recommendation-2023
- Elinor Ostrom, Nobel Lecture: https://www.nobelprize.org/uploads/2018/06/ostrom_lecture.pdf
- Sociocracy 3.0 principles and pattern changelog: https://sociocracy30.org/the-details/principles/ and https://patterns.sociocracy30.org/changelog.html

## 3. Paradigm shifts worth pursuing

### 3.1 From talent marketplace to capability commons

A marketplace matches a person to a vacancy. A capability commons also helps a community create the conditions under which people can become capable and do useful work:

- shared tools, labs, equipment, data and workspace;
- structured learning and supervised practice;
- mentoring and paid assessment;
- team formation and project incubation;
- procurement, production and shared services;
- cooperative ownership, patronage and commons allocation;
- dispute resolution, safety and member support;
- portable work evidence and member-controlled opportunity access.

The benchmark is not profile count. It is whether people gain durable, transferable capability and meaningful access to useful work.

### 3.2 From résumé claims to evidence graphs

Represent a work claim as a graph of scope, activity, contributor, artifact, observation, assessor, authority, acceptance, time and provenance. Preserve what each source actually establishes.

A signed credential proves an issuer made a signed claim; it does not prove that a physical job was safely completed or that a customer accepted the outcome. An artifact digest proves byte identity, not correctness. A work log proves that a record was submitted, not that every recorded hour occurred.

Measure false acceptance, false rejection, missing provenance, stale status, assessor calibration, correction latency and independent replay. The user should be able to inspect, selectively disclose, contest and correct claims.

### 3.3 From universal rank ladder to composable authority

Membership, learning relationships, capability claims, professional assessment authority and governance assignments are independent records. Permission is an exact action in an exact domain, granted by a verified authority, for a bounded time and subject to revocation and conflicts of interest.

A cultural honorific can remain culturally meaningful without becoming a machine permission. Founder provenance is historical. A facilitator is not automatically a treasurer. A federation delegate does not automatically control member admission in a local guild.

### 3.4 From one payment rail to plural, typed economic agreements

A single work agreement can reference fiat compensation, mutual credit, time exchange, cooperative participation, grants, commons access, in-kind resources and explicit voluntary contribution. Each component has its own issuer, unit, owner, obligation and acceptance evidence.

Different units are not automatically fungible. A quote is not an exchange. A payment status is not final settlement. Craft records work terms and evidence; the owning Finance, community currency, cooperative or resource authority performs its own effects.

### 3.5 From opaque person-score to explainable opportunity support

Avoid collapsing a person into one score. Recommendations should expose the specific required skills, evidence and confidence gaps, learning opportunities, constraints, provenance and relevant user preferences. Support abstention and a human review path. Never infer competence from age, social status, guild rank, wealth, token balance or unrelated reputation.

## 4. Qualification suites

### GOV-CORE-1: Guild authority safety

Frozen adversarial cases must cover:

- non-member, on-leave, suspended and exited subjects;
- founder-only and honorific-only subjects;
- facilitator attempting treasury actions;
- assessor attempting unrelated certification domains;
- local delegate attempting federation mutations;
- federation delegate attempting local membership changes;
- expired, revoked, future-dated and malformed assignments;
- duplicate IDs, duplicate actions and conflicting interests;
- scope aliases, Unicode normalization and identifier collisions;
- authority revocation during a concurrent or partitioned request.

**Hard targets:** zero unauthorized grants in the frozen corpus; 100% scope mismatch denials; 100% expiry/revocation boundary correctness; zero authority inferred from display labels or founder status; zero unresolved duplicate-ID ambiguity. Expand to at least 100,000 generated/property-based cases before production qualification.

**Performance target:** report p50/p95/p99 for 1, 100, 1,000 and 10,000 assignments on a named reference host. Initial target: p95 below 1 ms for 10,000 in-memory assignments, with correctness and snapshot integrity taking priority over speed. This is a target, not a measurement.

### ECON-CORE-1: Economic agreement safety

Frozen adversarial cases must cover:

- unknown, stale, duplicate, suspended, retired and out-of-period profiles;
- issuer/profile identifier collisions and ticker aliases;
- owner mismatch, unit-profile mismatch and scale mismatch;
- zero, overflow-boundary and malformed quantities;
- missing, duplicate, withdrawn, future or wrong-digest consent;
- duplicate component IDs and repeated owner-obligation IDs;
- cross-system quantity addition or inferred parity attempts;
- expired, unauthenticated, unaccepted and incorrectly bound conversion quotes;
- unconsented fallback/substitution;
- replayed owner effects and contradictory settlement observations;
- non-transferable reputation presented as compensation or collateral.

**Hard targets:** zero unknown/stale profile admissions; zero inferred conversion/parity; zero unlike-unit arithmetic; zero duplicate owner effects; zero unconsented fallback; zero Craft-owned ledger mutations; 100% frozen-corpus classification.

**Performance target:** report profile lookup and agreement validation p50/p95/p99 for 1, 10, 100 and 1,000 components. Initial target: p95 below 5 ms for 1,000 components on a named reference host. This is a target, not a measurement.

### WORK-EVIDENCE-1: Portable work proof

Measure:

- provenance completeness and issuer diversity;
- independent replay determinism;
- tamper detection and canonicalization consistency;
- stale/revoked credential detection;
- false-accept and false-reject rates by work domain;
- assessor calibration and inter-rater agreement;
- disclosure size and privacy leakage;
- subject correction, challenge and appeal time;
- cross-language golden-vector agreement.

Hard safety target: no unsupported claim is upgraded during a standards crosswalk. Add independent Rust/TypeScript or Rust/Go implementations before claiming cross-language interoperability.

### APPRENTICESHIP-1: Learning-to-work outcomes

Use matched cohorts and report:

- competency gain against a pre-registered rubric;
- time to first paid work and median time to independent practice;
- apprenticeship completion and 6/12-month retention;
- learner income, unpaid hours and out-of-pocket costs;
- mentor compensation, workload and learner-to-trainer ratio;
- workplace safety incidents and resolution time;
- accessibility and pathway completion for people without conventional credentials;
- transfer between employers without losing earned evidence;
- apprentice understanding and control over data disclosure.

Do not optimize placement counts while ignoring pay, safety, quality of instruction or learner agency.

### GUILD-1: Governance quality

Compare at least six charter profiles: one-member-one-vote, consent-based circles, delegated/representative, board/association, multi-stakeholder and federated hybrid.

Measure by charter and decision domain:

- member participation and representation;
- decision latency and unresolved backlog;
- appeal and correction rates;
- time to revoke a compromised delegation;
- concentration of decision rights and treasury access;
- conflict-of-interest detection and recusal;
- local rule adaptation and federation coordination cost;
- member-reported voice, trust, safety and procedural fairness.

Do not rank constitutions on one universal scale. Report trade-offs and whether each local charter's own rules were followed.

### PLURAL-ECON-1: Economic system fit

Run the same work cases through distinct profiles for fiat, cooperative participation, mutual credit, time exchange, grant funding, commons allocation, in-kind exchange, contribution allocation, voluntary contribution and hybrid agreements.

Measure:

- time and cost to reach informed agreement;
- obligations fulfilled, delayed, disputed and indeterminate;
- fees, spreads, default loss and recovery time;
- worker income, resource access and liquidity by arrangement;
- participant comprehension of unit meaning and conversion risk;
- local retention and external-rail dependency;
- issuance, treasury and clearing concentration;
- accessibility for unbanked/new members;
- operational burden and privacy leakage.

Include a no-currency control. Transaction volume, token count and circulation are not sufficient measures of community flourishing.

## 5. Matched comparison methodology

1. Publish the exact task set, data schema, synthetic cases, consent model and threat model.
2. Run Craft and each baseline on the same opportunities, skills, evidence and acceptance criteria.
3. Compare the relevant capability, not the entire vendor: matching against matching, payment protection against payment protection, credential verification against credential verification.
4. Report quality, cost, time, user agency, false outcomes, accessibility, data retention and operator effort.
5. Separate lab simulation from real community pilots. Obtain consent and independent ethical review for human-subject work.
6. Pre-register primary outcomes and publish failures, negative controls and confidence intervals.
7. Do not use a synthetic pass rate as proof of legal compliance, worker welfare, physical safety, monetary solvency or real-world competence.

## 6. Initial measurable targets

| Dimension | Initial target |
| --- | --- |
| Authority escalation | 0 unauthorized grants in frozen adversarial suite |
| Scope isolation | 100% exact-scope enforcement |
| Consent binding | 100% terms-digest and participant matching |
| Profile lifecycle | 0 stale/suspended profile admissions for new obligations |
| Unit safety | 0 implicit conversions or cross-unit additions |
| Ledger boundary | 0 Craft-originated balance/journal/settlement mutations |
| Replay safety | 0 duplicate owner effects under replay/concurrency tests |
| Evidence claims | 100% source and claim-ceiling traceability for qualified records |
| Interop | 100% frozen crosswalk vectors preserved or explicitly marked unsupported |
| Governance | report all six constitutional profiles without declaring one universally best |
| Apprenticeship | report paid work, safety, competency and retention—not placements alone |
| Plural economics | at least one no-currency and one hybrid pilot before claiming broad fit |

## 7. Implementation and release gates

- **Gate A — Reference semantics:** the two Rust crates compile, all unit tests pass, API docs build, and adversarial tests are reviewed.
- **Gate B — Independent verification:** a second implementation reproduces the frozen vectors; signatures, status and issuer authorization are verified in the owner domains.
- **Gate C — Holochain integration:** versioned V2 records and coordinators enforce the reference contracts without mutating the serialized legacy GuildRole enum.
- **Gate D — Security qualification:** property tests, fuzzing, replay/concurrency, partition and revocation tests pass with exact-head evidence.
- **Gate E — Human governance review:** multiple real guilds review terminology, authority and charter profiles; provide correction/appeal paths.
- **Gate F — Pilot outcomes:** matched, consented pilots report workforce, governance and economic results with negative findings.

Current repository code covers only part of Gate A. It is not Holochain integration, an independently verified implementation, a legal determination, or a field outcome.

## 8. Release and claim rules

Use distinct statuses: implemented; locally tested; CI tested at exact SHA; structurally inspected; independently reproduced; pilot measured; legally reviewed. Never collapse these into a single green badge.

The target is not to become a bigger job board or a universal currency. The target is an open, plural, evidence-carrying infrastructure through which people and communities can build capability, organize work, govern themselves and coordinate resources without surrendering their local sovereignty.
