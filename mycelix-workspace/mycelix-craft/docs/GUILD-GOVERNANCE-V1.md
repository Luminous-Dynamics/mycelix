# Mycelix Craft Guild Governance V1

- **Work item:** CRAFT-GUILD-GOVERNANCE-001
- **Status:** Pure Rust reference core added; Holochain integration, issuer verification and qualification remain pending
- **Version:** 1.0-draft
- **Owner:** Mycelix Craft
- **License:** AGPL-3.0-or-later

## 1. Decision summary

Craft guilds should be **self-governing professional and productive communities**, not universal rank ladders or credential gatekeepers.

A guild may define its own purpose, membership rules, learning practices, professional standards, economic model, governance process, and relationships with other guilds. Craft supplies interoperable records and narrow authority contracts; it does not impose one political constitution, one cultural vocabulary, one professional hierarchy, or one economic system.

The central change is to stop representing five different things with one `GuildRole` enum:

1. membership in a community;
2. participation in a learning or mentoring relationship;
3. temporary governance responsibilities;
4. professional capability and assessment authority;
5. decision rights under a particular guild charter.

These dimensions must be represented separately. No title, age, tenure, reputation score, founder status, currency balance, or governance office automatically proves professional competence.

### 1.1 Executable reference core

The companion crate `crates/guild-governance-v1` provides a pure Rust authorization evaluator for exact-scope assignments, charter-selected principal eligibility, inclusive-start/exclusive-expiry, revocation, declared conflicts of interest, duplicate assignment IDs, and fail-closed denial. An assignment carries an exact charter-policy reference and chooses either `ActiveMemberRequired` or `VerifiedPrincipalNotSuspendedOrExited`. The latter can authorize a verified external facilitator for only the exact actions and scope delegated; it does not create membership, professional assessment authority or federation inheritance. Unknown identity/membership, suspended and exited states remain denied. The trusted host must authenticate the principal, verify membership state, validate adoption of the referenced charter policy, and verify grantor authority before constructing the evaluator's input snapshot. The pure evaluator does not perform those trust checks itself.

## 2. Why the current model needs a redesign

The current `guild_integrity::GuildRole` defines `Observer`, `Apprentice`, `Journeyman`, `Master`, and `Elder`, with minimum consciousness permille thresholds. In the current coordinator:

- a new guild creator is automatically assigned `Elder`;
- the creator receives a hardcoded 900-permille join value;
- `Elder` is a prerequisite for establishing federation links;
- `Master` and `Elder` can promote members and create certification paths;
- join and promotion decisions derive a scalar from eight sovereign-profile dimensions;
- Craft Bridge can fall back to a development bootstrap profile if Identity is unreachable.

Those semantics conflate governance status, professional capability, reputation, and founder provenance. The result is an implied universal hierarchy that does not follow from a guild's actual charter or from scoped professional evidence.

The existing enum is serialized into Holochain entries. Do not reinterpret or remove its variants in place. A migration must preserve historical records and introduce versioned V2 authority records before any coordinator behavior changes.

## 3. Recommended terminology

### 3.1 Replace rank labels with responsibilities and relationships

| Current label | Recommended semantic model | What it means |
| --- | --- | --- |
| Observer | Visitor / Prospective member | Can view or participate in the explicitly public parts of a guild |
| Apprentice | Learner | Participates in an agreed learning pathway; not a lesser human or a restricted class of person |
| Journeyman | Practitioner | Has evidence of specified work or capability in a declared scope |
| Master | Do not use as a universal rank | Mentorship and assessment are separate roles; a practitioner may be a mentor without being an assessor |
| Elder | Domain Delegate / Federation Delegate, where relevant | Holds a bounded, documented, time-limited governance delegation; not a seniority or consciousness tier |

Additional role names should describe actual responsibilities:

- **Member:** holds membership rights under the charter.
- **Learner:** participates in a learning relationship or pathway.
- **Mentor:** agrees to support another person's development under a defined scope and duty of care.
- **Practitioner:** has one or more scoped capability claims; it is not a universal level.
- **Assessor:** has separately verified authority to assess specified criteria in a domain and jurisdiction.
- **Facilitator / Convenor:** facilitates a meeting or process; does not gain general decision authority by doing so.
- **Domain Delegate:** is delegated a particular decision domain, with explicit limits and an expiry/review date.
- **Records Custodian:** maintains agreed records and access controls; does not own members' evidence.
- **Treasury Delegate:** may make only the narrowly specified treasury proposals or actions allowed by the relevant finance owner and charter.
- **Federation Delegate:** represents a guild within the explicit federation agreement; cannot bind the guild outside that agreement.

**Do not replace every occurrence of Elder with Steward.** Mycelix Finance already uses Steward as a reputation/fee-tier label, and a new universal Steward rank would recreate the same ambiguity. Prefer an action-specific title such as Domain Delegate or Federation Delegate.

### 3.2 Keep culturally meaningful honorifics optional

The word **Elder** is an honorific in some communities and cultures. The system should not erase a community's self-chosen language or assume the word is inherently inappropriate.

The recommendation is narrower:

- remove `Elder` as a universal machine-level authority role;
- allow a guild to define a local honorific or display alias if its members choose it;
- document the locally understood meaning and the community's consent to that vocabulary;
- never infer age, wisdom, competence, vote weight, professional authority, or federation power from the alias;
- keep the neutral machine role and its permissions separate from the displayed honorific.

The same principle applies to `Master`, `Journeyman`, and `Apprentice`: a guild may preserve a historical or craft-specific vocabulary, but it must map to explicit semantic relationships rather than silently granting permissions.

### 3.3 Separate membership, learning, capability, governance and authority

The V2 model should use independent records:

**Membership record**
- guild identity and member identity;
- invitation/application and acceptance;
- status: prospective, active, on leave, suspended under due process, or exited;
- membership rights under the charter;
- join/exit/correction events and appeal references.

**Learning relationship**
- learner and mentor identities;
- goal, scope, expected time, consent and compensation terms;
- mentor responsibilities and safeguards;
- milestones and evidence references;
- completion, withdrawal and dispute state.

**Capability claim**
- specific capability, rubric version, context and evidence;
- observed conditions, assessor and limitations;
- provenance and current status;
- no universal mastery or employability score.

**Governance assignment**
- person, circle or verified external principal receiving a delegated responsibility;
- exact decision domain and permitted actions;
- explicit principal-eligibility rule selected by the adopted charter (active member only, or verified principal not suspended/exited);
- exact adopted charter-policy ID, revision and digest;
- grantor, selection process, term, review, revocation and conflict declarations;
- resource/budget limits and escalation route.

External-principal eligibility is not a universal default. A guild may require active membership for every delegated action. Another may appoint an external facilitator or affected-community representative for a narrow process role. The policy must be explicit, versioned, authorized by the adopted charter and checked independently from professional assessment authority. A non-member is distinct from an unknown identity or unknown membership state.

**Professional assessment authority**
- implemented as a separate reference contract in `crates/professional-assessment-authority-v1` (qualification pending);
- a separate issuer-bound authority record;
- exact capability scope, assessment method, jurisdiction, standards profile/revision/digest and validity;
- key status, revocation, conflict screening and assessor eligibility;
- references CRAFT-WORK-EVIDENCE-001 and the Identity trust authority;
- does not follow automatically from guild office or membership, and does not require guild membership when independently authorized.

**Decision-rights policy**
- who can propose, deliberate, consent, vote, appeal and ratify;
- applicable electorate or stakeholder group;
- quorum and conflict rules;
- how affected members are represented;
- version and effective dates.

## 4. Guild governance should be configurable, explicit and evolvable

Craft should support several governance patterns without declaring one universally best:

- **One-member-one-vote cooperative:** equal member voting rights, with elected or delegated operational roles.
- **Consent-based circle:** decisions proceed when no reasoned objection demonstrates a material risk to the circle's purpose or constraints.
- **Representative / delegated governance:** members select delegates for defined domains and terms, with reporting and recall procedures.
- **Board or association governance:** a board or committee acts under a published constitution and applicable legal obligations.
- **Professional standards committee:** independent, scoped authority to maintain rubrics and assessment policies.
- **Multi-stakeholder governance:** workers, learners, users, community members, funders or resource stewards have explicitly described representation.
- **Federated hybrid:** local circles retain authority over local matters while jointly governing only explicitly shared matters.

A guild charter must state which model applies to which decision domain. It may combine patterns. The system should not pretend that a single generic `GovernanceModel` field captures the whole constitution.

### 4.1 Decision domains

At minimum, distinguish:

1. **Constitutional decisions:** purpose, charter, membership rights, dissolution and amendments.
2. **Operational decisions:** routine scheduling, work allocation and process changes.
3. **Professional-standard decisions:** rubric versions, scope definitions, assessor eligibility and assessment appeals.
4. **Economic decisions:** budgets, compensation policy, shared resources and surplus allocation.
5. **Membership and conduct decisions:** admission, suspension, discipline, restoration and exit.
6. **Federation decisions:** agreements with other guilds, shared standards, shared infrastructure and joint obligations.

A person authorized to facilitate a meeting is not thereby authorized to approve a budget. A professional assessor is not thereby authorized to discipline a member. A founder is not thereby authorized to bind a federation.

### 4.2 Participation and minority protection

A guild constitution should specify:

- access to proposals and relevant information;
- reasonable notice and accessible participation modes;
- a way to raise concerns and formal objections;
- how conflicts of interest are declared and managed;
- independent or cross-circle appeal options;
- protections against retaliation;
- correction and dissent records;
- membership exit and portable personal records;
- how people affected by a decision can participate even if they are not full members.

A vote count does not make an unsafe, discriminatory, unlawful or unauthorized decision valid. Nor should a single unqualified objection silently veto every decision. Define the material-risk threshold, escalation process and appeal path in the charter.

### 4.3 Authority should be bounded, delegated and revocable

A governance assignment should have:

- a stable assignment ID and version;
- subject and assigning authority;
- role code and exact domain;
- explicit action set;
- resource or financial limits where relevant;
- valid-from and valid-until times;
- selection and supporting decision reference;
- conflict-of-interest policy;
- reporting obligations;
- review, recall, suspension and revocation events.

Authority is not transitive by default. Membership in a federation does not give the federation power to mutate a member guild's membership or currency. A delegate cannot sub-delegate unless the charter and assignment expressly allow it.

### 4.4 Founder status is historical provenance, not permanent power

A founding event may be recorded as a historical fact. It should not automatically confer a permanent governance role, professional qualification, budget access, voting weight or federation authority.

A founder may subsequently be elected or delegated specific roles under the same rules as other members. Any transitional authority must be explicit, narrowly scoped and time-bounded.

## 5. Federation should be a network of agreements, not a universal hierarchy

A guild may belong to multiple federations. Each federation edge should state:

- participating guilds and their authority to enter the agreement;
- shared purpose and scope;
- rules and standards that are shared;
- matters reserved to each member guild;
- representative and decision-selection rules;
- mutual recognition policy for scoped professional claims;
- liability and exposure boundaries;
- dispute, suspension, exit and recovery procedures;
- agreement version, validity and renewal.

A federation may recognize another guild's assessor authority for a particular purpose. That does not make every credential or role universal. Recognition should be explicit, scope-specific, jurisdiction-specific and revocable without rewriting historical evidence.

## 6. Guild economic models must remain plural

Guilds should be able to organize as worker cooperatives, professional associations, open-source communities, mutual-aid groups, commons institutions, unions, project collectives, non-profits, community enterprises or other lawful forms.

Craft records the guild's chosen charter and work agreements. It does not impose one legal structure or economic system. See the companion [Economic Pluralism and Craft Agreements V1](ECONOMIC-PLURALISM-AND-CRAFT-AGREEMENTS-V1.md).

## 7. Benchmark programme

All targets below are proposed qualification gates, not measured results.

### Safety and authority

| Benchmark | Target |
| --- | ---: |
| Professional assessment authority inherited from governance rank | 0 |
| Founder-only authority grants without explicit ratification | 0 |
| Out-of-domain or expired governance actions accepted | 0 |
| Authority inherited transitively across federation edges | 0 |
| Revoked assignments remaining effective in current authorization | 0 |
| Historical membership/governance decisions silently overwritten | 0 |
| Frozen positive and negative policy fixtures classified correctly | 100% |

### Governance quality

Measure each dimension separately; do not collapse them into a universal guild-quality score:

- role concentration and repeated tenure;
- proportion of members who can propose and amend policies;
- participation and speaking-time distribution;
- decision latency by decision class;
- appeal volume, resolution time and reversal rate;
- conflict-of-interest prevalence and management;
- member retention, voluntary exit and reasons;
- accessibility and multilingual participation;
- mentor capacity, mentor compensation and learner completion;
- assessment agreement and false-accept / false-reject rates;
- federation dependency and local autonomy;
- administrative cost per completed decision.

### Comparative evaluation

Compare matched guilds and workflows against:
- traditional professional associations and licensing bodies;
- worker cooperatives and platform cooperatives;
- DAO-style token voting;
- consent-based circles and Sociocracy 3.0 patterns;
- conventional employer-led professional development.

Use matched tasks, comparable member counts, equivalent authority scopes, declared decision classes, independent evaluators and prospective outcome measures. Measure both safety and operating cost. A system that decentralizes voting but concentrates information, proposal control or financial access has not demonstrated distributed power.

## 7.1 Industry and institutional baselines

Craft should borrow proven institutional patterns rather than claim that guild governance is new.

| Baseline | Existing strength | Craft opportunity |
| --- | --- | --- |
| Professional associations / licensing bodies | Occupational standards, member services, professional discipline and formal recognition | Interoperate with their scoped authority; do not pretend a Craft guild is a regulator or substitutes for statutory licensing |
| Enterprise talent-intelligence platforms such as Eightfold | Skills inference, candidate matching, internal mobility, project staffing and development recommendations | Add subject-controlled external work evidence, explainable evidence gaps, independent assessors and member-governed opportunity rules |
| Platform cooperatives | Shared worker/user ownership, democratic control, co-design and cooperation among cooperatives | Give member-owned guilds reusable evidence, work and federation infrastructure without taking their ownership or governance |
| Sociocracy 3.0 / consent-based circles | Explicit domains, semi-autonomous circles, delegated roles, role selection, consent and continuous policy evolution | Represent those roles and delegations as inspectable, scoped, revocable records |
| DAO-style governance | Transparent proposals, votes and execution records | Avoid treating token holdings or on-chain participation as universal professional competence or legitimate authority |

Relevant references:
- Eightfold product and capability documentation: https://docs.eightfold.ai/ and https://docs.eightfold.ai/en_US/ta-core/skills-related-insights
- Platform Cooperativism Consortium: https://platform.coop/
- OECD, *Platform Cooperatives and Employment*: https://www.oecd.org/content/dam/oecd/en/publications/reports/2023/09/platform-cooperatives-and-employment_8e8a1d61/3eab339f-en.pdf
- Sociocracy 3.0 role and circle patterns: https://patterns.sociocracy30.org/all.html

The differentiator to measure is not whether Craft can create a guild, hold a vote or match a skill. It is whether it enables multiple independent communities to recognize precisely scoped evidence, govern their own rules, cooperate on productive work and preserve member rights without a universal hierarchy.

## 8. Migration plan

Do not mutate the current serialized `GuildRole` enum in place.

1. Preserve legacy records and their original enum values as historical data.
2. Add versioned V2 membership, learning, governance-assignment and professional-authority records.
3. Introduce a read-only legacy projection with an explicit `legacy_unqualified` status.
4. Do not translate legacy `Elder` automatically into `FederationDelegate` or another privileged role.
5. Require a new, valid charter decision to issue V2 assignments.
6. Stop using `consciousness_at_join_permille` or aggregate reputation as authorization evidence.
7. Remove the development bootstrap credential from every consequential production gate; if Identity is unavailable, fail closed or enter an unmistakable local-development mode.
8. Add mixed-version tests, historical replay tests, authority-revocation tests and a rollback plan before switching coordinators.

## 9. Implementation sequence

- **GOV-001:** this charter and role-separation design.
- **GOV-002:** pure Rust scoped-governance reference contract with explicit charter-derived principal eligibility and adversarial fixtures.
- **GOV-003:** versioned Holochain V2 records and legacy projection.
- **GOV-004:** member-controlled charter policy and decision evidence.
- **GOV-005:** professional assessor authority integration with CRAFT-WORK-EVIDENCE-001 and Identity.
- **GOV-006:** multi-guild federation agreements and recognition policy.
- **GOV-007:** prospective governance-quality benchmark and pilot.

Do not implement all governance models as one giant enum. Begin with a stable policy envelope and testable decision-specific profiles.

## 10. Research references

- Elinor Ostrom design principles summarized in the World Bank's *Design Principles Illustrated by Long-Enduring Common-Pool Resource Institutions*: https://documents1.worldbank.org/curated/en/596191468179340869/pdf/101134-WP-P124655-PUBLIC-Box393259B.pdf
- Sociocracy 3.0 Practical Guide, current guide and patterns: https://patterns.sociocracy30.org/all.html
- Sociocracy 3.0 January 2026 guide update: https://sociocracy30.org/2026/01/update-to-the-practical-guide-2026-01-26/
- OECD, *Platform Cooperatives and Employment*: https://www.oecd.org/content/dam/oecd/en/publications/reports/2023/09/platform-cooperatives-and-employment_8e8a1d61/3eab339f-en.pdf
- Platform Cooperativism Consortium: https://platform.coop/
- ILO Recommendation 208, Quality Apprenticeships: https://www.ilo.org/resource/other/r208-quality-apprenticeships-recommendation-2023
- APA Inclusive Language Guide, including cultural nuance around the title “Elder”: https://www.apa.org/about/apa/equity-diversity-inclusion/language-guidelines

## Claim boundary

This document is a design proposal. It does not establish legal recognition, professional licensing, democratic legitimacy, fair outcomes, a production-safe governance engine, or a universally appropriate governance vocabulary. Each guild and affected community must choose and review its own charter and terminology.
