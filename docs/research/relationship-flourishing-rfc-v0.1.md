# Relationship Flourishing Protocol: Mutual Matching, Safety, and Outcomes

**Status:** Research RFC / design proposal v0.1  
**Date:** 2026-10-17  
**Scope:** Public Mycelix / Pulse composition; Symthaea, Sol-Atlas, and Symtropy are proposed collaborators, not claims of current product integration.  
**Implementation status:** Specification only. No dating feature, measured matching result, privacy audit, or production safety capability is claimed by this document.

## 1. Thesis

Build a system that helps consenting adults discover mutually wanted, respectful connections. Do not optimize for keeping people swiping, maximize message volume, rank human beings by universal desirability, or make prolonged loneliness a revenue dependency.

Legitimate outcomes include a mutually wanted date, a lasting romantic relationship, friendship, a useful community connection, a safe decision not to proceed, a pause, or leaving the product after finding what the user came for.

**Success is user-defined progress toward connection—not continued product engagement.**

This RFC proposes a measurable design direction. It is deliberately not a promise that AI can predict love, infer someone's character, or eliminate abuse.

## 2. Why work on this now?

Research and recent safety evidence suggest several specific gaps worth addressing:

- A 2023 *International Economic Review* field experiment evaluated a two-sided matching recommender on a dating platform. The paper reports reduced recommendation inequality and more matched couples than its comparison recommenders. This is a precedent for testing mutual matching rather than one-sided clicks; it is not proof that one algorithm generalizes to every population or improves relationship quality.
- A 2025 systematic review included 45 studies; more than 85% of studies examining body-image outcomes reported significant negative impacts associated with dating-app use. A separate 2026 meta-analysis of 23 studies and 26,068 participants found a modest association between app use and poorer psychological health/well-being, while warning that predominantly cross-sectional designs cannot establish causation.
- On 2026-10-13, the U.S. Joint Economic Committee's minority report described dating-platform scam-prevention deficiencies and increasing exposure to scam profiles on Tinder. Separately, the FTC warns about relationship-building followed by urgent money requests, investment pitches, and pressure to move off-platform.
- The W3C Verifiable Credentials Data Model 2.0 is a 2025 Recommendation and discusses selective disclosure and correlation risks. A verifiable credential can prove that an issuer made a claim; it does not make every claim true or prove that its holder is safe.

These findings support designing and testing a better alternative, not declaring existing services universally harmful or assuming an AI feature is inherently safer.

## 3. Product constitution

The following principles are release constraints, not optional UI copy.

1. **Adult, explicit, revocable consent.** A recommendation is not permission to contact someone. A single-sided acceptance must not expose a private profile, create a shared chat, or reveal an undisclosed preference. Either person can withdraw.
2. **No universal human score.** No public desirability, trustworthiness, mental-health, compatibility, or risk score. Do not turn a private preference or an allegation into a durable social-credit record.
3. **Hard boundaries before ranking.** Stated dealbreakers, relationship intentions, age eligibility, blocking, availability, and current consent must be checked before any ranking or exploration.
4. **Explainable suggestions, not judgments.** Every recommendation must cite permitted, user-controllable reasons such as shared interests or compatible stated intentions. Unknown means unknown. Do not infer protected or sensitive traits from text, images, contacts, location, or behavior.
5. **Safety is a system property.** Block/report, anti-impersonation, scam containment, data minimization, incident response, appeals, and independent review are foundational—not paid features.
6. **Privacy includes relationships and metadata.** Protect who is interested in whom, when people interact, location, preferences, and the existence of a sensitive connection—not only message content.
7. **No engagement trap.** No infinite swipe feed, urgency timers, streak pressure, hidden pay-to-be-seen, or shame-based reactivation. A useful session can end after one good introduction.
8. **User sovereignty.** Export, correction, deletion requests, revocation, pause, and departure must be accessible. Explain retention/replication limits honestly; never promise universal erasure from third-party copies or immutable records.
9. **Plural outcomes.** Romantic dates, friendship, activity partners, respectful declines, and a decision to stop are distinct outcomes. Never silently repurpose a friendship or community interaction as a dating lead.
10. **Evidence over marketing.** Claims about matching quality, safety, fairness, or well-being require exact-subject evidence and a documented evaluation design.

### Adult eligibility

Any dating pilot is adults-only. Age eligibility must be checked before dating discovery or messaging through a proportionate, privacy-reviewed flow with an accessible appeal route. Do not make face biometrics the only route. Do not retain identity documents, selfies, or raw verification artifacts unless an explicit legal and security review establishes necessity. A verified age or identity claim is not a promise of safety.

## 4. Paradigm shifts

### 4.1 From one-sided engagement ranking to two-sided mutual feasibility

Many recommenders learn from clicks or engagement, although one person's interest does not mean the other person is interested. First determine whether an introduction is admissible for both people. Then rank only the mutually eligible set.

Candidate algorithm families to compare:

- **B0 — feasibility-only deterministic pair ordering:** enforce hard eligibility, then use a stable pseudo-random-looking priority independent of declared interest utility. It is a neutral ranking control, not cryptographic randomness.
- **B1 — one-sided declared-utility ranking:** rank by the viewer's synthetic declared-interest utility after eligibility filters. This is a transparent offline baseline, not a learned engagement recommender.
- **B2 — pairwise mutual-utility ranking:** rank each candidate by the minimum and sum of both sides' declared utilities. This is not a globally coordinated or stable-marriage algorithm.
- **B3 — exposure-aware multi-objective allocation:** use pairwise mutual utility plus market-level coverage/exposure objectives, user-controlled exploration, and transparent tie-breaking. Pairwise mutual utility alone is not expected to remove recommendation concentration; allocation-level constraints are a separate intervention.

Do not collapse everything into one magic compatibility number. Keep a vector of interpretable dimensions; prefer lexicographic constraints or Pareto comparisons where practical; disclose the ranking policy and uncertainty. Any exploration must remain within consent and hard-constraint boundaries.

### 4.2 From attention capture to user-chosen outcomes

Make the default experience a small, adjustable set of introductions—not an endless queue. Let people choose pace, set an introduction budget, suppress notifications, pause without penalty, and opt into reminders. Avoid incentives that reward repeated use regardless of outcome.

Measure whether the experience helps users make wanted connections, not whether it keeps them online.

### 4.3 From profile marketplace to shared experience

Offer optional routes to connection: conversation-first prompts, common-interest groups, volunteering, local events, and public activity suggestions. Photos and attraction remain user-controlled parts of dating; they simply should not be the only information architecture.

Symtropy may provide voluntary shared activities and reflection prompts. These must not be disguised psychological tests, mandatory compatibility exams, or clinical assessments.

Sol-Atlas may surface suitable public venues and community events using coarse, purpose-limited location. It must not reveal a person's live location, routines, or presence to a potential match without explicit, one-time disclosure.

### 4.4 From verification badges to scoped evidence

Verification should answer a narrow question: what was checked, by whom, when, under what method, and when does it expire? Examples include age eligibility, control of an account, or attendance at a specific event. Each check needs a separate purpose and disclosure decision.

- Identity verification is not proof of sincerity, safety, or relationship compatibility.
- A credential's cryptographic validity is not proof that its claim is true.
- No public trust score, shared blacklist, automatic guilt state, or immutable allegation history.
- No user should have to disclose government identity to every match just to prove a narrow fact.
- Use selective disclosure when technically suitable, and assess issuer trust, metadata leakage, correlation, expiration, revocation, and appeal behavior.

### 4.5 From reactive moderation to adversarial lifecycle safety

Threat actors can create accounts, build emotional trust, move conversations off-platform, pressure people for money or intimate images, and return through replacement accounts. Safety therefore has to cover onboarding, discovery, conversation, off-platform warnings, block/report, investigation, victim notification, and account recovery.

Use layered risk signals, rate limits, proportionate friction, user-facing warnings, fast blocking, trained human review, evidence preservation with clear retention, and meaningful appeals. Models can prioritize review; a model score alone must not permanently ban a person or publicly label them a scammer.

Instrument drift and attack adaptation. Define named owners, review intervals, incident response, and a kill switch before launching any classifier that changes reach or account access. Do not hard-code a model that is never reviewed, and do not equate a new model version with a safer model.

### 4.6 From “decentralized means private” to explicit disclosure control

Encryption does not automatically hide sender, recipient, timing, thread structure, shared-group membership, public actions, or access patterns. Mycelix's existing Pulse issue [#5121](https://github.com/Luminous-Dynamics/mycelix/issues/5121) already identifies a concrete metadata boundary in the documented V2 message path.

Do not publish sensitive dating intentions, match graphs, precise locations, intimate reports, or identifying social edges to a public replicated store merely because message bodies are encrypted. Keep sensitive fields off immutable public paths by default. For every data flow, name the observer, purpose, retention, authorized action, and residual leakage.

### 4.7 From platform monopoly to portable participation

Users should be able to export non-sensitive profile data and their own content where lawful and safe, withdraw credentials, and leave without a punitive loss of all identity. Portability must not export another person's private messages, safety reports, location, or consent history without authority.

Community governance can change product rules, but it must not overrule an individual's consent or reveal protected safety data.

## 5. System boundaries and ownership

Compose existing work rather than create a competing social substrate.

| Component | Proposed responsibility | Explicitly does not own |
|---|---|---|
| **Mycelix / Pulse** | Canonical user-controlled social actions, message protocols, consent/block/report semantics, provenance and access boundaries | A global attractiveness/trust score; model-generated authority |
| **Symthaea** | Candidate explanations, constrained ranking proposals, simulation, uncertainty and calibration analysis | Access grants, consent, identity adjudication, automatic punishment |
| **Mycelix Identity / credential wallet** | Optional scoped proofs under holder control and explicit verifier purpose | A mandatory universal identity dossier or public reputation |
| **Sol-Atlas** | Coarse public-event/place discovery under location and safety policy | Live location sharing or tracking prospective dates |
| **Symtropy** | Optional shared activities, reflection prompts, coordination and user feedback | A psychological test, relationship verdict, or hidden behavioral surveillance |
| **Human safety/governance process** | Review, victim support, appeals, policy and incident accountability | Unreviewable model authority |

Before connecting a dating UI, build on the existing Pulse social-semantic line: [#3728](https://github.com/Luminous-Dynamics/mycelix/issues/3728), [#3729](https://github.com/Luminous-Dynamics/mycelix/pull/3729), [#3746](https://github.com/Luminous-Dynamics/mycelix/issues/3746), [#3772](https://github.com/Luminous-Dynamics/mycelix/pull/3772), as well as the scam/security and metadata threads [#5110](https://github.com/Luminous-Dynamics/mycelix/issues/5110) and [#5121](https://github.com/Luminous-Dynamics/mycelix/issues/5121). Verify their exact current status and head before relying on any qualification claim.

Potential transport-neutral contract families to review before coding:

- **ConnectionIntentV1:** what type of connection a person currently wants, audience, and expiry.
- **EligibilitySnapshotV1:** the exact, purpose-bound hard-constraint result for one recommendation run.
- **MutualRecommendationV1:** two-sided admissibility, explanation references, algorithm version, and expiry—never a person score.
- **ConsentTransitionV1:** explicit offer/accept/decline/withdraw transitions and actor evidence.
- **SafetyActionV1:** scoped block/mute/report/review/appeal observations without a public guilt field.
- **EvaluationManifestV1** and **EvaluationReceiptV1:** frozen input, algorithm, seed, metrics, uncertainty, negative controls and limits.

These are candidate names, not existing implemented types. No new Holochain zome or second message database is justified until the current semantic and durable-message contracts are shown to be insufficient.

## 6. Threat model

At minimum, test against:

- Impersonation, synthetic personas, stolen photographs, and duplicate/coordinated accounts.
- Romance fraud, investment scams, extortion, coercive control, harassment, unsolicited sexual material, and attempts to evade a block.
- Stalking, exact-location inference, event-attendance leakage, doxxing, and correlation of supposedly unlinkable credentials.
- Fake or overbroad verification claims, compromised verifiers, malicious insiders, and unjustified credential retention.
- Coordinated false reports, retaliatory accusations, discrimination, brigading, and appeals abuse.
- Popularity feedback loops, recommender gaming, cold-start disadvantage, exposure concentration, and preference-distribution shifts.
- Model memorization, sensitive-trait inference, explanation laundering, prompt injection through profiles, and leaked private context.
- Data retention failures, analytics/log leakage, backup/replica assumptions, weak account recovery, and unauthorized export.

Define attacker capability and vantage for each test; don't label a threat “solved” because a happy-path unit test passes.

## 7. Benchmark suite

All numerical values below are **proposed acceptance targets**, not current results. Freeze the dataset generator, baseline implementations, cohort definitions, metric formulas, statistical plan, and thresholds before running experiments. A changed baseline or threshold requires a new manifest and explicit explanation.

### 7.1 Lane A — protocol and safety invariants

| ID | Metric / invariant | Proposed gate |
|---|---|---|
| A1 | Mutual eligibility | 100% of issued recommendations satisfy both parties' current consent, age eligibility, hard constraints, and block state at the authoritative decision point. |
| A2 | One-sided interest privacy | 0 profile disclosures, chat creation, or notification leaks that reveal the other party's non-consent or hidden interest. |
| A3 | Block enforcement | 100% of same-account contact attempts after an effective block are suppressed; adversarial tests also cover replacement-account and alias behavior without claiming perfect identity resolution. |
| A4 | Withdrawal / stale state | 100% of withdrawn, expired, or superseded offers are rejected; retries and replay do not recreate withdrawn consent. |
| A5 | Safety controls | Block/report tools function from every live communication surface; protective action is immediate; human-review service levels are published and met before a live pilot. |
| A6 | Sensitive-data disclosure | 0 unauthorized disclosures in the complete negative-control suite; unknown authorization fails closed. This is a test-scope claim, not proof that all real-world leaks are impossible. |
| A7 | Human authority | 0 permanent bans or public guilt labels caused solely by an ML risk score; adverse actions have provenance, explanation, review and appeal. |
| A8 | Traceability | 100% of recommendation reasons map to declared, permitted inputs and the exact algorithm version; no post-hoc fabricated explanation. |

### 7.2 Lane B — synthetic matching and population health

Build a deterministic synthetic market generator with configurable population size, feasible-match scarcity, preference heterogeneity, new-user cold starts, uneven activity, coordinated manipulators, and multiple user-defined outcomes. Synthetic profiles must contain no real people's personal data.

Compare B0–B3 from Section 4.1 on a frozen corpus; repeat across a preregistered seed suite and report variance and confidence intervals. Keep algorithm quality separate from feasibility, privacy/safety invariants, and simulated exposure opportunity. Exposure Gini must be reported beside a candidate's number of feasible counterparts (opportunity/degree); raw Gini can otherwise confuse lack of feasible options with ranking bias.

| Metric ID | Metric | Proposed gate for exposure-aware B3; compare against baselines B0–B2 |
|---|---|---|
| M1 | Eligible-user coverage | Initial gate: mean candidate coverage of at least 99.5% across a preregistered seed suite for opted-in users who have at least one feasible counterpart in a single allocation batch, with no seed below 98%. Report infeasible users separately and compare every baseline on identical inputs. Calendar-window and repeat-session effects require a later temporal simulation. |
| M2a | Raw exposure concentration | Initial gate: at least 8% mean per-seed relative reduction in recommendation-exposure Gini versus B1's one-sided declared-utility baseline; 20% is a stretch goal. Preserve hard constraints and report raw exposure counts. |
| M2b | Opportunity-normalized exposure concentration | Initial gate: at least 10% mean per-seed relative reduction in Gini of incoming exposure divided by each candidate's number of feasible counterparts, versus B1; review any scenario-family aggregate more than 2% worse than baseline. This complements raw Gini rather than replacing it. |
| M3 | Reciprocal-shortlist proxy | Count unordered pairs where each person independently receives the other in their own top-k list. This is a synthetic recommendation-consistency proxy—not real mutual interest, acceptance, a date, or relationship quality. Report counts per 100 eligible users and seed-level uncertainty; no positive lift is assumed. |
| M4 | Proxy non-inferiority | Do not accept exposure-aware B3 if reciprocal-shortlist-pair count is more than 5% lower than the strongest simpler baseline, unless a preregistered multi-objective rationale and human-outcome study justifies that trade-off. Never relabel this proxy as acceptance. |
| M5 | Subgroup examination | Report coverage, raw and opportunity-normalized exposure, reciprocal-shortlist proxy and simulated decline/block cases across appropriately consented/privacy-protected slices. Investigate unexplained relative gaps over 10%; do not force equal outcomes regardless of preference feasibility or publish small-cell data. |
| M6 | Robustness | Eligibility and consent gates remain 100% enforced under cold-start, malformed profile, preference change, stale cache, replay, coordinated account and model-failure cases. |

The thresholds are proposed engineering gates, not established scientific norms. Freeze them before the qualification run and revise only with an explicit benchmark-version change. Raw exposure Gini is affected by the feasible market graph: report candidate exposure conditional on opportunity, coverage at multiple top-k budgets, distribution tails (including zero exposure among feasible users), and the seed-level spread. A synthetic pass does not establish human compatibility or real-world fairness.

**Exploratory sizing note — not Rust qualification or human-outcome evidence.** After aligning the Python reference generator with the Rust generator's inclusive ranges, explicit discovery-opt-in field, and round rotation, a quick reference simulation (30 seeds, 200 synthetic participants per seed, top-k=5) yielded mean raw exposure Gini of roughly 0.344 for feasibility-only ordering, 0.345 for one-sided utility, 0.347 for pairwise mutual utility, and 0.317 for exposure-balanced allocation. Mean opportunity-normalized exposure Gini was approximately 0.0540, 0.0545, 0.0566, and 0.0298, respectively. Mean candidate coverage was approximately 98.2% for feasibility-only, 98.1% for one-sided utility, 97.9% for pairwise mutual utility, and 99.98% for exposure-balanced allocation. Mean per-seed raw Gini reduction was about 8.2% versus one-sided utility (range about 0.4–16.2%), and opportunity-normalized Gini fell by about 43% on average (range about 14–59%); both improvements occurred in all 30 simulated seeds. This is only a preliminary sizing observation from a separate Python reference implementation—not execution of the current Rust source, not proof the checked-in tests pass, not a generalizable effect, and not real-world outcome evidence. It suggests testing allocation-level intervention separately from pair scoring, using 8% raw-Gini and 10% opportunity-normalized-Gini reductions as provisional initial targets; 20% and 25% are stretch goals, respectively. Recalculate every metric from the exact Rust head across the preregistered corpus and multiple market-topology scenarios before any qualification claim.

### 7.3 Lane C — scam and abuse resistance

Create a versioned, held-out adversarial corpus with provenance, attack class, test owner, and negative controls. Keep the corpus private where disclosure would help attackers; publish non-sensitive counts and definitions.

- Measure recall, precision, calibration, false-positive rates, time-to-detection, time-to-protection, and slice performance; never publish a single accuracy figure alone. Detection metrics apply only to the named held-out corpus and label policy, not to the prevalence or truth of all scam activity in the wild.
- Provisional offline target: at least 90% recall and 95% precision on the specifically defined held-out scam corpus, with confidence intervals and per-attack-class results. If the corpus is too small or non-representative, report “insufficient evidence” instead of passing.
- No automated permanent ban based solely on this classifier. A low-confidence signal may trigger a warning or rate limit only if the policy is defined and proportionate; stronger action requires the stated review standard.
- Re-check drift at least weekly during a pilot; conduct a named human policy/model review at least monthly and after a material incident or detected distribution shift. These are proposed operating targets, not substitutes for a staffed response function.
- Report immediately protective controls separately from human-review turnaround. Proposed SLO: 90% of high-severity reports receive qualified human triage within 24 hours, with a clear after-hours fallback. Do not launch a feature whose promise requires staffing the project does not have.
- Never expose a reporter's identity to a reported person by default, publish user allegations, or promise that identity/face verification guarantees safety.

### 7.4 Lane D — privacy and sovereignty

- 100% of fields in each live data flow have a documented sensitivity class, purpose, audience, retention/expiry, controller/steward, and deletion/export behavior.
- 0 intimate preferences, dating-graph edges, precise location, or safety-report bodies placed in public replicated/immutable storage by default.
- 100% pass rate on authorization-denial, cross-account isolation, purpose-mismatch, revoked credential, export-redaction, and retention-expiry negative controls.
- 100% of export, correction, account pause, revocation, and deletion workflows tested against exact stated policy and replicas/backups/derived-index handling.
- Claims about anonymity, unlinkability, or deletion must name the attacker and storage model and report residual leakage. A successful payload-encryption test is not metadata-privacy evidence.

### 7.5 Lane E — user outcomes and well-being

Run this lane only after protocol, safety, privacy, moderation, and governance gates pass. Use opt-in participants, an independent ethics/research review appropriate to the jurisdiction, a pre-registered analysis plan, a power calculation, a clear withdrawal path, and a no-penalty control experience. Never vary the safety protections across experimental arms.

Primary outcomes should be chosen with participants and measured using appropriate validated instruments where available:

- User-rated quality and mutuality of connections.
- Feeling respected and in control; unwanted contact; ability to decline or pause safely.
- Loneliness, dating-app exhaustion/burnout, body-image pressure, and emotional distress.
- User-reported progress toward their stated goal, including positive exits and friendship.
- Unwanted session time, notification pressure, and satisfaction with pausing or leaving.

Do not adopt retention or messages sent as the primary outcome. Pre-register a minimum practically meaningful benefit and a non-inferiority margin for harms with an independent research adviser. Report null, harmful, mixed, and inconclusive results. A promising pre-set exploratory goal is a 0.2 standard-deviation improvement on a co-designed connection-quality outcome; the final threshold must be justified and powered before recruitment, not selected after seeing the data.

Stopping rules: pause the experiment after a credible signal of increased harassment, material privacy exposure, systematic subgroup harm, or deterioration beyond the pre-registered harm margin. A successful pause is a safety outcome, not a product failure.

### 7.6 Lane F — software and reproducibility

- Reference target for the pure ranking stage: p95 below 250 ms and p99 below 750 ms for top-20 ranking from 10,000 pre-filtered synthetic candidates on a pinned reference machine. Record CPU, memory, build profile, dataset digest, seed and workload; revise the target only through a documented benchmark change.
- All algorithm outputs deterministic for a fixed input, policy version and seed; where privacy requires randomized outputs, record the randomness contract without logging secrets.
- Every evidence receipt binds exact commit SHA, corpus/generator digest, configuration, seed, environment, code version, metric definitions, output digest and negative-control results.
- Queued CI is not a pass. A result applies only to the exact evaluated head and recorded environment. No result may be silently transferred to another commit.

## 8. Evaluation protocol

### Stage 0 — contract review

Review the product constitution, data-flow inventory, threat model, pilot jurisdiction and existing Pulse semantic contracts. Resolve overlaps with existing social/security work before adding types or storage. This RFC alone does not authorize a production route.

### Stage 1 — deterministic offline harness

Implement a small Rust-first pure evaluation crate or reuse an appropriate existing crate after repository inspection. It should:

1. Generate seeded synthetic markets with explicit discovery opt-in separated from account activity, and publish a canonical corpus manifest.
2. Run feasibility-only B0, one-sided utility B1, pairwise mutual-utility B2, and exposure-aware allocation B3 on identical inputs.
3. Measure eligibility failures separately from ranking quality; discovery opt-in is not pair-specific consent or permission to reveal reciprocal interest.
4. Compute exposure Gini, opportunity-normalized exposure, candidate coverage, reciprocal-shortlist proxy, subgroup diagnostics, seed-level intervals and runtime distributions. Keep real participant mutual acceptance for a later consent-based pilot; synthetic profiles cannot supply it.
5. Exercise negative controls for block, withdrawal, stale preferences, malformed inputs, replay, cold starts, coordinated behavior and privacy violations.
6. Emit a deterministic JSON/TOML report and digest; make missing metrics or missing test vectors fail closed.
7. Run a multi-round allocation experiment that compares stateless per-batch exposure balancing with historical exposure allocation, reporting cumulative raw and opportunity-normalized inequality across at least 10 rounds.
8. Compare the quality–fairness frontier using experimental 70%, 80%, and 90% minimum mutual-utility floors relative to each viewer's best feasible counterpart. Report utility retention, cumulative exposure inequality, candidate coverage, and per-scenario paired comparisons. These thresholds are benchmark controls, not product defaults.
9. Normalize cumulative exposure by feasible-counterpart opportunity degree as well as reporting raw exposure. A candidate with three times the feasible opportunities should not automatically receive the same target raw exposure as a candidate with one opportunity.
10. For quality-constrained allocation, also compute a policy-qualified opportunity denominator: the number of viewers for whom each candidate passes the same per-viewer quality floor used by that policy. Report broad hard-feasibility metrics and policy-qualified metrics separately; they answer different questions and should not be silently substituted for one another.
11. Treat exposure history as experimental state only. Before production use, define its scope, retention, deletion, update atomicity, market-change semantics, and recovery behavior; a local counter is not a durable or privacy-qualified ledger.

No real profiles, private messages, or safety reports are needed for Stage 1.

#### Required synthetic robustness families

A single generator configuration is insufficient to qualify exposure allocation. The initial harness suite therefore runs the same policies over these deterministic, seeded stress families:

| Family | Failure mode under test |
|---|---|
| Baseline | Preserve comparability with the original synthetic generator |
| Sparse market | Low feasible degree, rare interests, isolated candidates, unstable relative metrics |
| Dense market | High feasible degree, larger candidate pools, top-k congestion |
| Opt-out heavy | Low discovery participation and high inactivity; verify denominators do not hide excluded people |
| Skewed goals | Uneven distribution of stated connection goals across the two abstract partitions |
| Concentrated preferences | Deliberate popularity-concentration negative control; stable tie-breaking must not be mistaken for quality |
| Adversarial blocking | Coordinated blocks against a small target set; every block must remain authoritative |
| Cold start | Participants with minimal and uncommon declared interests |

Use the same seed sequence for each family, but compute a separate corpus fingerprint for every family/seed pair. A family name documents the transformation, not a claim that the scenario represents the prevalence or psychology of real people.

#### Interpretation and non-regression rules

- Publish each family separately; never let an aggregate mean hide a catastrophic sparse, opt-out, or adversarial slice.
- Report cumulative results across repeated batches as well as one-batch results. A per-batch Gini reduction is insufficient if the same people are systematically overexposed across time. Compare stateless balancing to a historical-exposure policy using identical market/seed sequences.
- Report the quality–fairness Pareto frontier, including a mutual-utility-floor retention proxy alongside raw and opportunity-normalized Gini. Identify dominated policies instead of declaring a single winner from Gini alone. A candidate may be less frequently exposed because they have fewer feasible counterparts; compare normalized exposure as well as raw counts.
- The exposure ledger currently normalizes past counts by the current feasible-opportunity graph. This is useful only as a fixed-market experiment. Dynamic markets require a policy for time-varying opportunity denominators, profile/opt-in changes, account deletion, privacy scope, retention, and replay/recovery before this metric can qualify a product.
- Under a per-viewer utility floor, distinguish the broad hard-feasible candidate pool from the policy-qualified directed opportunity pool. Report raw inequality on a common hard-feasible pool for comparability, then report opportunity-normalized inequality and coverage over the opportunity pool the policy could actually allocate. Never count policy-excluded pairs as if they were available opportunities without labeling that denominator.
- Require zero invalid recommendations and zero eligibility/consent negative-control violations across every tested family, seed, and policy. A fairness benefit cannot compensate for one hard-invariant violation.
- Report the count of comparable seeds whenever a relative Gini reduction is calculated. When baseline Gini is zero or a denominator is not meaningful, report the metric as non-comparable rather than a false zero-percent improvement.
- Retain the initial ≥8% raw-Gini and ≥10% opportunity-normalized-Gini relative-reduction targets as overall experimental targets, not universal guarantees for every market topology.
- No family may suffer more than a 5% relative reduction in reciprocal-shortlist pairs against the strongest simpler baseline without a preregistered explanation. Also report feasible-pair density, coverage, exposure tails, and excluded/no-opportunity counts together.
- The initial 30-seed run is exploration. For a release decision, preregister a larger seed suite and scenario-specific sample/uncertainty requirements before inspecting the outcomes.

These are proposed evaluation rules. The new Rust scenario generators and sweep runner remain unqualified until the exact head has been compiled, executed, and checked against the documented corpus and toolchain.

### Stage 2 — adversarial security and privacy review

Map every field and observer, review age assurance and identity choices, define human moderation and appeals, and test account recovery and replacement-account abuse. Have an independent reviewer challenge the assumptions. Do not invent cryptography: evaluate established standards and get specialist review for privacy-enhancing cryptographic constructions.

### Stage 3 — small opt-in pilot

Only after release gates pass, conduct a consent-based pilot with clear scope, staffing, support, accessible feedback and withdrawal. Randomize ranking approaches only; do not withhold safety protections. Publish a preregistered aggregate report with uncertainty, missingness, limitations, adverse events and changes from the analysis plan.

### Stage 4 — expand or stop

Expand only if outcomes meet pre-set gates and independent review does not identify unacceptable unresolved harm. If the effect is absent, harmed users are disproportionately affected, or the evidence is underpowered, document that conclusion and improve the design before another trial.

## 9. Monetization and governance

Prefer transparent memberships, optional community-event services, or user-requested premium utilities. Prohibit selling intimate data, paid safety, hidden visibility auctions, or manipulative scarcity. Never make a user's continued lack of a partner financially valuable to the system.

Before a pilot, establish:
- an accountable operator and staffed safety escalation;
- a privacy/data-stewardship policy for the pilot jurisdiction;
- documented appeal and abuse-report procedures;
- incident disclosure and rollback rules;
- community/user consultation that does not expose reporters or private relationship data;
- a defined product exit and data-retention plan.

Decentralization does not eliminate legal duties, moderation responsibilities, or accountability to people harmed by the service.

## 10. First implementation sequence

1. **RFC review:** resolve terminology and ownership with existing Pulse social-contract, safety, and metadata work.
2. **Rust synthetic-market harness:** deterministic generator, B0/B2 baselines, hard-constraint assertions, exposure/coverage metrics, corpus digest.
3. **Adversarial corpus:** block/withdrawal/stale-consent/identity-replacement/privacy negative controls with explicit expected dispositions.
4. **B3 proposal:** add exposure-aware multi-objective allocation and explanations only after baselines are stable; compare pairwise mutual scoring against allocation-level exposure balancing separately.
5. **Independent review:** matching methodology, human-subject research, safety operations and privacy assessment.
6. **Pilot decision:** no launch date is set until evidence, staffing, governance and user research support it.

The initial vertical slice is intentionally modest: prove that the system never recommends across a hard constraint, separate explicit discovery opt-in from pair-specific consent, compare feasibility-only/one-sided/pairwise-mutual/exposure-aware ranking on synthetic data, and produce a reproducible exposure/coverage receipt. The term “mutual” in this offline harness refers only to a two-sided utility formula or reciprocal shortlist, not actual consent or human attraction. It can make useful progress without waiting for unrelated CI jobs or claiming that a prototype is production-ready.

## 11. Non-goals

This RFC does not propose a universal compatibility score, attraction ranking, diagnosis, lie detector, mental-health inference, biometric-only age gate, public blacklist, immutable allegations, covert sentiment surveillance, automatic relationship advice, a tokenized dating economy, or autonomous enforcement. It does not claim that blockchain, AI, credentialing, or decentralization independently makes a product safe.

## 12. Research references

1. Kuan-Ming Chen, Ya-Wen Hsieh, Ming-Jen Lin (2023), [“Reducing Recommendation Inequality via Two-Sided Matching: A Field Experiment of Online Dating,” *International Economic Review* 64(3), 1201–1221](https://doi.org/10.1111/iere.12631). Supports testing two-sided matching and exposure distribution; one platform's results must not be assumed universal.
2. Zac Bowman et al. (2025), [“Dating apps and their relationship with body image, mental health and wellbeing: A systematic review,” *Computers in Human Behavior* 165, 108515](https://doi.org/10.1016/j.chb.2024.108515). The review synthesizes 45 studies and reports substantial body-image concerns; study designs and samples limit causal inference.
3. Liesel L. Sharabi et al. (2026), [“Dating app use, psychological health, and psychological well-being: A systematic review and quantitative meta-analysis,” *Computers in Human Behavior* 177, 108879](https://doi.org/10.1016/j.chb.2025.108879). Reports a modest association across 23 studies (N=26,068) and explicitly notes cross-sectional limitations.
4. U.S. Joint Economic Committee, Minority (2026-10-13), [report release on deficiencies in dating-platform romance-scam prevention](https://www.jec.senate.gov/public/index.cfm/democrats/press-releases?ID=83F458A5-B37B-427C-B3A8-D64A5A13CE8A). Treat platform-provided data and committee interpretations as the report's findings, not a universal estimate of every service.
5. U.S. Federal Trade Commission (2025-02), [“Looking for love? Watch out for scammers”](https://consumer.ftc.gov/consumer-alerts/2025/02/looking-love-watch-out-scammers).
6. W3C (2025-05-15), [Verifiable Credentials Data Model v2.0 — Recommendation](https://www.w3.org/TR/vc-data-model/). See privacy, selective-disclosure, verification and correlation guidance. The model's existence does not select an identity provider or prove credential claims true.
7. Yingqiang Ge et al. (2021), [“Towards Long-term Fairness in Recommendation,” WSDM 2021](https://arxiv.org/abs/2101.03584). Motivates measuring exposure fairness dynamically rather than only in one-shot batches; its item-popularity setting is not direct evidence about human-connection markets.
8. Nil-Jana Akpinar et al. (2022), [“Long-term Dynamics of Fairness Intervention in Connection Recommender Systems”](https://arxiv.org/abs/2203.16432). Analyzes feedback-driven bias amplification and reports that interventions which look fair in aggregate can fail to mitigate bias over time in its stylized model.
9. Yildiz Zoralioglu and Emre Yalcin (2026-01-14), [“Dynamic feedback loops in recommender systems: Analyzing fairness, popularity bias, and user group disparities,” *Journal of Intelligent Information Systems* 64, 2091–2119](https://doi.org/10.1007/s10844-026-01025-y). Reports longitudinal calibration, diversity and exposure disparities across popularity-, diverse-, and niche-focused groups on two benchmark datasets. The results motivate temporal testing; they are not results for Mycelix or human-connection outcomes.

## 13. Qualification statement

**Current qualification: not run / not applicable to implementation.** This commit is a design RFC. No benchmark has run, no algorithm has been implemented, no human-participant study has been conducted, and no current Pulse capability is promoted by this document. Follow-on PRs must attach exact-head evidence and preserve these claim boundaries.
