# Creator Sovereignty Control Plane (CSCP): Research and RFC v0.1

**Status:** Research proposal; not implemented or validated.
**Audience:** Mycelix, Symthaea, Symtropy, and Sol-Atlas maintainers.
**Primary design partner hypothesis:** Independent musicians who livestream or publish short-form video, with architecture intended to generalize to streamers and other creator careers.
**Date:** 2026-10-17.

## Executive proposal

Build a creator-owned career control plane that connects creative work, audience relationships, income, workload, agreements, and rights into one auditable working model. It should help creators make better decisions and retain more value—not merely maximize views, generate more posts, or migrate creators into another closed platform.

The first product should be narrow: **a Creator Control Room for career accounting and post-session learning**. It should accept creator-provided exports and manually entered records before depending on platform APIs. It can establish a trustworthy baseline for income, work hours, content outcomes, and direct relationships, then produce explainable weekly experiments. It should not publish, message, accept contracts, move money, or license a creator's likeness without explicit human approval.

This RFC is a product/architecture proposal, not a statement that the four projects already provide a production-ready creator suite.

## Why now: evidence and limits

1. **Creator advertising is a substantial and expanding market.** IAB's November 2025 report estimated US creator advertising spend at $37 billion for 2025 and projected $44 billion for 2026. The same report identified creator selection, measurement, standards, and operational tools as opportunity areas. These are market estimates, not guaranteed demand for this product. Source: [IAB, 2025 Creator Economy Ad Spend & Strategy Report](https://www.iab.com/insights/2025-creator-economy-ad-spend-strategy-report/).
2. **Creators are looking beyond follower counts.** Patreon’s *State of Create* research describes changing creator priorities toward creative quality, fan relationships, and financial stability, and reports creator concerns about reliable access to fans. This is useful directional evidence, but it is research published by a company that sells creator tools; treat its estimates and market sizing as interested-source data, not independent proof. Source: [State of Create](https://stateofcreate.co/).
3. **Portability is not a permissionless technical assumption.** TikTok’s current Data Portability API documentation says the feature covers TikTok users in the EEA and UK; access also requires user consent plus separate Data Portability API and Login Kit approvals. TikTok's current FAQ says review may take 3–4 weeks. YouTube also maintains API quotas and policy/compliance requirements. We must support manual records and creator-authorized exports first, and ship each connector only when its scope, permissions, deletion behavior, geographic eligibility, and current terms are documented. Sources: [TikTok Data Portability API: Get Started](https://developers.tiktok.com/docs/en/data-portability-api-get-started), [TikTok Data Portability API FAQ](https://developers.tiktok.com/docs/en/data-portability-api-faq), and [YouTube API quota and compliance audits](https://developers.google.com/youtube/v3/guides/quota_and_compliance_audits).
4. **Sponsorship operations carry real disclosure obligations.** The US FTC advises influencers to disclose material connections clearly and conspicuously; a platform's built-in control may not, by itself, make the disclosure adequate. The product can offer checklists and evidence records, but must not claim that a generated checklist certifies legal compliance. Source: [FTC, Disclosures 101 for Social Media Influencers](https://www.ftc.gov/node/60343).

5. **Creator income is unevenly distributed.** CreatorIQ and Influencers.club's August 2026 survey of 5,095 creators across 100 regions reports that 67% of respondents earned less than $10,000 from content creation in the prior year and 62% said it was not their primary income source. This is vendor-associated survey data, not proof of our product's demand or impact. A separate CreatorIQ brand-payment analysis reports concentration among top earners; do not combine its dataset with the broad survey. Sources: [State of Creators 2026](https://www.creatoriq.com/press/releases/creatoriq-state-of-creators-report-2026) and [State of Creator Compensation](https://www.creatoriq.com/press/releases/state-of-creator-compensation-).
6. **Creator resilience matters alongside output.** Patreon's 2025 survey reports that many creators experience burnout and pressure to post constantly; these are interested-source survey results and motivate a hypothesis to test, not a clinical or causal conclusion. Source: [State of Create](https://stateofcreate.co/).
7. **Music rights depend on identifying the right kind of asset.** IFPI reports global recorded music revenue of US$31.7 billion for 2025, while CISAC reports EUR13.97 billion in total creator royalty collections across multiple creative sectors for 2024. These are distinct industry measurements, not our addressable market. More importantly, the official ISWC system identifies musical works and the ISRC system identifies recordings; neither identifier by itself establishes rights ownership or shares. Sources: [IFPI Global Music Report 2026](https://www.ifpi.org/global-music-report-2026-global-recorded-music-revenues-grow-6-4-as-record-companies-drive-innovation/), [CISAC Global Collections Report 2025](https://www.cisac.org/cisac-global-collections-report-2025), [ISWC](https://www.iswc.org/iswc), and [ISRC](https://isrc.ifpi.org/faqs).
8. **Voice and likeness permission must be scoped separately.** The U.S. Copyright Office's 2024 digital-replicas report recommended that Congress consider a federal right against knowing distribution of unauthorized digital replicas. That is a policy recommendation, not a claim that a new federal statute was enacted by the report. Product requirements should therefore separate permission to analyze or publish an asset from permission to clone a voice/likeness or train a model, with applicable law reviewed for the jurisdictions involved. Source: [U.S. Copyright Office, Digital Replicas Report](https://copyright.gov/ai/Copyright-and-Artificial-Intelligence-Part-1-Digital-Replicas-Report.pdf).

These facts support an experiment, not a claim that the market is underserved in every segment. Existing tools cover many individual features. Differentiation must be demonstrated by outcomes, portability, and trust—not by aggregating more dashboards.

## Competitive reality: do not rebuild an existing creator stack

Official product material already advertises meaningful bundles:

- [Beacons](https://website.beacons.ai/i/blog/announcing-beacons-2-0) combines a link-in-bio surface, store, email marketing, media kit, AI recommendations, and creator operations.
- [Fourthwall](https://fourthwall.com/creators) combines merchandise, memberships, donations, a branded creator hub, and supporter-facing experiences; its [membership product](https://fourthwall.com/memberships) advertises subscriber analytics and retention views.
- [StreamElements](https://streamelements.com/) supplies streaming alerts/overlays, chat tools, merch, and sponsorship offers.
- [Passionfroot](https://www.passionfroot.me/creators) offers media kits, sponsor discovery, storefronts, booking, scheduling, and payment workflows.
- Patreon and similar membership products compete strongly on paid community and direct-to-fan offers; the [State of Create](https://stateofcreate.co/) report should be read as useful vendor research, not as neutral market measurement.

These products make a generic "all-in-one creator dashboard" a weak thesis. Do not rebuild storefronts, merch logistics, streaming overlays, or membership hosting in the MVP. Integrate or export to tools creators already choose, subject to explicit permissions and available interfaces.

**The initial wedge is independent cross-tool career accounting and decision quality:** reconcile creator-controlled income, costs, work time, and observed content outcomes; show how every metric was derived; provide lossless export; and run small evidence-linked experiments without forcing a platform migration. This gives the creator value even if no sponsor marketplace, social integration, or new community network ever ships. A later product may expand only when design-partner evidence shows that adjacent workflows are painful enough to justify replacing an incumbent.

## Paradigm shifts

### 1. Platform-first → creator-owned control plane

Treat YouTube, Twitch, TikTok, Instagram, email, storefronts, ticketing services, and payment processors as replaceable external systems. The control plane maintains only data the creator has supplied or authorized. It never assumes followers, DMs, analytics, or platform identity can be transferred.

- Make CSV/JSON import, manual records, and complete export first-class capabilities.
- Keep each adapter's permissions, source, last successful sync, freshness, known omissions, and deletion policy explicit.
- A disconnected or revoked source degrades gracefully; it must not corrupt the creator's canonical records.
- Never scrape private pages, evade rate limits, or violate source terms to simulate portability.

### 2. Vanity metrics → sustainable creative prosperity

Views and follower counts are diagnostic signals, not the objective function. The product should help a creator understand money, time, creative quality, audience relationships, and resilience together.

Core metric:

`net creator income per work-hour = (cash receipts - refunds - platform/payment fees - attributable operating costs) / recorded creator work-hours`

Keep taxes, capital expenditure, non-cash benefits, and allocated fixed costs separately visible so that comparisons are not falsely precise. Show raw figures, assumptions, missing data, and uncertainty. Never silently estimate hours or expenses and present them as measured facts.

Track platform/revenue concentration, returning voluntary supporters, direct-contact permissions, schedule stability, self-reported workload, and creator-rated creative quality alongside gross reach. Do not convert burnout or well-being into an automated diagnosis or a score that penalizes creators.

### 3. Content optimization → preregistered decision experiments

Replace generic “post more at 7 PM” advice with small, testable, creator-approved experiments.

Each experiment records:
- hypothesis and reason for choosing it;
- baseline period and metric definition;
- one change at a time where practicable;
- confounders (release timing, paid promotion, collaborations, seasonality, viral events);
- result, uncertainty, and the next decision;
- a human-readable record of why the result does or does not support the hypothesis.

Do not claim causation from a correlation. Use holdouts or randomized comparisons only when ethical, practical, and consent-compatible. Record null and negative results. Recommendation acceptance is not evidence that the recommendation was correct.

### 4. AI content factory → consentful creative augmentation

Symthaea should help a creator review their own work and make deliberate choices, not flood their accounts with generic output. Default to drafting, summarizing, indexing, and explaining. The human remains the publisher and decision-maker.

- No default training on private recordings, unreleased tracks, messages, or contracts.
- No synthetic voice, face, likeness, or style imitation without explicit, scoped permission from the rights holder.
- Keep AI-generated suggestions separate from observed facts and source records.
- Store source links/timestamps for proposed highlights and require creator review before publication.
- Support a correction trail when the AI's summary or attribution is wrong.

### 5. Data hoarding → minimum viable, creator-controlled evidence

Use the minimum data needed for each feature. Sensitive source content should stay in a creator-controlled or encrypted private store where possible; do not put private messages, raw analytics, personal information, unpublished media, or financial details into a public DHT.

A record should identify its owner, purpose, source, observed time, consent/authority scope, retention rule, and correction/supersession history. Revocation must stop future access and processing; it cannot falsely promise deletion from third-party platforms or immutable copies already lawfully published. Public attestations should reveal only what is explicitly intended to be public.

### 6. Extractive competition → fair collaborative production

Treat credits, contribution evidence, licenses, deliverables, usage scope, revenue splits, due dates, and dispute status as linked records. Let contributors agree to terms before relying on them. Build collaboration receipts and exportable credits before inventing a tokenized marketplace.

Mycelix reputation must not be treated as money, proof of artistic worth, or a substitute for contract terms. Do not implement pay-to-play rankings, speculative tokens, or founder/fan monetary privilege as prerequisites for this product.

### 7. Digital reach → physical-world opportunity

Only after the career ledger is useful, Sol-Atlas can help compare venue options, route costs, pop-up events, collaborators, and region-level demand. Geographic inference must be based on opted-in or properly aggregated data, with minimum aggregation thresholds and no public exposure of an individual fan's location.

Symtropy can later enable music-reactive stages, persistent fan spaces, and interactive stream overlays. Those are differentiated extensions, not first-release dependencies.

## System responsibilities

The projects are research and infrastructure substrates; the creator product must not imply that they are already integrated or production-ready.

| System | Proposed responsibility | Non-goal / guardrail |
|---|---|---|
| **Mycelix** | Portable identity/claims, explicit permissions, contributor credits, membership and agreement evidence, revocation/correction lineage, and an interoperable record layer where justified. | No public storage of sensitive creator/fan records; no token required for basic use; no assumption that DHT persistence equals legal truth or complete privacy. Existing Mycelix work is pre-alpha and must pass its own maturity gates. |
| **Symthaea** | Explainable, advisory session summaries, content indexing, creator-approved experiment proposals, evidence-aware career analysis. | Advisory only; cannot publish, message, accept agreements, spend money, or claim verified outcomes without human approval and evidence. |
| **Symtropy** | Later, interactive and music-responsive experiences, virtual stages, fan participation primitives. | Not required for initial product value; do not let engagement mechanics become manipulative or addictive by design. |
| **Sol-Atlas** | Later, venue/event opportunity mapping, route and tour planning, aggregate demand analysis. | No individual location tracking or unsupported demand forecasts. Avoid duplicating general maps when a creator's problem does not need spatial analysis. |

### Proposed canonical concepts

The first schema should be small and versioned. Candidate records:

- `CreatorWorkspace`: tenant/owner identity, settings, access grants, export policy.
- `CreativeArtifact`: creator-defined ID, type, version, rights-holder claims, source reference, provenance, and publication state.
- `WorkSession`: start/end or recorded duration, activity type, participant consents, linked artifacts, and optional creator notes.
- `DistributionObservation`: platform/source, content reference, metric, value, observation window, units, collection method, and freshness.
- `RevenueEvent`: amount/currency, receipt/refund/fee/cost classification, effective date, source evidence, and reconciliation status.
- `AudiencePermission`: contact or pseudonymous supporter reference, purpose, provenance, scope, timestamp, revocation, and retention expiry; collect only fields required by the chosen feature.
- `Agreement` and `RightsGrant`: parties, approved terms/version, deliverables, usage scope, compensation/split, due dates, disclosures required, signatures/acceptances, and correction trail.
- `Experiment`: preregistered hypothesis, baseline, treatment, outcome, confounders, result, uncertainty, and decision.
- `EvidenceReceipt`: source, digest or reference, exact time, transform/version, and validation result, without forcing confidential source material into public storage.

Every numeric observation needs units and a source. Every derived metric needs a versioned formula and a list of contributing records. The implementation must reject ambiguous currency, duplicate financial events, incompatible time windows, and stale values presented as current. Any conflict or correction should supersede the prior record; never silently rewrite history.

## MVP: prove one valuable loop

**Target cohort hypothesis:** 10–20 independent musicians who livestream or publish short-form content and manage at least two income or distribution channels. Recruit actual design partners; this is not a claim of existing participation.

### Alpha workflow

1. Import a creator-controlled CSV/JSON export or enter a minimal manual record. Show field mapping and omissions before accepting the import.
2. Create a weekly ledger of work hours, content sessions, receipts/fees/costs, platform outcomes, and direct supporter permissions where the creator chooses to track them.
3. Produce a source-linked weekly debrief: what changed, what data is missing, which result is merely correlational, and one optional low-risk experiment.
4. Let the creator accept, edit, or reject the experiment. Record the decision and its outcome without penalizing rejection.
5. Export the entire workspace as documented JSON/CSV with a manifest and schema version; verify round-trip import.
6. Collect time-use data and creator-reported usefulness for a 4-week baseline followed by an 8–12-week pilot.

### Explicit non-goals for MVP

- Building a new streaming/social network.
- Scraping platforms or promising follower portability.
- Auto-posting, auto-DMing, accepting contracts, changing prices, spending money, or executing rights transfers.
- Token, blockchain, or reputation requirements for ordinary creator workflows.
- AI-generated music/video intended for mass publishing.
- Predictive claims about virality or guaranteed earnings.
- Treating an internal Phi/consciousness metric as evidence of creator welfare, artistic merit, or system safety.

## Benchmark programme

See the companion [CSCP Benchmark Protocol](CSCP_BENCHMARK_PROTOCOL.md) for 15 adversarial fixtures, anti-Goodhart invariants, evidence-abstention gates, and staged evaluation rules. See [Creator Resilience and Creative Optionality](CSCP_CREATOR_RESILIENCE_MODEL.md) for the next paradigm shift: model volatility, concentration, runway scenarios, creator-defined constraints, and room for self-directed work—not just revenue growth. The [Deal-to-Cash and Repeat-Booking Model](CSCP_DEAL_TO_CASH_MODEL.md) explores a narrower operational wedge: evidence-backed agreement, delivery, acceptance, invoice, payment, and repeat-booking tracking. The [Creative Asset Passport and Rights Graph](CSCP_CREATIVE_ASSET_PASSPORT.md) extends this to modern music careers by distinguishing compositions, recordings, versions, approved splits, usage grants, and royalty statements.

All targets below are **proposed gates for testing**, not achieved results, guaranteed outcomes, or external industry standards. Freeze metric definitions before confirmatory evaluation and publish negative results.

| Area | Proposed target | Measurement contract |
|---|---|---|
| First value | First useful weekly report in ≤5 minutes after a supported import; basic workspace setup in ≤10 minutes for the test cohort. | Time from starting the workflow to a creator-confirmed useful report; report missing fields rather than quietly inventing data. |
| Administrative time | ≥30% median reduction by the end of the pilot. | Same task taxonomy and time diary during a 4-week baseline and 8–12-week pilot; report median and p25/p75, attrition, and workload shifted to setup. |
| Career economics | Exploratory target: +10% median net creator income per recorded work-hour relative to the creator's baseline. | Report separately from revenue growth; adjust/stratify obvious confounders and do not claim causation from the initial small cohort. Replicate before marketing this as an outcome. |
| Decision quality | 100% of material factual claims are source-linked or explicitly labeled unverified; zero high-severity unsupported claims in the preregistered evaluation set. | Predefined rubric, independent blinded reviewer, source-to-claim audit, adversarial fixtures, and error severity. Acceptance rate is secondary, never the pass criterion by itself. |
| Experiment quality | 100% of claimed experiments have a stated baseline, hypothesis, outcome window, and confounders; no causal wording unless design supports it. | Automated schema validation plus adversarial cases (seasonality, promotion spikes, missing data, survivorship bias, delayed payments). |
| Portability | 100% of supported user-owned records survive export → fresh workspace import → semantic comparison with no unexplained loss. | Golden fixtures for schema versions, Unicode, time zones, currency precision, permissions, revoked records, and partial imports; manifest hashes and independently generated comparison. |
| Consent and authority | Zero unauthorized publications, messages, contract acceptances, rights transfers, or payments. | Fail-closed action gate tests, revocation tests, least-privilege connector tokens, and audit receipts. Any violation blocks release. |
| Data safety | Zero sensitive fields leaked into public DHT/logs/test fixtures; deletion and retention behavior documented for every connector. | Synthetic/fuzz fixtures, structured redaction tests, retention-expiry tests, and a manual threat-model review. |
| Financial correctness | No silent duplicate or double-counted event in golden/hostile accounting fixtures. | Exact currency minor units where available, explicit rounding, idempotency keys, refund/reversal lineage, and independent reconciliation. |
| Trust and creator agency | ≥4/5 median creator-rated clarity/usefulness, with no material cohort subgroup below 3.5/5 during pilot. | Anonymous survey plus interviews; report sample size and distributions. Creator rejection of advice is a valid outcome. |
| Reliability | Local ledger and export functions work without platform API availability; connector failures are visible and do not corrupt existing data. | Fault injection, interrupted import/export tests, stale-data labels, and a reproducible test manifest. |

### Evaluation integrity

- Separate engineering benchmarks, creator-reported outcomes, and causal impact claims.
- Use synthetic fixtures and data volunteered with explicit research consent. Do not ingest private creator analytics or fan data for benchmarks by default.
- Preregister the primary outcome, cohort inclusion rules, exclusions, and stopping criteria before the pilot's confirmatory window.
- Publish null/negative results, sample size, attrition, uncertainty, source limitations, and exact software/schema revisions.
- Do not infer general creator-market effects from 10–20 participants; use this cohort to establish workflow feasibility and surface failure modes.

## Threat model and guardrails

1. **Account/source compromise:** least-privilege tokens, encrypted secret storage, revocation, and no raw credentials in logs.
2. **Misleading analytics:** provenance and freshness on every external metric, explicit missingness, formula versioning, and no fabricated cross-platform comparability.
3. **AI hallucination or overreach:** source-linked assertions, uncertainty, constrained action tools, human approval, red-team tests, and correction history.
4. **Rights/likeness abuse:** affirmative scoped consent; separate permission to analyze from permission to publish or train; revocable future-use grants; no default voice/face cloning.
5. **Fan privacy and stalking:** opt-in contact collection, purpose limitation, retention expiry, pseudonymous IDs, aggregation thresholds, and no public geolocation of individual fans.
6. **Sponsorship misrepresentation:** disclosure checklist linked to deliverable/channel, record of human review, and clearly stated legal limitations.
7. **Revenue or popularity gaming:** no pay-to-rank, fake-engagement automation, hidden affiliate conflicts, or single opaque creator-quality score.
8. **Governance capture:** ordinary creator features cannot depend on MYCEL balances, founder privilege, or reputation thresholds. Compensation/restoration proposals remain separate and must preserve Mycelix's existing safeguards.
9. **Platform dependency:** documented connector contract, export fallback, conservative sync, and explicit handling when API permission is denied or terms change.

## Delivery sequence: work can start without CI

1. **P0 — contract and benchmark spec:** settle metric definitions, schema, data boundaries, and test fixtures in this RFC. No runner-dependent result is needed to do this design work.
2. **P1 — Rust core:** implement a deterministic local career ledger and versioned import/export schema. Start with CSV/JSON; add no network permissions in this stage.
3. **P2 — independent verifier:** independently recompute derived metrics and verify manifests, event idempotency, financial arithmetic, and round-trip export. The verifier must not reuse the implementation code path as its sole oracle.
4. **P3 — evidence-linked weekly debrief:** add advisory summaries over locally supplied records, keeping source facts, derived metrics, and recommendations as separate types.
5. **P4 — creator pilot:** recruit consenting design partners and run baseline/pilot measurement; do not claim income uplift before evidence.
6. **P5 — selective Mycelix integration:** only after the local data model proves useful, decide which claims, permissions, agreements, and credits benefit from Holochain-backed sharing.
7. **P6 — Symtropy / Sol-Atlas extensions:** choose interactive performance or event mapping based on pilot demand, not architectural novelty.

Each implementation PR should include a scope statement, exact tested commit, local reproduction steps, data migration behavior, and independent-verifier evidence. A queued CI run is neither a pass nor a substitute for the documented local/independent checks.

## Relationship to existing work

This is a product architecture and evaluation contract, not a replacement for Mycelix's financial/governance design. In particular, [Mycelix issue #4443](https://github.com/Luminous-Dynamics/mycelix/issues/4443) already addresses dignity, creator/stewardship claims, contractual compensation, and the distinction between MYCEL reputation and compensation. CSCP should link to those semantics only where needed; it must not merge career analytics, fan permissions, or ordinary revenue records into the special stewardship-claim lifecycle.

The current Mycelix README identifies the project as pre-alpha and warns that passing unit tests is not evidence that every cluster has production-grade integration or UI. Product claims and rollout gates must match the maturity of each subsystem. See [Mycelix](https://github.com/Luminous-Dynamics/mycelix).

## Open questions for design partners

- Which is most valuable weekly: understanding finances/time, turning sessions into approved clips, sponsorship administration, or building direct fan relationships?
- Which source exports can creators reliably obtain and legally share, and which fields do they consider too sensitive to import?
- What information is required to negotiate a sponsorship without building an unlicensed talent agency or pretending to provide legal advice?
- Which benchmark outcomes would make a creator willing to pay after a month, even if follower count stayed flat?
- What data, AI behaviors, and collaboration permissions must be local-only by default?

## References

- IAB. [2025 Creator Economy Ad Spend & Strategy Report](https://www.iab.com/insights/2025-creator-economy-ad-spend-strategy-report/), published 2025-11-20.
- Patreon. [State of Create](https://stateofcreate.co/), creator/fan survey and market framing; interpret as vendor-published research.
- TikTok for Developers. [Data Portability API application guidelines](https://developers.tiktok.com/docs/en/data-portability-api-application-guidelines/), updated 2026-09-22.
- Google for Developers. [YouTube API quota and compliance audits](https://developers.google.com/youtube/v3/guides/quota_and_compliance_audits).
- US Federal Trade Commission. [Disclosures 101 for Social Media Influencers](https://www.ftc.gov/node/60343).
