# CSCP Benchmark Protocol: Creator Career Intelligence Without Goodharting the Creator

**Status:** Research protocol; proposed gates only, no empirical results reported.
**Version:** 0.1
**Date:** 2026-10-17
**Parent:** [Creator Sovereignty Control Plane RFC](https://github.com/Luminous-Dynamics/mycelix/pull/5166)
**Companion:** [Creator Resilience and Creative Optionality](CSCP_CREATOR_RESILIENCE_MODEL.md)
**Operational workflow:** [Deal-to-Cash and Repeat-Booking Model](CSCP_DEAL_TO_CASH_MODEL.md)
**Music/rights extension:** [Creative Asset Passport and Rights Graph](CSCP_CREATIVE_ASSET_PASSPORT.md)
**Scope:** Measurement semantics, adversarial benchmark design, and staged evaluation before platform integrations or creator-facing causal recommendations.

## Core thesis

A creator operating system should not optimize a single opaque score. A single target such as views, follower growth, engagement, or even income per hour is easy to game and can conflict with the creator's chosen artistic direction, time constraints, privacy, rights, and resilience.

The proposed paradigm shift is **creator-defined constraints + a multi-objective career model + source-linked evidence + reversible experiments**.

The product should offer a small set of legible trade-offs instead of claiming to know the creator's one true objective. It must never treat artistic worth, human worth, health, or consciousness as a measurable creator score.

## Why export-first is a product requirement, not just a shortcut

TikTok's current Data Portability API documentation says the data-portability feature currently covers TikTok users in the EEA and UK. Access requires application approval, user consent through Login Kit, and approval of both products; TikTok notes that the review may take 3–4 weeks. The implication is concrete: an otherwise useful creator product cannot make portability, setup, or evaluation contingent on a single provider's API approval.

Sources:
- [TikTok Data Portability API: Get Started](https://developers.tiktok.com/docs/en/data-portability-api-get-started)
- [TikTok Data Portability API FAQ](https://developers.tiktok.com/docs/en/data-portability-api-faq)
- [TikTok Data Portability Application Guidelines](https://developers.tiktok.com/docs/en/data-portability-api-application-guidelines)

Therefore the first complete workflow should use creator-controlled exports and manual records, with explicit source labels, timestamps, and missing-data states. Platform connectors are optional adapters, never the canonical data store.

## Paradigm shifts to evaluate

### 1. Replace the global score with a bounded multi-objective model

Keep separate, interpretable dimensions:
- net contribution by currency and creator-recorded work time;
- income stability and revenue-source concentration;
- creator-selected audience relationship indicators;
- creative goals declared by the creator, not inferred as universal truth;
- workload and schedule boundaries that the creator elects to record;
- portability, consent, and rights state.

Hard constraints come first. An action that violates a creator's explicit rights, budget, privacy, schedule, or consent constraint must be rejected even if an engagement model predicts more reach. Among actions satisfying constraints, show alternatives and trade-offs. Let creators select priorities and change them. No hidden weight changes and no default optimization for screen time or posting frequency.

### 2. Treat evidence lineage as the primitive, not the dashboard

Each surfaced claim must be one of:
- **Observed:** imported or directly recorded data with source and window.
- **Derived:** a versioned formula over identified observations.
- **Hypothesis:** a proposed explanation that is not established by the available data.
- **Recommendation:** a reversible next action with the reason and expected trade-off.
- **Unknown:** insufficient, stale, incomparable, or conflicting evidence.

Never blur these types. A visualization is not a provenance system. Every derived metric needs its source records, formula version, currency/units, observation window, data freshness, and missingness disclosure.

### 3. Change the unit of success from a post to a decision

A content recommendation is not successful because it was accepted, generated, or followed by a view spike. Evaluate whether it improved a pre-declared creator-selected outcome over an appropriate window, at acceptable cost and without violating constraints.

Use low-risk experiments with one principal change at a time where feasible. Document confounders including promotions, collabs, releases, seasonal effects, distribution changes, and unusually viral events. Use randomized or matched comparisons only when practical, ethical, and approved by the creator. Otherwise report association rather than causation.

### 4. Optimize the creator's portfolio, not algorithmic dependency

Measure how much of the creator's income and audience access relies on each platform or buyer. Use concentration as a risk signal, not an instruction to abandon the creator's best channel. A good suggestion may be to retain a high-performing platform while building a low-friction backup channel.

Do not assume portability means followers or private messages can be transferred. Count a relationship as portable only when the creator can legitimately export or re-establish it with consent.

### 5. Prefer reversible assistance to autonomous growth hacking

AI may summarize, index, draft, calculate, explain, and propose. Publishing, fan messaging, contract acceptance, payments, rights transfers, training on private material, or use of a voice/face/likeness require explicit, scoped authorization and a recorded approval. Rejection of an AI recommendation must not reduce a reputation score or system capability.

### 6. Build for uncertainty and abstention

The system must be comfortable saying that data are missing, incomparable, or insufficient to infer the answer. A calibrated abstention is better than a confident fabricated result. When platform exports are stale or partial, present the last observed window and ask for an update rather than presenting it as current.

## Proposed benchmark corpus

Build synthetic, deterministic fixture cases with known ground truth. Every case should have an input manifest, expected invariants, adversarial mutation(s), and an exact software revision. No real creator analytics, private messages, fan contact data, or unpublished media belong in the benchmark by default.

| ID | Adversarial scenario | Required behavior |
|---|---|---|
| B01 | A viral post has very high views but low receipts and unusually high production hours. | Do not rank it as the best career outcome by views alone. Show net contribution, work hours, missing costs, and trade-off. |
| B02 | Revenue rises after a creator changes both posting schedule and paid promotion. | Do not attribute uplift to the schedule change; label the cause unresolved or recommend a better-isolated experiment. |
| B03 | A platform export is stale or only a partial period is present. | Show freshness, missing window, and incomparable period; do not silently fill gaps. |
| B04 | USD and EUR events are mixed in one import. | Keep totals separate unless an explicit, sourced FX policy and dated rate are selected; default is no conversion. |
| B05 | The same payment row is imported twice; a later row reuses its ID with changed amount. | Identical replay is idempotent; conflicting replay fails closed and does not alter totals. |
| B06 | Work sessions overlap or have invalid timestamps. | Reject overlap in a single-creator ledger; reject invalid intervals; never inflate recorded work hours. |
| B07 | A refund/chargeback arrives after the original receipt or the import includes a duplicate reversal. | Preserve source and reversal lineage when supported; idempotently handle identical replay; surface unmatched reversals instead of silently assigning them. |
| B08 | One channel reports views, another reports watch time, and a third lacks a comparable window. | Do not create an unlabelled unified engagement score. Keep metric definitions and windows explicit. |
| B09 | One high-performing post dominates a small sample. | Surface sample size and uncertainty; do not assert a stable creator pattern from one observation. |
| B10 | Aggregation across audience segments reverses the per-segment trend (Simpson's paradox). | Offer stratified results and warn that the aggregate may mislead; do not present aggregate association as causal. |
| B11 | Creator has explicitly prohibited synthetic voice/face usage, DMs, or posting during a declared rest window. | Block incompatible recommendations and actions regardless of predicted reach. |
| B12 | Sponsorship brief requires disclosure and a specific usage license, but terms or approval are missing. | Identify missing requirements and block automated publication/acceptance; do not claim the checklist certifies legal compliance. |
| B13 | A claim cites an aggregate estimate but no data source or formula revision. | Reject it as an unsupported derived fact or label it unverified. |
| B14 | A creator has negative net contribution for a period. | Preserve the negative value and show contributing categories; never saturate to zero or hide the loss. |
| B15 | A creator revokes an integration or deletes data imported from it. | Stop future collection, apply documented deletion rules to local source data, retain only legally/technically required minimal audit metadata, and honestly disclose limits on third-party/immutable copies. |

Each fixture should have at least one mutation that a naive dashboard or recommendation system gets wrong. Negative controls should be retained; never weaken or remove a failing fixture merely to make a release pass.

## Benchmark gates and metric definitions

The targets below are proposals. They are not external standards or achieved results.

### Current deterministic scenario-engine contract

A first implementation slice now exists in the draft Rust prototype [PR #5172](https://github.com/Luminous-Dynamics/mycelix/pull/5172). It performs only user-specified source-decline arithmetic. The current contract is deliberately limited:

- The declared observation window must be valid; source shocks must reference known, unique source IDs and use a decline between 0 and 10,000 basis points.
- Every used currency must have exactly one explicit deductions row. Currencies are never combined.
- Refunds, fees, and operating costs remain frozen in this version; this is displayed as an assumption, not inferred behavior.
- Fractional minor-unit losses are rounded up conservatively. This may overstate the stress impact at tiny amounts; it must never be described as a statistical estimate.
- The report carries creator-declared completeness or partial/unknown coverage. Partial observations must not be presented as a complete career ledger.
- Scenarios never mutate observed ledger rows, imply a probability, or execute financial/creator actions.

The draft's source has 12 scenario unit tests, but the Rust toolchain is not available in the current editing environment and those tests have not been executed here. These tests are design evidence only until compiled and run on a Rust-enabled environment.

### Gate 0 — arithmetic and data integrity

- 100% exact agreement with golden integer-minor-unit totals for supported classifications.
- Zero cross-currency summation without an explicit conversion policy.
- Zero unexplained record loss on supported export/import round trips.
- Identical replays are idempotent; conflicting identity reuse is rejected.
- Zero double-counted work time from overlapping intervals in the single-creator model.
- Missing work hours produce an unknown rate, not a fabricated zero or a guessed denominator.
- 100% of reported derived totals are independently recomputable from source records.

### Gate 1 — claim quality and uncertainty

- Every material factual claim is source-linked or explicitly marked unverified.
- Zero high-severity unsupported claims in the preregistered adversarial set.
- Zero causal claims when the available design supports only correlation.
- 100% of stale, partial, currency-incompatible, or low-sample cases expose that limitation.
- 100% of creator-declared hard constraints block incompatible suggestions/actions.

Review these with a blinded human rubric that records false claims, missing caveats, severity, and false abstentions. Report counts and confidence bounds; do not use a single blended “trust score”.

### Gate 2 — workload and usability

- Useful first ledger/report within 5 minutes of a supported import for the test cohort.
- Basic setup within 10 minutes where no provider approval is required.
- At least 30% median reduction in the defined administrative task time after an 8–12-week pilot, compared with a 4-week baseline.
- At least 4/5 median creator-rated clarity/usefulness, while reporting the full response distribution and subgroup harms.

Measure time spent in setup/import as part of total workload so the system cannot game the metric by shifting administrative effort earlier.

### Gate 3 — economic value and decision quality

Primary descriptive measure:

`net contribution per recorded work-hour = (receipts - refunds - fees - attributable operating costs) / creator-recorded work-hours`

Keep results separated by currency, accounting policy, window, and data completeness. Do not use the rate when work time is absent. Cash-basis MVP figures are not tax or accrual accounting.

- Exploratory goal: +10% median net contribution per recorded work-hour against each participant's baseline.
- Treat this as a research hypothesis, not a promised result or a release gate for a small cohort.
- Report revenue concentration, income variability, and direct relationship portability separately; do not conceal trade-offs in the primary metric.
- Assess decision value by preregistered outcome vs. current-tool baseline, not recommendation acceptance or click-through rate.

### Gate 4 — causal claims

Before claiming the product caused increased income or reduced workload, run a preregistered study with:
- a defined population, outcome, time window, and inclusion/exclusion policy;
- a sample-size/power or uncertainty analysis appropriate to noisy creator outcomes;
- randomization or a credible comparison design when feasible;
- known confounders, missingness, attrition, and protocol deviations;
- independently reproducible analysis and publication of null/negative outcomes.

A 10–20 creator design-partner cohort is appropriate for usability, workflow feasibility, and failure discovery. It is not enough by itself to establish a generalizable causal improvement in earnings.

## Anti-Goodhart release tests

Before any recommendation-capable release, test the following invariants:

1. Replacing views with a large synthetic spike while holding receipts and work hours constant must not automatically improve the declared economic outcome.
2. Increasing revenue while increasing attributed costs and work time may reduce net contribution per hour; the model must not hide this.
3. Violating a creator's hard constraint must invalidate an action even if the predicted utility is high.
4. Missing source data must widen uncertainty or trigger abstention; it must not increase apparent performance.
5. Creator preference weights must be visible, changeable, and versioned; the product must disclose when two goals conflict.
6. No single ranking combines money, audience, creative quality, personal health, and reputation into an opaque score.
7. Recommendation acceptance rate, post frequency, screen time, and retention are not primary success metrics.

## A domain-specific wedge: the release-to-tour loop

A creator-facing product becomes more distinctive when it models a complete creative project, not an account or a post. For an independent musician, the project spine could run:

1. **Create:** song or performance concept, session/work logs, source assets, contributor credits, explicit rights claims, and versioned approvals.
2. **Prepare:** artist-selected goals, release costs, release window, approved campaign variants, and clear usage/license boundaries.
3. **Publish and learn:** source-linked observations from creator-authorized exports; comparisons use declared windows, surface missingness, and avoid claiming one change caused a result when several things changed.
4. **Convert voluntarily:** connect a release to creator-chosen fan memberships, direct opt-in mailing, pre-orders, tickets, or merchandise via existing tools. Do not assume that follows or platform DMs are exportable or that accounts across services identify the same person.
5. **Reach the physical world:** only after opt-in or appropriately aggregated demand supports it, estimate venue/route alternatives, cost, expected break-even, and uncertainty. Do not expose individual fans' location or manufacture demand from noisy engagement signals.
6. **Reconcile:** compare cash receipts, refunds, costs, work time, collaboration splits, event outcomes, and rights/disclosure completeness. Feed the evidence into the creator's next decision.
7. **Preserve:** export the project record so the work, credits, financial evidence, and future license boundaries remain usable when one platform disappears or changes policy.

This is a **project graph**, not an attempt to replace every tool in the chain. Each existing service remains replaceable; the control plane maintains the relationships among the work, authorized observations, decisions, receipts, approvals, and outcomes.

### How the four projects can matter in this loop

- **Symthaea:** summarize approved materials and source-linked results, propose bounded experiments, and disclose uncertainty; never decide whether art is universally good or publish without approval.
- **Mycelix:** later support scoped permissions, collaborator claims, agreement versions, and portable evidence; avoid exposing fan-level data in public DHT storage.
- **Symtropy:** later supply responsive stage/visual experiences or audience participation that a musician can reuse across performances.
- **Sol-Atlas:** later compare venue, routing, travel cost, and aggregated geographic demand where the data and consent justify it.

None of these integrations is a prerequisite for the first ledger. They earn their place only by improving a measurable creator workflow.

### Release-loop metrics (kept separate, never collapsed to one score)

- **Project accounting completeness:** percentage of declared supported receipts, refunds, fees, costs, and work-session records reconciled to source evidence. Release goal for supported records: 100%, with unknown/unsupported categories explicitly listed.
- **Rights/credit completeness:** percentage of published project assets with creator-confirmed rights-holder/credit metadata and the required approved usage scope. Report not-applicable and unknown separately; do not infer ownership from file possession.
- **Fan-consent integrity:** zero outbound fan contact or data reuse outside the purpose/permission recorded by the creator; export does not imply permission to contact.
- **Release efficiency:** net contribution per recorded work-hour, by currency and declared accounting policy, with missing time treated as unknown.
- **Voluntary conversion:** supporters, pre-orders, or ticket sales attributed only to a defined and defensible measurement method. Do not claim cross-platform attribution when the creator cannot establish it.
- **Live-event planning quality:** compare pre-event cost/break-even forecasts with observed outcomes and report forecast error and uncertainty; do not turn estimated local interest into a guarantee of attendance.
- **Next-decision usefulness:** blinded review of whether the post-release debrief identified evidence-supported options and trade-offs, not whether the creator obeyed the recommendation.

A pilot should follow multiple releases or events over time where feasible. A single viral release is not evidence of a repeatable business model, and an event that makes money may still be a poor decision if it consumed excessive unrecorded labor or violated creator constraints.

## Evaluation workflow independent of hosted CI

The synthetic corpus can be reviewed and exercised locally without platform tokens, external services, or GitHub runners:

1. Freeze the metric definitions and the expected outputs before evaluating a candidate implementation.
2. Store fixtures and manifests in version control; hash inputs and result artifacts in the evaluation harness once a selected hash implementation is available.
3. Have an independent checker recompute totals from the source records rather than importing the implementation's aggregation function.
4. Run hostile mutations against the fixtures and confirm the verifier catches altered totals, IDs, windows, currency, and permission scopes.
5. Record exact toolchain, OS, source revision, commands, results, and known limitations. A queued check is not a pass.
6. Keep runtime performance targets provisional until a reference hardware profile and workload are frozen. Do not compare timings from unspecified machines.

## Product sequencing implied by these benchmarks

1. Deterministic local ledger and independent arithmetic checker.
2. Versioned import/export with lossless round-trip checks, source provenance, and deletion behavior.
3. Evidence-linked static reports and adversarial test corpus.
4. Human-reviewed, preregistered experiment suggestions with uncertainty and abstention.
5. Opt-in pilot with creator-defined outcomes.
6. Selective Mycelix permissions/claims integration where it adds a documented benefit.
7. Symthaea deeper career reasoning, Symtropy interactive experiences, and Sol-Atlas venue/tour intelligence only after creator research indicates demand.

## Gate 5 — asset identity, split claims, and consent scope

The initial implementation slice is tracked in [draft PR #5182](https://github.com/Luminous-Dynamics/mycelix/pull/5182). These are release thresholds, not achieved results.

| Fixture family | Required oracle | Release threshold |
|---|---|---:|
| ISWC vs ISRC | Work identifiers cannot be assigned to recordings; recording identifiers cannot be assigned to works; malformed presentation syntax rejected | 100% correct fixed-fixture outcomes |
| Identifier collisions | Same canonical external identifier on different local assets raises a conflict; no automatic merge | Zero silent merges |
| Rights-pool arithmetic | Evaluate each asset and pool separately; a complete claim is exactly 10,000 basis points | Zero normalization; zero cross-pool aggregation |
| Partial / unknown shares | Report known basis points and unattributed residual; do not assign the residual to a participant | Zero invented shares |
| Claim state | Proposed and disputed claims remain distinguishable from confirmed arithmetic | Zero disputed claims reported as complete |
| Permission scope | Asset, grantee, action, purpose and time interval all match the grant | Zero false allows in adversarial fixtures |
| Sensitive use | Metadata analysis, publication, distribution, commercial use, model training, derivatives, and voice/likeness synthesis are not interchangeable | Zero cross-action permission expansion |
| Expiry / revocation | Start is inclusive, expiry is exclusive, and revocation blocks evaluation at/after its timestamp while retaining historical evaluation before it | 100% fixed-fixture agreement |
| Revocation alongside original grant | At/after revocation timestamp, zero allows regardless of history order; historical requests before revocation remain allowed | Zero resurrection of revoked consent |
| Conflicting grant ID reuse | Same ID with different immutable terms or inconsistent revocation times blocks any request whose exact scope is implicated; unrelated valid permissions remain evaluable | 100% conflict detection; no first-match wins or catalogue-wide denial of service |
| Asset boundaries | A grant for one recording does not authorize a second recording or derivative asset | Zero implicit grant inheritance |

The golden fixture corpus must include at least these negative controls: an ISWC attached to a recording; an ISRC attached to a composition; a cross-asset duplicate identifier; a 6,000-basis-point partial split; a complete split with an unknown share; a 10,500-basis-point over-allocation; duplicate participants in one pool; an analysis grant used for model training; an explicit training grant reused for a different model-purpose token; an expired grant; a revoked grant; and a grant for another asset. Add positive controls for valid syntax, an exact 6,000/4,000 complete split, an exact matching permission, and a request made before revocation.

**Decision rule:** one false allow, silent identifier merge, invented share, or unmarked incomplete split blocks release. Aggregate accuracy cannot compensate for any of those failures. These fixtures only assess internal policy behavior; they do not prove legal ownership, grantor authority, identity, consent authenticity, or compliance with territory-specific law.


Also include a history-ordering fixture containing both the original grant and its revocation. Evaluate the request against both grant-then-revocation and revocation-then-grant orderings. Both must deny at and after the revocation timestamp and allow a historical query strictly before it. Reuse of a grant ID for a different asset, party, action, purpose, evidence binding, validity window, or incompatible revocation timestamp must block authorization for any request whose exact scope is implicated. Unrelated scopes must remain evaluable to avoid a catalogue-wide denial of service; the evaluator must never use "first matching grant wins."

Do not claim this gate is met until an independently reviewed build runs the corpus at the exact candidate commit with a pinned Rust toolchain and recorded exit status. The current PR records source-level tests only; compilation and runtime evidence are still outstanding.

## Gate 6 — portable grant history and replay

The implementation slice in [draft PR #5182](https://github.com/Luminous-Dynamics/mycelix/pull/5182) now includes an in-memory append-only issue/revocation event log. Events carry an event identifier, event time, actor identifier, evidence reference, and typed action. The log requires nondecreasing event times, treats identical event-ID replay as idempotent, rejects conflicting event-ID reuse and duplicate grant issuance, and validates revocation against the currently effective grant. It reconstructs effective state by replaying the event stream.

This is still not a portable or signed history. Actor IDs and evidence references are caller-provided; event authenticity, grantor authority, evidence truth, durable persistence, deletion/export semantics and independent cryptographic verification remain outside the current implementation.

| Fixture family | Required result | Threshold |
|---|---|---:|
| Exact event replay | Re-appending the exact same event ID/content returns idempotent outcome and does not grow history | 100% |
| Conflicting event ID | Reusing event ID with different content leaves history unchanged | 100% rejected |
| Duplicate grant issuance | Second event attempting to issue an existing grant ID leaves state unchanged | 100% rejected |
| Unknown or already-revoked grant | Revocation is rejected atomically | 100% rejected |
| Event ordering | Decreasing event time is rejected; equal-time distinct events are deterministic | 100% |
| Failed transition atomicity | Both history length and effective state remain unchanged after rejection | Zero partial mutation |
| Replay reconstruction | Freshly replayed effective state exactly equals append-derived state | 100% fixed fixture agreement |
| Attribution truth | Caller-provided actor/evidence IDs are not described as authenticated | Zero unsupported authenticity claims |

Keep the claim ceiling explicit: replay demonstrates deterministic internal reconstruction, not the truth of an event or legal validity of consent. A content hash can demonstrate byte equality only; it does not authenticate the issuer or prove rights. A future portable envelope should be versioned and canonicalized, carry the exact source schema/version, keep raw external messages or their cryptographic digests, and make unsupported fields visible rather than inventing defaults.

## Gate 7 — canonical portable grant-history envelope

The implementation slice in draft PR #5182 now exports the ordered grant/revocation event history through an experimental versioned binary envelope with deterministic canonical bytes and a domain-separated BLAKE3-256 content digest. Two frozen byte vectors are checked against a standard-library-only Python reference encoder used only for testing, as well as the Rust encoder: an issue/revocation history and a UTF-8/absent-expiry case. This remains a tiny corpus; there is not yet a published cross-language BLAKE3 digest vector. The envelope schema and stable numeric tags are described in the public research crate README. It covers grants and revocations only, not the complete asset registry, contribution/split claims or source payloads.

| Fixture family | Required outcome | Threshold |
|---|---|---:|
| Deterministic encoding | Repeated encoding of the same event history yields byte-identical output | 100% |
| Decode/encode round trip | Re-encoding a decoded envelope yields byte-identical canonical bytes and preserves reconstructed effective grant state | 100% |
| Digest | Same canonical bytes yield the same domain-separated digest; modified content changes fixture digest | 100% |
| Unknown format version/action tag | Reject; never reinterpret an unknown tag as a permissive action | 100% rejected |
| Truncation, invalid UTF-8, oversized fields/counts or trailing bytes | Reject import without partial state creation | 100% rejected |
| History replay after import | Imported event stream reconstructs the same effective grants as append-derived state and independent reconstruction | 100% |
| Cross-language interoperability | A second implementation reproduces multiple frozen canonical-byte and digest golden vectors, including negative/boundary fixtures | 100% golden-vector agreement before format freeze; currently two byte vectors exist and the digest vector remains outstanding |
| Authorship/consent claim | No digest is presented as signature, trusted timestamp, consent proof or title evidence | Zero unsupported authenticity claims |

The digest is carried separately from the envelope. BLAKE3 detects byte changes relative to an expected digest, but an attacker can replace both bytes and digest unless a separately authenticated channel/signature anchors the expected value. Keep source-system identifiers, raw message hashes, parser versions, and unresolved fields visible in a future full passport envelope. Do not silently drop unsupported fields during import/export.

The current implementation is an experiment, not a ratified interoperability standard. Do not freeze the wire format or claim cross-language interoperability until a separately implemented encoder/decoder passes published golden vectors, property tests, mutation tests, and size-limit cases.

## Gate 8 — whole-passport export integrity

The implementation slice in draft PR #5182 now includes a separate CreativeAssetPassportEnvelope that combines the ordered asset registry (including typed external identifiers), split claims, and the grant/revocation history envelope into one versioned canonical export. The inner grant-history format remains separate and unchanged. The outer envelope has two frozen byte vectors checked by Rust and a test-only standard-library Python reference encoder: a work/ISWC/complete-split fixture and a Unicode recording with ISRC/other identifiers and a disputed unknown-share claim. This remains a small conformance corpus.

| Fixture family | Required outcome | Threshold |
|---|---|---:|
| Full snapshot round-trip | Asset kinds/titles/IDs, identifiers, split claims, grant history, and revocation state survive import/export exactly | 100% fixture agreement |
| Identifier insertion order | Different valid insertion order results in identical canonical bytes | 100% |
| Claim insertion order | Reordering claim input does not change canonical bytes | 100% |
| Cross-record reference integrity | Every split claim and all grants, including revoked grants, point to an asset in the exported registry | Zero unresolved references accepted |
| Digest sensitivity | Changing an asset field, a split claim/evidence reference, or a grant-event evidence reference changes the outer snapshot digest | 100% fixed fixtures |
| Unknown tags/version, non-canonical order, truncation or trailing bytes | Reject import without silently normalizing untrusted source bytes | 100% rejected |
| Snapshot reference vectors | Independent encoder reproduces frozen outer bytes for identifiers, splits, and embedded empty history across normal and Unicode/disputed fixtures | 100% of frozen vectors; currently two |
| Diverse outer golden corpus | Include multiple asset kinds, multiple identifiers, disputed/proposed claims, nonempty grant history, Unicode, missing optional fields, unknown identifiers, and size boundaries | No format freeze until all cases pass |
| Cross-language BLAKE3 digest | Independent implementation yields the same domain-separated digest for frozen full-snapshot bytes | 100% before format freeze |

Do not describe the digest as proof of rights/consent or the snapshot as a full evidence archive. The snapshot currently omits original external source payloads and provenance hashes for those imports; it carries the record/evidence references, not proof that their contents are genuine. The format is experimental, not standards-compliant or ratified for interoperability. Preserve claims and disputes rather than turning split arithmetic into an automated ownership judgment.

## Gate 9 — field-level import provenance and failure atomicity

Draft PR #5182 now adds a standalone import-provenance sidecar. It computes BLAKE3 from exact raw payload bytes; records source/schema/parser versions and import time; associates each parser-enumerated field with a mapping rule or explicit retained-raw/unsupported treatment; records diagnostics; and binds the outcome to before/after passport digests. The sidecar is not embedded in the full passport snapshot v1 and does not prove that the payload reference stays available or that the source is authentic.

| Fixture family | Required outcome | Threshold |
|---|---|---:|
| Raw payload verification | Exact observed bytes match length and digest; changed bytes or length are rejected | 100% |
| Source-field disposition | Each parser-enumerated source path has exactly one treatment; duplicate paths fail closed | 100% |
| Unsupported/retained field | Every field not mapped into canonical state has an explicit field-level diagnostic and remains in the raw source | Zero silent drops |
| Clean import label | A clean Imported result requires parser-reported complete inventory, all declared fields mapped, and zero diagnostics | 100% rejected if preconditions fail |
| Rejected import | An error diagnostic exists and after snapshot digest equals before snapshot digest | Zero mutations |
| Partial/unknown inventory | Outcome cannot be labelled clean Imported | 100% warning or rejection |
| Mapping version | Parser/mapping version changes are represented in provenance and later produce a reviewable before/after diff | 100% manifest completeness |
| Canonical sidecar | Decode/encode preserves exact manifest bytes; malformed version, invalid fields, unknown diagnostic paths, duplicate field paths and trailing bytes reject | 100% fixture agreement / invalid cases rejected |
| Independent reference encoder | Python reference encoder reproduces the frozen manifest bytes independently of Rust code | 100% of vectors; currently one minimal clean-import vector |
| Cross-language digest | Independent implementation reproduces domain-separated BLAKE3 manifest digest for frozen source fixtures | Required before format freeze; outstanding |
| Persistence enforcement | Backing ingress layer confirms rejected import did not mutate persisted snapshot, not merely that a manifest says so | 100% |
| Raw-source availability | Source locator resolves under declared access controls, and bytes match recorded digest | 100% for records marked retained |

A parser's "complete inventory" is only a parser assertion, not independent proof that every source field was discovered. A digest is integrity evidence only; it does not prove authenticity, consent, or rights. The integration layer—not this manifest constructor—must enforce transaction boundaries and verify the post-import snapshot digest from the actual persisted state.

## Gate 10 — idempotent import replay

Draft PR #5182 adds a separate in-memory ImportReplayLedger. Its key binds source system/record identity, raw payload digest/length, schema/parser versions, and enumerated mapping/treatment versions. It omits import time and raw storage locator. A separate result fingerprint binds the inventory assertion, outcome, before/after snapshot digests and diagnostics. Exact retries are idempotent; the same processing key with a different result fails closed. A schema/parser/mapping upgrade creates a new key for explicit reprocessing.

| Fixture family | Required outcome | Threshold |
|---|---|---:|
| Exact retry | Repeated source payload + schema/parser/mapping identity and same result returns AlreadyPresent without adding an entry | 100% |
| Repeated upload at a different timestamp/storage locator | Same payload bytes and mapping identity remains an idempotent retry | 100% |
| Changed source bytes | New raw-payload digest gives a distinct processing key | 100% |
| Parser/schema/mapping upgrade | Version change produces a new key rather than being silently deduped as the previous interpretation | 100% |
| Conflicting effect under same key | Different outcome, diagnostics, or before/after snapshot digest returns ResultConflict without ledger mutation | Zero silent overwrites |
| Concurrent retries | All concurrent copies converge to one durable effect and one durable receipt | Zero duplicates under 1/10/100/1,000-client stress |
| Crash after commit but before acknowledgement | Retry resolves the committed replay key and does not apply the mutation a second time | Zero duplicate rights/royalty/cash-link records |
| Process restart | Previously recorded replay keys survive restart through durable storage | 100% for persisted receipt corpus |
| Source truth boundary | Idempotency result is not presented as sender authentication, legal truth or proof of entitlement | Zero unsupported trust claims |

The current ledger is in-memory and does not provide the concurrency, restart, or crash guarantees in the stress rows. Those rows are release targets for the future transaction-backed integration, tracked by [CSCP-REPLAY-001](https://github.com/Luminous-Dynamics/mycelix/issues/5190). Do not market this prototype as exactly-once processing.

## Research references

- TikTok for Developers, [Data Portability API: Get Started](https://developers.tiktok.com/docs/en/data-portability-api-get-started). Documents access prerequisites, separate Login Kit approval, user consent, and the export lifecycle.
- TikTok for Developers, [Data Portability API FAQ](https://developers.tiktok.com/docs/en/data-portability-api-faq). Updated 2026-09-22; documents review and access limits.
- TikTok for Developers, [Data Portability Application Guidelines](https://developers.tiktok.com/docs/en/data-portability-api-application-guidelines). Documents purpose limitation, data deletion, data protection, and scope minimization.
- IAB, [2025 Creator Economy Ad Spend & Strategy Report](https://www.iab.com/insights/2025-creator-economy-ad-spend-strategy-report/). Useful market context, not evidence that this product has demand or improves outcomes.
- Patreon, [State of Create](https://stateofcreate.co/). Vendor-published survey research; interpret as interested-source evidence.
- U.S. Federal Trade Commission, [Disclosures 101 for Social Media Influencers](https://www.ftc.gov/node/60343). Operational reference for disclosures; not a substitute for legal advice.
