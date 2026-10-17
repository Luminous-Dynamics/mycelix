# CSCP Benchmark Protocol: Creator Career Intelligence Without Goodharting the Creator

**Status:** Research protocol; proposed gates only, no empirical results reported.
**Version:** 0.1
**Date:** 2026-10-17
**Parent:** [Creator Sovereignty Control Plane RFC](https://github.com/Luminous-Dynamics/mycelix/pull/5166)
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

## Research references

- TikTok for Developers, [Data Portability API: Get Started](https://developers.tiktok.com/docs/en/data-portability-api-get-started). Documents access prerequisites, separate Login Kit approval, user consent, and the export lifecycle.
- TikTok for Developers, [Data Portability API FAQ](https://developers.tiktok.com/docs/en/data-portability-api-faq). Updated 2026-09-22; documents review and access limits.
- TikTok for Developers, [Data Portability Application Guidelines](https://developers.tiktok.com/docs/en/data-portability-api-application-guidelines). Documents purpose limitation, data deletion, data protection, and scope minimization.
- IAB, [2025 Creator Economy Ad Spend & Strategy Report](https://www.iab.com/insights/2025-creator-economy-ad-spend-strategy-report/). Useful market context, not evidence that this product has demand or improves outcomes.
- Patreon, [State of Create](https://stateofcreate.co/). Vendor-published survey research; interpret as interested-source evidence.
- U.S. Federal Trade Commission, [Disclosures 101 for Social Media Influencers](https://www.ftc.gov/node/60343). Operational reference for disclosures; not a substitute for legal advice.
