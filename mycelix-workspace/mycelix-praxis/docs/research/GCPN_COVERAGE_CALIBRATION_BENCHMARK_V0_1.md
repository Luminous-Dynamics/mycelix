# GCPN coverage calibration and common-mode benchmark v0.1

**Status:** frozen public synthetic DGP specification; generator and metrics evaluator are not yet implemented.  
**Tracking:** Mycelix #6008.  
**Composition:** #6001/#6004 observability semantics, #5973 clustered inference, #5979 holdout integrity, #5967 paired non-degradation, #5904/#5923 dependency hypergraph, #5451 evidence receipts.

## Paradigm shift: coverage is not confidence

Current sensor products and evaluations often present detection, track, classification or identification capability together. Luminous should go further by separating five quantities that cannot safely be substituted for one another:

1. **Coverage / observability:** was this exact source and modality applicable and available to observe the declared abstract scope/time under the pinned conditions?
2. **Probability of detection (Pd):** conditional probability of a positive source observation given independent event truth *and* valid source observation exposure.
3. **Probability of false alarm (Pfa):** false alerts or deduplicated event clusters per eligible source-hour or other predeclared exposure unit.
4. **Score calibration:** does a declared probability correspond to the empirical frequency of the claim in this sampling frame?
5. **Evidence confidence:** are provenance, rights, freshness, completeness, independence, clock quality and receipts sufficient to support the claim? This is not probability of physical truth.

The system should be allowed to say: “The source reported this; the source was only qualified for these conditions; the observation interval had this gap; this probability is calibrated only on this partition; common dependency X weakens corroboration.” That is a stronger public-interest claim than an unqualified confidence badge.

## What industry standards cover—and do not cover

- **ASTM F3411-22a** specifies message formats, transmission methods and testing for broadcast/network Remote ID in participating systems. Its own scope says it does not solve identification needs for UAS not participating in Remote ID or operators deliberately circumventing it. Remote ID protocol conformance is not universal aircraft observability. [ASTM F3411-22a](https://store.astm.org/standards/f3411).
- **EASA U-space** sets a service-performance approach that includes data quality, latency, availability, integrity and service-specific requirements tied to the airspace/use case. The consolidated rules include Commission Implementing Regulation (EU) 2023/203 information-security risk-management requirements applicable from 22 February 2026. EASA also announced the enhanced online U-space Easy Access Rules as part of its March 2026 publication set. This is a better comparison point than a single detector accuracy score. [EASA U-space rules and amendment scope](https://www.easa.europa.eu/en/document-library/easy-access-rules/easy-access-rules-u-space-regulation-eu-2021664) · [EASA 21 May 2026 publication update](https://www.easa.europa.eu/en/newsroom-and-events/news/easa-publishes-new-set-enhanced-online-easy-access-rules).
- **European Commission JRC's 2025 C-UAS review** covers acoustic, EO, radar and RF modalities, their limitations and sensor fusion. Its subject includes detection, localization, tracking, classification and identification. [JRC140692](https://publications.jrc.ec.europa.eu/repository/handle/JRC140692).
- A 2026 systems survey emphasizes the divergence between nominal and field performance under clutter, NLOS, occlusion, multipath and environmental degradation, and evaluates SWaP, integration and maintenance alongside detection performance. It is a review, not a standardized benchmark or universal SOTA table. [Survey record](https://www.sciencedirect.com/science/article/pii/S1874548226000429).

These scopes are complements. We must not claim ASTM, EASA, ICAO, IMO or national conformity from this synthetic programme.

## Benchmark definitions

### Probability of detection

Report a numerator and a valid denominator: positive observations among independently confirmed eligible events during valid source observation exposure. An event occurring while the source was unobservable, unavailable or outside profile is not an ordinary miss by that source. Report non-observed exposure separately. If the eligible exposure denominator is zero, Pd is **undefined**, not zero.

Use exact binomial or Wilson intervals for simple independent trials. For clustered trials, preserve top-level site/session/source-dependency blocks and use the qualified paired/clustered method from #5973. Do not bootstrap individual correlated windows.

### False alarm

Report alert rate per eligible source-hour/exposure unit and separately report deduplicated false event clusters. One persistent false event can create dozens of overlapping alerts; those are not dozens of independent errors. Keep both raw operator-review burden and unique event burden.

### Score calibration

Use Brier score and Brier Skill Score against a prevalence baseline, log loss, calibration intercept/slope and reliability plots with support counts and uncertainty. ECE is secondary: its value depends on binning and it can conceal sparse subgroup failures. Always report calibration by source/device revision, class, environment and source group; do not let a dominant class conceal a rare-class failure.

### Coverage calibration

Treat a coverage assertion as a probability or categorical eligibility claim about an abstract scope/condition/time cell. Compare declared coverage with independently stewarded realized exposure. Report coverage calibration separately from Pd and from the model's event score. A vendor advertised range is not a qualified coverage oracle.

### Common-mode dependence

Measure how calibration, Pd/Pfa and abstention change when a dependency group is removed or fails. Include collector, clock, power, network, common model artifact, common data provider and common configuration. Two named feeds sharing one upstream dependency are not two independent confirmations.

### Selective risk

Plot correctness/false-claim risk against the fraction of cases on which the system elects to issue a qualified observation claim. Preserve `UNKNOWN`, `UNOBSERVABLE`, `FEED_UNAVAILABLE`, `STALE`, `WITHHELD` and `CONFLICT`; do not force all examples into present/absent.

## Frozen DGP specification

The machine-readable DGP contains 21 deterministic scenario definitions:

- well-specified calibration positive control;
- rare-class overconfidence hidden by pooled metrics;
- underconfident but rank-preserving scores;
- prior prevalence shift;
- conditional distribution shift;
- missing-at-random exposure;
- informative missingness tied to difficult conditions;
- shared collector common-mode failure;
- synthetic independent-source control;
- shared classifier/model dependency;
- zero eligible observation exposure;
- positive observation in a partial window;
- stale calibration/coverage profile;
- qualified source conflict;
- synthetic Gaussian noise versus a distinct interference holdout;
- imbalanced cluster sizes / Simpson reversal;
- false-alarm bursts and overlapping windows;
- source/device revision shift;
- coverage probability calibration by abstract condition cell;
- source-attestation failure / content-digest tampering, which must be rejected before scoring;
- end-to-end multi-hop latency exceeding a predeclared budget, with event-time/receipt-time and bounded clock uncertainty kept separate.

Corpus: `gcpn-coverage-calibration-dgp-v0.1.json`  
Schema: `gcpn-coverage-calibration-dgp-v0.1.schema.json`

The DGP is a specification, not a run. It records no measured performance. The PCG-XSH-RR-64/32 primitive and SHA-256 domain-separated per-case stream derivation are implemented and covered by the reference output vector and reproducibility tests; the scenario-specific DGP generator and metric runner have not yet been written.

## Acceptance metrics and reporting

- Per-source and fused Pd with intervals and eligible-exposure denominators.
- Pfa per eligible exposure and per deduplicated event cluster.
- Brier/log loss/calibration slope/intercept, prevalence baseline and subgroup reliability.
- Coverage calibration by abstract condition cell and gap/availability interval completeness.
- Risk/coverage curves and reason-specific abstention.
- Dependency-group/common-mode and leave-one-dependency-group-out analysis.
- Exact input, source, coverage, model, preprocessing, code, split and evaluator digests.
- All runs including failures, abstentions, timeouts and incomplete outputs; no silent exclusions.

Hard gates: zero unknown/no-exposure promotion to zero detections; zero model-score promotion to coverage probability or source independence; zero physical-absence overclaims; zero restricted-source use; zero privacy/authority violations; zero silent run exclusions.

Do not set universal Pd, Pfa or ECE thresholds. Pre-register per-domain/modality/class/environment/hazard operating points, with the exposure denominator and target interval width. Any numeric acceptance threshold must be justified prospectively by that context.

## South African context

The benchmark does not authorize receiving, recording or processing any physical RF/communications feed. Before any Track B work, obtain South African legal and regulator review for the exact equipment, frequency, receive/transmit mode, purpose and data fields. ICASA's official licensing and type-approval requirements already documented in the programme remain a hard gate; receive-only should not be assumed exempt.

## Non-goals

No active RF, live sensor activation, tracking an operator/pilot/crew, personal location, identity, intent, threat score, interception, flight/navigation/ATC/VTS control, mitigation, jamming, spoofing, takeover or enforcement trigger. No benchmark pass is evidence of physical sensor performance or operational readiness.

## Information integrity and end-to-end timeliness

A valid sensor observation must still be tied to an authenticated source assertion, pinned content digest and current rights/purpose. A digest/signature failure is not a detector false positive or false negative; the assertion is rejected or quarantined before it enters calibration. Preserve the source-authentication failure as its own metric/quality dimension.

Service latency must be measured end-to-end across all responsible hops, with each sub-budget and the overall budget declared before the run. Keep event time, source time, ingest time and receipt time distinct; clock uncertainty is part of the timing record. A latency budget is use-case-specific, not a universal threshold. The two new cases CC-020 and CC-021 lock these controls.
