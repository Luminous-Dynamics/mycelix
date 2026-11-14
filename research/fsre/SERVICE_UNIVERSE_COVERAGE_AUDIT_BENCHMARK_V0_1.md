# FSRE service-universe coverage audit benchmark v0.1

**Date:** 2026-11-14  
**Status:** Synthetic reference contract. No real-world coverage, population completeness, statistical estimate or field validation is claimed.

## 1. Research question

Before FSRE reports service outcomes for declared cohorts, can it determine whether the evaluated cohort register is reconciled against a separately defined service-universe reference frame over the complete study horizon?

The cohort-manifest work answers whether submitted outcome lanes match a declared register. This benchmark adds a separate boundary question: whether the declared register itself has been compared with a second source frame, with omissions, duplicates, unresolved links, unrepresented in-scope records, out-of-scope records and temporal drift kept distinct.

The result is deliberately named service-universe reconciliation, not population coverage certification. A reference register may itself be incomplete, biased, unauthenticated or out of date.

## 2. Why this matters

Statistics South Africa's Statistical Quality Assessment Framework explicitly includes frame coverage, under-coverage, over-coverage, duplication, out-of-scope units and misclassification. It calls for comparisons between alternative frames and matching operations, and notes that excluded units can differ systematically from included ones. FSRE adopts these categories as distinct failure modes rather than collapsing them into a single completeness percentage.

The UNECE Recommendations for the 2030 Round of Population and Housing Censuses distinguish gross coverage error (omissions, duplicates and erroneous enumerations) from net undercount/overcount. They describe independent post-enumeration surveys, alternative administrative sources and record linkage as coverage-evaluation methods. These are statistical design methods, not a licence to treat any two correlated registers as independent captures.

A 2026 practitioner study of entity resolution across six benchmarks (864 to 5 million records) reports that matcher performance is dataset-dependent, that precision and recall need different controls, and that one false-positive bridge can collapse distinct clusters. FSRE therefore stores adjudicated crosswalk decisions as explicit evidence, forbids transitive closure from being treated as proof, and refuses to estimate recall from an unaudited list overlap.

## 3. Paradigm shift

**From closed-world cohort accounting to open-world frame reconciliation.**

The system should never silently assume that:
- every affected service unit appears in a supplied manifest;
- a missing link means a unit is out of scope;
- an ID match proves the same physical service asset across ownership or boundary changes;
- two public datasets are statistically independent;
- a list-to-list overlap percentage measures population recall;
- a current identifier has the same meaning over a historical study horizon.

The audit keeps three layers distinct:

1. **Frame definition:** unit type, service boundary, source owner, source reference, source snapshot, reference period and validity interval.
2. **Crosswalk adjudication:** observed record to reference record, explicit match, unresolved candidate, known in-scope non-match or out-of-scope disposition, and the evidence owner/reference supporting that adjudication.
3. **Universe qualification:** exact reconciliation against the declared reference frame, while real-world reference-frame completeness and source authentication remain separate unknowns.

Half-open intervals [from, until) are used consistently. A frame, record and crosswalk assertion must all cover the complete study horizon. A valid mapping for only part of the period is not a valid longitudinal identity.

## 4. v0.1 reference implementation

The dependency-free crate tools/fsre-service-universe-audit accepts one frozen audit file with:
- a declared enumerated reference frame;
- an observed comparison frame;
- a scope and unit type;
- full-horizon frame and record validity;
- per-observed-record crosswalk dispositions;
- evidence owner and reference for each adjudication;
- an explicit crosswalk-independence assessment.

It reports separate counts for:
- reference and observed units;
- one-to-one matched reference units;
- reference units with no match (omissions relative to the declared reference);
- reference units matched by multiple observed records (duplicate coverage);
- reference units with unresolved candidate links;
- observed in-scope records with an explicit no-match disposition;
- observed out-of-scope records;
- observed records with unknown scope;
- unresolved observed records and crosswalk edges.

Input fails closed on duplicate IDs, unknown metadata, missing crosswalk disposition, unknown reference targets, invalid decision/scope combinations, repeated assertions and validity intervals that do not cover the study horizon.

The supported exact comparison basis is DECLARED_ENUMERATED_REGISTER. A sample or unknown reference basis returns NOT_ESTIMABLE_REFERENCE_BASIS_UNSUPPORTED; v0.1 does not pretend a sample is a census.

The output always states:
- reference_population_completeness=NOT_ESTABLISHED;
- population_coverage_estimate=NOT_COMPUTED;
- source_authentication=ASSERTED_NOT_AUTHENTICATED.

RECONCILED_TO_DECLARED_REFERENCE_FRAME means only that the records reconcile one-to-one to the declared enumerated register and all observed rows have explicit dispositions. It does not certify that the reference register contains every real-world unit.

## 5. What v0.1 intentionally does not do

- No probabilistic/fuzzy entity resolution, automated clustering or transitive-closure inference.
- No capture–recapture / dual-system population estimate.
- No sample-based estimator, confidence interval, design effect or non-response correction.
- No geometry intersection, areal weighting, geocoding or spatial completeness claim.
- No inference that different owners imply independent sources.
- No authentication of owner, source, crosswalk adjudicator, legal permission or evidence bytes.
- No automated classification of a new facility, retired unit, split/merged asset or boundary transfer.
- No claim that a register is the true service universe.

For split, merged, renamed, relocated or transferred facilities, a one-to-one link is insufficient. The next schema version should add explicit event-sourced identity transitions with effective dates and reviewed predecessor/successor cardinality. Until then, those cases must remain unresolved rather than be forced into a false match.

## 6. Benchmark programme

### 6.1 Synthetic conformance corpus

The reference implementation should be tested against frozen scenarios:
1. exact one-to-one frame agreement;
2. omitted reference unit;
3. duplicate observed links to one reference unit;
4. unresolved observed-to-reference candidate;
5. in-scope observed unit absent from the reference register;
6. out-of-scope observed unit;
7. unknown-scope unit;
8. duplicate and malformed identifiers;
9. stale frame, record or crosswalk interval;
10. sample-based reference frame;
11. unknown crosswalk target;
12. contradictory link decisions.

Hard target for deterministic fixtures: 100% expected disposition agreement and zero silent conversion of unknowns into matches, omissions or out-of-scope units.

### 6.2 Gold-standard matching benchmark

Build an independently adjudicated pair corpus with difficult cases, not only exact identifiers:
- renamed or re-numbered works;
- shared site but distinct treatment trains;
- one facility split into multiple operational works;
- multiple works merged under a regional operator;
- municipal boundary and WSA changes;
- ownership transfer and private/municipal category changes;
- temporary shutdown, decommissioning and recommissioning;
- incomplete or conflicting geospatial attributes;
- sparse records that create false transitive bridges.

Report precision, recall, false-positive bridge rate, unresolved rate, duplicate-cluster rate and calibration separately. Use a held-out adjudication set and preserve original records and reversible crosswalk decisions. Require zero false-positive identity merges in the adversarial safety corpus; do not maximize recall by relaxing hard vetoes. Report abstention as a first-class outcome.

Targets are proposed gates, not achieved results:
- match precision: at least 99.5% on the held-out high-confidence match class;
- recall: at least 95% on the independently labelled candidate population, with a 95% confidence interval;
- false-positive bridge rate: zero in the adversarial bridge corpus;
- unresolved rate: reported, not hidden; reduce it without degrading precision;
- 100% reproducible output from the same frozen inputs and versioned adjudications.

The precision target is intentionally stricter than ordinary record-linkage F1 optimisation because a false merge can contaminate identity, time-series, denominators and decision-regret calculations downstream.

### 6.3 Statistical coverage estimation

Only a future, separately qualified design may estimate real population under-coverage. It must declare the target population, time reference, frame inclusion/exclusion rules, sample design, matching error, dependence between captures, population closure or movement model, non-response and uncertainty. An independent post-enumeration survey or other justified design may be appropriate.

Capture–recapture must not be run merely because two lists overlap. If list dependence, heterogeneous capture probability, population movement or linkage error is not adequately modelled, the estimate is NOT_ESTIMABLE. Exact register reconciliation and statistical population estimation are different claims.

### 6.4 Operational performance

Measure:
- throughput and memory by frame size (10^3, 10^4, 10^5, 10^6 and 10^7 records);
- candidate-pair generation cost separately from adjudication cost;
- deterministic replay time;
- memory ceiling;
- time-to-resolve for changed or disputed identity links;
- stale-crosswalk detection after an identity or boundary change;
- regression rate for previously adjudicated hard negatives.

Do not claim SOTA until the same labelled corpus, source snapshots, hardware, matching thresholds and error taxonomy are used for all comparators.

## 7. Industry comparison and positioning

This increment is not a substitute for mature entity-resolution engines, GIS topology checks, statistical register maintenance, hydraulic network models or enterprise asset-management systems. Those systems supply candidate matches, geometry, authoritative identifiers and domain knowledge.

FSRE's intended differentiator is the assurance layer around their outputs:
- versioned reference and observed frames;
- explicit evidence-backed match adjudication;
- time-bounded identity validity;
- one-to-one reconciliation and hard-negative handling;
- preserved uncertainty and abstention;
- a clear distinction between declared-frame reconciliation and real population coverage;
- downstream propagation of identity uncertainty into cohort eligibility and decision regret.

The competitive question is not whether a small Rust crate matches more records than a commercial ER engine. It is whether a combined workflow can prevent a silent false identity merge or missing population frame from becoming a falsely qualified service-resilience decision.

## 8. Acceptance and falsification

The v0.1 reference is falsified by any of the following:
- a duplicate observed record silently increases matched coverage;
- an unknown-scope record is treated as out of scope;
- an unmatched in-scope record is silently discarded;
- an unresolved identity is counted as a deterministic match;
- a stale frame or crosswalk is accepted outside the full study horizon;
- a sample frame is reported as an exhaustive register;
- row order changes the output;
- output claims real-world population completeness or source authentication.

Passing this synthetic contract is necessary but not sufficient for production. A production qualification requires independently sourced and rights-authorized reference data, a reviewed identity transition model, external source authentication and an independent adjudication corpus.

## 9. Research references

- Statistics South Africa, South African Statistical Quality Assessment Framework: Operational Standards and Guidelines, First Edition (2010), especially accuracy / frame coverage and record-linkage quality.
- UNECE, Recommendations for the 2030 Round of Population and Housing Censuses (draft circulated 4 March 2025), sections on coverage management, gross/net coverage error and coverage evaluation.
- Pavani et al., Entity Resolution in Practice: Lessons from a Self-Serve Pipeline (28 July 2026), six benchmarks from 864 to 5 million records, precision/recall separation and false-positive bridge risk.
