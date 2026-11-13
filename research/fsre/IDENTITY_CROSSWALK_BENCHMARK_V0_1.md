# FSRE Temporal Identity Continuity and Boundary Benchmark v0.1

**Status:** Research protocol; the Rust reference validates synthetic transition semantics only.  
**Version:** 0.1.0  
**Date:** 2026-11-13  
**Claim ceiling:** No real Green Drop system identities have been linked. No real asset identity, ownership, legal status or operational state is asserted.

## 1. Research problem

A national total is not an entity history. A system can persist, change name, transfer operator, merge with another facility, split into multiple systems, be commissioned, be decommissioned, or disappear from a report because of missing submission or a scope change. A naïve name join can turn any of these into a false continuation.

For South Africa's Green Drop evidence, three denominators must remain separate: WRO describes 995 works in the 2022 all-sector audit; SAnews reports 850 municipal systems in the 2022 municipal comparison; and the 2025 full audit covers 848 municipal systems across 144 WSAs. SAnews says the 850-to-848 municipal count difference reflects decommissioning or merging. These published aggregate facts do not identify which individual systems continued or changed. System-level linkage must not be inferred from the national totals.

A useful new external comparator is Unmapped Dataworks UM-006. Its 12 November 2026 release describes 3,103 records from Green Drop 2024, Blue Drop 2023/2024 and No Drop 2025, with Demarcation Board codes on municipality records. Its public catalogue says links between the same system across assessments remain future work. The release is early-access, its API requires a key and limits use, and its terms say underlying DWS facts remain subject to source-owner rights. FSRE records the release metadata and fingerprint as a comparator lead only; it does not download, copy or re-publish the underlying records. Any future access requires permission from the data owner and provider.

## 2. Identity-transition event contract

The reference event model distinguishes:
- CONTINUITY: one source ID to one target ID;
- RENAMED: one source ID to one target ID with a changed label/code;
- TRANSFERRED: one source ID to one target ID with a changed operator/steward;
- MERGED_INTO: multiple distinct source IDs to one target ID;
- SPLIT_INTO: one source ID to multiple distinct target IDs;
- DECOMMISSIONED: source ID only, no successor asserted;
- CREATED: target ID only, no predecessor asserted;
- UNRESOLVED: source identity exists but no successor is accepted.

Every event group binds source/target namespace, effective date, evidence reference, evidence class and review state. Merge and split cardinality is explicit; they cannot be represented as a silent one-to-one alias. A transition without an evidence reference, with conflicting dates or namespaces within one event, or with conflicting same-date source/target assignments is invalid.

Accepted records require an official crosswalk, a source-native change record or independent field evidence. An operator assertion alone is not enough for acceptance. Synthetic evidence can only be marked SYNTHETIC_ONLY. Unresolved, disputed and provisional records remain unresolved; they are not force-matched to improve the apparent sample size.

## 3. Executable reference

The dependency-free Rust crate at tools/fsre-identity-crosswalk validates transition event shape and cardinality. Its ten-row synthetic fixture includes continuity, rename, transfer, a two-to-one merge, a one-to-two split, decommissioning, creation and unresolved identity. A bounded generated test also exercises 50 synthetic transition groups (62 rows) across the eight event types. The 19-test suite additionally rejects grouped transitions that disagree on relation, source namespace, effective date, evidence reference or review state. The fixture contains no actual municipal or treatment-works identifiers. The generated sweep is not an independent sealed challenge set and says nothing about real-world matching accuracy. Its output explicitly states that real identity linkage is not established, source authenticity is not authenticated and operational authority is none.

This reference is a contract validator, not an entity-resolution model. It does not ingest Green Drop PDFs, match names, geocode facilities, infer ownership, validate legal transfer or decide whether two real records refer to the same facility.

## 4. Benchmark tasks and metrics

### Tier A — transition semantics

Use synthetic, independently prepared cases covering all eight event types, missing source/target IDs, duplicate IDs, invalid effective dates, evidence conflicts, unresolved identities, and namespace errors. The current generated 50-group test is a development smoke test; the release benchmark still requires a separately authored, reviewer-controlled sealed set with an independently specified expected outcome for each case.

Targets:
- 100% correct relation cardinality on a sealed deterministic challenge set;
- 0 synthetic-to-accepted promotion;
- 0 unresolved/disputed-to-accepted promotion;
- 0 same-date competing assignments silently accepted;
- deterministic order-invariant output.

### Tier B — boundary-aware longitudinal linkage

Only after rights-cleared system-level source rows become available, assemble independently labelled pairs and event groups. Include the 2022 full audit, 2023 Progress Assessment and 2025 full municipal audit only where a comparable population and stable record references exist. Keep municipal, DPWI and private ownership classes separate. Bind WSA and municipality demarcation codes to time-qualified crosswalks; do not treat WSA identity as treatment-works identity.

Ground truth must be dual-reviewed by personnel independent of the matching system, cite the source page/table/row and retain disagreements as unresolved. Split by organisation/WSA and report cycle to prevent near-duplicate leakage. No model may see sealed adjudication labels.

Metrics:
- pairwise precision/recall and F1 by transition relation;
- false-continuity rate, especially across merge/split/decommissioning;
- merge/split event exact-match accuracy;
- unresolved/abstention precision and coverage;
- boundary-scope error rate (municipal versus all-sector);
- temporal validity and effective-date accuracy;
- provenance completeness and dual-review agreement;
- performance by ownership class, province, WSA and source-vintage;
- manual adjudication hours and cost per accepted transition.

### Tier C — matched industry comparisons

Compare:
1. exact source-native IDs where publishers provide them;
2. deterministic alias/code crosswalks and manually maintained registers;
3. string-only name matching;
4. fuzzy name plus geography and WSA/municipality hierarchy;
5. temporal graph/event resolution using merge/split/transfer evidence;
6. owner-operated incumbent data integration or master-data-management workflows;
7. a human expert adjudication baseline;
8. FSRE's evidence-qualified transition contract and its best matching implementation.

All methods must receive the same frozen records, permitted auxiliary data, geographic boundaries, review budget and hardware. Report model/version, thresholds, feature availability, geography vintage, manual effort and false-continuity cost. A false merge can be much more harmful than an unresolved record; report relation-specific costs rather than one aggregate F1 only.

## 5. Research targets (prospective, not achieved)

- Zero accepted links without explicit evidence reference and review state.
- Zero false accepted continuity on sealed merge, split, decommissioning and scope-change controls.
- At least 99% precision for accepted identity links before any consequential longitudinal claim; coverage must be reported alongside precision.
- 100% of unresolved identities retained as unresolved in output.
- At least 95% of system rows assigned an explicit population scope, assessment cycle and owner class before trend analysis; lower coverage means no national longitudinal trend claim.
- Independent reproduction of each accepted transition from source page/record citation and crosswalk version.
- No single score that hides poor performance in DPWI/private or low-data WSA groups.

The 99% target is a proposed release gate for high-consequence longitudinal claims, not a measured result or confidence bound. It must be revised if label quality, sample size or error-cost analysis indicates a stricter requirement.

## 6. Falsification

The proposed evidence-qualified crosswalk adds no value if source-native stable identifiers and incumbent MDM systems already provide equal or better longitudinal precision, merge/split handling, provenance and operational cost. It is also falsified if its acceptance gates reduce coverage so far that no useful claim can be made, or if independent adjudicators cannot reproduce accepted transitions from source records. The appropriate outcome may be to adopt the incumbent identifier service and retain only the common claim/reporting contract.

## 7. Immediate next work

1. Use the synthetic Rust validator to stabilize relation cardinality and negative controls.
2. Resolve DWS rights for the 2025 report resource before system-level extraction.
3. Request source-native system identifiers and change/decommissioning logs from DWS; do not infer identities from counts.
4. Inspect the Unmapped Dataworks release metadata as a comparator only; request permission from both DWS and provider before using its records.
5. Build a dual-reviewed, rights-cleared transition challenge set, separated by WSA and report cycle.
6. Evaluate deterministic crosswalks, incumbent MDM, fuzzy matching and graph/event methods under equal constraints.
7. Only then produce a time series or compare individual facility trajectories.

**Current status:** Tier A has a synthetic contract reference. Tiers B and C are protocols. No real system-level records, real identity links or longitudinal municipal wastewater performance results have been created.
