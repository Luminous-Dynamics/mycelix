# FSRE cohort-set manifest and source-use qualification protocol v0.1

**Date:** 2026-11-14  
**Status:** Protocol proposal accompanying the synthetic manifest reference. It does not establish authority to reuse any external dataset.

## 1. Purpose

A distributional outcome report needs to answer two different questions:

1. **Was the declared cohort set reconciled?** Are all expected cohort IDs present, unique, correctly scoped, denominator-consistent and valid for the entire study horizon?
2. **Was the evidence lawfully and legitimately usable for this purpose?** Does the source owner permit this processing, purpose, derivation, storage, publication and retention?

These questions must not collapse into one boolean. A perfectly reconciled manifest can be legally inadmissible, and a licensed dataset can still contain incomplete or incorrect population identities.

The v0.1 code in `fsre-cohort-outcome-envelope` implements the first question relative to a supplied manifest. Source-use authorization remains a separate gate and is not inferred from machine readability, public availability, a third-party platform's "open" label, or a checksum.

## 2. Cohort manifest contract

The manifest is a frozen owner-declared register for one service, service family, cohort set and outcome snapshot. It declares:

- manifest ID, manifest snapshot ID, source owner and opaque source reference;
- service and service-family IDs;
- cohort-set ID and retrospective outcome snapshot ID;
- denominator unit;
- each expected cohort ID;
- the denominator value or explicit UNKNOWN;
- identity-source owner/reference;
- denominator-source owner/reference;
- half-open membership validity interval [valid_from_seconds, valid_until_seconds).

Each submitted lane must match the manifest on identity source, denominator source, denominator value, population unit, service scope, cohort-set scope and outcome snapshot. Membership validity must cover the entire horizon. Duplicate manifest IDs, duplicate lane IDs, missing lanes, extra lanes, unit or denominator drift, source-reference drift, and expired membership are hard negatives.

The output distinction is deliberate:

- `COMPLETE_FOR_DECLARED_MANIFEST`: the submitted lanes reconcile exactly to the declared register.
- `INCOMPLETE_OR_MISMATCHED_TO_DECLARED_MANIFEST`: the submitted lanes do not reconcile exactly.
- `ESTIMABLE_FOR_DECLARED_MANIFEST`: in addition to an exact manifest reconciliation, every submitted lane has known observation coverage across the declared horizon.
- `NOT_ESTIMABLE`: manifest reconciliation or lane observation coverage is incomplete.

`COMPLETE_FOR_DECLARED_MANIFEST` is not equivalent to complete real-world population coverage. The manifest could itself omit people, assets, service zones, private operators or a class of affected users. A source-owner signature, content digest, independent register review and a lawful population-boundary audit are separate qualification requirements.

## 3. Source-use admissibility is an independent gate

Before processing a real dataset, record a distinct use-authorization claim:

- source custodian and publishing authority;
- exact source product, version, snapshot and digest;
- legal instrument or permission reference;
- allowed purposes and prohibited purposes;
- permitted transformations, derived outputs and redistribution;
- territory, effective start/end, retention and deletion obligations;
- whether third-party contribution or database rights apply;
- reviewer, review date and unresolved legal questions.

Use one of the explicit states:

- `AUTHORIZED_FOR_DECLARED_PURPOSE`: an owner- or rights-holder-backed authorization covers the declared use and period.
- `REVIEW_REQUIRED`: the licence, rights holder, permitted purpose or downstream rights are unclear or contradictory.
- `NOT_AUTHORIZED`: permission is absent, denied or expired.

`REVIEW_REQUIRED` and `NOT_AUTHORIZED` must block production ingestion and external publication. A public URL, PDF, API key, open download, mirrored dataset, public hash or third-party assertion of openness does not override the source custodian's terms. A checksum establishes byte identity only relative to the bytes hashed; it does not prove authenticity, ownership, completeness or permission.

Keep this gate separate from the service outcome disposition. If source-use authorization is not established, do not calculate or publish a production service outcome from those records. The software can still use synthetic fixtures authored for testing, under their own declared synthetic-test scope.

## 4. South African Green Drop: a live source-governance test case

The South African Government's 1 April 2026 release says the 2025 Green Drop report audited 848 municipal wastewater treatment systems for the 2023/24 financial year. It reports 396 systems (47%) in a critical state, compared with 334 (39%) in 2022, and 66 systems (8%) in good or excellent condition. These are national aggregate findings, not a fixed longitudinal cohort of system-level outcomes.

The WRO catalogue for the DWS Green Drop reports says:
- the 2022 report covered 995 works across municipal, DPWI and private ownership;
- the 2025 report covered 848 municipal systems in 144 Water Services Authorities;
- the source resources were published as PDF reports and system-level tables were not extracted into machine-readable form in that catalogue record;
- its visible licence area displays "No License Provided", while its additional-information fields also list "Open (Creative commons)" and a CC BY 4.0 URL.

Those two licence statements are inconsistent within the same catalogue entry and must be resolved with the custodian before relying on the latter as an authorization.

A third-party Unmapped Dataworks dataset page updated from a 12 November 2026 release describes 3,103 records, including all 998 Green Drop 2024 works: 848 municipal, 114 DPWI and 36 private-sector works. It also lists Blue Drop audit/risk-assessment records and No Drop 2025 records. The page reports a public SHA-256 fingerprint and machine-readable access through an API or Parquet download. However, it also says DWS publishes no reuse terms and that the publisher has requested permission; its statement "free and open meanwhile" is not a substitute for written authorization from the rights holder. The page also says cross-cycle linkage between the same wastewater system is still future work.

The apparent difference between 848 and 998 is therefore first an ownership/scope distinction in the third-party dataset's description, not evidence that 150 systems disappeared. The 2022 count of 995 and 2025 municipal count of 848 must not be compared as a fixed population without ownership, identity, boundary and inclusion-rule reconciliation.

### Required investigation before production use

1. Obtain the original DWS IRIS reports and a written statement of applicable reuse rights for the precise purpose and derived outputs.
2. Ask DWS to resolve the WRO catalogue's conflicting licence fields and clarify whether any database or report-specific rights apply.
3. Request a machine-readable, owner-authorized system register and historical identifiers from DWS, including owner, WSA, reporting boundary, dates, capacity, flow and source page/table references.
4. Treat the Unmapped dataset as a third-party candidate discovery/index source only until the underlying source rights and downstream reuse permissions are cleared. Preserve its release fingerprint and version metadata in the investigation record, but do not infer source authenticity or permission from the fingerprint.
5. Independently reconcile the 2022 municipal/DPWI/private population, the 2025 municipal population and any later system transfers, splits, closures, new works, renamed works or changed WSA boundaries.
6. Keep unknown identity links as UNKNOWN. Do not auto-merge by fuzzy name, geography or similar capacity without independent adjudication.
7. Do not convert a municipality-level Green Drop score into safe service delivered to a population. Identify the service boundary, denominator, receiving environment, effluent-quality measures and observation coverage separately.

## 5. Benchmark: manifest and identity qualification

The independent manifest challenge should include:

- exact cohort-set match;
- omitted cohort and extra cohort;
- duplicate cohort ID in the manifest and in submitted lanes;
- denominator changed while identity stays the same;
- same denominator with changed source reference;
- changed identity-source reference with same display name;
- unit mismatch (persons versus connections);
- membership valid-from after the study begins;
- membership valid-until before the horizon ends;
- service, service-family, cohort-set or outcome-snapshot mismatch;
- unknown denominator;
- cohort merge/split and ownership transfer;
- ID reuse after closure;
- same facility with multiple owners/operators;
- boundary/WSA changes;
- stale manifest snapshot;
- contradictory source licence and downstream permission claims.

A reviewer who did not implement the parser should author a disjoint corpus and adjudicate expected answers. Report precision, recall, coverage, unknown rate, false-merge rate, false-split rate and manual-review workload. Do not report only accepted-link precision: a system that abstains on most identities can appear artificially precise.

## 6. v0.1 implementation boundary

The current manifest-backed Rust reference:
- requires exact set equality between manifest cohort IDs and supplied lanes;
- checks the declared service, family, cohort set, outcome snapshot and population unit;
- checks exact denominators, identity and denominator owner/reference pairs;
- checks membership validity across the complete analysis horizon;
- preserves unknown observations and blocks outcome estimability where any lane has unknown time;
- does not authenticate the manifest, its owner, its source bytes or its licence;
- does not establish that the manifest is exhaustive for the real population;
- does not resolve identity merges/splits automatically;
- does not calculate population-weighted or normative equity metrics.

The next engineering step is a content-bound, owner-attested manifest receipt and a separate source-use authorization envelope. These must be independently reviewed before any production Green Drop data ingestion.

## References

- South African Government, Department of Water and Sanitation (2026-04-01), *Water and Sanitation crowns Steve Tshwete Local Municipality as 2025 Green Drop Champions*. https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
- Water Research Observatory, *Green Drop Reports 2009-2025: South African Municipal Wastewater Management Performance*. https://wrcwro01.arc.agric.za/en/dataset/green-drop-reports-2009-2025-south-african-municipal-wastewater-management-performance
- Unmapped Dataworks, dataset UM-006, *Water quality by treatment works*, release 2026-11-12. https://unmapped.co.za/datasets/water-quality
- Graham, P. M., Pattinson, N. B. & Still, D. (2025-04-01), *The state of wastewater management in South Africa: data gaps, missing wastewater, and Green Drop reporting*. Water SA 51(2). https://journals.co.za/doi/10.17159/wsa/2025.v51.i2.4152
