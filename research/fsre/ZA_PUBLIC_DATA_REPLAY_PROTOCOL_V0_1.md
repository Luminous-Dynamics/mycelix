# FSRE South African public water-data replay protocol v0.1

**Date:** 2026-11-13  
**Status:** Extraction and validation protocol; no Treasury fact rows or DWS system tables have been extracted in this increment.  
**Primary scope:** Public, non-personal data for South African water and wastewater service context.

## 1. Objective

Build a reproducible historical evidence package that establishes which water-sector claims can be supported by public data, at their actual published geographic and temporal grain.

The first replay is an evidence-readiness exercise. It is not an assessment of current Johannesburg service reliability, plant condition, water quality, physical losses, intervention causality or municipal performance.

Use the machine-readable source register at [`ZA_PUBLIC_SOURCE_REGISTER_V0_1.psv`](ZA_PUBLIC_SOURCE_REGISTER_V0_1.psv). It is a discovery register; every row must be upgraded from `DISCOVERED_NOT_EXTRACTED` only when a source snapshot, raw digest, retrieval time and extraction receipt exist.

## 2. National Treasury Municipal Finance API

### 2.1 Verified API structure

The National Treasury / OpenUp Municipal Money API documentation identifies:
- HTTPS API base: `https://municipaldata.treasury.gov.za/api`;
- API cubes, each with a `/model` endpoint for field and dimension metadata;
- `/facts`, `/members/<member>` and `/aggregate` endpoints;
- filtering via `cut`, pagination through `page` and `pagesize`, and JSON or CSV format;
- integer Rand measures, with no cents;
- a 10,000-row default maximum page size;
- quarterly snapshots of Section 71 municipal submissions;
- a current catalogue covering 292 municipalities and financial years 2008/09–2025/26, including disestablished municipalities.

As of 13 November 2026, the landing page reports that the latest 2026 Q2 figures are still being verified for completeness and integrity. They are not to be treated as a fully qualified audited panel.

### 2.2 Candidate water-relevant cubes

| Cube | Period | Relevant items / dimensions | Intended use |
|---|---|---|---|
| `repmaint_v2` | 2019/20 onwards | 6001 employee-related costs; 6002 inventory consumed; 6003 contracted services; 6004 operational costs | Repairs/maintenance expenditure context |
| `incexp_v2` | 2019/20 onwards | 0400 service charges - water; 0500 service charges - wastewater management | Revenue/service-charge context; not service outcome |
| `demarcation_changes` | 2009 onwards | Old/new municipality code and transition date | Temporal municipality identity crosswalk |
| `municipalities` | Current metadata | Demarcation code/name, province, category and parent fields | Identity resolution at the cube's published revision |
| `repmaint` | Older v1 period | 5001–5005 legacy repair/maintenance item taxonomy | Historical context only; do not splice to v2 without a reviewed crosswalk |

The v1 and v2 cubes are not assumed to be semantically identical. The 2019/20 mSCOA transition changed item coding and structures. Preserve cube and item-code provenance; do not combine a 5005 total from v1 with 6001–6004 v2 records as though the taxonomy were unchanged.

### 2.3 Safe query sequence

For each cube, obtain and retain the model before requesting facts:

1. `GET /api/cubes` — discover current cube names.
2. `GET /api/cubes/repmaint_v2/model` — retain the full model response.
3. `GET /api/cubes/incexp_v2/model` — retain the full model response.
4. `GET /api/cubes/demarcation_changes/model` — retain the identity-transition model.
5. Query membership for the exact municipality and amount-type codes from the selected cube; do not assume a code is unchanged across all cubes.
6. Query a narrowly filtered fact slice with `cut`, `page`, `pagesize` and `format=json`.
7. Follow all pages until the server's total fact/cell count is satisfied.
8. Reconcile selected values against the corresponding Treasury Section 71 report and, for audited actuals, audited annual financial statements.

Illustrative fact query shape (replace the example code only after the model and member lookup have confirmed it):

```text
GET /api/cubes/repmaint_v2/facts?pagesize=10000&cut=financial_year_end.year:2025|amount_type.code:"<verified-code>"|demarcation.code:"<verified-code>"
```

The documentation shows `cut` filters as `dimension.attribute:value`, with string values quoted and multiple cuts separated by `|`. Query encoding must preserve the separators and quote characters correctly.

### 2.4 Required dimensions and controls

Keep at minimum:
- `demarcation.code` and `demarcation.label`;
- `financial_year_end.year`;
- `financial_period.period`;
- `period_length.length`;
- `amount_type.code` and label;
- `item.code` and label;
- raw measure `amount`;
- cube name and cube-model digest;
- retrieval time and source snapshot/release marker;
- raw response digest and page metadata.

Amount type is non-aggregatable. Never sum budget, actual, adjusted budget and audited actual together. The API docs state that audited actual amounts are audit-outcome figures matched by Treasury to Auditor-General-audited annual financial statements. Retain those categories separately.

The API documentation also states:
- a returned `null` measure should be treated as zero where a municipality did not fill in that value;
- a missing fact from a query indicates that the information was not supplied in an acceptable form.

Therefore store both `raw_amount` and `interpreted_amount` plus a `null_interpretation_rule` field. A fact row that is entirely absent is not zero expenditure. Preserve this distinction in all missingness reports.

### 2.5 Financial interpretation limits

Repairs/maintenance expenditure is a financial input, not proof of:
- physical asset condition;
- work-order completion;
- competent maintenance;
- service uptime;
- water quality;
- causal effect on an outage or recovery.

Water/wastewater service charges are municipal revenue categories, not delivered volume, household access, affordability or continuity.

A municipal-level financial row does not identify a particular treatment plant, reservoir, pump, pipe or neighbourhood. Do not downscale or spatially allocate the values without a defensible source and explicit uncertainty.

## 3. Stats SA General Household Survey

For GHS 2025, retain:
- the survey year and publication/release date (26 May 2026);
- table/variable ID and definition;
- population/household denominator;
- weighting and uncertainty fields supplied by Stats SA;
- exact national/provincial/subgroup grain;
- missing, suppressed and "do not know" states;
- source document and raw digest.

Published national findings state that 87.4% of households had piped water access in the dwelling, yard or communal tap, and 84.0% had improved sanitation access in 2025. They are access indicators, not continuity, safety, pressure, water quality, affordability or utility-level performance. GHS 2024 values must remain a separate survey-year record and must not be compared without verifying definitions and survey methodology.

No provincial estimate is to be interpolated into a municipality, ward or neighbourhood estimate.

## 4. DWS Green Drop, Blue Drop and No Drop

### 4.1 Green Drop
The 2025 Green Drop assessment covers 848 municipal wastewater treatment systems for the 2023/24 financial year, across 144 Water Services Authorities. The cited public result reports 396 systems (47%) in a critical state.

Required extraction:
- exact report PDF and revision;
- Water Services Authority identifier/name;
- wastewater-system identifier/name;
- assessment period;
- score/risk category and underlying indicators;
- denominator and system coverage;
- notes and data-quality caveats;
- retrieval date and raw PDF digest.

The WRO collection notes that system-level tables have not been extracted into machine-readable form and shows inconsistent licence metadata. Until DWS confirms the redistribution/licensing terms and the raw reports are acquired, store only source links and manually verified attributed facts; do not redistribute extracted report tables as open data.

### 4.2 Blue Drop
Keep Blue Drop drinking-water system-risk classifications separate from water sample results. Capture report cycle, system identity, assessment denominator, categories and underlying fields. Do not treat a risk classification as a current water sample, point-of-use result or continuous compliance measurement.

### 4.3 No Drop
Keep the 2025 No Drop progress-report and parliamentary briefing values separate from Green Drop. Parliament's 9 June 2026 briefing reported national non-revenue water of 47.3%, compared with 47.4% in the 2023 Full No Drop Report. This is a national water-distribution-efficiency indicator, not a Johannesburg baseline or a physical-leakage-only measure.

## 5. Cross-source identity resolution

Municipal names are not stable identifiers. Use:
- National Treasury demarcation codes and its `demarcation_changes` old/new code transitions;
- DWS Water Services Authority identifiers and published system identifiers;
- Stats SA geography codes at their actual publication grain.

Create a time-qualified crosswalk with:
- source system and source-native ID;
- source-native label;
- valid-from/valid-to dates;
- target/canonical identity ID;
- mapping relation (same, renamed, split, merged, transferred, uncertain);
- supporting source reference;
- reviewer and verification state.

A many-to-many or uncertain crosswalk remains unresolved. Do not force historical municipality or WSA records into a current name where the boundary has changed.

## 6. Data product layers

Keep three distinct layers:

1. **Raw immutable source snapshot:** original JSON/CSV/PDF/XLSX bytes, retrieval time, URL, license terms, source revision, file digest and page/query metadata.
2. **Normalised analytical table:** parsed rows with explicit source IDs, grain, period, denominator, units, raw and interpreted values, null/missingness state, crosswalk version and transformations.
3. **Derived study view:** selected variables and predeclared analysis for a named hypothesis. Every derived value traces to a normalised row and raw source location.

Never overwrite an earlier snapshot with a later revision. Keep corrections and restatements as new versions.

## 7. First historical-replay question

The first study should ask:

> Can public access, published wastewater/drinking-water assessment and municipal financial data jointly improve the identification of evidence-ready candidate service studies, without claiming that they establish actual service reliability or causal intervention effects?

Primary outcomes for this data-readiness study:
- source coverage and source-period coverage;
- municipality/WSA identity mapping rate and unresolved mapping rate;
- availability of comparable pre/post mSCOA financial records;
- missing and null-value frequency by source and cube;
- proportion of rows with valid denominators and observation periods;
- report revision/change detection;
- number of candidate service claims supported, unsupported or indeterminate by public evidence;
- reproducibility of the extracted table from raw source snapshots.

The no-runner tranche can begin with source-register validation, model metadata capture, identity crosswalk design and extraction receipts. Do not make causal or service-outcome claims until a utility-authorised service record and an appropriate evaluation design are available.

## 8. Claim ceiling

This protocol identifies discoverable public sources and reproducible extraction semantics. It is not a completed municipal dataset, a Johannesburg service baseline, a physical asset inventory, a causal study or a public performance ranking. No Treasury fact values or DWS system-level tables are claimed as extracted in this increment.

## 9. References

- National Treasury Municipal Finance API documentation: https://municipaldata.treasury.gov.za/docs
- Treasury Municipal Finance Data landing page: https://municipaldata.treasury.gov.za/
- Treasury cube list API: https://municipaldata.treasury.gov.za/api/cubes
- Stats SA GHS 2025 key findings: https://www.statssa.gov.za/?PPN=P0318&SCH=74563&page_id=1856
- Stats SA GHS 2025 media release (26 May 2026): https://www.statssa.gov.za/publications/P0318/MEDIA%20RELEASE%20-%20%20GHS%202025%20.pdf
- DWS IRIS latest published results: https://ws.dws.gov.za/IRIS/latestresults.aspx
- South African Government 2025 Green Drop announcement: https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
- Parliament water-sector progress briefing (9 June 2026): https://www.parliament.gov.za/press-releases/media-statement-water-committee-warns-deepening-crisis-municipalities-fail-implement-corrective-water-plans
