# FSRE South African public-data inventory — water and sanitation v0.1

**Reviewed:** 2026-11-13  
**Status:** Source inventory and admissibility plan; no operational baseline frozen.  
**Study boundary:** Public national/provincial/municipal evidence for research and historical context only. No live utility telemetry, private asset register, household-level personal data or operational control data is included.

## 1. Research objective

Establish a reproducible public-data starting point for a South African water/wastewater service study without confusing fundamentally different indicators.

The first public-data layer can establish:
- household access context;
- published drinking-water, wastewater and water-efficiency assessment results;
- municipal financial capacity and spending context;
- historical source coverage, reporting gaps and source-quality limitations.

It cannot by itself establish current service hours, pressure, water quality at a tap, actual physical losses, wastewater effluent quality at a treatment plant, repair duration or causal effect of a maintenance intervention. Those require utility-authorised operational data and appropriate independent verification.

## 2. Public sources and admissibility

| Source | Grain / period | Useful fields or indicators | Admissible use | Key limitations |
|---|---|---|---|---|
| Stats SA General Household Survey (GHS) 2025 | Household-survey estimates, national/provincial and published subgroups, survey year 2025 | Published key findings report 87.4% of households with access to piped water in the dwelling, yard or communal tap; 84.0% with improved sanitation | Broad access and equity context; identify areas for deeper study where published geography permits | Access is not continuity, safety, pressure, affordability, source reliability or utility performance; do not infer individual-household conditions |
| Stats SA GHS 2024 publication and time series | Annual survey estimates; selected variables 2002–2024 | Tables and downloadable Excel time series | Historical trend context and methodological comparison with GHS 2025 | Survey design, categories, weighting and question definitions must be checked before constructing a continuous series; estimates are not operational telemetry |
| National Treasury Municipal Finance Data / Municipal Money API | Quarterly Section 71 submissions; 292 municipalities; 2008/09–2025/26 in the current catalogue; current page reports latest 2026 Q2 data | Income/expenditure, water and wastewater service charges, repairs and maintenance, cash flow, grants, audit opinions and municipal identifiers | Financial capacity, spend/maintenance context, time-series source and budget/actual analysis | The landing page explicitly says the latest figures are still being checked for completeness and integrity. Data is only as complete/trustworthy as municipal submissions. Financial spending does not prove maintenance quality or service outcomes |
| DWS Green Drop 2025 | Wastewater-treatment systems in 144 Water Services Authorities; assessment basis 2023/24 municipal financial year; 848 systems | System and authority Green Drop assessment, risk and performance indicators | Wastewater risk and historical system/authority comparison, after verifying system identifiers and report definitions | Published source collection is PDF reports; the WRO collection says system-level tables have not been extracted into machine-readable form. It is not live plant telemetry or current effluent sampling |
| DWS Blue Drop 2025 | Drinking-water systems; 2025 assessment/reporting cycle | Drinking-water quality risk categories and assessment results | Published drinking-water system-risk context | Do not equate a risk classification with current point-of-use sample results or continuous quality compliance |
| DWS No Drop 2025 progress reporting and Parliament briefing | National water-conservation / distribution-efficiency results reported in 2026 | Parliament's 9 June 2026 briefing reported national average non-revenue water of 47.3%, versus 47.4% in the 2023 Full No Drop Report | Dated national-level context and an indicator family for later utility-level water-balance studies | National aggregate is not a Johannesburg baseline, a physical-leakage-only measure or a measure of household service hours |
| Parliament Portfolio Committee briefing, 9 June 2026 | National water-sector and workforce context from the 2025 Full Green Drop, Blue Drop and No Drop Progress Reports | 47% / 396 of 848 wastewater systems critical; 2025 Blue Drop low-risk share 61.9% vs 60.2%, critical-risk share 7.9% vs 9.9%; 1,151 process controllers employed of 2,431 required, and 84 supervisors short | Dated sector-level context and workforce-capacity hypothesis generation | Parliamentary summary of report results, not the underlying asset/time-series dataset or a local operating baseline |

## 3. Source facts that must remain separate

The 2025 Stats SA GHS key findings report that 87.4% of households had access to piped water in the dwelling, yard or at a communal tap, and 84.0% had access to improved sanitation. These are household access indicators. They are not service-continuity or water-quality metrics.

The 2025 Green Drop assessment covered 848 municipal wastewater systems for the 2023/24 financial year. The government reported 396 systems (47%) in a critical state. The 2025 Blue Drop findings concern drinking-water quality risk categories. The No Drop programme concerns water conservation and distribution efficiency.

Parliament's 9 June 2026 briefing reported national non-revenue water of 47.3%, compared with 47.4% in the 2023 Full No Drop Report. This remains a national aggregate, not a local utility measurement. It should not be described as equivalent to physical leakage alone.

The same briefing reported a shortage of skilled process controllers: 1,151 employed against 2,431 required, plus 84 supervisors short. This is a workforce-capacity signal, not proof that any particular facility is understaffed or that staffing caused any specific service failure.

**Never combine GHS household access, Blue Drop risk, Green Drop wastewater performance, No Drop non-revenue water, and Treasury expenditure into one service score.** They have different populations, units, observation periods and causal interpretations.

## 4. Source quality and version controls

### 4.1 National Treasury

The Municipal Finance API documentation states:
- the API is updated quarterly from snapshots of the Local Government Database;
- submissions originate from Section 71 municipal reporting;
- municipal managers and CFOs sign submissions as accurate;
- audited-actual amount types are audit-outcome figures matched by National Treasury to the Auditor-General-audited annual financial statements;
- the data documentation warns that the API is only as complete and trustworthy as the municipal data submitted;
- the current landing page says 2026 Q2 figures are still undergoing completeness/integrity verification.

The API exposes relevant v2 cubes and item taxonomies:
- `repmaint_v2` (2019/20 onward) includes items 6001 employee-related costs, 6002 inventory consumed, 6003 contracted services and 6004 operational costs.
- `incexp_v2` (2019/20 onward) includes 0400 water service charges and 0500 wastewater-management service charges.
- `demarcation_changes` records old/new municipality codes and transition dates and should support the temporal identity crosswalk.

The v2 cubes use mSCOA and do not share a one-to-one item-code taxonomy with the older v1 cubes. Keep periods before and after the 2019/20 boundary distinct until a reviewed item-code crosswalk exists.

Implementation rules:
1. Preserve amount type: budget, actual, adjusted budget, audited actual and other classifications are not interchangeable.
2. Preserve the quarter/snapshot and retrieval time.
3. Distinguish audited actuals from unaudited Section 71 management submissions.
4. Preserve corrections and restatements rather than silently overwriting prior snapshots.
5. Preserve raw API nulls and the source's documented interpretation separately: the API documentation states that a returned `null` measure should be interpreted as zero where the municipality did not fill in that value; a fact that is not returned for the requested query means it was not supplied in an acceptable form and must remain missing/unknown, not zero. Never convert an absent fact row into zero expenditure.
6. Reconcile important values to the corresponding Treasury Section 71 report or audited financial statements before use in a consequential comparison.

### 4.2 DWS Green Drop / Blue Drop / No Drop

The DWS IRIS results page is the official discovery point for current published results. The Water Research Observatory's Green Drop 2009–2025 collection points to the published report PDFs and states that system-level tables have not been extracted into machine-readable form. As checked on 13 November 2026, both the WRO dataset page and the individual 2025 PDF resource page state `No License Provided`; do not assume download, extraction or redistribution rights. Resolve rights directly with the DWS custodian before using the PDF.

Keep Green Drop audit populations distinct. The WRO catalog describes the 2022 full audit as 995 works across municipal, Department of Public Works and private ownership. SAnews' 1 April 2026 account describes 850 municipal systems in the 2022 comparison and explains that 848 municipal systems were audited in 2025 after some systems were decommissioned or merged. The published critical/good-excellent rates (334/850 and 118/850 in 2022; 396/848 and 66/848 in 2025) are therefore stored with the municipal denominators, never with 995. No system-level identity crosswalk or fixed-cohort comparison has been performed.

A third-party comparison source is Unmapped Dataworks UM-006, whose 12 November 2026 public metadata release reports 3,103 records from Green Drop 2024, Blue Drop 2023/2024 and No Drop 2025. It describes 998 wastewater works (848 municipal, 114 DPWI and 36 corporate/private), 958 Blue Drop 2023 supply systems, 1,003 Blue Drop 2024 risk-assessment systems and 144 No Drop WSAs; it records Demarcation Board codes for municipalities. Its accuracy page reports a 50-record sample check and it publishes a release fingerprint. Its documentation also says cross-assessment system linking remains future work. The dataset is early access, its API is key-gated and limited, and the provider says the underlying DWS facts have no published reuse terms while it seeks permission. Treat this as public metadata and an external benchmark lead only; do not scrape, mirror or republish the records. The FSRE source register marks it DISCOVERED_NOT_EXTRACTED.

Implementation rules:
1. Store the exact source report, retrieval date, publication date, assessment period and report revision.
2. Preserve the original DWS system and Water Services Authority labels.
3. Maintain an explicit crosswalk for renamed, merged, transferred or decommissioned systems and municipalities.
4. Record whether a result is a full assessment, progress assessment or summary.
5. Do not interpolate annual values between assessment cycles.
6. Preserve unknown, not-applicable and not-reported categories as distinct states.
7. Do not infer a present-day operational state from a report covering an earlier financial year.

### 4.3 Stats SA household survey

Implementation rules:
1. Preserve survey year, publication release, table/variable, denominator, weighting and estimate uncertainty where supplied.
2. Do not downscale provincial estimates into municipality or neighbourhood estimates.
3. Retain "do not know", unspecified and suppressed/small-cell values as distinct missingness states.
4. Treat household access as context, not as a service-quality or reliability result.

## 5. Data needed from a willing water-services authority for a qualified service study

Public sources are sufficient to start contextual analysis and data-quality work. A service-outcome pilot needs operator-approved records with a defined data-sharing and purpose agreement.

Minimum candidate data domains:

| Domain | Minimum fields / grain | Qualification requirements |
|---|---|---|
| Service outcomes | Service-area/cohort, time interval, service fraction or delivered volume, declared target and quality threshold | Measurement method, denominator, coverage, unit, time alignment, missingness and uncertainty |
| Water balance | System input volume, billed authorised consumption, unbilled authorised consumption where applicable, apparent and real losses | Meter calibration, reconciliation method, reporting period, boundary and uncertainty |
| Water quality | Sampling point, sample time, analyte, result, unit, threshold, laboratory and method | Chain of custody, sample representativeness, lab accreditation where applicable, validation and non-compliance handling |
| Pressure / flow / outage | Approved aggregate or asset-level readings, outage start/end, pressure/flow and restoration state | Instrument metadata, calibration, clock quality, maintenance state, sampling interval and safe disclosure controls |
| Wastewater | Influent/effluent sampling, process indicators, bypass/overflow events and compliance results | Sampling location/time, laboratory provenance, applicable permit/threshold and uncertainty |
| Energy | kWh, delivered/treated kL, meter/plant boundary and tariff period | Meter identity/calibration, allocation method and normalisation assumptions |
| Maintenance | Work-order creation, diagnosis, dispatch, repair, parts, crew, closure and repeat fault | Exact event times, status history, duplicate/reopened work orders and independent completion evidence |
| Skills and resources | Credential/role, shift availability, spares, chemical inventory and mobilisable suppliers | Privacy, labour rules, safety, competence validity and source freshness |
| Finance / procurement | Contract, purchase order, invoice, payment, budget and lifecycle cost | Procurement authority, contract version, financial classification and delivery verification |

Do not request or ingest personal household records where aggregated or purpose-bound statistics suffice. Do not publish sensitive OT asset coordinates, control networks, physical security information, credential data or exploitable single points of failure. Operational access requires the responsible operator's explicit permission, data minimisation, security review and a safe read-only starting boundary.

## 6. First no-runner work package

The following tasks can proceed using public sources and deterministic local/repository checks:

1. **Source register:** freeze each source URL, publisher, version, retrieval time, report period, geography, denominator, unit, rights and known limitations.
2. **Municipal identity crosswalk:** map DWS Water Services Authorities, Treasury municipal codes and current demarcation versions; preserve many-to-many and temporal relationships.
3. **Indicator dictionary:** define GHS access, Blue Drop, Green Drop, No Drop, NRW, finance, maintenance and workforce indicators separately.
4. **Missingness report:** quantify missing municipalities, missing periods, suppressed values, source revision changes and unresolved identity mappings.
5. **Public-data historical replay:** only after exact source extraction and a pre-registered study boundary; keep national/provincial and municipal statistics at their actual published grain.
6. **Operator data request specification:** describe the minimum approved operational evidence required for a service-level study; do not collect operational data without the responsible party's agreement.

## 7. First bounded study hypothesis

**Hypothesis:** Combining dated wastewater-system condition assessments, municipality-level financial and maintenance context, and published service-access context can identify *where evidence is insufficient and which candidate service studies are feasible* better than any one source family alone.

This is an evidence-readiness and study-selection hypothesis, not a causal claim that expenditure changes system condition or that a national assessment predicts household service reliability.

A first evaluation may measure:
- proportion of records linked to a valid authority/system identity;
- proportion with known observation period and denominator;
- temporal staleness and revision frequency;
- disagreement between summary and detailed reports;
- completeness of financial/maintenance data by municipality and amount type;
- uncertainty and suppression coverage;
- number of service claims that the public data can and cannot substantiate.

The study is successful even if it concludes that the public data are insufficient for service-outcome claims. It should make the next required evidence acquisition explicit rather than filling gaps with model assumptions.

## 8. Primary sources

- Stats SA, GHS 2025 key findings: https://www.statssa.gov.za/?PPN=P0318&SCH=74563&page_id=1856
- Stats SA, GHS 2024 publication and downloadable additional tables: https://www.statssa.gov.za/?PPN=P0318&SCH=74038&page_id=1854
- National Treasury Municipal Finance Data: https://municipaldata.treasury.gov.za/
- National Treasury Municipal Finance API documentation: https://municipaldata.treasury.gov.za/docs
- DWS IRIS latest published results: https://ws.dws.gov.za/IRIS/latestresults.aspx
- WRO Green Drop reports 2009–2025 collection: https://wrcwro01.arc.agric.za/en/dataset/green-drop-reports-2009-2025-south-african-municipal-wastewater-management-performance
- South African Government 2025 Green Drop announcement: https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
- Parliament Portfolio Committee on Water and Sanitation, 9 June 2026 briefing: https://www.parliament.gov.za/press-releases/media-statement-water-committee-warns-deepening-crisis-municipalities-fail-implement-corrective-water-plans
