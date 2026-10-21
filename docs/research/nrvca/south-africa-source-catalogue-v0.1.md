# South Africa NRVCA Source Catalogue v0.1

**As-of date:** 2026-10-21  
**Status:** discovery catalogue; no source bytes ingested or licensed for production use yet.  
**Purpose:** choose a small, evidence-bounded pilot and specify what must be recorded before a source can feed a report.

This is a register of candidates identified through publisher pages. It is **not** a claim that every listed dataset is current, openly licensed, downloadable in machine-readable form, complete, or suitable for national estimation. A publication page being accessible does not settle dataset licensing. Before ingest, pin the exact downloaded bytes, publisher revision/vintage, extraction code, schema, units, geographic/time scope, and terms of use.

## Recommended first vertical slice

Use the following order:

1. **Official monetary baseline:** Stats SA GDP release and accompanying time series.
2. **Physical energy flow bridge:** Stats SA's experimental Physical Energy Flow Accounts (PEFA), 2015–2022.
3. **Water resource bridge:** selected catchment accounts (Breede, uMngeni, Mooi), 2015/16–2020/21.
4. **Water service / performance evidence:** DWS water-services benchmarking, only with coverage and submission-confidence flags retained.
5. **Service/asset context:** DWS water-resource monitoring and planning evidence, once specific data product access and interpretation are confirmed.

This sequence can establish a small historical crosswalk without pretending that the catchment data are a complete national water account or that physical energy flows themselves measure welfare.

## Candidate source table

| ID | Publisher/source | Known scope | Status and permitted initial use | Main caveat |
|---|---|---|---|---|
| SA-GDP-P0441-Q2-2026 | [Stats SA P0441, GDP Q2 2026](https://www.statssa.gov.za/?PPN=P0441&page_id=1854) | National quarterly GDP; release page dated 8 Sep 2026; offers the publication and GDP time-series download | **Official statistical baseline.** Reconstruct a few published aggregates, preserve the release vintage and price basis | GDP is a revised monetary production flow; don't mix nominal/real values, quarterly/annual values, or subsequent revisions |
| SA-PEFA-2022 | [Stats SA D0401.7, Physical Energy Flow Accounts for 2022](https://www.statssa.gov.za/?PPN=D0401.7&SCH=74271&page_id=1856) | Physical energy flows; extends South African PEFA history to reference year 2022, following 2015–2021 | **Experimental statistics.** Compare physical energy input/use flows to the monetary production baseline at explicitly aligned years and sector definitions | Stats SA describes this as a discussion document with experimental accounts, not official statistics. It is not an energy reliability, affordability, or welfare measure |
| SA-WATER-CATCHMENT-2015-2021 | [Stats SA D0401.4, Sub-national Water Resource Accounts](https://www.statssa.gov.za/?PPN=D0401.4&page_id=1854) | Six hydrological years 2015/16–2020/21; Breede, uMngeni and Mooi catchments in two Water Management Areas; hydrological years run October–September | **Experimental/sub-national accounts.** Establish time and unit alignment, reproduce selected source tables, then relate water accounts to spatial and service context | Not national coverage; catchments were selected for water supply relevance to major cities. These are resource accounts, not a complete measure of municipal delivery, quality, access, or value |
| SA-NCA-ECOSYSTEM-INVENTORY | [Stats SA ecosystem accounting catalogue](https://www.statssa.gov.za/?page_id=14717) | Inventory identifies land and terrestrial ecosystem accounts 1990–2014; protected areas 1900–2020; strategic water source areas 1990–2020; sub-national water accounts 2015–2021; experimental biodiversity-based tourism estimates 2013–2019 | **Discovery / experimental account index.** Use to plan later joins and avoid re-creating published accounts | Inventory is not a single homogeneous dataset; definitions, granularity, periods, account status and file formats differ by publication |
| SA-NCA-STRATEGY | [Stats SA National Natural Capital Accounting Strategy](https://www.statssa.gov.za/?page_id=14714) | Ten-year strategy and institutional programme for national natural-capital accounts | **Methodology/policy context.** Use to align vocabulary, institutional roles and future work | Not a source of quantitative time series. Stats SA's forum page mentioned a strategy review pathway for 2026; its completion/status must be verified rather than assumed |
| SA-DWS-WATER-RESOURCE-PLANNING | [DWS Strategic Water Resource Planning](https://www.dws.gov.za/iwrp/NWRP.aspx) and [NWRS-3](https://www.dws.gov.za/dnwrs/default.aspx) | Water availability/requirements reconciliation, Water Management Areas, national resource planning and strategy | **Planning context / potential evidence locator.** Use to map institutional definitions and identify underlying source models or tables | Strategy and planning assessments are not themselves observed water deliveries. Dated assumptions and modeled values must not be promoted to measured capacity |
| SA-DWS-MONITORING-NETWORKS | [DWS National Water Resource Monitoring network review](https://www.dws.gov.za/Projects/NWRM/) | Page identifies national hydrological, geohydrological, chemical, microbial, eutrophication, radioactivity, toxicity, aquatic-health, estuarine and wetland monitoring networks | **Candidate operational-measurement source.** Ask DWS for specific series, formats, calibration/QC metadata, spatial coverage and access terms | The project page documents networks and their review; it does not prove any specific time series is downloadable, complete, current or licensed for our intended use |
| SA-DWS-NO-DROP-2021-22 | [DWS National State of Water Report 2023, water security section](https://www.dws.gov.za/Projects/National%20State%20of%20Water%20Report/Documents/National%20State%20of%20Water%20Report%202023-%20Water%20Security%208.pdf) | Describes national water-loss benchmarking using 2021/22 data and No Drop Watch 2023; the cited report says 88 datasets (61%) were received from water services authorities and documents high/medium/low confidence categories | **Coverage/quality context.** Use the confidence taxonomy to design missingness and evidence-status rules; later obtain underlying data if permissible | The report is not a full 2026 service observation. It notes that non-submitting WSAs may have water balances calculated, extrapolated or estimated; those values must retain a modelled/estimated status rather than be mislabeled as observed. Submitted-data coverage and confidence vary; nonresponse is not zero consumption or zero water loss |

Source-level statuses above reflect the linked publisher materials, not a fresh data-file audit. The JSON companion records license_status unknown and ingest_state discovery_only until the source's exact terms and bytes have been reviewed.

## First crosswalk: monetary output × physical capability

The first historical work should be a **non-causal descriptive bridge**, not a composite index:

- GDP: published monetary flow by quarter and sector, carrying release vintage and current/constant-price basis.
- PEFA: physical energy supply/use quantities in the source-defined units and classifications.
- Water accounts: physical water stock/flow variables at catchment and hydrological-year scale.
- Service performance: actual supply, non-revenue water/losses, interruptions and quality only where valid service-area observations exist.
- Outcomes: population/business coverage and unmet essential demand only where denominators and service definitions are supported.

Do not force quarterly GDP to match a hydrological year by silently averaging or prorating. Retain original periods and add an explicit, versioned alignment method only for comparisons that can justify one. Where the intervals cannot be defensibly aligned, show separate time series.

## Minimum ingestion receipt

Before a source enters an NRVCA report, record:

- publisher, dataset and release identifiers;
- exact URL and retrieval timestamp;
- exact bytes SHA-256 and digest scope;
- licence/terms decision and allowed reuse;
- release vintage, statistical status (official or experimental), data reference period and revision policy;
- geography/CRS, spatial resolution, time basis, calendar or hydrological-year convention;
- units, sector/catchment classifications, source definitions and transformations;
- coverage denominator, nonresponse, revisions, flags, uncertainty and documented limitations;
- exact parser/transform version, output digest and independent reconstruction result.

If any field is unknown, report it as unknown and restrict the claim accordingly. Never put a mutable webpage URL alone in place of a content digest.

## Immediate research tasks

1. Retrieve the Stats SA GDP Q2 2026 time-series file and publication, record digests and recreate a small table of published values.
2. Retrieve the PEFA 2022 discussion document and machine-readable companion table; check energy units and classifications against PEFA 2015–2021.
3. Retrieve the sub-national water account workbook(s), inspect hydrological-year labels, units, missingness, and any modelled-versus-observed distinctions.
4. Determine legal/permission status before redistributing source extracts; publish only derived outputs or excerpts permitted by the applicable terms.
5. Request/locate DWS service-quality datasets with explicit denominator/coverage metadata. Do not infer actual household delivery from resource availability alone.
6. Freeze an initial bridge dataset and comparison protocol before testing correlations, nowcasts or proposed policy effects.

## Research anchors

- [2025 System of National Accounts](https://unstats.un.org/unsd/econstats/national-accounts/sna2025)
- [UN SEEA](https://seea.un.org/en)
- [Stats SA Natural Capital Accounting Strategy](https://www.statssa.gov.za/?page_id=14714)
- [Stats SA ecosystem-account catalogue](https://www.statssa.gov.za/?page_id=14717)
- [Stats SA GDP Q2 2026](https://www.statssa.gov.za/?PPN=P0441&page_id=1854)
- [Stats SA PEFA 2022 key findings](https://www.statssa.gov.za/?PPN=D0401.7&SCH=74271&page_id=1856)
- [Stats SA Sub-national Water Resource Accounts](https://www.statssa.gov.za/?PPN=D0401.4&page_id=1854)
- [DWS National Water Resource Monitoring project](https://www.dws.gov.za/Projects/NWRM/)
- [DWS National State of Water Report 2023, water security](https://www.dws.gov.za/Projects/National%20State%20of%20Water%20Report/Documents/National%20State%20of%20Water%20Report%202023-%20Water%20Security%208.pdf)
