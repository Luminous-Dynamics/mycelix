# NRVCA Household Capability and Distributional Outcomes — Protocol v0.1

**Parent:** [NRVCA-020 #5991](https://github.com/Luminous-Dynamics/mycelix/issues/5991)  
**Programme:** [NRVCA-001 #5374](https://github.com/Luminous-Dynamics/mycelix/issues/5374)  
**Status:** source-registry and metric-definition foundation; no microdata ingested and no empirical estimate recomputed  
**Profile:** South Africa v0.1

## 1. Purpose

Build an auditable bridge from national-account and physical-capital measures to the distribution of household outcomes. The protocol keeps GDP, national income, household survey responses, utility service telemetry, wealth estimates, physical stocks, and modelled scenarios as distinct evidence classes.

The research question is:

> Do aggregate economic and capital-stock improvements correspond to improved, broadly distributed household capabilities and sustainable service access, and which groups, places or future cohorts remain underserved or exposed?

The first objective is **measurement integrity**, not policy ranking. Do not infer causal effects from descriptive co-movement.

## 2. Paradigm: linked accounts, not a replacement GDP

The architecture is a set of linked but non-collapsed accounts:

1. **Production and national accounts:** output, income and expenditure by release vintage and price basis.
2. **Household outcomes:** income categories, housing, health, education, food security, work and reported access to services.
3. **Physical service accounts:** delivered water, energy, transport and other service quantities with clear measurement boundaries.
4. **Distribution accounts:** quantiles, group gaps, deprivation and access gaps where the source supports them.
5. **Capital and inheritance accounts:** produced, financial, human, natural and institutional stocks, maintenance/depletion and intergenerational exposure.

A change in GDP is not itself evidence of improved household welfare. A survey-reported access indicator is not continuous telemetry. A utility bulk-supply metric is not household delivery. A wealth-distribution estimate is not a household microdata record.

The OECD Well-being Framework is a strong international measurement comparator: it separates 11 dimensions of current well-being, distribution across groups/top-bottom performers/deprivations, and four future-capital categories (economic, natural, human and social). It is a framework and comparison anchor, not a ready-made South African household dataset:
https://www.oecd.org/en/publications/how-s-life-2024_90ba854a-en/full-report/component-5.html

UN SEEA Ecosystem Accounting provides spatially explicit ecosystem extent, condition and ecosystem-service accounts. It should supply a natural-capital lens without monetizing or replacing physical household outcomes:
https://seea.un.org/en/methodology/ecosystem-accounting

## 3. First data sources

### 3.1 Stats SA General Household Survey 2025

DataFirst catalogue:
https://datafirst.uct.ac.za/dataportal/index.php/catalog/1144

- Reference ID: `zaf-statssa-ghs-2025-v1`
- DOI: `10.25828/a4a6-p161`
- Producer: Statistics South Africa; distributor: DataFirst / University of Cape Town
- Version: v1, edited/anonymised for public distribution, June 2026
- Data access metadata says Creative Commons Attribution (CC BY); cite the published DataFirst citation exactly and supply DataFirst with a copy/link to resulting publications as specified.
- Coverage: all nine provinces; minimum published geography is province and metropolitan municipality where applicable.
- Population: private households in the nine provinces and residents in workers' hostels. Other collective living quarters and military populations are outside the stated survey universe.
- Unit of analysis: households and individuals.
- Design: two-stage stratified sampling using the 2013 Master Sample; approximately 3,080 PSUs. Preserve PSU/cluster, strata, weight and design metadata.
- Weights account for selection probabilities, subsampling/segmentation, frame exclusions, nonresponse, trimming and benchmarking to Stats SA population estimates.
- Survey period: January–December 2025; cross-sectional.

Stats SA publication P0318:
https://www.statssa.gov.za/?PPN=P0318&page_id=1854

DataFirst release note (9 October 2026):
https://dfru.uct.ac.za/latest-release-south-africas-general-household-survey

**Important:** the GHS is a cross-sectional household survey. It does not provide continuous utility telemetry, complete household wealth balance sheets, a causal estimate of a policy, or evidence of uninterrupted/safe/affordable service unless the relevant question directly measured that construct. Low-level disaggregation is cautioned by the dataset metadata. Changes in provincial boundaries and historical master-sample frames require a comparability review before trend statements.

### 3.2 Stats SA rebased and benchmarked GDP release

Stats SA published the 2022-base benchmarked/rebased GDP estimates on 29 October 2026:
https://www.statssa.gov.za/?PPN=Report+04-04-04&SCH=74595&page_id=1854

The accompanying release notice:
https://www.statssa.gov.za/?p=20039

The South African Reserve Bank confirms that the revision covers 2018 Q1–2026 Q2 and comprises revised nominal GDP at current prices and real/volume GDP at constant 2022 prices:
https://www.resbank.co.za/en/home/publications/publication-detail-pages/media-releases/2026/statistics-revision

The prior 2015-base series in NRVCA-011 / PR #5430 is an immutable legacy vintage. Do not overwrite it or silently splice the 2022-base series into it. A separate 2022-base source identity and overlap/revision bridge are required. At the 9 November 2026 snapshot, SARB says balanced production/expenditure/income national-account series through 2026 Q3 are expected with the 15 December 2026 Quarterly Bulletin, while 1946–2017 backcasts remain future work.

### 3.3 World Inequality Report 2026

https://wid.world/document/world-inequality-report-2026/

The WIR 2026 country sheet is a separately constructed income/wealth estimate series. Use it for independently attributed national distribution context only after its exact series definitions, methodology, units, year, redistribution concept and applicable data-use terms are pinned. Do not treat it as microdata, join its group shares to GHS respondents, or assume its top/bottom categories have identical denominators or definitions to the GHS.

The report text indicates CC BY-NC-SA 4.0. That does not automatically establish the licence for every underlying downloadable data series. Check the series-specific terms before copying/rehosting any data; do not embed report narrative or tables in a redistributable product without rights review.

## 4. Metric semantics

The initial dictionary contains three **proposed metric definitions only**:
- household access to piped water in the dwelling, yard or communal tap;
- household-reported inadequate/severely inadequate food access;
- household access to mains electricity.

Stats SA's GHS 2025 published findings use these concepts, but the exact questionnaire variable codes, recode rules, valid-response denominator and missingness must be reconciled against the v1 questionnaire, DDI and microdata before producing any statistic. Metric examples intentionally contain no values and are marked pending codebook mapping.

For each metric, preserve:
- the exact question/text and definition version;
- unit of analysis and universe;
- numerator/denominator and valid-response rule;
- weight, stratum, PSU and variance estimator;
- geography and subgroup support;
- valid period, release date and known-at time;
- source file digest and data rights;
- missingness/nonresponse, imputation and any transformations;
- comparability state against prior periods.

An access indicator must not be relabelled as continuity, quality, safety, affordability, adequacy or delivered quantity unless it directly measures that attribute.

## 5. Distributional analysis

Where permitted and statistically supported, report:
- weighted mean and median;
- quantiles and top/bottom shares;
- top/bottom ratios or Palma ratio;
- Gini only with an explicit, comparable underlying distribution;
- group gaps by published/available geography and demographic strata;
- deprivation share using a declared threshold and denominator;
- confidence intervals / design-based uncertainty and effective support.

Do not hardcode a universal Gini limit or universal welfare target. A threshold can only be a locally declared, reviewable administrative or normative trigger—not a universal statistical law.

Do not treat all statistics with the same name as comparable. Income before/after tax, disposable income, consumption, household vs individual equivalence scales, wealth net of debt, PPP/current ZAR, survey year vs income reference year, and top-coded/truncated distributions are materially different constructs.

## 6. Required cohorts and evidence classifications

Keep source and analysis classifications distinct:

- `official_published_statistic`: a value published by Stats SA/SARB for the stated vintage.
- `survey_estimate`: weighted estimate derived from GHS microdata using documented design.
- `external_distribution_estimate`: independent WID or other third-party estimate with its own method.
- `physical_measurement`: a utility/asset/instrument record with measurement boundary.
- `derived_statistic`: a reproducible computation over admitted inputs.
- `model_output`: output of a calibrated or uncalibrated model, with profile and limitations.
- `scenario_assumption`: hypothetical input, never relabelled as measured.
- `normative_threshold`: a declared policy/value choice, not a measured fact.

Every missing cohort, region, service or indicator remains explicitly missing/unassigned. No silent pro-rating or population weighting to invent causal incidence.

## 7. Benchmark protocol

### B0 — source and semantic integrity
- Validate release/vintage, source, period, definition, population, geography, frequency and units.
- Same label with changed denominator, questionnaire, boundary, price-base, survey frame or KPI rule must not be joined without a bridge.
- Negative fixtures must reject future-known or revised values used in an earlier historical frontier.

### B1 — survey reproduction
- Reproduce selected Stats SA GHS 2025 summary tables from exact acquired v1 microdata, DDI, questionnaire and official publication.
- Use supplied survey weights and sampling-design metadata.
- Preserve PSU/cluster and strata; do not treat household rows as independent replicates.
- Compare only within published precision and predeclared rounding tolerance.
- Do not claim small-area validity below the published geography or without a separately validated estimator.

### B2 — distribution and deprivation
- Report top/bottom groups, group gaps and deprivations independently.
- Publish sample/weighted support and uncertainty per stratum.
- Do not merge WID estimates with GHS records; a metric-definition crosswalk must state unresolved differences.

### B3 — national-to-household diagnostic bridge
- Compare GDP/sector revisions, household survey indicators, physical-service accounts and capital stocks only over compatible time windows and explicit vintages.
- Treat disagreement as an observable diagnostic; do not infer causation from co-movement.
- Preserve nominal/current-price and real/volume series separately.
- Report missing datasets, nonresponse, boundary changes and revision effects.

### B4 — policy / scenario evaluation
- A causal claim requires a named intervention, target population, comparison, timeline, outcome and defensible identification strategy.
- Symtropy simulations remain scenarios until external calibration/validation. Use model-family holdouts and independent physical/accounting oracles.
- Normative allocation tests compose with Symtropy #3610 and its Pareto/essential-floor reference rather than redefining justice or adding a new optimizer.

## 8. Scorecard — no single welfare scalar

For each metric, group, geography and period report:
- source value/estimate, unit and definition;
- weighted mean/median/quantiles where supported;
- deprivation/service-floor incidence;
- absolute and relative group gaps;
- support count, effective sample size and uncertainty;
- source completeness, missingness, nonresponse and data vintage;
- comparability state and reasons for non-comparability;
- physical, financial, ecological and inherited-capital outcomes in native units;
- Pareto trade-offs across capabilities, distribution, ecological condition, economic resources and structural power.

## 9. First implementation sequence

1. Freeze the source manifest and metric dictionary. No raw microdata are committed in this first documentation tranche.
2. Acquire exact source artifacts only under their documented terms. Record exact byte sizes, hashes, retrieval times and license decisions. A publication locator is not a source digest.
3. Review the GHS questionnaire, DDI and codebook. Map variables and exact valid-response rules; do not guess column names or category codes.
4. Implement an independent weighted-estimate reproducer on a small subset; compare with Stats SA published tables.
5. Add group-gap, top/bottom and deprivation outputs with design-based uncertainty.
6. Add the GDP 2015-to-2022 vintage bridge as a separate linked artifact; do not rewrite PR #5430's source.
7. Compose physical energy/water/economic capital and future-cohort incidence after each upstream owner is qualified.
8. Only then build dashboard projections in Sol-Atlas; display source vintage, support, uncertainty, missingness and rights alongside every value.

## 10. Claim ceiling

A PASS establishes only that the declared source, metric, survey design, estimator, cohort/geography, period and evaluator reproduce the declared source statistics under the stated assumptions. It does not establish a universal definition of well-being, person-level wealth, causal policy impact, current service continuity, or an authoritative replacement for Stats SA/SARB. No automatic eligibility, rights, political authority or public-resource allocation may be derived from these metrics.


## 3.4 Public DDI mapping update (9 November 2026)

The public DataFirst JSON metadata export is accessible separately from the login-gated microdata page:
- DDI/JSON locator: https://datafirst.uct.ac.za/dataportal/index.php/metadata/export/1144/json
- Study reference ID: `zaf-statssa-ghs-2025-v1`.

The export identifies the F3 household file as `ghs-2025-household-v1` (20,095 records; 206 variables) and maps these variables:
- `wat_drinkwat` F3/V347 — Main source of water; categories 1 (tap in dwelling), 2 (tap on site/in yard), 3–16 (other source types).
- `eng_light` F3/V378 — Energy for lighting; categories include mains electricity (1) and generator (2), plus other source categories. Codes 9 and 10 both have a None label in the metadata and remain distinct pending questionnaire clarification.
- `fsd_hung_adult` F3/V435 — Insufficient food for adult; ordinal categories Never, Seldom, Sometimes, Often, Always, Not applicable (no adults), Unspecified.
- `fsd_hung_child` F3/V436 — Insufficient food for children; code 6 is Not applicable (no children).
- Supporting food hardship items: `fsd_worried` V437; `fsd_healthy` V438; `fsd_fewfoods` V439; `fsd_fewfoods_days` V440; `fsd_ateless` V441; `fsd_ateless_days` V442; `fsd_ranout` V443; `fsd_ranout_days` V444.
- Survey-design fields: `psu` F3/V330, `prov` F3/V331, `stratum` F3/V528, `geotype` F3/V529, `metro` F3/V530, `rotation` F3/V532 and `house_wgt` F3/V533.

The DDI variable-level question text field is blank for these variables. Therefore variable labels/category codes are mapped, but exact interviewer question wording, routing/applicability and questionnaire changes remain to be checked against the questionnaire/documentation before defining final estimators. The metadata's displayed category frequencies are not weighted national estimates. The microdata download remains login-gated; no microdata bytes were acquired and no weighted estimate has been computed.
