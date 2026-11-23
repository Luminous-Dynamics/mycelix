# South African soil measurement and decision economics benchmark v0.1

**Status:** synthetic cost-ledger engine plus published-tariff reference catalogue. The strategy fixture is manufactured. Public tariffs are external price references, not a current farm quote or a claim of method equivalence.

## 1. Why this benchmark is needed

A South African deployment must beat the actual local alternative, not an abstract lab price or a European per-hectare figure. Publicly posted 2026/27 tariffs provide useful anchors:

| Provider / validity | Example published price | Critical comparability note |
|---|---|---|
| Western Cape Department of Agriculture / Elsenburg, 1 Apr 2026–31 Mar 2027 | Sample preparation R20; pH(KCl) R20; citric-acid P/K/Ca/Mg/Na R30 combined; exchangeable acidity R20; Walkley–Black C R32; 3-fraction particle size R32, 5-fraction R85. | Method-specific and a published department tariff; the listed sample preparation is a separate item. This is not an all-in sampling, courier and interpretation price. |
| KZN Department of Agriculture and Rural Development, 2026/27 | Soil fertility R180/sample; inorganic N R210; total C by Dumas R230; total N by Dumas R230; texture 3 fractions R200; 7 fractions R330; exchangeable Na R200. | Dumas total carbon is not directly interchangeable with the Western Cape Walkley–Black result; a line item is not necessarily a bundled panel. |
| Soil Health Support Centre, 2026 | Volumetric aggregate stability R315; microbial respiration R475; combined VAS + microbial respiration R500. | Different functional assays from standard fertility chemistry. Do not compare the combined package as though it were a single pH/P/K test. |
| Proximal sensing service literature review, 2026 | Company-service estimates €142–€362/ha; literature-derived estimates €15.5–€130/ha. | European estimates; use as an industry cost-accounting comparator, not a South African tariff. Fieldwork, travel/access, personnel, equipment, analysis and reporting matter. |

Primary tariff/source URLs are retained in `published-tariff-references.json`. All values need renewed source verification and scope review before procurement. Public tariff data does not establish what an actual farmer pays after field sampling, courier, VAT, report interpretation, repeat testing or service support.

## 1A. South African laboratory / spectroscopy landscape

The published tariff list is not the full industry map. Publicly documented service and capability surfaces include:

- **ARC Soil, Climate and Water Analytical Services:** the ARC page describes soil, water and plant analytical services, QC, sample submission and an established inter-laboratory proficiency-scheme context. Its page states participation in the AgriLASA and WEPAL proficiency schemes for soil/water/plant/fertilizer matrices, but says its current SANAS accreditation is for water anions. Do not describe all ARC soil chemistry as SANAS-accredited merely because the lab participates in proficiency schemes. Obtain the exact current scope/accreditation certificate per analyte/method before selecting it as a reference laboratory.
- **ARC / ICFR spectroscopy capabilities:** ICFR's Forestry Research Analytical Laboratory describes NIR reflectance research for low-cost, non-destructive analysis, including soil organic carbon and soil texture, and work to transfer applications to portable instruments. This is a valuable South African research comparator and possible calibration-transfer partner; it is not evidence that an NIR method is already qualified for all farms, crops or soil properties.
- **SGS South Africa:** publicly describes configurable soil nutrient and physical-property testing, pH, organic carbon, nematodes/pathogens, and accredited testing by appropriate laboratory/network methods. Exact analyte, laboratory site, method, accreditation scope, turnaround, sampling and reporting must be confirmed by quotation.
- **AgriLASA member directory:** useful for building an auditable supplier sampling frame; membership listing alone does not establish accreditation, method suitability, price or service quality.
- **ARC expertise locator:** lists mobile soil testing / soil spectroscopy expertise as a capability lead; confirm current availability, geographic service area, technology and fees before treating it as a deployable mobile service.

The benchmark should distinguish at least four supplier classes: reference analytical laboratories; physical / biological soil-health laboratories; research spectroscopy platforms; and proximal mapping / sensing service providers. Compare method scope, analytical QA/proficiency evidence, service coverage, turnaround, pricing basis and field logistics separately. A supplier's statement of "accredited testing" needs an analyte/method/laboratory scope match, not a logo-level assumption.

## 2. Paradigm shift: price per assay → cost per correct, qualified decision

The primary economic metric is **total cost per correctly resolved decision at a stated coverage and error profile**.

A cheaper spectrum is not a win if it leads to an unsupported phosphorus recommendation, misses a high-risk sample, or forces repeat sampling. Likewise, a low public lab tariff does not capture travel, sample preparation, farmer time, slow turnaround or interpretation cost.

Report each strategy separately:
- total cost, cost/ha, cost/sample, and cost/correctly resolved decision;
- coverage / resolved fraction and accuracy among resolved cases;
- false-negative rate among reference-positive cases that were resolved;
- unresolved-positive and unresolved-negative fractions;
- test referral rate, turnaround, and complete category-level cost decomposition;
- the strategy's measurement methods, applicability domain and reference status.

No aggregate “best strategy” score is emitted. Show the cost–coverage–risk Pareto frontier.

## 3. Cost ledger contract

Every strategy must account for all ten cost categories. A category is either:
- `priced`: quantity × unit price in ZAR; source/evidence class and scope are declared;
- `not_applicable`: explicitly justified, not silently set to zero.

Categories:
1. site access / travel;
2. sampling labour;
3. sample preparation;
4. sensor or laboratory assay;
5. instrument capital / depreciation;
6. calibration and quality control;
7. transport / courier;
8. analysis and reporting;
9. compute and storage;
10. repeats / retests.

Price evidence classes:
- `published_tariff`;
- `written_quote`;
- `measured_actual_cost`;
- `synthetic_assumption`.

Published tariffs and quotes require a source, validity dates, price basis and method/service scope. Actual cost entries require a cost-record reference and measurement period. Synthetic assumptions are permitted only in synthetic scenarios and must remain explicitly synthetic.

One-off capital, annual maintenance and marginal scan costs need explicit allocation units (for example instrument-year, campaign, scan or sample). No automatic amortisation is inferred.

## 4. Decision quality contract

For an evaluation cohort, the independent reference supplies:
- total eligible decisions;
- reference-positive and reference-negative counts;
- TP, TN, FP, FN among resolved decisions;
- unresolved-positive and unresolved-negative counts.

The counts must reconcile exactly. Accuracy is only among resolved decisions. False-negative rate is among resolved reference-positive cases; unresolved-positive fraction is reported separately. A strategy cannot improve apparent accuracy by silently dropping difficult cases.

This v0.1 synthetic auditor does not estimate confidence intervals, prevalence transport, causal impact or future deployment cost. Real benchmarks require a blinded, independent reference set and matched thresholds.

## 5. Required comparison arms

- all-reference wet-chemistry workflow;
- physical / biological soil-health test panel;
- NIR-only within a validated property-specific domain;
- fixed-rate confirmatory referral;
- applicability-domain / uncertainty referral;
- decision-risk / EVSI referral;
- proximal sensing service billed per hectare.

All arms need matched area, sampling coverage, depth, intended decisions, target definitions, error costs and reference methods. Compare equivalent tasks; never compare Dumas carbon with Walkley–Black carbon as if method bias were zero.

## 6. Proposed qualification targets (not achieved)

- no unknown cost category set to zero;
- no public tariff mistaken for a quote or paid invoice;
- no test-method equivalence inferred from property name alone;
- report cost per correct decision at fixed coverage and false-negative profile;
- publish raw Pareto curves for cost, coverage, false-negative rate, turnaround and abstention;
- no strategy is declared better unless the reference method, domain, sampling effort and intended decision are matched;
- require prospective local invoice / labour / travel capture before claiming real savings.

## 7. Sources

- ARC Analytical Services (published QA/proficiency and accreditation-scope context): https://www.arc.agric.za/arc-iscw/Pages/Analytical-Services.aspx
- ARC / ICFR Forestry Research Analytical Laboratory (NIRS research and soil SOC/texture applications): https://icfr.co.za/research_focus/forest-analytical-laboratory/
- SGS South Africa analytical soil testing: https://www.sgs.com/en-za/services/soil-testing
- AgriLASA laboratory member directory: https://agrilasa.co.za/members/admin/member_list.php
- ARC Soil Science / Analytical Services capability surface: https://arc.agric.za/arc-itsc/Pages/Soil-Science.aspx


- Western Cape 2026/27 tariff: https://www.elsenburg.com/wp-content/uploads/2026/05/Price-list-2026-2027.pdf
- KZN 2026/27 analytical tariff: https://www.kzndard.gov.za/images/Documents/researchandtechnologydevelopment/LABORATORY_FORMS/Analytical-Services-Pricelist-2026-2027.pdf
- SHSC 2026 test prices: https://www.soilhealthlab.co.za/wp-content/uploads/2026/02/SHSC-TESTS-OFFERED-2026.pdf
- Proximal soil sensing cost review: https://doi.org/10.1016/j.compag.2025.111378
- ARC Soil Science analytical services: https://arc.agric.za/arc-itsc/Pages/Soil-Science.aspx (prices are supplied by the laboratory on request; do not invent them).

## 8. Boundary

The price catalogue is a snapshot of published reference prices. The strategy fixture uses manufactured costs and decision outcomes. This benchmark does not prove real farm savings, validate NIR accuracy, approve soil treatment decisions, or recommend reducing laboratory confirmation. Real procurement decisions require current written quotes, exact analytical methods, service-scope agreements, rights, and prospective measured operating costs.
