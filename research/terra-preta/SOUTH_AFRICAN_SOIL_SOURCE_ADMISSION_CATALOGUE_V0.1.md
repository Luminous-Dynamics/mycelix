# South African soil-source admission catalogue v0.1

**Status:** source discovery and qualification matrix only. No listed dataset is admitted as farm/plot ground truth by this document. No source files were acquired, digested or modelled here.

## Governing rule

A soil layer is usable only for the spatial, depth, time and property support it actually measures or predicts. A national polygon, continental point survey, modelled raster, pedotransfer estimate, laboratory result and plot sensor are not interchangeable evidence.

Every source is classified before data fusion:
- **Measured observation:** field/laboratory value tied to an identifiable sampling unit, date, depth, method and reference basis.
- **Cartographic inventory:** a mapped soil class / landtype / association; useful for stratification and prior context, not a direct numerical observation at every map pixel.
- **Digital soil map prediction:** an estimated property with model provenance, prediction support and uncertainty.
- **Pedotransfer-derived estimate:** an indirect estimate from soil class or other inputs; never labelled as directly measured.
- **Spectral model prediction:** derived from an instrument spectrum plus a calibration model; preserve instrument, model, preprocessing, domain and reference-method provenance.
- **Historical / legacy record:** preserve original date, method, location accuracy and provenance; do not treat as current merely because it is accessible.

## Source candidates

| Candidate | What it contributes | Native support / key boundary | Intended role |
|---|---|---|---|
| **AGIS / Soil MapServer** (Department of Agriculture) | National layers: modal profiles, generalized soil patterns, landtype, SOTER associations, water-holding capacity, natural organic carbon/pH/CEC/P status, leaching, physical constraints, erosion and conservation. | The parent REST service exposes 37 sublayers with WGS84 service coordinates. Individual sublayer metadata must be queried for vintage, geometry, attributes, scale, units, custodianship and licence; parent service does not establish a common scale. Examples include dated layers from 1992–2007. | National/regional stratification and source discovery. Never treat a mapped polygon as a contemporary plot assay. |
| **Soils4Africa field-survey release** (Zenodo 17699297, v1) | Field observations from a continental campaign; the 55-column field table includes coordinates, survey date, plot/TSU IDs, country, erosion, landform, slope, vegetation, crop type, field size, tillage, residue/manure/inorganic inputs, irrigation and grazing context. | Published 26 June 2026. `s4a_field_data.csv` is 7.4 MB, publisher MD5 `23416634c2a7ae89e715980f77021229`; its 5.6 kB tableschema has MD5 `4cdfe065de45814241fff7e09a35a405`. Zenodo describes over 15,000 locations across 33 countries; the 2026 framework paper reports 24,951 soil samples from 14,311 locations across 33 countries. The Zenodo README gives 2020-06-01 to 2025-05-31, while the paper describes the coordinated campaign as 2022–2025: preserve source-specific date statements until row-level dates are audited. Quality control is ongoing and further versions are expected. Zenodo rights metadata is not explicit; the Soil Data of Africa portal points to a data licence agreement for point observations. | Contextual field observations and sampling-frame data for strata, management and site effects. The field table schema does not itself establish that each row has reference chemistry; do not treat it as a soil-property assay or field/plot prescription. |
| **Soils4Africa MIR-derived soil-property estimates** (announced 5 Oct 2026) | Announced 19 soil-property estimates derived from MIR spectra for more than 11,000 soil samples at over 6,100 locations in 21 countries; separate pesticide-residue data cover 272 locations. | Separate from the Zenodo 17699297 field-context file. The exact versioned Zenodo record for the MIR-derived property product was not pinned in this review; locate and inspect its schema, calibration/reference lineage, licence and exact artifact before admission. The announcement explicitly describes MIR-derived estimates, not necessarily direct wet-chemistry measurements. Further locations and properties were still being processed. | Candidate for cross-country spectral-domain / transfer research and contextual priors after exact record and model lineage are acquired. Never use MIR-derived outputs as independent wet-chemistry truth until calibration targets and validation basis are known. |
| **Soil Data of Africa gridded products** | GeoTIFF maps for texture, bulk density, organic carbon and chemical properties, derived from field observations using spatial statistics and machine learning; the portal reports a per-pixel accuracy estimate. | Separate from point observations and MIR-derived sample estimates. Preserve native grid resolution, spatial support, model/training lineage and pixel-level uncertainty. The portal recommends national-to-continental application and discourages sub-national / field-level use. | Regional context and broad stratification only; map pixels remain model predictions, not direct plot measurements. |
| **ARC-ISCW Soil Information System** | National soil profile and map custodianship, landtype and soil survey archives. | A published ARC brochure (2014-era) describes 19,000 profile records, landtype/climate zone information and thousands of maps/reports. Those counts are historical descriptions, not a live current inventory. Current record-level metadata, availability, access terms and update dates must be requested from the custodian. | Potential primary source for measured historical profiles and detailed survey records after source-native admission. |
| **SOTER South Africa / SOTERSAF / SOTWIS** | Soil-terrain units and profile-linked estimates for organic C, N, pH(H2O), CEC, texture, bulk density, available water capacity and other properties. | Regional mapping at about 1:1M or 1:2M depending on product. Some profile attributes are missing and filled using taxonomy-based pedotransfer procedures. Distinguish measured values from filled/derived values in the source. | Regional stratification / broad priors, not farm-scale ground truth. |
| **Digital soil map comparisons for South Africa (WRC report)** | South African comparison of SWAT-SA, iSDA, Africa SoilGrids 250m, HWSD and DSSAT-oriented maps against measured observations from selected studies/soil types. | Performance differs by product, region, parameter and soil type. The report explicitly discusses the limitations of global maps for local South African soil-landscape relations. | Benchmark design and local-model motivation; not a blanket endorsement of any one map. |
| **Western Highveld paired NIR/MIR v2 (Kock & Van Zijl 2026)** | 790 composite cultivated topsoil samples (0–15 cm) from one 2020 winter season; pH(KCl), Bray-1 P, exchangeable K/Ca/Mg/Na and paired NIR/MIR. | Only 178 points have released WGS84 field coordinates. Article/Zenodo version-link discrepancy and raw-reflectance vs converted-absorbance semantics remain explicit; v2 CSV not yet downloaded here. | Highveld local spectroscopy / metrology seed, not a field-treatment dataset or whole-Highveld map. |
| **Western Highveld MIR 2024 cohort** | Published local MIR calibration results on a 979-sample cohort after sample-name reconciliation. | Exact and semantic cohort overlap with the 2026 790-sample release is unresolved until a reviewed crosswalk is available. | Published comparator and potential shared-sample crosswalk, not independent validation by default. |
| **NeoSpectra handheld NIR spectral library v1.2** (Zenodo 13122321) | 2,106 mineral-soil samples scanned across nine handheld NeoSpectra scanners; includes 2,016 US samples and 90 African samples from Ghana, Kenya and Nigeria, with six replicate scans in the replication data and model products for eight soil properties. | Release v1.2 published 29 July 2024. NIR range 1350–2550 nm. The 1,976-US-sample subset was used for Cubist model calibration; paired MIR spectra are available for KSSL-linked samples. The African subset is small and excludes South Africa, so it is a transfer challenge seed, not broad African validation. Keep sample/device replicates grouped. | Device-to-device repeatability, scanner holdout, NIR calibration transfer and external-domain comparison. Evaluate simple local baselines and explicit abstention before claiming portable instrument generalization. |

## Source locators

- AGIS/Soil REST service: https://ndagis.nda.agric.za/arcgis/rest/services/AGIS/Soil/MapServer
- Soil Data of Africa: https://africasis.isric.org/
- Soils4Africa field-survey release v1: https://doi.org/10.5281/zenodo.17699297
- Soils4Africa field data / point and map downloads: https://africasis.isric.org/download.html
- Soils4Africa MIR-derived 19-property announcement: https://isric.org/news/soils4africa-makes-11-000-soil-data-points-from-21-african-countries-available/
- Soils4Africa 2026 field campaign framework: https://doi.org/10.3390/soilsystems10070079
- NeoSpectra handheld NIR library v1.2: https://doi.org/10.5281/zenodo.13122321
- Soil Data of Africa procedures: accessible from the live portal's Procedures area.
- ARC-ISCW soil information note: https://www.arc.agric.za/arc-iscw/News%20Articles%20Library/Soil%20Information%20at%20the%20ARC.pdf
- WRC, *Digital soil maps for South Africa*: https://www.wrc.org.za/wp-content/uploads/mdocs/3121%20final.pdf
- SOTER South Africa dataset description: https://datahub.moa.gov.et/data/datasets/soil-and-terrain-database-soter-for-south-africa
- SOTWIS Southern Africa dataset description: https://data.moa.gov.et/dataset/soter-based-soil-parameter-estimates-sotwis-for-southern-africa
- 2026 Western Highveld Data in Brief article: https://doi.org/10.1016/j.dib.2026.113160
- Zenodo v1: https://zenodo.org/records/17510653
- Zenodo v2: https://zenodo.org/records/21640671
- 2024 local MIR comparator: https://doi.org/10.1111/ejss.70014

## Admission contract for each source artifact

Before a source joins the Mycelix evidence graph or a Sol-Atlas layer, record:

1. **Identity:** source ID, exact canonical URI, custodian, dataset/product name, immutable version/date, download timestamp and SHA-256 of exact bytes.
2. **Rights:** explicit file/data licence, purpose limitation, redistribution rights, access conditions, consent or public authority where relevant. Article open access does not imply raw-data redistribution rights.
3. **Support:** CRS / datum, native geometry, map scale / raster resolution, pixel size, point positional accuracy, depth/horizon interval, temporal window and aggregation unit.
4. **Semantics:** measured / mapped / predicted / pedotransfer / spectral prediction / historical class, variable name, unit, analytical method, extraction/reference basis and transformation history.
5. **Uncertainty:** analytical, sampling, model-prediction, spatial-support and temporal-representativeness uncertainty, plus known systematic limitations.
6. **Coverage:** spatial/time/domain coverage, missingness, sampling design, class distribution and known selection bias.
7. **Dependencies:** originating study, sample identity, field/plot/batch/instrument/calibration, upstream model and source products; explicit independence groups to prevent leakage.
8. **Qualification:** structural, source-semantic, reference-method, model-validation and intended-use status kept as separate fields.
9. **Change control:** a new source version creates a new immutable artifact; no silent overwrite or substitution.

## Soils4Africa and NeoSpectra: separate products, distinct evidential roles

The current public source surface exposes at least four non-interchangeable Soils4Africa products:

1. **Field-observation table:** Zenodo record 17699297, v1, includes a field-data CSV and a 55-column table schema. The schema describes coordinates (EPSG:4326), survey date, country and plot/TSU identifiers, landform/slope, erosion, vegetation, crop type, field size, tillage, residue/fertiliser inputs, irrigation and grazing. It is valuable for context and site/management covariates; it is not automatically an analytical chemistry table.
2. **MIR-derived sample estimates:** ISRIC announced 19 estimates derived from MIR spectra for more than 11,000 samples at over 6,100 sites in 21 countries on 5 October 2026. The exact versioned Zenodo artifact was not pinned in this review. Until calibration/reference lineage is recovered, these are model-derived values, not independent wet-chemistry truth.
3. **Gridded soil-map products:** distinct GeoTIFF prediction grids derived from field observations using spatial statistics / machine learning, with per-pixel accuracy estimates.
4. **Pesticide-residue data:** a separate release describing residues from 272 locations; keep it distinct from soil fertility chemistry and assess its own rights / analytical basis.

The campaign counts also have different denominators: the Zenodo field-data description says over 15,000 locations across 33 countries; the 2026 framework paper reports 24,951 samples from 14,311 locations across 33 countries. Its README describes the broader project period as 1 June 2020–31 May 2025, while the framework paper describes the coordinated field campaign as 2022–2025. Do not replace one count or date range with another without auditing sample, plot and survey identifiers.

The Soil Data of Africa portal recommends national-to-continental scale application, not sub-national / field-level use. It also points to a data licence agreement for point observations and separate licences for catalogue products. Zenodo's rights metadata for record 17699297 was not explicit in the record view reviewed; verify the exact data-use agreement before redistribution.

A complementary NIR-transfer candidate is the NeoSpectra library v1.2 (Zenodo 13122321): 2,106 mineral-soil samples scanned across nine handheld scanners, including 90 African samples from Ghana, Kenya and Nigeria. The release documents six replicate scans per sample/device; a 1,976-US-sample subset was used to train Cubist models for eight properties. This can support scanner-to-scanner and geography-transfer research, but the African subset is too small and geographically narrow to establish pan-African or South African transfer. Group replicates by physical sample and hold out entire scanners / sites before claiming transfer.

These sources make a useful nested benchmark possible: global / continental context → regional spectral calibration → Highveld / South African local calibration → field-specific referral and validation. At each step, evaluate whether transfer holds, what additional local labels are needed, and when the model must abstain. Do not let MIR-derived estimates or digital-map pixels masquerade as independent lab assays.

## How the source classes should enter the model

- Use AGIS / SoilGrids / iSDA / SOTER / Soils4Africa for *stratification*, gap analysis, candidate soil domains and spatially broad priors, subject to native support.
- Use ARC profile records, university datasets and laboratory/field studies as empirical observations only after method, date, depth, unit, sampling and rights audits.
- Use the Highveld paired NIR/MIR dataset to compare instrument/property calibration, not to infer untreated/treatment effects.
- Use local soil and hydrology measurements for site-specific validation; external map agreement cannot replace independent field sampling.
- Any digital soil map used for training may not also be a nominally independent test label unless its training-source lineage is known and shared source points are excluded.
- Do not upsample or interpolate broad products to parcel/plot resolution and imply added information. Maintain native support in maps and calculations.
- When two products disagree, retain both with source, support and uncertainty; do not average them into an artificial consensus.

## Benchmark design

Three distinct evaluations are required:

1. **Regional mapping:** whole-region / spatial-block validation with source and survey lineage held out; report map-scale appropriate uncertainty, calibration and spatial transfer.
2. **Local farm/plot transfer:** independent samples, exact depth and reference methods, field/plot assignment groups and prospective measurements; broad national/continental products are priors, not the local reference.
3. **Spectral transfer:** hold out site/field/batch/instrument/season and cohort identities; report per-property errors, reference-method uncertainty, calibration-transfer loss, applicability-domain detection and confirmatory referral cost.

Compare with the best source-native/local product, not simply the most convenient global raster. Report property-specific errors, bias, interval coverage/width, spatial error and subgroup transfer. No one scalar R² should hide a failure in phosphorus, potassium, a soil class or a local stratum.

## Admission boundary

This catalogue records source discovery. It does not say that a dataset is downloaded, licensed for redistribution, complete, fit for field-level use or qualified for an agronomic decision. No source-specific SHA-256 is claimed until exact source bytes are acquired and hashed.
