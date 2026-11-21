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
| **Soils4Africa / Soil Data of Africa** (ISRIC-hosted) | Georeferenced point data across African agricultural land; catalogue, manuals and sampling SOPs. | The live portal recommends national-to-continental application and explicitly discourages sub-national / field-level use. It also warns that completeness/accuracy is not warranted and that point data are subject to a data licence agreement; catalogue layers have their own licences. | Regional context, harmonized comparative prior and survey-design context; not field prescription. |
| **ARC-ISCW Soil Information System** | National soil profile and map custodianship, landtype and soil survey archives. | A published ARC brochure (2014-era) describes 19,000 profile records, landtype/climate zone information and thousands of maps/reports. Those counts are historical descriptions, not a live current inventory. Current record-level metadata, availability, access terms and update dates must be requested from the custodian. | Potential primary source for measured historical profiles and detailed survey records after source-native admission. |
| **SOTER South Africa / SOTERSAF / SOTWIS** | Soil-terrain units and profile-linked estimates for organic C, N, pH(H2O), CEC, texture, bulk density, available water capacity and other properties. | Regional mapping at about 1:1M or 1:2M depending on product. Some profile attributes are missing and filled using taxonomy-based pedotransfer procedures. Distinguish measured values from filled/derived values in the source. | Regional stratification / broad priors, not farm-scale ground truth. |
| **Digital soil map comparisons for South Africa (WRC report)** | South African comparison of SWAT-SA, iSDA, Africa SoilGrids 250m, HWSD and DSSAT-oriented maps against measured observations from selected studies/soil types. | Performance differs by product, region, parameter and soil type. The report explicitly discusses the limitations of global maps for local South African soil-landscape relations. | Benchmark design and local-model motivation; not a blanket endorsement of any one map. |
| **Western Highveld paired NIR/MIR v2 (Kock & Van Zijl 2026)** | 790 composite cultivated topsoil samples (0–15 cm) from one 2020 winter season; pH(KCl), Bray-1 P, exchangeable K/Ca/Mg/Na and paired NIR/MIR. | Only 178 points have released WGS84 field coordinates. Article/Zenodo version-link discrepancy and raw-reflectance vs converted-absorbance semantics remain explicit; v2 CSV not yet downloaded here. | Highveld local spectroscopy / metrology seed, not a field-treatment dataset or whole-Highveld map. |
| **Western Highveld MIR 2024 cohort** | Published local MIR calibration results on a 979-sample cohort after sample-name reconciliation. | Exact and semantic cohort overlap with the 2026 790-sample release is unresolved until a reviewed crosswalk is available. | Published comparator and potential shared-sample crosswalk, not independent validation by default. |

## Source locators

- AGIS/Soil REST service: https://ndagis.nda.agric.za/arcgis/rest/services/AGIS/Soil/MapServer
- Soil Data of Africa: https://africasis.isric.org/
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
