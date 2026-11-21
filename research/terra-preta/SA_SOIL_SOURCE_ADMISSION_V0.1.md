# South African soil-source admission contract v0.1

**Status:** executable metadata qualification contract with synthetic examples only. No live source bytes are included and no source is admitted to field/plot decision use here.

## Scientific premise

A regional map, a soil survey point, a lab reference, a spectral model, and a pedotransfer estimate have different observation processes and spatial supports. Their values must never be silently harmonized into one interchangeable "soil truth" table.

This contract is designed for the South African candidate-source catalogue, including AGIS, Soil Data of Africa / Soils4Africa, ARC-ISCW profile archives, SOTER/SOTWIS, South African digital-soil-map products, and the Western Highveld NIR/MIR cohort.

## Evidence classes

| Value semantics | Meaning | Permitted language |
|---|---|---|
| `directly_measured` | A measured sample / field / laboratory observation with a method, unit, support and reference basis. | Measured value under stated method; uncertainty and representativeness remain explicit. |
| `mapped_class` | A cartographic polygon/class derived from survey or map interpretation. | Mapped class at native mapping support, not a point assay. |
| `digital_map_prediction` | A modelled grid/cell/polygon estimate. | Predicted value under the named model/version and support. |
| `pedotransfer_estimate` | A derived value inferred from other soil data / taxonomic class. | Pedotransfer estimate, not direct measurement. |
| `spectral_prediction` | Prediction from a spectrum and calibration model. | Model prediction for the instrument, preprocessing and applicability domain. |
| `other_derived` | Other explicitly described transformation. | Derived quantity with full lineage. |

Dataset-level source type does not replace per-variable semantics. A point survey can contain direct measured chemistry and later predictions; these are separate value records with separate lineage.

## Native support is non-negotiable

- **Point samples** require coordinate reference system and positional uncertainty / accuracy or an explicit unknown state.
- **Soil profiles** require horizon / depth semantics and vertical support.
- **Polygons / regional units** require native mapping scale or equivalent support description.
- **Raster products** require CRS and native cell size / resolution.
- Map scale or pixel size is never improved by upsampling a layer in a visualization.
- A map polygon does not become a field observation because a query point intersects it.
- Unknown depth support is retained as unknown, not silently labelled 0–15 cm.
- Time support / collection vintage is explicit. Historic map layers do not become current measurements.

## Rights and purpose

Public availability is not automatically a right to redistribute. The record separately tracks licence review, access terms, intended uses and release permissions. A pending / unknown licence blocks public redistribution. A data access agreement can permit internal research use without allowing republishing the raw file.

## Acquisition / digest state

- `not_acquired`: only metadata is known; no byte-level claim is made.
- `publisher_checksum_only`: publisher checksum captured; local SHA-256 not yet verified.
- `sha256_verified`: exact bytes acquired and a lowercase SHA-256 digest calculated.

A metadata profile must not invent a SHA-256 for an unavailable file.

## Dependency / leakage lineage

Record study, survey, site, field, sample, horizon, batch, instrument, calibration, lab and upstream model independence groups. A derived map trained on survey observations is not an independent external test against those same source samples. Split by connected source-dependency groups, not row IDs alone.

## Synthetic fixture

The examples have four manufactured records:
1. An AGIS-like polygon map inventory with mapped-class semantics.
2. A Soils4Africa-like field-observation table with direct field-observation semantics (e.g. slope position), not laboratory soil chemistry.
3. A distinct Soils4Africa-like MIR-derived soil-property estimate product with model-prediction semantics and unresolved calibration lineage.
4. A SOTER-like pedotransfer product with explicitly derived values and model lineage.

Names are analogues used to test metadata validation, not declarations about the real source files or live licences.

## Intended use / qualification

`regional_stratification` and `sampling_design` are separate from `local_validation`, `field_plot_inference` or `public_redistribution`. Metadata validation only proves that the package states its semantics consistently. It does not prove a source's scientific accuracy, currentness, completeness, representativeness, licence validity or suitability for a real agronomic decision.

## Sources to review per artifact

- AGIS/Soil REST: https://ndagis.nda.agric.za/arcgis/rest/services/AGIS/Soil/MapServer
- Soil Data of Africa: https://africasis.isric.org/
- ARC-ISCW soil information: https://www.arc.agric.za/arc-iscw/News%20Articles%20Library/Soil%20Information%20at%20the%20ARC.pdf
- South African digital soil maps review: https://www.wrc.org.za/wp-content/uploads/mdocs/3121%20final.pdf
- SOTER South Africa: https://datahub.moa.gov.et/data/datasets/soil-and-terrain-database-soter-for-south-africa
- SOTWIS Southern Africa: https://data.moa.gov.et/dataset/soter-based-soil-parameter-estimates-sotwis-for-southern-africa
- Western Highveld paired NIR/MIR article: https://doi.org/10.1016/j.dib.2026.113160
- Western Highveld Zenodo v2: https://zenodo.org/records/21640671

## Current Soils4Africa release distinctions

The 2026 public source surface describes separate products that must remain separate in the admission graph:

- **Field observations:** Zenodo 17699297 v1 (published 26 June 2026) includes a field-data CSV and its tableschema. The schema describes coordinates, survey date, plot/TSU identifiers, country, erosion, landform/slope, vegetation, crop type, tillage, residue/fertiliser inputs, irrigation and grazing context. It does not establish that this file is an analytical soil-chemistry table.
- **MIR-derived estimates:** ISRIC announced 19 soil-property estimates derived from MIR spectra for over 11,000 samples at over 6,100 locations in 21 countries (5 October 2026). The exact versioned data record was not pinned during this review. These values are model-derived estimates and require model/reference lineage before use.
- **Gridded map predictions:** separate GeoTIFF products derived from survey data with spatial-statistical / ML methods and per-pixel accuracy estimates.
- **Pesticide residues:** a separate product from 272 locations, with its own analytical and rights contracts.

The Zenodo field release describes more than 15,000 locations across 33 countries; the 2026 framework paper reports 24,951 soil samples from 14,311 locations across 33 countries. The README describes 1 June 2020–31 May 2025, while the paper describes the coordinated campaign as 2022–2025. These are source-specific counts/date descriptions that must be reconciled from sample/plot IDs and row dates, not silently replaced.

A complementary handheld NIR transfer dataset is the NeoSpectra library v1.2 (Zenodo 13122321): 2,106 soil samples across nine scanners, including 90 African samples from Ghana, Kenya and Nigeria. It is relevant for device and geographic transfer research, but the African subset is small and cannot support country-level or South African validation alone. Keep replicate scans grouped by physical sample and scanner in all splits.

## Boundary

This contract does not download, mirror, redistribute or independently qualify source files. It does not authorize farm access, sampling, input application, fertilizer recommendations or field/plot decisions.
