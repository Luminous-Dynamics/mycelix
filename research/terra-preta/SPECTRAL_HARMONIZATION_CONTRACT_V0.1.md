# Source-aware soil spectral harmonization contract v0.1

**Status:** synthetic metadata / transform-chain auditor. No OSSL or Highveld source file is acquired or harmonized by this fixture.

## Why harmonization must be explicit

Open Soil Spectral Library v1.3 (Zenodo 23815431, released 30 October 2026) contains 20 source datasets and distributes per-source archives with manifest checksums; it does not currently provide one harmonized cross-dataset table. v1.2 and v1.3 differ in source inventory, KSSL coverage and dataset codes. A global ML table cannot safely assume all source property columns and spectra have identical meaning.

**Preserve native source data.** A harmonized model view is a derived artifact with its own operations, code/config digest, output support and applicability domain. Never replace source measurements with the transformed view.

## Four independent compatibility questions

1. **Spectral representation:** reflectance, absorbance, derivative, normalized or other. The Highveld article describes reflectance acquisition but a converted absorbance representation in the merged file; do not transform again without evidence from the pinned file.
2. **Spectral axis:** wavelength / wavenumber units, range, direction, native resolution and observation operator. Resampling does not change sensor support; never extrapolate outside the acquired range.
3. **Target semantics:** property definition, unit, analytical reference method, extraction chemistry, sample depth and uncertainty. Identical property/column names do not imply analytical equivalence.
4. **Sample independence:** physical sample, source dataset, field/site, survey, batch, instrument, calibration, replicate scan and derived-row lineage. All replicates and cross-library copies of the same physical sample must stay in one split group.

Spectral compatibility and target compatibility are separate. Two sources may share a target method but have different native spectral axes, or share similar spectra while their analytical targets are not comparable.

## Transformation chain

Each operation carries ordered sequence, operation ID, operation family, input/output representation and axis, parameters, code/config digest, software version, lossy flag and extrapolation policy. The chain is continuous. The final state must match the declared analysis view.

V0.1 recognizes reflectance→absorbance (one time only), resampling inside native range, smoothing, first/second derivative, SNV and MSC. An absorbance source cannot receive a reflectance→absorbance transform. No chain may create coverage outside the native source range.

## Target-comparison graph

Each target includes property, definition, unit, depth interval, analytical method ID and extraction/reference basis. A direct-comparability edge is accepted only when these semantics match exactly. A bridge-required edge needs separate validation dataset/model digests, paired sample count and independent validation-group count. Otherwise mark the mapping non-comparable or unresolved and keep separate model heads.

## Sample identity / split grouping

Each row maps to a physical sample group and split group. Every row, replicate scan and cross-library copy sharing a physical sample group must share the same split group and split label. A split group itself cannot span train/validation/test. Duplicate row IDs are rejected.

## Benchmark design

Compare native within-source PLSR / Cubist / RF baselines; source-held-out without harmonization; source-held-out with an explicit common spectral view; source-held-out with a qualified target-method bridge; global pretraining with small local calibration; local-only calibration and abstention. Pin exact OSSL v1.2/v1.3 releases, per-source manifests, sample lineage and common property targets. Report per-property MAE/RMSE/bias, interval coverage/width, calibration-transfer loss, applicability-domain coverage, method-transfer bias and cost. PLSR remains a required baseline.

## Qualification targets (not achieved)

- Zero double-transform and zero wavelength extrapolation.
- Zero direct-comparability claims across method/unit/depth mismatches.
- Zero physical-sample or replicate leakage across splits.
- Every transformed view replayable from source and operation digests.
- Report source exclusions, unresolved target bridges and applicability-domain abstention.
- A pooled model cannot qualify if target semantics or source rights are unresolved.

## Sources

- OSSL v1.3, Zenodo 23815431: https://doi.org/10.5281/zenodo.23815431
- OSSL v1.3 update / per-source files and checksums: https://soilspectroscopy.org/update-2026/
- OSSL data access / per-source manifest: https://docs.soilspectroscopy.org/db-access.html
- Western Highveld paired NIR/MIR Data in Brief (2026): https://doi.org/10.1016/j.dib.2026.113160
- African soil spectroscopy review (2026): https://doi.org/10.3390/environments13040228

## Boundary

All source and row identifiers in the fixture are synthetic. This is not an OSSL import, Highveld artifact audit, spectral preprocessing implementation or model benchmark result. Real admission requires exact archive bytes, manifest digest verification, licence review and native file inspection.
