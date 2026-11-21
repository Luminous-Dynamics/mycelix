# Western Highveld NIR/MIR release reconciliation — 2026-11-21

**Status:** source-metadata reconciliation only. The CSV bytes have not been acquired in this environment; no actual row audit, SHA-256, model fit or calibration result is claimed.

## Canonical article

Kock, A.-L. & Van Zijl, G. (2026), *A soil property and spectral near- and mid-infrared dataset from cultivated fields sampled in winter 2020, Western Highveld, South Africa*, Data in Brief 68, 113160. DOI: https://doi.org/10.1016/j.dib.2026.113160. Open full text: https://pmc.ncbi.nlm.nih.gov/articles/PMC13543106/

The article describes 790 composite cultivated-grain-field topsoil samples, 0–15 cm, collected in one 2020 winter sampling season from NWK/GWK routine fertility submissions. It includes pH(KCl), Bray-1 P and exchangeable K/Ca/Mg/Na with paired NIR and MIR spectra. The stated chemistry methods are:
- pH: 1:2.5 soil : 1 M KCl.
- P: Bray-1 extraction with colorimetry.
- K, Ca, Mg, Na: 1 M ammonium acetate at pH 7.

The article reports one NeoSpectra handheld FT-NIR scan of 15 seconds per sample over 1350–2500 nm and Bruker Alpha II DRIFT MIR at 2 cm⁻¹ over 3997–600 cm⁻¹ with 30 co-added scans. Thirteen flat/blank MIR spectra were removed before the 790-row release. WGS84 coordinates were released for 178 samples only; all other coordinates remain unavailable and must not be inferred.

## Version reconciliation

The article's data-availability section links to Zenodo record 17510653, published as v1, containing 803 sample rows and file MD5 `b13d51f905f22a3c15adbec11e8a733b`.

The Zenodo v1 page now identifies a newer version. The latest version page is Zenodo record 21640671 (v2), DOI https://doi.org/10.5281/zenodo.21640671, published 28 July 2026, containing the same filename `NWU_NIR_MIR_Dataset.csv`, 790 sample rows, 19.6 MB, and publisher MD5 `c351dca52863991d7e862182464d0e9c`.

These are distinct immutable release artifacts:
- v1 / record 17510653 / 803 rows / MD5 `b13d51f905f22a3c15adbec11e8a733b`.
- v2 / record 21640671 / 790 rows / MD5 `c351dca52863991d7e862182464d0e9c`.

The current profile pins v2. Do not silently replace it with v1 or assume the 13-row difference is the only possible change. Download the exact version, verify publisher MD5 as a transport check, and compute SHA-256 locally before running the structural auditor. No SHA-256 is recorded in this change because the 19.6 MB CSV was not retrievable here.

Zenodo v1 and v2 pages:
- https://zenodo.org/records/17510653
- https://zenodo.org/records/21640671

## Spectral value semantics: source conflict retained

The article's accessible text does not provide one internally consistent description:
- The specifications describe NIR as reflectance across 1350–2500 nm.
- The spectral-column discussion says all spectral columns are absorbance, but separately calls the NIR block reflectance percentage.
- The acquisition method states that the handheld instrument recorded reflectance (R).
- The data-merging section describes wavelength-to-wavenumber conversion for raw NIR reflectance, while the technical-validation figure caption describes converted NIR absorbance.
- The accessible text does not state an exact reflectance-to-absorbance transformation equation or provide enough detail to reproduce all preprocessing steps unambiguously.

Accordingly, the v0.2 profile labels value semantics as unresolved. It records the axis conversion (nm to cm⁻¹) separately from any intensity transformation. Do not infer that wavelength-to-wavenumber conversion also transforms reflectance to absorbance. Do not apply (-log_{10}(R)), (log_{10}(1/R)), percent-to-fraction scaling, baseline correction, or any other transform until the delivered CSV values and source processing code / author clarification establish what was actually applied.

The structural auditor should preserve the values as delivered, report all numeric/axis and missingness properties, and carry a warning about the unresolved semantics. Model results should be embargoed from confirmatory comparison until the transform interpretation is resolved or an explicit raw-reflectance track is analysed separately.

## License and redistribution

The Data in Brief article is CC BY 4.0. This does not automatically establish the licence for the underlying Zenodo CSV. The Zenodo versioned page displayed no explicit licence in the accessible metadata lines during this review. Keep the current profile's license state conservative and confirm the exact Zenodo file licence before mirroring or redistribution. Public availability is not equivalent to a redistribution right.

## Benchmark implications

This release is a useful local metrology bridge, not an external field-effect dataset:
- The sample population is commercial cultivated topsoil, not natural soil profiles or representative all-land-use Highveld coverage.
- All samples share one 2020 winter collection window; there is no repeated-season estimate from this dataset.
- NIR and MIR are paired on the same soil samples, enabling within-sample modality comparison, but this is not independent validation.
- A single 15-second NIR scan versus 30 co-added MIR scans creates an acquisition precision / throughput comparison, not a clean causal instrument comparison unless scan replication and preparation effects are modelled.
- Only 178 samples are georeferenced; do not infer spatial autocorrelation structure or apply a spatial split to all 790 rows unless reliable spatial grouping metadata exists.
- The 2024 local MIR paper used a 979-sample cohort after sample-name reconciliation from 1,000 selected samples. Exact and semantic crosswalks must be reviewed before any shared sample identities are declared independent.

Recommended evaluation sequence:
1. Freeze the v2 file digest and profile.
2. Audit sample IDs, duplicate identifiers, reference chemistry columns, spectrum lengths, axis monotonicity, boundary gaps, missingness and source-value semantics.
3. Reconcile exact and semantic identity against the 2024 MIR cohort under a source-reviewed crosswalk.
4. Reproduce simple property-specific local MIR baselines first (PLSR, Cubist, RF where article settings/data support reproduction).
5. Compare paired NIR-only, MIR-only and fusion under identical grouped sample splits.
6. Use whole-field/group holdouts only where trustworthy IDs exist; otherwise label the strongest available grouping and limitations.
7. Report bias, MAE/RMSE, proper predictive scores, interval coverage/width, applicability-domain detection and decision utility by property—not one overall R².
8. Keep reference wet-chemistry uncertainty in the model/error budget; do not assume the laboratory target is error-free.

## Source record

- Article record: https://pmc.ncbi.nlm.nih.gov/articles/PMC13543106/
- Article DOI: https://doi.org/10.1016/j.dib.2026.113160
- Zenodo v1: https://doi.org/10.5281/zenodo.17510653
- Zenodo v2: https://doi.org/10.5281/zenodo.21640671
- 2024 local MIR comparator: https://doi.org/10.1111/ejss.70014
- Related Mycelix track: #6414
