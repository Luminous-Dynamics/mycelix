# SOC stock accounting and equivalent-soil-mass benchmark v0.1

**Status:** deterministic synthetic accounting primitive. This is not a carbon-credit calculator, approved MRV methodology, or net-removal estimate.

## Scientific and standards context

FAO's Global Soil Organic Carbon MRV Protocol explains that fixed-depth stocks can be biased when bulk density changes between treatments or over time. It recommends equivalent soil mass (ESM) normalization. Verra's current VM0042 v2.2 is active (21 October 2025) and has corrections/clarifications effective 11 June 2026. Verra's proposed VM0042 v3.0 revision is still in methodology development; it is not an active approved standard. The proposal adds cumulative accounting, repeated-measurement/model-error true-up, and more standardized ESM, sampling, and laboratory guidance.

Wadoux et al. (2026), DOI 10.1038/s44264-026-00125-0, demonstrate a digital-soil-mapping MRV workflow on a hypothetical seven-field US project. They report cross-validation RMSE 10.56 Mg C/ha, R² 0.61 and PICP 91%, and illustrate the importance of spatial residual correlation and uncertainty deductions. These are study-specific figures, not South African targets. The transferable lesson is to validate both point predictions and uncertainty, model spatial covariance, and carry uncertainty through aggregation and change estimation.

## Physical definitions

The input explicitly defines:
- fine-earth bulk density as oven-dry fine-earth mass per fine-earth volume, excluding coarse fragments (kg/m³ fine-earth volume);
- coarse-fragment fraction as coarse-fragment volume divided by total core volume;
- SOC concentration as g C per kg dry fine earth.

For layer thickness dz (m), fine-earth bulk density rho (kg/m³) and coarse-fragment volume fraction f:

fine-earth mass (Mg/ha) = rho × dz × (1 - f) × 10

SOC stock (Mg C/ha) = fine-earth mass × SOC concentration (g/kg) / 1000

The factor 10 converts kg/m² to Mg/ha. The coarse-fragment correction is applied once. Do not apply it again if the supplied density has already been corrected for coarse fragments.

## Fixed-depth and ESM

- **Fixed-depth stock:** integrate layer contributions to the declared accounting depth, taking a proportional part of the terminal layer if necessary.
- **ESM reference mass:** calculate baseline fine-earth soil mass to the declared reference depth. Freeze that mass as the comparison target.
- **ESM follow-up stock:** integrate follow-up layers until exactly the baseline reference fine-earth mass is reached. If the mass boundary falls within a layer, v0.1 assumes that layer's SOC concentration and density are uniform through the layer and takes the corresponding proportional layer depth.
- **Insufficient follow-up mass:** abstain. Do not extrapolate below measured profile depth.

Report fixed-depth and ESM values side by side. A density change can produce different stock-change conclusions under the two bases; that difference is informative and must not be hidden.

## Separate ledgers and attribution

The output reports:
1. observed SOC stock change at fixed depth;
2. observed SOC stock change at equivalent fine-earth soil mass;
3. separately calculated applied amendment-carbon input, if declared.

Amendment input is not retained soil carbon. It is never added to observed SOC stock or subtracted from native SOC automatically. Without isotope/tracer or other qualified source-attribution evidence, attribution between native SOC and amendment-derived carbon remains unresolved.

This auditor does not calculate net CO2e removal, N2O/CH4 emissions, leakage, additionality, reversal risk, permanence, project baselines, issuance quantities or carbon credits.

## Evidence requirements

- two repeated profiles from the same declared plot / experimental unit;
- chronological baseline and follow-up dates;
- same SOC and reference method in v0.1;
- exact profile source digest, lab batch identity and depth layers;
- contiguous layers beginning at 0 m, with no gaps/overlaps;
- positive fine-earth density, non-negative SOC and coarse-fragment fraction in [0,1);
- analytical standard errors retained in input for future uncertainty work but **not propagated by v0.1**.

## Future MRV comparison

A production MRV system must compare design-based probability sampling / remeasurement, measure-and-model biogeochemical approaches, DSM/spatial ML, joint spatial-temporal models with cross-time error correlation, and hybrid observation assimilation with independent physical/lab verification.

Report bias, RMSE, interval coverage/width (PICP and proper scores), spatial residual covariance, field/project aggregate variance, baseline/follow-up error correlation, ESM conversion uncertainty, and measurement cost. Split by whole field/site/soil group and exclude samples used to fit map products from nominally independent validation.

## Proposed targets (not achieved)

- analytic mass/stock cases within floating-point tolerance;
- zero ESM reports when follow-up fine-earth mass cannot reach the frozen baseline reference mass;
- zero fixed-depth / ESM semantic substitution;
- zero coarse-fragment double correction;
- later calibrated nominal 90% change intervals target 85–95% coverage on independent field/site holdouts;
- report full sampling, lab and modelling cost per resolved Mg C/ha change—not just point-level R².

## References

- FAO, *Global Soil Organic Carbon MRV Protocol*: https://www.fao.org/fileadmin/user_upload/GSP/MRV/GSP_MRV_Protocol_A4_EN_002.pdf
- Verra VM0042 v2.2 (active): https://verra.org/methodologies/vm0042-improved-agricultural-land-management-v2-2/
- Verra M0264 proposed v3.0 revision (under development): https://verra.org/methodologies/revision-to-vm0042-methodology-for-improved-agricultural-land-management/
- Wadoux et al. (2026), *A digital soil mapping approach to soil carbon monitoring, reporting and verification (MRV)*, DOI https://doi.org/10.1038/s44264-026-00125-0.

## Boundary

All fixtures are synthetic. The implementation does not meet or claim compliance with VM0042, FAO or any registry methodology, and it does not issue credits or determine net climate benefit. Local methods, sampling, reference procedures, rights and independent validation must be qualified for the intended use.
