# Layer-resolved soil organic carbon stock / equivalent soil mass benchmark v0.1

**Status:** synthetic-only stock-calculation benchmark. It is not a carbon-credit method, local baseline, sequestration estimate or MRV certification.

## Why this matters

SOC concentration and SOC stock are different quantities. Concentration (for example, g C kg⁻¹ fine earth) describes a mass fraction. Stock (for example, Mg C ha⁻¹) requires the mass of soil represented per unit area. Compaction, tillage, swelling, erosion, deposition and coarse fragments can change the soil mass represented by a fixed depth.

FAO's GSOC-MRV protocol and ISO 23400 guidance emphasize soil-mass-aware monitoring, profile support, sampling design and uncertainty. A 2025 Geoderma study found fixed-depth / fixed bulk-density stock estimates can produce unacceptable errors and reported a 3.5 Mg ha⁻¹ minimum detectable change for its single-layer ESM method. A 2026 Geoderma empirical evaluation found that, in its two evaluated soil orders, a 30–35 cm correction layer substantially reduced systematic error when measuring 0–30 cm stocks. Rutjens et al. (2026) compared ESM10, ESMμ, EBD10 and EBDμ across nine eastern Australian farms; mean-based variants reduced variability by up to 38–86% relative to ESM10 in that dataset. None of these numerical results is a South African estimate or proof of one universal best variant.

## v0.1 data basis

Each core profile supplies contiguous layers from the surface. For each layer:

- `dry_soil_mass_megagrams_per_ha`: total oven-dry soil mass per hectare **including coarse fragments**;
- `coarse_fragment_mass_fraction`: dimensionless coarse-fragment **mass fraction of total dry soil mass** (0–1);
- `soc_grams_carbon_per_kg_fine_earth`: organic-carbon concentration in g C per kg fine earth.

Fine-earth mass is calculated as:

`fine_earth_mass = dry_soil_mass × (1 − coarse_fragment_mass_fraction)`

The coarse-fragment basis must be explicitly mass-based. A volume fraction cannot be inserted as though it were a mass fraction. v0.1 rejects an unknown or volume-only basis.

SOC concentration must represent organic carbon, either directly measured or total carbon corrected for inorganic carbon. Unassessed inorganic-carbon interference blocks stock calculation.

## Calculations

### Fixed depth (FD)

For the declared fixed depth, sum complete layers whose lower boundary equals that depth:

`SOC_stock_FD = Σ fine_earth_mass_i × SOC_i / 1000`

The factor 1000 converts g C kg⁻¹ to Mg C per Mg soil.

The v0.1 fixed-depth boundary must coincide exactly with a supplied layer boundary. It does not silently split or interpolate a layer.

### Equivalent soil mass (ESM)

For the declared reference fine-earth mass, walk from the surface downward. In each layer, consume the lesser of the layer's available fine-earth mass and the remaining reference mass. The layer SOC concentration is treated as piecewise constant across that supplied interval:

`SOC_stock_ESM = Σ consumed_fine_earth_mass_i × SOC_i / 1000`

If the observed profile does not contain the full reference fine-earth mass, the profile is rejected. No extrapolation beyond the measured profile is allowed.

## What this v0.1 does not do

- It does not estimate the reference mass from a field sampling design. The reference fine-earth mass is declared and digest-bound in the input.
- It does not implement ESM10, ESMμ, EBD10 or EBDμ target-selection procedures. Those must be compared as distinct estimators in a later multi-core MRV study; do not assume mean-based variants are unbiased everywhere.
- It does not propagate laboratory, coarse-fragment, core-sampling, layer-interpolation or shared-method covariance.
- Core-to-core standard deviation is reported as within-round subsample dispersion; cores are not treated as independent treatment plots.
- First-to-last differences are descriptive. No causal management effect, project baseline, permanence, leakage, reversal or credit quantity is estimated.
- Amendment-derived / pyrogenic carbon, native SOC and total carbon must remain separate ledgers in future work.

## Benchmark controls

The synthetic compaction-only fixture keeps layer SOC concentration constant and scales dry soil mass by 10% between rounds. The fixed-depth stock should rise by 10%, while the declared ESM stock should remain unchanged. This is a known-truth arithmetic control, not empirical evidence.

Other negative controls reject:
- coarse-fragment volume fraction relabelled as mass fraction;
- total carbon without inorganic-carbon correction;
- layer gaps / overlaps or inconsistent boundaries;
- fixed-depth boundaries not present in the layer profile;
- insufficient observed soil mass for the declared ESM target;
- changing experimental unit across temporal rounds.

## Scientific comparison required before field qualification

Compare FD, multi-layer ESM, ESM10, ESMμ, EBD10 and EBDμ where each method's data requirements are satisfied. Evaluate bias against known-truth synthetic profiles first, then independent measured profiles across soil orders, texture, mineralogy, compaction, stoniness and sampling depth. Report accuracy, variance, minimum detectable change, cost, and sensitivity to short cores / obstructions. Use whole farms / plots as validation groups.

## Sources

- FAO (2020), *GSOC-MRV Protocol*: https://www.fao.org/fileadmin/user_upload/GSP/MRV/GSP_MRV_Protocol_A4_EN_002.pdf
- ISO 23400:2021, *Soil quality — Guidelines for the determination of soil organic carbon and nitrogen stocks*: https://www.iso.org/standard/72423.html
- Boivin et al. (2025), *The single-layer equivalent soil mass method for the evaluation of soil organic carbon stocks: Sources of errors, simplification, and associated detectable change*, Geoderma 456, 117279. DOI: https://doi.org/10.1016/j.geoderma.2025.117279
- Empirical evaluation and recommendations for ESM adjustment (2026), Geoderma 469, 117801. DOI: https://doi.org/10.1016/j.geoderma.2026.117801
- Rutjens et al. (2026), *Alternatives to Equivalent Soil Mass in Monitoring, Reporting and Verification of Changes in Soil Carbon*. DOI: https://doi.org/10.1111/sum.70213
- Wuest (2025), *Measuring soil carbon stocks with greater simplicity, accuracy, and repeatability*. DOI: https://doi.org/10.1002/saj2.70012

## Boundary

Only manufactured values are included. No live samples or source files are included. This is not a South African carbon stock estimate, carbon-credit claim, project baseline, sequestration rate or evidence of an intervention effect. A real MRV protocol requires qualified local review, approved methods, source rights, spatially representative sampling and a preregistered design.
