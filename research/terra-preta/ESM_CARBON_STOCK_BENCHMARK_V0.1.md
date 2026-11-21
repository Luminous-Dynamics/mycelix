# Equivalent-soil-mass carbon stock benchmark v0.1

**Status:** synthetic-only, layered-core stock calculator and method-comparison audit. It demonstrates the measurement basis; it does not estimate a real farm's carbon stock, sequestration, carbon-credit eligibility or causal management effect.

## Why this matters

SOC concentration is not SOC stock. A fixed-depth sample contains more or less soil mass when bulk density changes through compaction, decompaction, tillage, grazing or amendment. Multiplying concentration by the changing mass in a fixed volume can create an apparent stock change even when the soil's carbon concentration is unchanged.

Equivalent soil mass (ESM) compares carbon in a common mass of fine-earth soil rather than a fixed volume. But ESM is not a single magic number: reference-mass choice, layer resolution, coarse fragments, core variability, and extrapolation all matter.

Recent literature:
- Rutjens et al. (18 April 2026), DOI [10.1111/sum.70213](https://doi.org/10.1111/sum.70213), compared ESM10, ESMμ, EBD10 and EBDμ using nine intensively sampled farms. Mean-based ESMμ/EBDμ reduced variance by 38–86% versus ESM10 in that dataset.
- A May 2026 Geoderma empirical comparison, DOI [10.1016/j.geoderma.2026.117801](https://doi.org/10.1016/j.geoderma.2026.117801), reports multi-layer ESM as most accurate in its comparison.
- A further 2026 Geoderma comparison of nine USDA-NRCS Dynamic Soil Properties projects, DOI [10.1016/j.geoderma.2026.117950](https://doi.org/10.1016/j.geoderma.2026.117950), reports fixed-depth estimates were biased where management caused consistent bulk-density differences, while tested ESM reference choices were similar in that dataset.

These results motivate method-specific benchmarking; they do not establish one universal ESM method for all soils or South African MRV applications.

## V0.1 method contract

Each core is a contiguous profile of depth layers. Per layer the input supplies:
- top and bottom depth (cm);
- SOC concentration (g C / kg dry fine earth);
- dry fine-earth bulk density (g cm⁻³, defined excluding coarse fragments);
- coarse-fragment volume fraction.

The fine-earth soil mass for a layer is:
`M_i = BD_i × Δz_i × 100 × (1 - CF_i)`
where BD is g cm⁻³ = Mg m⁻³, depth increment Δz is cm, and output M_i is Mg fine earth ha⁻¹.

The layer carbon stock is:
`C_i = M_i × SOC_i / 1000`
in Mg C ha⁻¹.

- **Fixed-depth (FD) stock:** sum C_i over the full observed profile.
- **ESM stock:** integrate layer carbon concentration against fine-earth soil mass until a single predeclared reference mass is reached. If the target falls within a layer, v0.1 uses that layer's measured mean concentration for the remaining mass (piecewise-constant layer approximation).
- No ESM extrapolation beyond observed profile mass is permitted.
- All comparison rounds and cores must have the same vertical layer boundaries and analytical basis.

The reference mass is supplied before the comparison; v0.1 does not choose it from post-treatment bulk-density outcomes. Future variants should separately qualify baseline-mean, ESM10, ESMμ, EBD10/EBDμ and interpolation-based multi-layer methods on adequate replicated core data.

## Pseudoreplication boundary

Core samples are spatial subsamples nested within one declared experimental unit (e.g. plot). Their mean describes that unit's sampled profile; the core count is not the number of independent treatment replicates. V0.1 reports core-to-core values and round means but does not estimate field-level treatment uncertainty.

## Known-truth compaction control

The manufactured fixture uses identical SOC concentration across baseline and follow-up layers while increasing bulk density at follow-up:
- fixed-depth stock rises because the same depth contains more fine-earth soil mass;
- ESM stock at the predeclared baseline reference mass remains unchanged.

A passing control demonstrates only that the equations and basis semantics are correct. It does not show real sequestration, stock stability, or ESM superiority in field observations.

## Future benchmark ladder

1. Reproduce FD and a simple ESM calculation on known-truth layered cores.
2. Compare one-layer and multi-layer ESM, explicitly report interpolation/extrapolation assumptions.
3. Compare ESM10, ESMμ, EBD10, EBDμ, core-depth and gouge-auger methods where the actual sampling data support them.
4. Add coarse-fragment/stony-soil methods and uncertainty for stone volume/mass and soil mass.
5. Add hierarchical variance decomposition: analytical, core, plot, block, site, time and method.
6. Estimate minimum detectable change and cost per detectable Mg C ha⁻¹ using independent experimental-unit replication.
7. Hold out whole farms, soil groups, laboratories and protocols; avoid repeated-core leakage.

## Proposed qualification targets (not achieved)

- Exact synthetic mass closure within declared numeric tolerance.
- Zero false SOC-stock promotion from pure compaction/decompaction known-truth controls.
- No ESM estimate when target reference mass exceeds any required core's observed fine-earth mass.
- Zero vertical support or unit mismatch.
- Explicit core-subsample vs experimental-unit identity.
- Report FD and ESM independently; never overwrite one with the other.
- No causal claim or carbon-credit claim from this software.

## Boundaries

The benchmark does not establish a South African SOC reference mass, sampling depth, carbon-credit methodology, MRV protocol or sequestration claim. Those require qualified local soil science, source-admitted reference methods, appropriate field permissions, an approved sampling design and independent validation.
