# Hydraulic Path, Hysteresis and Flux Benchmark v0.1

**Status:** Research and software design only. No soil amendment formulation, application rate, field-trial authorization, or claim of local efficacy is implied.

## 1. Why a single retention curve is not a complete hydraulic model

A water-retention curve describes water content as a function of matric pressure/suction under a defined measurement path and method. It does not, on its own, fully describe the dynamics of infiltration, redistribution, drainage, evaporation or root extraction.

Three distinctions are essential:

1. **Retention state:** water content `θ(h)` at a specified pressure head/suction and soil history.
2. **Conductivity state:** hydraulic conductivity `K(h)` or `K(θ)`, which governs the rate at which water can move.
3. **Hydraulic history:** wetting, drying and intermediate scanning paths can produce different water contents and conductivities at the same nominal pressure head.

A drying-only curve must not silently stand in for a wetting branch or for the complete hydraulic function.

Hysteresis can arise from pore-throat / ink-bottle geometry, contact-angle differences, entrapped air, pore connectivity and soil structure. Main drying and main wetting curves bound multiple scanning curves during cyclic wetting/drying. The state of a soil therefore depends on its hydraulic history, not only its current pressure head.

Dynamic nonequilibrium effects add another layer: under transient flow, pressure and water-content changes may not be in instantaneous equilibrium. A curve measured quasi-statically may not fully describe rate-dependent transient behaviour.

## 2. Evidence motivating a higher-fidelity benchmark

- Assouline et al. (2013) review soil hydraulic properties as a coupled water-retention and hydraulic-conductivity function, noting limitations in inferring conductivity only from retention and highlighting hysteresis and structural change.
- Likos, Lu and Godt (2014) experimentally assessed uncertainty in simplifying wetting parameters inferred from drying curves; their results show that such parameter relationships carry soil-dependent uncertainty and air entrapment can reduce wetting-path saturated water content.
- Rudiyanto et al. (2020) proposed functions spanning saturation to oven dryness that include hysteresis in water retention and conductivity; they evaluated the formulation against 241 hydraulic-property datasets spanning soils from sands to clays. This is a strong comparator, not a guarantee that one parameterization is best in every domain.
- Moret-Fernández and Latorre (2025) investigated estimating hydraulic properties from infiltration plus drainage curves using QEI and HYDRUS-VG inverse approaches, evaluating synthetic data, sieved soil and undisturbed soil cores. This points toward an informative measurement-design challenge: what can dynamic flow observations identify beyond a static retention curve?
- Yan et al. (2026) review dynamic nonequilibrium retention, rate-dependent effects, path-dependent hysteresis and challenges in coupling dynamic capillarity with soil deformation.

These findings motivate a benchmark that jointly considers retention, conductivity, hydraulic history and transient flux.

## 3. Research questions

1. How much prediction error is caused by substituting one drying branch for the wetting branch under measured wetting/drying cycles?
2. Does an explicit hysteresis model improve predictions of infiltration, drainage and moisture profiles on unseen cores, soils or laboratories?
3. When does a conventional non-hysteretic van Genuchten–Mualem model provide adequate fidelity, and when does it fail?
4. What information is identifiable from retention points alone, and what requires conductivity or transient-flow observations?
5. Can infiltration-plus-drainage experiments reduce the cost of measuring hydraulic functions without making unvalidated assumptions?
6. How do biochar/material amendments change both branches, scanning behaviour, conductivity and transient flow—not merely a single reported AWC value?
7. Are observed hydraulic changes persistent after aging, compaction, wetting/drying cycles and soil-particle movement?

## 4. Evidence and measurement contract

Every curve/flow experiment must carry:

- immutable study, site, soil, experimental unit, core/specimen and replicate identifiers;
- sample preparation, undisturbed/disturbed status, bulk density, texture/mineralogy and coarse-fragment metadata where available;
- pressure head / matric potential with explicit sign, unit and reference convention;
- water-content basis, unit, water density and bulk density when conversion is required;
- curve branch: main drying, main wetting, scanning wetting, scanning drying, or unknown;
- hydraulic history and reversal points for scanning curves;
- pressure-step equilibration protocol or transient measurement times where applicable;
- instrument, calibration, operator/laboratory, temperature and method uncertainty;
- hydraulic conductivity values with measurement method, units, saturation state and pressure head;
- infiltration, drainage, evaporation and boundary flux time series, where measured;
- sample depth, geometry, initial conditions, boundary conditions and test duration;
- original source digest, raw values, transformation provenance and exclusions.

Unknown branch or missing history must remain explicit. Never infer a curve branch from water-content monotonicity alone when the experiment history is not known.

Keep these evidence classes separate: synthetic fixtures, literature-extracted data, measured retention curves, measured conductivity, transient column experiments, field sensors, and plant/root water extraction observations.

## 5. Model hierarchy and comparators

All candidate models receive identical measured inputs, boundary conditions, calibration data and frozen train/test splits.

Required comparator hierarchy:

1. **Observed-data baseline:** interpolation within a measured branch's support; no branch substitution or extrapolation.
2. **Conventional non-hysteretic baseline:** van Genuchten–Mualem or another justified conventional retention/conductivity model, with its assumptions and fit domain made explicit.
3. **Hysteretic constitutive model:** a preregistered path-dependent retention/conductivity representation with main wetting/drying and scanning behaviour where data support it.
4. **Full-range / capillary-plus-adsorption comparator:** for datasets spanning near saturation through very dry states, include a suitable model that accounts for adsorption/film flow and drying/wetting hysteresis.
5. **Inverse transient benchmark:** estimate parameters from infiltration-plus-drainage observations; compare against parameters obtained from retention/conductivity laboratory curves.
6. **Reference solver:** compare transient simulations against HYDRUS-1D only for matched governing equations, constitutive model options, units, geometry, initial conditions, boundary conditions, and numerical tolerances. Do not claim solver agreement from a parameter-fit-only or retention-only comparison.

For each fitted model report parameter identifiability, convergence, bounds, covariance or posterior uncertainty, and sensitivity. A good fit on the calibration curve does not establish transient-flow validity.

## 6. Physical validation tasks

### A. Curve branch reproduction

- Reproduce main drying and main wetting observations separately.
- Where scanning curves exist, test measured reversal points and closed/open cycle behaviour.
- Retain entrapped-air and residual-water effects where the model supports and the data identify them.
- Evaluate prediction error by pressure-head range and soil class.
- Reject branch-mixed records and unobserved curve extrapolation.

### B. Conductivity validation

- Compare predicted `K(h)` and `K(θ)` against independent conductivity measurements.
- Do not treat a conductivity function inferred from retention as independently measured.
- Report error on log-conductivity where scientifically appropriate, plus absolute error and bias in original units.
- Report saturated and unsaturated conductivity separately.

### C. Transient flow and conservation

For a soil column or field control volume, account for the applied water, drainage, evaporation/other declared losses, and storage change. Define the balance residual as:

`R_w(t) = input(t) - output(t) - Δstorage(t)`

Cumulative fluxes and storage must use compatible units, area and time basis. Interpret residuals against the combined measurement uncertainty. If the uncertainty budget is missing, mark the balance unqualified rather than pass/fail against an invented tolerance.

Validate time series for:
- infiltration front / cumulative infiltration;
- drainage onset and cumulative drainage;
- moisture/pressure profiles at observed depths;
- time to wetting-front arrival;
- redistribution after input stops;
- evaporation and root extraction only where independently measured.

### D. Amendment-effect validation

Use paired soil-only and amended-soil observations for the actual material and batch. Measure whether treatment changes:
- main wetting and drying curves;
- hysteresis magnitude and scanning behaviour;
- `K_s` and unsaturated `K(h)`;
- infiltration / drainage / redistribution;
- pore/aggregate structure and bulk density;
- the above properties over aging and cycles.

Do not add a generic biochar multiplier to a soil constitutive law. Do not infer that a change in AWC entails improved infiltration, lower water stress, crop yield, or water productivity.

## 7. Evaluation protocol and proposed targets

Freeze splits by soil/core/site/study/laboratory and, for temporal or treatment claims, by material batch and experiment. Never split individual pressure points from the same curve across training and test.

Report:

- MAE/RMSE/bias for retention and conductivity;
- error by pressure-head, water-content and conductivity regimes;
- main-wetting, main-drying and scanning-path performance separately;
- transient curve error, infiltration-front timing error and cumulative flux error;
- water-balance residual relative to documented uncertainty;
- interval coverage, width and proper interval score;
- fit failure, parameter plausibility and out-of-domain abstention rates;
- leave-soil-out, leave-core-out, leave-lab-out and prospective evaluation where possible.

**Provisional research targets (not achieved results):**

- 100% branch and unit declaration for every confirmatory curve/flow record.
- Zero silent branch mixing, unlogged smoothing or unsupported extrapolation.
- Nominal 90% predictive intervals achieving 85–95% empirical coverage on independent held-out groups, with interval width and proper score reported.
- At least 10% lower predeclared transient-flow prediction error than the strongest valid baseline, with group-bootstrap evidence that improvement is positive and no critical domain regression. This is an engineering target, not a universal scientific minimum.
- Water-balance residuals consistent with combined independent measurement uncertainty.
- Explicit abstention where wetting history, conductivity or boundary-condition data are insufficient for a supported prediction.

## 8. Symtropy / Mycelix integration

- **Mycelix:** preserve raw curves, branch history, sample custody, methods, calibrations, core/site dependencies, batch identity, dataset hashes and independent review.
- **Symtropy:** implement the constitutive and transient-flow kernels only at the fidelity supported by measured inputs; retain explicit units and numerical tolerances; compare to matched reference solvers.
- **Symthaea:** identify missing branch/conductivity/flux measurements, maintain competing model hypotheses, and rank only approved measurement candidates by expected reduction in decision uncertainty.
- **Independent verifier:** recompute conservation balances, validate held-out partitions, reproduce parameter fits and check that no model has inferred unmeasured physical state as fact.

A curve fit, simulation, hash or signed record is not a measurement certificate, efficacy result, safety approval or regulatory determination.

## 9. Work packages

1. Extend curve schema to distinguish drying, wetting and scanning branches plus reversal history.
2. Define hydraulic-conductivity and transient-flux observation schemas.
3. Create synthetic loop and reversal fixtures with known branch behaviour and negative controls.
4. Extend the curve auditor to report branch-specific support and reject mixed histories.
5. Add model bake-off for non-hysteretic and hysteretic constitutive functions.
6. Add infiltration-plus-drainage inverse-identifiability benchmark.
7. Create matched HYDRUS-1D transient benchmark with independently checked inputs and conservation residuals.
8. Compare soil-only/amended-soil trajectories by batch, texture/mineralogy and aging.
9. Publish independent group-held-out reports before any local efficacy claim.

## 10. References

- Assouline, S. et al. (2013). “Conceptual and Parametric Representation of Soil Hydraulic Properties: A Review.” *Vadose Zone Journal*. https://doi.org/10.2136/vzj2013.07.0121
- Likos, W.J., Lu, N. & Godt, J.W. (2014). “Hysteresis and uncertainty in soil water-retention curve parameters.” *Journal of Geotechnical and Geoenvironmental Engineering*. https://doi.org/10.1061/(ASCE)GT.1943-5606.0001071
- Rudiyanto et al. (2020). “Simple functions for describing soil water retention and the unsaturated hydraulic conductivity from saturation to complete dryness.” *Journal of Hydrology* 588, 125041. https://doi.org/10.1016/j.jhydrol.2020.125041
- Moret-Fernández, D. & Latorre, B. (2025). “A novel laboratory method to estimate the soil hydraulic property from the inverse analysis of an infiltration curve measured in finite columns.” *Journal of Hydrology* 657, 133087. https://doi.org/10.1016/j.jhydrol.2025.133087
- Yan, G. et al. (2026). “Advances in dynamic soil water retention behaviour and implications for vadose zone hydrology.” *Capillarity* 19(1), 26–38. https://doi.org/10.46690/capi.2026.04.03
