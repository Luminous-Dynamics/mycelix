# Terra Preta soil-water retention metrology v0.1

**Status:** Measurement/evaluation contract draft. It is not a laboratory operating procedure, fertilizer recommendation, efficacy claim, or permission to conduct a trial.

## Why this exists

A model can have low prediction error while the target endpoint is ill-defined, measured on incompatible water-content bases, or extrapolated beyond the range actually measured. Before comparing a Symthaea/Symtropy prediction to a published baseline, make the laboratory quantity independently reproducible.

ISO 11274:2019 specifies laboratory methods for the soil-water retention characteristic on drying/desorption curves. Its described methods cover different pressure ranges, including suction tables near saturation and pressure-plate/membrane methods extending to −1,500 kPa. The standard is a method-selection reference, not proof that an individual dataset or curve was generated in compliance.

Available-water capacity is conventionally expressed as water retained between a defined upper limit (field capacity) and lower limit (often estimated at permanent wilting point). These are operationally defined conditions, not universal immutable constants. The trial protocol must predeclare the target matric potentials or justify a field-derived field-capacity endpoint for the soil/crop context.

## Versioned computational contract

The reference implementation is `tools/water-retention-audit/`.

For each independently identified sample and desorption curve:

1. Preserve one record per specimen/curve and the exact input bytes. Do not treat repeated pressure steps, technical subsamples, or multiple cores from the same plot as independent treatment replicates.
2. Declare water-content basis and units. Volumetric fraction is m³ water per m³ bulk soil. Gravimetric input is kg water per kg dry soil and is converted only when dry bulk density and water density are explicitly supplied:
   `theta_v = theta_g × rho_bulk / rho_water`.
3. Declare the signed matric-potential observations in kPa (zero or negative) and a field-capacity potential and permanent-wilting-point potential. Require `pwp < field_capacity < 0` in signed-kPa convention.
4. Interpolate only between measured bracketing points, using piecewise linear interpolation on log10 suction. Record the exact brackets and weight. Do not extrapolate.
5. Check desorption monotonicity. If water content increases with increasing suction beyond a predeclared tolerance, reject the curve; smaller deviations are retained raw and flagged. Never silently smooth, delete, or repair measurements.
6. Calculate `AWC_fraction = theta_v(FC) - theta_v(PWP)`. Report storage depth only for an explicitly declared soil-layer depth; do not assume a crop root depth or sum across horizons without a separately reviewed profile model.
7. Bind the report to the raw input SHA-256. Measurement uncertainty is explicitly unpropagated in this version.

The computational result is a curve-level measurement audit, not a treatment-effect estimator. A treatment effect requires a valid randomized/controlled contrast and correct experimental unit.

## Required data and method metadata

At minimum retain sample, curve, source-study, site, plot, treatment, comparator, core/specimen, depth interval, sampling date, laboratory, method, pressure/suction steps, water-content basis, drying/desorption branch, replicate identity, dry bulk density, water density where used, and any calibration/measurement uncertainty provided by the lab. Keep original units and values alongside normalized values.

Do not pool hysteresis branches. ISO 11274's published scope is drying/desorption; wetting/adsorption curves need a separately declared measurement and analysis contract. Record whether specimens are disturbed or undisturbed and the equilibration procedure/time. A pressure-plate curve is not automatically interchangeable with a field drainage-derived field-capacity observation.

## Benchmark targets

These are proposed acceptance targets for future datasets, not achieved performance:

- **Input fidelity:** 100% of evaluated curves have complete unit/basis/method metadata and a raw-file digest; no implicit unit conversion.
- **Coverage:** 100% of reported FC/PWP points are observed or interpolated inside measured curve support; zero extrapolated target values.
- **Arithmetic:** deterministic synthetic fixtures match independently calculated expected values within 1e−12 m³/m³ for well-conditioned double-precision inputs.
- **Integrity:** zero silent repairs, dropped observations, or unreported monotonicity violations; invalid physical units and target order fail closed.
- **Uncertainty:** future versions must propagate measurement and between-specimen uncertainty separately; v0.1 explicitly does not claim this.
- **Reproducibility:** same exact input bytes and tool version produce byte-stable numeric fields and the same input digest.
- **Transportability:** evaluate prediction of curve points and derived AWC on held-out study, laboratory/method, soil class, site, and material/batch groups. Report each domain separately; do not let a pooled mean hide a failed domain.
- **Decision value:** only after curve metrology, compare any model-driven selection policy with random and fixed expert schedules on cost per independently verified reduction in uncertainty, subject to safety and trial approvals.

Never make a single scalar score compensate for a failed data-integrity, safety, or transportability gate.

## Baselines and comparison fairness

Compare (1) direct interpolation from measured curve points, (2) simple empirical/texture-based pedotransfer functions where their input requirements are met, (3) established hydraulic parameterizations such as van Genuchten or Brooks–Corey when fitted only on training data and evaluated with the same support constraints, and (4) current published ML methods on identical source-held-out folds. A predictor must receive the same permitted covariates and the same target pressure support as its competitors.

Report MAE/RMSE in m³/m³, bias, target-specific error, calibration/interval score, failure/abstention rate, and group-bootstrap confidence intervals. Also report parameter-fit convergence/failure, physical monotonicity, out-of-support abstentions, and performance stratified by laboratory method, soil texture, bulk density, organic carbon, and pressure range.

Do not compare a curve interpolation metric with a learned full-curve model unless the observation budgets and input information are matched. Report both point-wise held-out pressure prediction and curve-level derived-AWC prediction; they answer different questions.

## Industry and standards context

- [ISO 11274:2019](https://www.iso.org/standard/68256.html) is the laboratory-method reference for water-retention characteristics on drying/desorption curves.
- The [USDA-NRCS Available Water Capacity indicator](https://www.nrcs.usda.gov/sites/default/files/2022-10/nrcs142p2_051590.pdf) describes AWC as water held between field capacity and permanent wilting point and notes its dependence on soil conditions and depth.
- A review of AWC methods describes use of soil samples equilibrated at preselected matric potentials and the practical role of laboratory pressure plates; see [Available water capacity from a multidisciplinary and multiscale viewpoint](https://link.springer.com/article/10.1007/s13593-022-00774-8).

These sources guide method and endpoint design; they do not establish one universally correct pressure threshold or certify the project’s measurements.
