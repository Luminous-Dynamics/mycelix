# GCPN Spatiotemporal Forecast Verification Engine v0.1 — Stage B

**Status:** Rust source authored; Cargo build, Rust tests, external-data adapter and CI remain outstanding.  
**Parent issue:** #5662.  
**Specification PR:** #5663.  
**Package:** `benchmarks/gcpn-spatiotemporal-forecast-verification`.  
**Dependencies:** none; isolated Cargo package.

## 1. Implemented scoring core

The first package increment implements four scoring primitives for supplied scalar/spatial ensembles:

1. Empirical ensemble CRPS.
2. Threshold Brier score from a predeclared threshold and ensemble-member event fraction.
3. Multivariate energy score on a complete spatial field vector.
4. Variogram score with caller-supplied pair weights and exponent.

All scoring inputs are checked for empty/non-finite values and dimension mismatches. The variogram scorer rejects duplicate pairs, self-pairs, invalid indices and negative/non-finite weights. Pair weights and exponent are explicit input parameters; they are not chosen to maximize a model's score.

This is a scorer for forecasts supplied by another system. It does not produce forecasts, set warning thresholds or authorize warnings.

## 2. Marginal versus joint-field negative control

The source includes a deliberate two-dimensional fixture:

- observation: `[0, 0]`;
- correct-dependence ensemble: `[[0,0], [1,1]]`;
- wrong-dependence ensemble: `[[0,1], [1,0]]`.

For each spatial dimension separately, both ensembles have exactly the same marginal member values. Therefore their marginal CRPS values are equal for each dimension.

Their joint-field structure differs. The energy score and variogram score distinguish the two ensembles under the frozen observation and pair-weight protocol.

This is a synthetic score-discrimination example, not a general proof that either joint score detects every spatial dependence failure. Energy and variogram scores have different sensitivities and must be reported separately.

## 3. Exact synthetic score oracles

The authored tests pin:
- `CRPS([0,1], y=0) = 0.25`;
- threshold Brier with event-member fraction 0.5 and observed event = 1 gives 0.25;
- energy score for the two-member, two-dimensional oracle equals `sqrt(2)/4`;
- correct-dependence variogram fixture equals 0;
- deliberately incorrect-dependence variogram fixture equals 1.

The tests also assert equal marginal CRPS but distinguishable joint energy/variogram scores.

## 4. Fair-comparison contract

`validate_fair_comparison` checks an archive panel before comparative scoring:

- at least two unique expected model IDs;
- all records use one non-empty verification source and version;
- forecast issue time precedes valid time;
- declared lead equals `valid_time - issue_time`;
- variable, units, region and grid policy are specified;
- ensemble fields and verifying observations are finite and dimensionally valid;
- each model has at most one record per exact case key;
- all models in an eligible case key share the exact same verifying observation;
- only complete expected-model case keys enter the paired denominator;
- incomplete case keys are counted and excluded explicitly, not imputed.

The case key includes case ID, issue/valid time, lead, variable, units, region and frozen grid policy. Verification source/version are required and uniform across the comparison panel. A future archive adapter must also pin source data checksums, actual dataset versions and spatial remapping/preprocessing metadata.

This is intentionally strict: a scorecard must not quietly compare models against different verifying analyses or different case denominators.

## 5. Authored tests

Ten Rust tests are included:

1. exact ensemble CRPS oracle;
2. threshold Brier/member-fraction oracle;
3. exact joint energy-score oracle;
4. exact variogram-score oracle;
5. same marginal CRPS but distinct joint score under wrong spatial dependence;
6. empty/non-finite/dimension-invalid scoring inputs fail closed;
7. duplicate/invalid variogram pairs and weights rejected;
8. fair comparison accepts identical case and verification vintage;
9. fair comparison rejects different verifying sources;
10. incomplete model case is excluded and counted while a complete case remains paired.

These tests are authored source; they have not been executed.

## 6. Limitations and next steps

Not implemented in this increment:
- actual WeatherBench 2 data reader, license/provenance capture or source checksum adapter;
- grid remapping/interpolation policy and verification-mask implementation;
- temporal block/event DGP generator;
- calibrated uncertainty intervals for score differences;
- CRPS reliability decomposition, PIT/rank histogram and spread-skill reporting;
- probabilistic threshold-stratified rare-event report;
- multi-variable/multi-lead aggregate report serialization;
- performance optimization for large spatial grids and ensemble sizes;
- independent verification against a second scoring library;
- Cargo build, Rust tests, CLI execution or CI.

The source is a first scoring/fairness core. It is not a complete WeatherBench 2 adapter or operational verification service.

## 7. Validation status

- Isolated Cargo manifest, Rust library and CLI are on the implementation branch.
- Source-level review checks score formulae, exact oracle arithmetic and fair-comparison safeguards.
- JSON profile and source files are fetched back from the exact branch.
- No Cargo build, Rust test execution, CLI run, external-data replay or CI pass is claimed because this authoring environment has no Cargo or rustc.

## 8. Research alignment and non-claims

The design follows WeatherBench 2's open, like-for-like evaluation principle and extends the metric set toward joint spatial and extreme-event evaluation. WeatherBench 2 itself notes that scalar CRPS/spread-skill do not capture spatial/temporal correlations or specifically focus on extremes. Different verifying analyses used for different model families must not be turned into an unqualified direct leaderboard.

No forecast superiority, SOTA model win, real weather-skill estimate, operational warning threshold or warning decision is claimed.
