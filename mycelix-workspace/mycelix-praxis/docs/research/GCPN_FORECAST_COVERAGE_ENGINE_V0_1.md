# GCPN Known-DGP Forecast Coverage Engine v0.1 — Stage B

**Status:** Rust implementation authored; compile/test/CI and coverage qualification remain outstanding.  
**Parent issue:** #5644.  
**Specification PR:** #5645.  
**Implementation package:** `benchmarks/gcpn-forecast-coverage-qualification`.  
**Dependencies:** none; isolated Cargo package.

## 1. Implemented research increment

This package moves the Stage A specification into an executable design, with an intentionally small first known-DGP family.

The DGP generates independent top-level systems. Each system has two event IDs that share a system-level latent risk state. Low-risk systems have one case per event; high-risk systems have four. This makes case rows and event IDs dependent within the actual top-level unit and deliberately creates a setting where resampling the wrong level can understate uncertainty.

The same outcome is used for candidate and reference Brier loss on every forecast case. Each bootstrap draw retains the paired case record; it never resamples model losses independently.

## 2. Exact analytical oracle

Default DGP parameters:

| Parameter | Value |
|---|---:|
| Independent systems per dataset | 40 |
| High-risk system probability | 0.20 |
| Events per system | 2 |
| Cases per event, low-risk system | 1 |
| Cases per event, high-risk system | 4 |
| Outcome probability, low-risk system | 0.10 |
| Outcome probability, high-risk system | 0.60 |
| Candidate forecast probability | 0.20 |
| Reference forecast probability | 0.35 |

For a Bernoulli outcome probability (q), expected Brier difference is:

`Delta(q) = (p_candidate - q)^2 - (p_reference - q)^2`.

The exact conditional differences are:
- low risk: `-0.0525`;
- high risk: `+0.0975`.

Expected cases per independent system:

`E[n] = 0.8 * 2 + 0.2 * 8 = 3.2`.

The case-weighted population estimand is:

`theta_case = (0.8 * 2 * -0.0525 + 0.2 * 8 * 0.0975) / 3.2 = +0.0225`.

The system-weighted estimand is:

`theta_system = 0.8 * -0.0525 + 0.2 * 0.0975 = -0.0225`.

The two estimands have opposite signs. This is intentional: it proves that case weighting and equal-system weighting answer different questions when cluster size is informative. The oracle is calculated analytically from the DGP parameters, not from generated sample data.

## 3. Implemented methods

The Rust core implements:
1. IID forecast-case percentile bootstrap — negative control.
2. Event-ID percentile bootstrap — under-clustering negative control because event IDs share one system state.
3. Independent-system percentile bootstrap — primary candidate under the DGP's declared independence assumption.
4. Reflected/basic independent-system percentile interval.
5. Independent-system studentized bootstrap-t candidate using a ratio-estimator cluster standard error.

Each bootstrap replicate draws the same number of complete declared units as in the observed sample and recomputes the case-weighted ratio of total paired score differences to total cases. Candidate/reference predictions and the shared outcome remain paired.

The studentized candidate recalculates the ratio estimator and a cluster-level delta-method standard error in each independent-system resample. Degenerate resamples are counted as abstentions. The implementation requires at least 80% valid studentized replicates to emit an interval; this is a declared engineering guard, not an empirically qualified threshold.

The original `coverage_study` driver measures the three resampling-unit percentile methods. A separate `alternative_coverage_study` compares basic and studentized independent-system intervals. Both report empirical coverage, MCSE, Wilson 95% coverage interval, mean width and abstentions; all coverage is conditional on an emitted interval.

The CLI prints one fixed-seed sample and the five interval constructions. It does not silently run or claim a confirmatory Monte Carlo qualification.

## 4. Authored tests

Thirteen Rust tests are included:

- analytical case-weighted and system-weighted oracle have opposite signs;
- fixed-seed generated datasets reproduce exactly;
- event IDs remain nested under shared independent systems;
- candidate/reference loss difference uses the same outcome;
- all bootstrap methods reproduce and report the declared resampling-unit count;
- coverage output includes MCSE, Wilson interval and abstentions;
- invalid DGP configurations fail closed;
- invalid event/system parent links are rejected;
- too few bootstrap replicates are rejected;
- invalid coverage confidence levels fail before the Monte Carlo loop rather than being reported as abstentions;
- the basic interval reflects percentile endpoints about the paired estimate;
- the studentized system interval is deterministic and reports degenerate bootstrap draws;
- the alternative coverage driver reports both basic and studentized methods with abstention accounting.

These tests are authored source, not executed results.

## 5. Deliberate limitations and next steps

This Stage B core is not yet the complete Stage C qualification programme.

Not implemented in this increment:
- the full DGP matrix from #5644;
- rare-event and serially dependent top-level blocks;
- balanced and multi-shape cluster-size families beyond this two-state design;
- regime-shift estimands;
- BCa and stationary/block bootstrap intervals;
- alternative-method 90%, 95% and 99% confirmatory coverage runs (the new alternative coverage driver is currently an exploratory facility);
- confirmatory 10,000-dataset runs at 90%, 95% and 99%;
- multiple-comparison corrections;
- JSON serialization and frozen case-level output fixtures;
- external statistical review.

The first run should be a pilot, not confirmatory evidence. Confirmatory DGPs, seeds, outer sample counts, bootstrap counts and method definitions must be frozen before qualification.

## 6. Validation status

- Cargo manifest and Rust source are present on the exact feature branch.
- Source-level review and analytical oracle arithmetic are documented.
- No Cargo build, Rust test execution, CLI run or CI pass is claimed because this authoring environment has no Cargo or rustc.
- No interval coverage result is claimed.

## 7. Safety and non-claims

Synthetic offline statistical-method research only. No live forecast data, alert generation, hazard thresholds, model auto-selection, public-warning action or operational recommendation. The synthetic DGP does not represent meteorological truth or establish superiority over ECMWF or any forecasting centre.
