# Multi-timescale soil carbon fate and disturbance vulnerability benchmark v0.1

**Status:** synthetic known-truth evaluator. This is not a calibrated soil-carbon process model, empirical field result, persistence certification, MRV method or carbon-credit instrument.

## Scientific motivation

Fernández-Catinot et al. (2026), *Temporal dynamics of particulate and mineral-associated carbon reveal three timescales of response to experimental manipulation*, SOIL 12, 805–819, DOI [10.5194/soil-12-805-2026](https://doi.org/10.5194/soil-12-805-2026), report that in their incubation experiment a two-pool POC/MAOC model did not simultaneously reproduce fraction-content changes and respiration, while a three-pool model with a faster litter-carbon compartment better represented the observed trajectories. The result motivates a structural adequacy challenge; it does not prove three pools are universally adequate.

Minich et al. (2026), *A Radiocarbon-Based Framework to Assess Soil Organic Carbon Persistence and Vulnerability Across Land-Use Types*, DOI [10.1111/gcb.70799](https://doi.org/10.1111/gcb.70799), combine radiocarbon-derived system age and respired-CO2 transit time with biological and thermal indicators. A 2026 dryland radiocarbon study (Nature Communications 17, 3565; DOI [10.1038/s41467-026-70623-9](https://doi.org/10.1038/s41467-026-70623-9)) likewise reports distinct age signals for bulk SOC and respired CO2.

## V0.1 purpose

The evaluator accepts a frozen set of observations and model predictions, then reports per-observable errors and a conjunctive adequacy status. It does not fit a process model. This isolates the benchmark/evaluator layer before connecting real Symtropy solvers.

Streams remain separate:
- total SOC stock;
- operational POC stock;
- operational MAOC stock;
- respiration flux;
- carbon system age;
- respired-CO2 transit time.

All observations carry a method, native unit, time and evaluation independence group. Each model declares its training groups; overlap with any evaluation group rejects.

## Model comparison

The synthetic fixture compares single-, two- and three-pool model predictions against the same observations. It intentionally gives the two-pool model a reasonable terminal/total-stock fit but poor rewetting respiration and respired-carbon transit-time predictions. This verifies that stock fit cannot compensate for a missed flux or wrong carbon clock.

No single weighted score is emitted. Each stream reports MAE, RMSE, bias, empirical 90% interval coverage and mean interval width. Qualification requires every predeclared stream to meet its own MAE and minimum-coverage criteria. Abstentions remain in the denominator and prevent joint adequacy in v0.1; they cannot improve a model's qualification by hiding difficult targets.

## Carbon accounting

The fixture keeps plant-derived and pyrogenic carbon inputs as separate ledger entries. It reports:
`initial stock + measured inputs - final stock - measured exports = unresolved residual`.

Respiration, dissolved export, erosion and harvest are separate. A residual outside the declared tolerance blocks the mechanistic claim ceiling. This is a simple synthetic closure check, not full field accounting: unmeasured roots, vertical transfers, sampling uncertainty, carbonate interference, lateral flows and unobserved gases require real measurement contracts.

## Negative controls

- Two-pool model fits total stock but misses respiration and respired transit time.
- Training/evaluation group overlap rejects.
- Missing model-target prediction rejects.
- An abstention carrying a numerical prediction rejects.
- Inverted prediction interval rejects.
- Carbon-balance residual outside tolerance remains explicit and blocks mechanistic interpretation.
- Existing output is never overwritten.

## Claim ceiling

Even a passing synthetic evaluator means only that the frozen predictions pass this manufactured multi-observable test. It does not validate mechanism identifiability, inferred transfer rates, the carbon age estimator, field generalization, multi-year persistence, climate benefit, permanence or MRV eligibility. Mechanism-level inference remains unqualified in this version.

## Integration

- Parent programme: Mycelix [#5527](https://github.com/Luminous-Dynamics/mycelix/issues/5527) and [#5525](https://github.com/Luminous-Dynamics/mycelix/issues/5525).
- Stock accounting: [#6449](https://github.com/Luminous-Dynamics/mycelix/issues/6449) / [PR #6524](https://github.com/Luminous-Dynamics/mycelix/pull/6524).
- Field GHG and event measurements: [#6405](https://github.com/Luminous-Dynamics/mycelix/issues/6405), [#6447](https://github.com/Luminous-Dynamics/mycelix/issues/6447), [PR #6558](https://github.com/Luminous-Dynamics/mycelix/pull/6558).
- Prospective model verification: [#6554](https://github.com/Luminous-Dynamics/mycelix/issues/6554).
- Causal treatment effects: [#6535](https://github.com/Luminous-Dynamics/mycelix/issues/6535).

## Next empirical gate

Acquire rights-cleared datasets and hash raw files; establish fractionation-specific measurement semantics; pair fraction stocks with temporally resolved respiration and, where feasible, radiocarbon/isotope measurements; preregister disturbance protocols; preserve plot/site/batch holdouts; and validate the observation operators. Do not infer missing historical fluxes from sparse stock data.
