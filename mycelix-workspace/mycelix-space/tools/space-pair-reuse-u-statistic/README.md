# Space pair-reuse U-statistic reference

A dependency-free Rust reference for reusing two independently propagated trajectory ensembles across all pairwise collision checks, with a confidence bound that accounts for the dependence induced by shared trajectories.

## Estimand and estimator

Let X_1,...,X_m be IID samples from the primary object's trajectory uncertainty distribution and Y_1,...,Y_n be IID samples from the secondary object's trajectory uncertainty distribution. The two ensembles must be mutually independent for this E0 implementation.

Let h(X_i,Y_j) in {0,1} be the fixed high-fidelity continuous-time event indicator over the declared risk horizon. The all-pairs estimator is:

    U = (1/(m*n)) * sum_i sum_j h(X_i,Y_j)

It is unbiased for E[h(X,Y)] under the declared product distribution.

The mn outcomes are not independent: every primary trajectory is reused across n pair checks and every secondary trajectory across m pair checks. The report therefore always marks `dependent_pair_outcomes=true`. It must never apply a binomial confidence interval as though mn were independent hits/trials.

## Distribution-free one-sided bound

Changing one primary draw X_i changes U by at most 1/m. Changing one secondary draw Y_j changes U by at most 1/n. Under independence of all m+n trajectory draws, McDiarmid's inequality gives:

    P(E[U] > U + t) <= delta
    t = sqrt(0.5 * (1/m + 1/n) * ln(1/delta))

The reference reports:

    upper = min(1, U + t)

This is a conservative one-sided upper bound with confidence level 1-delta. The quantity:

    concentration_scale = 1 / (1/m + 1/n) = m*n/(m+n)

is reported as a scale for the bound, **not** as an exact effective sample size. It is categorically different from mn pair comparisons.

## Computational tradeoff

The reference consumes:
- m high-fidelity propagated primary trajectories;
- n high-fidelity propagated secondary trajectories;
- m*n pairwise event evaluations.

Thus it can reuse expensive trajectory propagation while evaluating many more pairs. It does not reduce pairwise event-oracle work, and its honest concentration bound may be much wider than a binomial interval that incorrectly treats every pair as independent.

The intended benchmark compares matched-propagation budgets, not only pair counts. Report propagation time, pair-check time, memory, estimate, confidence-bound width and observed coverage separately.

## Dependence and model boundaries

This E0 implementation requires:
- IID primary trajectory draws;
- IID secondary trajectory draws;
- mutual independence of the two ensembles;
- distinct retained trajectory IDs;
- a fixed collision event definition and risk horizon;
- one versioned high-fidelity continuous-time event oracle;
- a retained collision matrix with exact sample-order and content digests.

It does **not** support correlated primary-secondary errors from common environmental nuisance variables, shared random numbers, reused seeds or a common latent density forecast. If those matter, use a separately derived shared-latent / clustered estimator. Do not force the independence assumption.

The high-fidelity oracle is the benchmark reference, not proof of physical truth. Its propagator, covariance inputs, force model, event search and environmental data still require independent qualification.

## References and motivation

NASA CARA's 2018 brute-force Monte Carlo study describes Level IV epoch sampling and states that it does not reuse primary/secondary propagated states because ordinary confidence intervals are invalid under sample reuse. It also reports the severe propagation cost at low Pc:
- Hejduk, *CARA Brute Force Monte Carlo Study Results* (June 2018): https://ntrs.nasa.gov/api/citations/20180004056/downloads/20180004056.pdf

NASA's Spacecraft Conjunction Assessment and Collision Avoidance Best Practices Handbook Appendix N gives an explicit sample-reuse example: N propagated primary and N secondary samples yield N² pair checks, but those pair checks are dependent and conventional binomial confidence intervals are invalid. It discusses the time cost of a 1e-7 event several days from epoch:
- https://explorers.larc.nasa.gov/2023ESE/pdf_files/NASA16.%20CARA%20Handbook%20%28OCE_51%29.pdf

This crate's contribution is a simple E0 reference bound using independent primary/secondary sample sets. It is not a claim of improved Pc accuracy over CARA, ESA or commercial conjunction systems.

## Claim ceiling

The guarantee is conditional on IID primary samples, IID secondary samples, mutual independence between the two ensembles, a fixed event kernel and correct high-fidelity event labels. It does not validate orbital truth, covariance realism, shared environmental-error handling, complete encounter search, ISO conformity, operational suitability or maneuver safety.
