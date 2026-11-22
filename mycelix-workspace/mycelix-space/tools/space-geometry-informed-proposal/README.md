# Space geometry-informed proposal reference

A dependency-free Rust reference for a **flat-boundary geometry-informed importance proposal** with defensive nominal support.

This is an analytical E0 slice inspired by GeoIS, not a reimplementation of the full curved-boundary method and not an orbital propagator.

## Target and event

The nominal target in whitened coordinates is p(z)=N(0,I_d). Each geometric mode declares a unit normal n, threshold a, and an orthonormal tangent basis. The event is the union of the declared half-spaces:

    A = union_j { z : n_j^T z >= a_j }

The geometric component draws tangent coordinates from N(0,I_(d-1)) and the normal coordinate as:

    s = a + E,    E ~ Exponential(lambda)

Its normalized density in the declared orthonormal frame is:

    q_j(z) = phi_(d-1)(t) * lambda * exp(-lambda*(s-a)) * 1{s>=a}

The geometry mixture is q_geo(z)=sum_j alpha_j q_j(z), with positive weights summing to one.

## Defensive mixture and estimator

The final proposal is:

    q(z) = epsilon*p(z) + (1-epsilon)*q_geo(z)

Therefore q(z)>=epsilon*p(z) and p(z)/q(z)<=1/epsilon. Every estimate uses the complete mixture density q(z), including all geometric and nominal components. The selected component ID is a diagnostic only and is never used in place of q.

The event contribution I_A(z)*p(z)/q(z) lies in [0,1/epsilon], so a fixed-sample empirical-Bernstein bound is available. It is not anytime-valid; repeated peeking needs the bounded-variable confidence sequence in PR #6495 under its own conditional-mean assumptions.

## Research context

GeoIS (Wang, Katsevich & Tong, arXiv:2610.27442, October 2026) uses a local Gaussian boundary approximation with an exponential inward shift and a second component to cover event mass missed by the boundary approximation:
https://arxiv.org/abs/2610.27442

Coverage-first constrained GMM (Lorek et al., arXiv:2610.07485, October 2026):
https://arxiv.org/abs/2610.07485

CVIS (Chakroborty et al., 2025):
https://doi.org/10.1137/24M1658498

NASA CARA public methods:
https://www.nasa.gov/cara/publicly-available-cara-software/

## Important limitations

- Boundary geometry is flat and explicitly supplied; no instanton optimizer or curvature/Hessian estimator is implemented.
- The event is the exact union of declared half-spaces. The crate does not propagate orbits or determine physical collision.
- Target is whitened standard Gaussian. Non-Gaussian state uncertainty, uncertain covariance, correlated state estimates and frame transformations need independent contracts.
- Nominal defensive support avoids zero proposal density when a geometry mode is missed, but does not guarantee small finite-sample error. A missed mode can still cause high variance.
- The fixed-sample empirical-Bernstein bound assumes independent evaluation draws from the frozen full proposal. It is not valid under optional stopping.

## Claim ceiling

This crate validates density and estimator mathematics for declared flat Gaussian half-space unions. It does not establish orbital Pc accuracy, covariance realism, model fidelity, high-fidelity event-oracle correctness, complete encounter search, regulatory conformity or maneuver safety.


## Synthetic rare-tail benchmark

The optional analytic half-space benchmark compares crude Monte Carlo with the defensive flat-boundary geometry proposal for standard-normal tails at thresholds 2.5, 3.5 and 4.5. A midpoint quadrature supplies the reference tail, and the benchmark reports RMSE, one-sided upper-bound coverage and event-contribution ESS over 64 deterministic seeded replicates of 8,192 samples each.

This is a smoke benchmark, not a calibrated coverage study: 64 replicates are far too few to establish nominal 95% coverage, and SplitMix/Box-Muller seed streams are not certified production randomness. It is limited to a one-dimensional half-space and does not establish superiority on curved or orbital collision events.
