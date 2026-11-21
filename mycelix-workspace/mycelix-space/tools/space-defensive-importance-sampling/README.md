# Defensive Gaussian-mixture importance-sampling reference

A dependency-free Rust reference for **fixed-proposal defensive Gaussian-mixture importance sampling** in a whitened standard-normal state space.

This crate is an estimator/evidence core, not an orbital propagator. The caller supplies state samples and high-fidelity continuous-time event classifications from the orbital engine.

## Mathematical model

The nominal target in whitened coordinates is:

    p(z) = N(z; 0, I)

The adaptive proposal is a finite full-covariance Gaussian mixture:

    q_adapt(z) = sum_k alpha_k N(z; mu_k, L_k L_k^T)

The proposal used for final evaluation is the defensive mixture:

    q(z) = epsilon p(z) + (1-epsilon) q_adapt(z)

where 0 < epsilon < 1. Since q(z) >= epsilon p(z), the likelihood ratio obeys:

    w(z) = p(z)/q(z) <= 1/epsilon

For a collision event indicator I_A(z) supplied by the declared high-fidelity continuous-time event oracle:

    Pc = E_q[I_A(Z) w(Z)]

The estimator is the arithmetic mean of independent weighted event contributions. Every contribution is in [0,1/epsilon].

The full mixture density q(z) is always used in the denominator, irrespective of which component generated a sample. Using only the selected component density generally produces the wrong estimator.

## Fixed-sample confidence bound

The crate calculates a one-sided empirical-Bernstein upper confidence bound for independent bounded contributions Y_i = I_A(Z_i)w(Z_i) in [0,1/epsilon]:

    U = min(1, mean(Y) + sqrt(2 s^2 log(2/delta)/n)
                   + 7 log(2/delta)/(3(n-1) epsilon))

where s^2 is the unbiased sample variance. This is the Maurer-Pontil empirical-Bernstein form, scaled to the defensive contribution bound. It is fixed-sample only; it is not valid under optional stopping or repeated adaptive peeking.

Reference:
- Maurer & Pontil (2009), Empirical Bernstein Bounds and Sample Variance Penalization: https://arxiv.org/abs/0907.3740

## Proposal training versus final evaluation

The evidence contract requires distinct training and evaluation split identifiers, a frozen proposal, an IID evaluation design, a sample-order digest and a versioned high-fidelity continuous-time event oracle. These are caller attestations, not proofs. The estimator rejects split-ID reuse, non-IID evaluation claims, missing sampling evidence and low-fidelity event labels.

The intended workflow is:
1. Fit/tune a Gaussian or Gaussian-mixture proposal using a low-fidelity conjunction model or a dedicated training split.
2. Freeze the complete defensive mixture, including epsilon, component means, Cholesky factors and mixture weights.
3. Draw an independent evaluation sample from the full defensive mixture.
4. Evaluate every final sample using the declared high-fidelity continuous-time collision oracle.
5. Compute the full mixture density and nominal density, the exact likelihood ratio, and the weighted event contribution.
6. Produce the fixed-sample estimate, bound and concentration diagnostics, then bind them into the Mycelix qualification package.

This version does not generate random numbers. It exposes a deterministic mapping from externally generated independent uniform and standard-normal variates into proposal samples. RNG selection, quality and seed independence must be qualified by the caller.

## Diagnostics and limitations

The report includes the raw estimate, sample variance, standard error, maximum observed likelihood ratio, likelihood-ratio ESS, collision-contribution ESS, maximum contribution share, confidence bound, proposal ID, training/evaluation IDs, sample-order digest and event-oracle revision.

ESS is diagnostic, not proof of estimator validity. A low standard error does not establish covariance realism, model fidelity or physical truth. No likelihood-ratio clipping or weight truncation is used because those change the estimator and can introduce bias.

The target is a whitened Gaussian distribution. Non-Gaussian target uncertainty, uncertainty in the target density, correlated samples, MCMC/SMC evaluation particles and adaptively changing proposals are not supported by this fixed-sample bound. They require separately derived estimators and inference.

## Relation to orbital collision research

- NASA CARA's public software includes 2D/3D Pc, brute-force Monte Carlo, collision consequence and covariance-realism tools: https://www.nasa.gov/cara/publicly-available-cara-software/
- Hall et al. (2018), high-fidelity brute-force Monte Carlo from orbit-determination epochs: https://ntrs.nasa.gov/citations/20180006154
- Sztamfater-García et al. (2023), ESA rare-event sampling with conjunction mapping, adaptive importance sampling and high-fidelity boundary checks: https://conference.sdo.esoc.esa.int/proceedings/neosst2/paper/46/NEOSST2-paper46.pdf
- Sztamfater-Garcia et al. (2025), approximate dynamical model and importance sampling: https://doi.org/10.1016/j.asr.2024.12.074
- Ketzner & Elgohary (2026), Gaussian and Gaussian-mixture cross-entropy proposals: https://doi.org/10.2514/1.G009204

The present crate is an auditable estimator baseline, not a claim of better collision-probability accuracy or faster high-fidelity propagation than these systems.

## Claim ceiling

The guarantee is conditional on correct target/proposal densities, independent draws from the frozen proposal, and correct high-fidelity event labels. It does not validate orbit determination, force models, covariance realism, event-search completeness, ISO conformity, operational suitability or maneuver safety.
