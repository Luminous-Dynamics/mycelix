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

## Exact analytical rare-event benchmark

The crate exposes `exact_standard_normal_2d_disk_probability(r)`, the closed-form value:

    P(||Z|| <= r) = 1 - exp(-r^2/2),    Z ~ N(0,I_2)

The standalone `src/bin/analytic_disk_benchmark.rs` runs both crude Monte Carlo and defensive importance sampling through the same estimator API at disk radii 0.10, 0.05 and 0.02. It compares replicate RMSE and mean fixed-sample upper bounds against exact truth. The proposal is frozen per replicate and the final event label is produced by the exact analytic disk predicate.

This is a regression benchmark, not orbital evidence. It uses a deterministic SplitMix64 + Box–Muller stream for reproducibility, not production RNG certification; its sampling evidence identifiers are fixture references. The analytic-oracle flag is permitted only for synthetic mathematical fixtures. Real conjunction estimates must use an identified high-fidelity continuous-time event oracle.

The benchmark's central metric is RMSE against analytical truth at an equal sample count, not only the number of proposal samples that hit the event. It does not include orbital propagation cost or proposal-fitting cost.

Run:

    cargo run --locked --manifest-path mycelix-workspace/mycelix-space/tools/space-defensive-importance-sampling/Cargo.toml --bin analytic_disk_benchmark

## Whitening and physical-state correspondence

The target distribution is evaluated in whitened joint-state coordinates z, while the high-fidelity event oracle operates on the physical state:

    x = mu + L z

A versioned `WhiteningTransform` binds the mean and Cholesky factor together with state layout, epoch, time scale, coordinate frame, origin and units. Each retained evaluation observation carries both z (used for the target/proposal density ratio) and the exact physical state supplied to the event oracle. Before accepting the event label, the estimator recomputes x from z and rejects any mismatch.

This prevents the importance weight for one whitened sample from being paired with a collision label from a different physical state or a different epoch/frame convention. The transform Jacobian cancels in p(x)/q(x) because both target and proposal are expressed under the same invertible whitening map; the estimator therefore computes the density ratio in z-space.

The metadata are caller-declared and must be bound to the exact source covariance / OD epoch and frame-conversion evidence. This validation establishes internal correspondence, not that the declared orbit frame, epoch or covariance is physically correct.

## Repeating encounters: shared trajectory, joint path event

The crate also exposes `estimate_repeating_encounter_risk`. One observation row contains:
- one whitened initial-state sample z;
- its exact physical state x = mu + Lz;
- an ordered vector of high-fidelity collision flags, one for each encounter in a fixed assessment horizon.

All encounter outcomes on a row therefore share the same sampled initial uncertainty and high-fidelity trajectory. The horizon event is the union:

    U(z) = 1{ any encounter k collides }

The union estimator uses one importance weight p(z)/q(z) for the row. It does not estimate encounters from unrelated draws, sum marginal probabilities, or assume event independence.

The report retains weighted marginal estimates for each encounter, weighted pairwise co-occurrence estimates and target-event covariance diagnostics. It also reports the naive independence approximation:

    Pc_union_independent = 1 - product_k (1 - Pc_k)

This is only a comparator. It is never substituted for the direct shared-path union estimate. It is absent if finite-sample marginal estimates fall outside [0,1]. The evidence contract binds the fixed encounter ordering digest, assessment horizon, one-trajectory-per-row design, full-horizon event-oracle identity, proposal and sample order.

A synthetic perfect-dependence fixture shows why this matters: two encounters each have marginal probability 0.5, but if they always occur together the horizon union is 0.5, not the 0.75 obtained by multiplying complements as if independent. The independent and mutually-exclusive fixtures produce 0.75 and 1.0 respectively.

NASA CARA has a high-fidelity brute-force Monte Carlo precedent for repeating LEO/GEO conjunctions: https://ntrs.nasa.gov/citations/20190028903. This implementation is a statistical composition layer; it does not replace the high-fidelity propagator/event-search engine.

## Paired counterfactual maneuver comparisons

The crate exposes `compare_counterfactual_maneuver_risk` for comparing one baseline plan with one candidate plan on the **same initial-state samples**, using the same frozen defensive proposal, encounter order, assessment horizon and event-oracle revision.

For sample j, the paired weighted difference is:

    D_j = w_j * (I_candidate,j - I_baseline,j)

The estimate is candidate minus baseline. Negative means the candidate has lower estimated event probability on the declared sample population. Since each event-weighted term is bounded by 1/epsilon, D_j lies in [-1/epsilon, +1/epsilon]. The crate reports a two-sided fixed-sample empirical-Bernstein interval using that known range.

This common-random-numbers approach can reduce the variance of the difference when baseline and candidate outcomes are positively correlated. It does not guarantee a variance reduction for all plan pairs; the report includes an unpaired-stream standard-error comparator and the measured paired/unpaired variance ratio.

The per-encounter ledger reports baseline and candidate marginal estimates, newly introduced collision samples and mitigated collision samples. A candidate can improve one encounter while introducing risk at another; the ledger keeps those changes visible. The path-level union is estimated directly from each plan's full-horizon outcomes.

All candidate comparisons must be predeclared or use an appropriate simultaneous error budget. Repeatedly evaluating many plans and selecting the best one with unadjusted intervals introduces selection bias. The paired interval is fixed-sample only, not an anytime-valid stopping rule. No maneuver authorization or command authority is provided.

The design binds baseline/candidate plan IDs and revisions, the shared encounter order, horizon and same-oracle requirement. These are evidence attestations; a physical propagator must still execute both plan variants from the same retained initial state and the same high-fidelity event oracle must screen all relevant secondary objects.

## Proposal training versus final evaluation

The evidence contract requires distinct training and evaluation split identifiers, a frozen proposal, an IID evaluation design, a sample-order digest and a versioned high-fidelity continuous-time event oracle. These are caller attestations, not proofs. The estimator rejects split-ID reuse, non-IID evaluation claims, missing sampling evidence and low-fidelity event labels. A mathematical test fixture may explicitly identify an exact analytic reference oracle; that permission is not available for physical mission estimates.

The intended workflow is:
1. Fit/tune a Gaussian or Gaussian-mixture proposal using a low-fidelity conjunction model or a dedicated training split.
2. Freeze the complete defensive mixture, including epsilon, component means, Cholesky factors and mixture weights.
3. Draw an independent evaluation sample from the full defensive mixture.
4. Evaluate every final sample using the declared high-fidelity continuous-time collision oracle.
5. Compute the full mixture density and nominal density, the exact likelihood ratio, and the weighted event contribution.
6. Produce the fixed-sample estimate, bound and concentration diagnostics, then bind them into the Mycelix qualification package.

This version does not generate random numbers. It exposes a deterministic mapping from externally generated independent uniform and standard-normal variates into proposal samples. RNG selection, quality and seed independence must be qualified by the caller.

## High-fidelity boundary sentinel validation

The crate also provides `assess_boundary_sentinels`, a separate descriptive validator for a frozen low-fidelity collision classifier against a declared high-fidelity continuous-time event oracle. Each record includes the whitened point, its transformed physical state, a frozen stratum label, the surrogate result and the high-fidelity result.

The report counts false negatives (surrogate safe / high-fidelity collision) and false positives (surrogate collision / high-fidelity safe) per stratum. It lists strata with boundary leakage and does not conflate these counts with Pc or the importance-sampling estimator.

Crucially, no population confidence interval is emitted. Sentinels are commonly selected deliberately near a predicted boundary or around suspected modes, so their empirical miss fraction is not automatically an IID estimate of a population false-negative rate. A population interval requires a predeclared sampling population and a qualified independent/stratified selection design, with per-stratum error control or valid design weights. When a stratum has no high-fidelity collision, its false-negative fraction is undefined and its status is `NO_HF_POSITIVE_DENOMINATOR`, not zero.

This boundary check is motivated by ESA/UC3M's published adaptive-importance-sampling work, which re-propagates selected collision-boundary samples under high-fidelity dynamics and explicitly notes that its small frontier-validation set is insufficient to establish broad predictability:
https://conference.sdo.esoc.esa.int/proceedings/neosst2/paper/46/NEOSST2-paper46.pdf

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


The boundary-sentinel API is implemented in `assess_boundary_sentinels`. Its evidence must freeze a `required_strata` list (for example, `inside`, `boundary-shell`, and `outside` for each mode); the verifier rejects any missing required stratum. Its counts are descriptive by design; callers must supply a separately qualified sampling design before population-level inference is allowed.
