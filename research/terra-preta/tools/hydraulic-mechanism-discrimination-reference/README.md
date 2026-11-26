# Hydraulic mechanism-discrimination reference v0.1

**Status:** executable synthetic known-truth benchmark. It is not soil data, field validation, or a reproduction of a published simulator-based BOED method.

## The research shift

Parameter precision and mechanism identification are not the same objective.

This fixture contains three competing hydraulic mechanisms:

| Mechanism | Steady total flow | Upper-path flow | Early transient flow |
|---|---:|---:|---:|
| Balanced parallel | 2.0 | 1.0 | 2.000 |
| Preferential path | 2.0 | 1.5 | 2.000 |
| Storage delay | 2.0 | 1.0 | 0.787 |

All three predict the same steady bulk conductance. Their pathway partition and transient predictions differ.

A bulk steady-flow observation can be highly informative about the scalar total-conductance parameter while providing zero information about which of these three mechanisms generated the data. The benchmark is designed to make that failure mode visible.

## Objectives kept separate

For discrete mechanism identity $M$ and a design's noisy observations $Y_D$:

- **Model discrimination:** $I(M;Y_D)=H(M)-E_{Y_D}[H(M\mid Y_D)]$.
- **Parameter precision:** worst-case scalar Fisher information for log total conductance, using the supplied physical derivative and declared observation noise.
- **Predictive information:** $E_{Y_D}[I(M;Y_*\mid Y_D)]$ for a held-out late-transient observable $Y_*$.
- **Decision value:** not yet implemented; it requires a named downstream action, loss matrix and decision threshold. It must not be inferred from model entropy.

The benchmark reports posterior model entropy, expected classification accuracy, Brier score and model log loss alongside information objectives.

## Deterministic numerical method

Candidate subsets are exhaustively enumerated under the acquisition-cost budget. Expected posterior entropy and expected predictive information gain are estimated by a fixed seven-point product Gauss-Hermite rule over independent Gaussian observation noise. This avoids Monte Carlo seed variation in the small reference problem; it is a deterministic quadrature approximation, not a closed-form integral.

A uniform portfolio baseline enumerates every distinct portfolio with exactly the selected model-discrimination design's total cost and gives each equal probability. It is not a uniform spatial sensor-placement comparator.

## Run

```bash
cargo fmt --manifest-path research/terra-preta/tools/hydraulic-mechanism-discrimination-reference/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/hydraulic-mechanism-discrimination-reference/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/hydraulic-mechanism-discrimination-reference/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/hydraulic-mechanism-discrimination-reference/Cargo.toml -- \
  research/terra-preta/tools/hydraulic-mechanism-discrimination-reference/fixtures/matched-steady-transient.json \
  /tmp/terra-mechanism-discrimination.json
```

The CLI hashes the exact input bytes and refuses output overwrite.

## Falsification controls

- Three mechanisms have identical steady bulk-flow predictions.
- Observationally equivalent mechanisms must produce zero model information gain.
- The parameter-precision design is expected to differ from the model-discrimination design.
- Candidate-order permutation must not change the selected physical measurements or objective.
- The exact-cost uniform baseline must have normalized portfolio probabilities and ordered quantiles.
- The Gaussian quadrature rule must integrate its normalization, first moment and second moment.

## Research comparators and next ladder

This is a small oracle, not an SOTA claim. The next benchmark should compare, at matched simulator/solve/acquisition budgets:

1. Random feasible and exact-cost uniform portfolios.
2. Nominal local Fisher / D-optimal design.
3. Finite-ensemble maximin D/E design (see [#6664](https://github.com/Luminous-Dynamics/mycelix/pull/6664)).
4. Expected model information gain / classification-based discrimination.
5. Expected predictive information gain (EPIG) on held-out observables.
6. Decision-focused expected value of sample information, with preregistered decisions and losses.
7. Faithful maximin robust Bayesian design with its KL ambiguity-set, Sibson-$\alpha$-MI and PAC-Bayes assumptions reproduced.

Relevant current literature includes:

- Abdulsamad et al., *Maximin Robust Bayesian Experimental Design*, NeurIPS 2026: https://arxiv.org/abs/2603.14094
- Zaballa & Hui, *Optimizing Likelihoods via Mutual Information: Bridging Simulation-Based Inference and Bayesian Optimal Experimental Design*, UAI 2026 / PMLR 337: https://proceedings.mlr.press/v337/zaballa26a.html
- Rainforth et al., *Modern Bayesian Experimental Design*, Statistical Science 2024: https://doi.org/10.1214/23-STS915

The SBI-BOED work is particularly relevant because it connects mutual-information objectives, surrogate-likelihood training and design under fixed simulator budgets. The robust BOED work provides a materially stronger comparator than simply taking a worst case over a hand-selected finite list of scenarios. These methods must be reproduced faithfully before claiming a comparison or improvement.

## Qualification targets (targets, not results)

- Exact portfolio optimum on the finite candidate fixture.
- Zero false discrimination under observational equivalence.
- Posterior calibration and simulation-based calibration before evaluating approximate posterior methods.
- At least 10 distinct mechanism families and 1,000 paired synthetic worlds before broad synthetic claims.
- Paired uncertainty intervals for predictive log score, CRPS, classification calibration and decision regret.
- Report simulator calls, solver/PCG work, wall time, memory, abstention and failure rate.
- No physical-soil claim until the observation support, noise and mechanism family have been validated against measured data.

## Claim boundary

The fixture has known synthetic truth and a deliberately small mechanism family. It cannot establish that the family spans real pore-scale or soil hydraulic behavior. A system that confidently selects the best model among a misspecified candidate set can still be wrong; the next iteration must include model-discrepancy detection and a safe abstain / request-new-measurement outcome.
