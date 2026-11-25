# Hydraulic robust experiment-design reference v0.1

**Status:** executable synthetic-only reference for robust sensor/experiment selection across segmentation and registration variants. It does not validate real soil sensors or field experiments.

## Research shift

The nominal experiment design in the adjacent reference chooses sensors for one idealized pore network. This reference evaluates the same design question across plausible segmentation/registration alternatives.

A candidate sensor is not assigned one universal sensitivity row. Each scenario supplies its own observation-operator Jacobian, noise and support description. These rows encode partial path mixing, probe registration and finite sensor footprints. All scenarios must use the same declared physical estimand and parameter IDs; the engine rejects scenario-local estimands that cannot be compared.

No probabilities are inferred from scenario count. The robust objectives are unweighted worst-case criteria.

## Objectives

For scenario v and design D:

`F_v(D)=F_{prior,v}+\sum_{e\in D}j_{v,e}^{T}R_{v,e}^{-1}j_{v,e}`.

The fixture declares a dimensionless parameter scaling and a prior-information matrix. Data-only sensitivity rank and the data-Fisher minimum eigenvalue are reported separately from prior-augmented information. D-optimal log-determinants include the declared prior plus explicit numerical regularizer; E-optimal selection maximizes the worst-case minimum eigenvalue of the data Fisher matrix, so a prior cannot make uninformative observations appear identifiable. Per-scenario oracle receipts report the prior-augmented logdet, data-only minimum eigenvalue, prior-augmented minimum eigenvalue and data-only rank separately.

The engine exhaustively enumerates feasible candidate subsets (up to 16 candidates) and reports:
- nominal D-optimal selection on the single designated nominal scenario;
- maximin D-optimal design using the smallest scenario log-determinant;
- maximin E-optimal design using the smallest scenario *data-Fisher* minimum eigenvalue;
- minimax normalized D-optimal regret against each scenario's own exhaustive oracle;
- per-scenario rank, nullity, minimum eigenvalue and log-determinant for each selected design.

For scenario v, normalized regret is the fraction of the best feasible experimental-design gain that a candidate design leaves unused:

`regret_v(D) = [logdet F_v(D*_v) - logdet F_v(D)] / [logdet F_v(D*_v) - logdet F_v(D_0)]`

where `D*_v` is that scenario's exhaustive feasible oracle and `D_0` is the no-new-acquisition baseline. The denominator is therefore the available improvement over baseline, not the absolute oracle log-determinant. This removes the arbitrary additive offset introduced by a change of parameter units. A scenario with no numerically resolvable available gain must also have no resolvable regret; otherwise evaluation fails closed. The report records both baseline log-determinant and available gain for every scenario.

Ties are resolved deterministically by lower cost and candidate ID. Scenario ordering does not affect results.

## Manufactured three-scenario fixture

The fixture uses four common log-conductance estimands for upper/lower two-throat paths. Three segmentation/registration scenarios have different observation support mappings:
- seg-a: nominal path-local sensors;
- seg-b: partially mixed sensor footprints;
- seg-c: stronger upper/lower path mixing, reduced support coverage and increased noise.

Candidates include two internal pressure probes, two nominal path-flow sensors and a repeat bulk boundary-flux measurement. The path-flow candidates are deliberately not assumed perfectly localized: the per-scenario observation rows mix upper and lower path flow in proportion to the stated support and registration alternatives.

The baseline bulk-flux observation is present in every scenario. Repeating the same bulk support may reduce variance in its direction but cannot raise structural sensitivity rank. The test checks this per scenario.

## Numerical qualification

- Symmetry and positive-semidefinite prior checks are performed relative to the matrix's own maximum-entry scale.
- The Jacobi eigensolver normalizes the matrix before applying its convergence threshold, avoiding an absolute-unit stopping criterion.
- Non-finite parameter-scaled sensitivities and inverse-variance overflow/underflow are rejected before Fisher accumulation.
- Uniform parameter reparameterization (including the prior and determinant regularizer) must leave design selections and normalized regret unchanged.

## Validation controls

- common-estimand mismatch rejected;
- missing scenario-specific candidate observation operator rejected;
- support-mixing, noise and coverage vary by scenario;
- input scenario order is permuted and robust selected design/objective must remain invariant;
- prior information is not counted as data rank;
- all selected designs remain within the stated cost budget;
- scenario count is not treated as a probability.

## Run

```bash
cargo fmt --manifest-path research/terra-preta/tools/hydraulic-robust-experiment-design-reference/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/hydraulic-robust-experiment-design-reference/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/hydraulic-robust-experiment-design-reference/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/hydraulic-robust-experiment-design-reference/Cargo.toml -- \
  research/terra-preta/tools/hydraulic-robust-experiment-design-reference/fixtures/segmentation-support-ensemble.json \
  /tmp/terra-robust-design.json
```

The CLI records SHA-256 of exact input bytes and refuses output overwrite.

## Claim boundary

This engine scores declared local Jacobians. It does not itself derive them from CT images, resolve pore split/merge correspondences or calibrate physical sensor support. Those are upstream inputs and must be independently justified. The common four-parameter path estimand is a manufactured fixture convenience; real pore ensembles should use common boundary-response moments/tensors or other physically invariant estimands, not variant-specific pore IDs. Minimax scores are decision criteria, not posterior probabilities.
