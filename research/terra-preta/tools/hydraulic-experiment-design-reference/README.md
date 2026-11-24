# Hydraulic experiment-design reference v0.1

**Status:** executable synthetic reference for local experiment design on a four-throat, two-parallel-path network. It does not validate a real sensor, soil specimen or field experiment.

## Scientific question

A single bulk effective conductance is one scalar observation of four positive throat conductances. It cannot identify all four local parameters. Which additional measurements are worth their cost?

The reference uses `eta_i = log(g_i)`, central finite differences for observation sensitivities, a Fisher information matrix, numerical sensitivity rank, and exhaustive cost-constrained D-optimal selection.

## Manufactured network

Two parallel paths connect left and right boundaries:

- upper path: throats g1, g2 in series;
- lower path: throats g3, g4 in series.

At the nominal point g=[2,2,2,2]:
- bulk effective conductance = 2;
- upper and lower intermediate pressures under unit boundary drop = 0.5 each;
- upper path flow = 1.

Candidate measurements:
- upper intermediate pressure probe (cost 1);
- lower intermediate pressure probe (cost 1);
- localized upper-path flow observation (cost 2);
- repeat of the same bulk conductance measurement (cost 1).

The declared cost budget is 4. The reference constructs a cost-matched conventional baseline that spends all four cost units on repeated bulk measurements. It compares the optimal design's regularized log-determinant and structural rank with this repeat-only protocol.

## Identifiability result

The local log-conductance sensitivity rows at the nominal point are:

`bulk:      [0.5, 0.5, 0.5, 0.5]`
`p_upper:   [0.25, -0.25, 0, 0]`
`p_lower:   [0, 0, 0.25, -0.25]`
`q_upper:   [0.5, 0.5, 0, 0]`

Therefore:
- bulk only has rank 1 and local nullity 3;
- bulk plus both interior pressure probes has rank 3;
- adding a genuinely localized upper-path flow measurement yields rank 4 at this nominal point;
- repeating the same bulk observation does not increase structural sensitivity rank;
- the cost-4 repeat-only bulk protocol retains rank 1 / 4, while the selected cost-4 spatially informative design reaches rank 4 / 4 and has a higher regularized Fisher log-determinant.

This last measurement must have genuine path-resolved support. An integrated whole-core flux or breakthrough curve is not automatically a localized upper-path flow measurement.

## D-optimal selection

For observation rows j_i and standard deviations sigma_i, the reference accumulates:

`F = sum_i (j_i^T j_i / sigma_i^2)`.

It exhaustively enumerates candidate subsets within the cost budget and ranks them using:

`log det(F + lambda I)`

with a declared small regularizer `lambda = 1e-8`. The regularized log-determinant is a ranking criterion, not proof that data identify every parameter. Numerical rank is reported separately. Exhaustive search is deliberately limited to 16 candidates and serves as an oracle for future greedy approximations.

## Run

```bash
cargo test --manifest-path research/terra-preta/tools/hydraulic-experiment-design-reference/Cargo.toml
cargo fmt --manifest-path research/terra-preta/tools/hydraulic-experiment-design-reference/Cargo.toml -- --check
cargo clippy --manifest-path research/terra-preta/tools/hydraulic-experiment-design-reference/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/hydraulic-experiment-design-reference/Cargo.toml -- \
  research/terra-preta/tools/hydraulic-experiment-design-reference/fixtures/two-parallel-paths.json \
  /tmp/hydraulic-design-report.json
```

The CLI hashes exact input bytes and refuses output overwrite.

## Claim boundary and next steps

The result is local to the supplied nominal conductance vector, candidate observation operators, noise values, costs and network topology. It does not show that four arbitrary throats are identifiable in a real soil. Measurement support, registration, observation error, model discrepancy and segmentation uncertainty must be admitted.

Next increments:
- D-, A- and E-optimal design with explicit parameter subspaces;
- robust design across segmentation and constitutive ensembles;
- sequential Bayesian design with prediction-before-observation receipts;
- pressure pulse and tracer schedule selection for dynamic systems;
- comparison with actual observation operators and measurement costs.
