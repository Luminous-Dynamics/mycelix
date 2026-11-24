# Response-operator reference v0.1

**Status:** executable reference for fixed, positive, linear, steady conductance graphs. All checked-in values are manufactured fixtures. This is not an empirical soil result or a nonlinear soil-flow solver.

## Why this slice exists

A bulk permeability label supervises a small number of boundary responses while a pore-network model may contain thousands of latent throat conductances. The inverse map can be many-to-one. Fitting one scalar permeability does not generally identify every throat.

This reference makes two mathematical objects explicit:

1. **Boundary response:** eliminate interior nodes with a Schur complement (Kron reduction), preserving the complete linear pressure-to-flux map on named boundary ports.
2. **Local identifiability diagnostic:** perturb each positive conductance multiplicatively and estimate the Jacobian of the symmetric boundary-response entries with respect to log-conductances. Numerical rank and nullity expose locally unresolved parameter directions for this observation set.

## Model

For an undirected edge e=(i,j) with positive conductance g_e, assemble the graph Laplacian:

`L = B diag(g) B^T`.

Partition nodes into boundary b and interior i. The exact linear steady boundary operator is:

`Lambda = L_bb - L_bi L_ii^(-1) L_ib,   q_b = Lambda u_b`.

For two boundary ports, the effective conductance is `-Lambda[1,2]`. The coarse boundary graph records `-Lambda[i,j]` for positive off-diagonal coupling. This preserves boundary response under arbitrary prescribed potentials at the declared ports, not merely one chosen pressure drop.

For parameter vector `eta_e = log(g_e)`, the diagnostic estimates:

`J[k,e] = d vech(Lambda)[k] / d eta_e`

using a central finite difference in log-conductance space. The reported nullity is `N_edges - rank(J)`. This is a local numerical diagnostic at the supplied conductances, not a global uniqueness theorem and not a posterior uncertainty interval.

## Manufactured example

The fixture has two parallel paths. Each path contains two series edges of conductance 2, so each path has equivalent conductance 1 and the total two-terminal conductance is 2.

A distinct assignment `[4, 4/3, 3, 1.5]` also has path conductances 1 and 1, and therefore the same bulk response. The reference tests this non-uniqueness and reports one observable sensitivity direction for four local conductance parameters (local nullity 3).

A second test uses a triangle with all three nodes exposed as boundary ports. Its full response matrix locally identifies its three edge conductances. This is a controlled illustration of how richer boundary access can add information; real experiments still require noise, geometry, boundary registration and model discrepancy to be handled.

The suite also proves two negative controls: node/edge storage permutations do not change physical boundary response, and repeating the same linear two-terminal excitation at a different pressure-drop magnitude does not increase structural sensitivity rank. Repetition may reduce independent measurement noise, but it cannot resolve an exact structural nullspace.

## Run

```bash
cargo test --manifest-path research/terra-preta/tools/response-operator-reference/Cargo.toml
cargo fmt --manifest-path research/terra-preta/tools/response-operator-reference/Cargo.toml -- --check
cargo clippy --manifest-path research/terra-preta/tools/response-operator-reference/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/response-operator-reference/Cargo.toml -- \
  research/terra-preta/tools/response-operator-reference/fixtures/two-parallel-paths.json \
  /tmp/response-report.json
```

The CLI hashes exact input bytes and uses create-new semantics for the report path. It refuses to overwrite existing output.

## Qualification coverage

- Schur complement equals an analytically reduced two-terminal graph.
- Endpoint reversal, edge-storage permutation and node-storage permutation preserve physical response.
- Repeating a scaled version of the same linear two-terminal excitation does not increase sensitivity rank.
- Distinct local conductances can produce the same effective bulk response.
- Jacobian rank/nullity is checked on a non-identifiable parallel-path fixture.
- A fully instrumented three-boundary triangle is locally identifiable.
- Unanchored components and nonpositive conductances fail closed.

## Claim boundary and next steps

Exactness applies only to the declared fixed linear steady conductance graph and its boundary ports. It does not imply equivalence for Richards flow, state-dependent unsaturated conductance, storage/transient response, hysteresis, multiphase flow, diffusion with reaction, or root/matrix exchange.

Next research increments:
- finite-volume / PNM comparison on a pinned open CT specimen with image-derived and independent reference values kept separate;
- noisy observation and sensor-placement studies;
- D-optimal / E-optimal selection of boundary excitations and interior pressure probes;
- nonlinear, state-dependent closures tested across saturation and wetting history;
- uncertainty propagation over segmentation ensembles and support-qualified hydraulic observations.

All targets remain research hypotheses until their corresponding tests and independent data gates pass.
