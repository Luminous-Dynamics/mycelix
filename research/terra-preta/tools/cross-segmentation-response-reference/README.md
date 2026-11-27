# Cross-segmentation response reference v0.1

**Status:** executable synthetic reference for the steady two-port response of each supplied segmentation-graph variant. It is not a CT segmenter, nonlinear soil solver, field validation, or hydraulic-activation evidence.

## Why this increment exists

The segmentation ensemble identifies topology changes, while the nominal experiment-design reference assumes one fixed graph and idealized point/path observations. This package connects those pieces without treating percentile variants as probabilities or pretending that a missing throat remains a measurable path.

## Manufactured fixture

The fixture is the same two-branch topology represented by the five variants in the segmentation-ensemble audit. All present edges have unit, dimensionless conductance so the experiment isolates topology:

| Variants | Through-path topology | Effective conductance | Upper probe pressure | Localized upper path |
|---|---|---:|---:|---|
| alpha-10 / 30 / 50 | Two branches | 1/3 | 1/2 | Resolved; flow 1/6; 1/2 of total |
| alpha-70 | Lower branch only; upper branch is a dead end | 1/4 | 3/4 | Incomplete; no complete-path estimate |
| alpha-90 | Source and sink disconnected | 0 | 1 | Incomplete; no through-flow fraction |

For a unit source-to-sink pressure drop, the exact boundary response matrices are:

- alpha-10 / 30 / 50: [[1/3, -1/3], [-1/3, 1/3]]
- alpha-70: [[1/4, -1/4], [-1/4, 1/4]]
- alpha-90: [[0, 0], [0, 0]]

The alpha-90 source-side and sink-side components are each anchored at one prescribed boundary. They therefore have zero through-response, not an unanchored singular interior component. An actually floating component fails closed.

## Observation-support rule

The localized path observation names a measurement edge and the ordered edge sequence required to support the full-path estimand. The tool reports a local edge flux if that edge exists, but promotes it to a complete-path flow only when every required path edge is present and the source and sink remain connected.

This distinction is intentional:
- a locally observed zero edge flux is not the same as a fully resolved path observation;
- a missing required throat is not assigned zero conductance;
- an integrated boundary flux or tracer curve is not substituted for a local throat measurement;
- a disconnected source/sink pair has no defined through-flow fraction.

The two pressure probes are reported only where their registered nodes exist in the variant. They remain pressure observations, not edge-activation certificates.

## Numerical method

For each variant the package assembles the graph Laplacian L = B diag(g) B-transpose, partitions the two boundary nodes from the interior nodes, and solves the interior Dirichlet system. It evaluates both columns of the two-port response matrix from unit boundary basis excitations. Effective two-terminal conductance is the negative off-diagonal response coefficient.

The validator rejects duplicate/unknown IDs, nonpositive or nonfinite conductances, missing endpoints, noncontiguous path declarations and interior components that are not connected to either prescribed boundary. Report generation hashes exact input bytes and uses create-new semantics to refuse overwriting prior output.

## Run

    cargo fmt --manifest-path research/terra-preta/tools/cross-segmentation-response-reference/Cargo.toml -- --check
    cargo test --manifest-path research/terra-preta/tools/cross-segmentation-response-reference/Cargo.toml
    cargo clippy --manifest-path research/terra-preta/tools/cross-segmentation-response-reference/Cargo.toml --all-targets -- -D warnings
    cargo run --manifest-path research/terra-preta/tools/cross-segmentation-response-reference/Cargo.toml -- research/terra-preta/tools/cross-segmentation-response-reference/fixtures/percentile-two-branch.json /tmp/cross-segmentation-response.json

## Qualification coverage

- exact manufactured conductance and pressure expectations across all five segmentation variants;
- source/sink disconnect and separately anchored components;
- fail-closed rejection of unanchored interior components;
- edge/node storage permutation invariance;
- boundary-response symmetry, zero row sums and flux balance;
- distinction between local edge flux and a complete-path observation;
- exact input digest and output no-overwrite negative control in CI.

## Claim boundary

All values are synthetic. The unit-conductance assumption isolates topological effects and is not inferred from CT geometry. The variant set is a deterministic sensitivity grid, not a posterior distribution. This reference does not validate hydraulic functions in real soil or establish that a physical sensor can be installed at a given support. Hysteresis, state-dependent conductance, storage, tracer transport and soil-scale closure remain separate qualification lanes.
