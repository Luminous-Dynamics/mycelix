# Sparse passive boundary-response reduction reference v0.1

Status: executable Rust reference for fixed positive linear conductance graphs with positive diagonal interior storage and ideal Dirichlet boundary ports. All included fixtures are synthetic.

## Why this follows the dense modal reference

The dense reference (#6617) explicitly diagonalizes the normalized interior operator. That is useful for small exact cases but does not scale to large pore networks. This reference uses CSR sparse storage, preconditioned conjugate gradients, and a port-driven rational Krylov snapshot basis. Only the reduced matrices are dense.

## Projection and passivity

For the normalized system

    zdot = -A z - B u
    q = D u + B^T z
    A = A^T > 0

a full-column-rank basis V gives

    A_r = V^T A V
    B_r = V^T B
    Lambda_0 = D - B^T A^-1 B
    D_r = Lambda_0 + B_r^T A_r^-1 B_r

and

    Lambda_r(s) = D_r - B_r^T (sI + A_r)^-1 B_r.

The reduced model has the same DC response. If the basis contains the zero-shift port snapshots `A^-1 B`, it also preserves the original high-frequency feedthrough `D`; without those snapshots, the DC correction may change the `s -> infinity` limit. The report exposes the feedthrough-limit error separately from finite-frequency sample error so exact DC cannot hide a high-frequency mismatch.

Its realization passivity matrix

    Q_r = [[D_r, B_r^T], [B_r, A_r]]

has Schur complement Lambda_0 with respect to A_r. Therefore, when A_r is positive definite and Lambda_0 is positive semidefinite, Q_r is positive semidefinite. The program checks these numerical conditions and reports CG residuals; floating-point certificates are tolerance-qualified.

The basis is generated from snapshots (A + sigma I)^-1 B for declared nonnegative rational shifts sigma and all boundary-port columns. This is a small, deterministic starting method, not claimed to be an optimal adaptive rational Krylov implementation.

## Adaptive shift selection

Adaptive mode is opt-in through the `adaptive` fixture object:

- `candidate_shifts`: a finite nonnegative candidate grid, disjoint from seed shifts and the untouched audit grid;
- `target_max_relative_error`: maximum reduced-vs-full port-response error sought over the candidate grid;
- `maximum_basis_dimension`: hard cap on reduced basis rank.

The implementation computes the full sparse multi-port response at every candidate shift once, then evaluates each input-port response column independently. It greedily adds the single unselected `(shift, input-port)` snapshot with the largest relative column error, rebuilding the projection after each addition. This grows basis rank one direction at a time instead of adding all boundary ports for every selected shift. The process stops when the worst candidate-pair error meets the target, the rank cap is reached, or the candidate-pair grid is exhausted. The report preserves each selected shift/port pair and its pre-addition error. The separate `laplace_samples` grid is used only for final audit and never for shift selection.

This first greedy selector is deterministic and port-response weighted, but it is not claimed to be globally optimal. Its costs include all full-order candidate solves and repeated projection rebuilds. The report records `cg_solve_count`, `total_cg_iterations`, maximum PCG iterations, maximum true relative residual, basis rank and candidate count. These costs must be included in any comparison with fixed shifts, PRIMA-style block Arnoldi or SPRIM-like symmetric reduction; basis rank alone is not a fair cost metric.

Run the adaptive manufactured fixture:

    cargo run --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml -- research/terra-preta/tools/sparse-passive-rom-reference/fixtures/adaptive-high-contrast-three-port.json /tmp/adaptive-rom-report.json

The adaptive fixture is synthetic. Meeting its candidate-grid target does not certify error outside the candidate and audit grids or establish soil hydraulic accuracy.

## Run

    cargo fmt --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml
    cargo fmt --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml -- --check
    cargo test --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml
    cargo clippy --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml --all-targets -- -D warnings
    cargo run --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml -- research/terra-preta/tools/sparse-passive-rom-reference/fixtures/two-interior-node-chain.json /tmp/sparse-rom-report.json

## Qualification

- sparse CSR assembly for the graph Laplacian and storage-normalized operator;
- anchored-component and positive-storage checks;
- PCG true residuals recomputed before convergence is accepted, with diagnostics recorded for DC, rational shifts and response samples;
- scale-aware basis, pivot, eigenvalue and passivity tolerances;
- port-driven rational Krylov snapshot generation and double-pass modified Gram-Schmidt;
- exact DC correction in the reduced feedthrough and separate high-frequency feedthrough-limit error;
- A_r positive definiteness and Lambda_0 positive-semidefinite checks;
- full sparse vs reduced response comparisons at declared nonnegative real Laplace samples;
- sparse-storage growth check on a generated 128-node chain;
- four-interior-node cross-linked parallel-pathway network with full-port response recovery;
- node/edge ordering invariance for the branched response;
- input SHA-256 and create-new output semantics.

The current test suite contains sixteen tests. Synthetic fixtures include a 128-node chain, a four-node branched network, and a high-contrast three-port network. Conductance/storage rescaling by 24 orders of magnitude tests scale invariance. Tests also verify that a zero-shift basis preserves both DC and high-frequency limits, while a nonzero-only basis reports its feedthrough-limit mismatch. None of these fixtures is empirical soil data.

## Claim boundary

This package does not implement complex-frequency responses, adaptive optimal shift selection, certified a-posteriori frequency error bounds, nonlinear Richards flow, state-dependent conductivity, hysteresis, multiphase flow, reactive transport, root uptake or empirical soil validation. A measured speed advantage is not claimed. The current purpose is to establish sparse structure and a passivity-preserving projection baseline against the dense exact reference.
