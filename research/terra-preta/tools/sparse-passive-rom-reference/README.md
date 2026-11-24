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

The current test suite contains fourteen tests. Synthetic fixtures include a 128-node chain, a four-node branched network, and a high-contrast three-port network. Conductance/storage rescaling by 24 orders of magnitude tests scale invariance. Tests also verify that a zero-shift basis preserves both DC and high-frequency limits, while a nonzero-only basis reports its feedthrough-limit mismatch. None of these fixtures is empirical soil data.

## Claim boundary

This package does not implement complex-frequency responses, adaptive optimal shift selection, certified a-posteriori frequency error bounds, nonlinear Richards flow, state-dependent conductivity, hysteresis, multiphase flow, reactive transport, root uptake or empirical soil validation. A measured speed advantage is not claimed. The current purpose is to establish sparse structure and a passivity-preserving projection baseline against the dense exact reference.
