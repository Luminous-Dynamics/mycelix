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

This first greedy selector is deterministic and port-response weighted, but it is not claimed to be globally optimal. It caches each candidate's full port response and residual-verified state solution, reuses the selected state vector when enriching the basis, and computes the exact DC operator once per adaptive run. Projection matrices are rebuilt from the in-memory basis without repeating old PCG solves. If the audit grid contains zero shift, its full response is reused as the exact DC reference rather than solved a second time. The report records `cg_solve_count`, `total_cg_iterations`, maximum PCG iterations, maximum true relative residual, basis rank and candidate count. These costs must be included in any comparison with fixed shifts, PRIMA-style block Arnoldi or SPRIM-like symmetric reduction; basis rank alone is not a fair cost metric.

Run the adaptive manufactured fixture:

    cargo run --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml -- research/terra-preta/tools/sparse-passive-rom-reference/fixtures/adaptive-high-contrast-three-port.json /tmp/adaptive-rom-report.json

The adaptive fixture has eight interior states and three boundary ports. The declared target is at most 5% worst relative port-column error on the candidate grid, with at most 1% maximum response error on the separate audit grid and rank capped at six. The dedicated workflow compares this with a fixed two-shift model at the same rank. Before incremental caching, the adaptive path used 87 PCG solves / 661 iterations; the refreshed implementation is designed to use 60 solves, reusing selected candidate state solutions, the one DC operator and the zero-shift audit response. The fixed two-shift baseline is designed to use 36 solves. Exact-head CI must verify the cost assertions; do not treat these as achieved until that run passes. This is a synthetic equal-rank comparison, not an equal-offline-cost comparison. Meeting the finite candidate-grid target does not certify error outside the candidate and audit grids or establish soil hydraulic accuracy.

## Run

    cargo fmt --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml
    cargo fmt --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml -- --check
    cargo test --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml
    cargo clippy --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml --all-targets -- -D warnings
    cargo run --manifest-path research/terra-preta/tools/sparse-passive-rom-reference/Cargo.toml -- research/terra-preta/tools/sparse-passive-rom-reference/fixtures/two-interior-node-chain.json /tmp/sparse-rom-report.json

## PRIMA-style zero-shift block-Krylov comparator

For adaptive fixtures, the analysis also builds a zero-shift block-Krylov baseline at exactly the adaptive model's final rank:

1. Seed the basis with the full port block `A^-1 B`.
2. Repeatedly apply `A^-1` to the current orthonormal basis vectors using the same residual-verified PCG solver.
3. Use double-pass modified Gram-Schmidt and stop at the requested rank.
4. Form the reduced model with the same congruence projection, exact-DC correction and passivity checks.

This is a PRIMA-style passive block-Arnoldi/Krylov comparator for the restricted symmetric conductance/storage realization. It is not a full implementation of the general circuit PRIMA or SPRIM algorithms. The report records its audit-grid error, basis rank, passivity result, PCG solve count and total iterations separately from the adaptive method.

The dedicated workflow asserts equal rank and successful passive construction for both models, and prints the error/work comparison. Full sparse audit solves are shared references; they are not charged twice. The adaptive method additionally pays for its candidate-grid full-order solves and repeated projection rebuilds. No superiority claim is made unless the measured audit error and total offline cost are compared explicitly.

## A posteriori response-error bound (numerically verified M-matrix supersolution)

For each admitted graph, the reference validates that the normalized interior operator is a symmetric positive definite Z-matrix (a Stieltjes/M-matrix in the anchored graph setting). It solves `A y_hat = 1` with residual-verified PCG, requires positive components, scales the vector until `A y >= 1` is independently rechecked componentwise, and obtains the conservative lower bound:

    lambda_min(A) >= 1 / ||y||_infinity

For each real nonnegative Laplace sample, it computes the reduced state lifted to the full graph, the full-space residual `R(s) = B - (A+sI) X_r(s)`, and the bound:

    ||Lambda(s) - Lambda_r(s)||_F
      <= ||D - D_r||_F + ||B||_F ||R(s)||_F / (lambda_min_lower + s)

The report includes the lower bound, supersolution size, PCG certificate work, absolute and relative error bounds, and per-sample verification that the bound dominates the measured full-vs-reduced error. If the M-matrix structure, positive supersolution, componentwise inequality or bound check fails, analysis returns an error rather than emitting a qualified certificate.

This is a **floating-point-verified supersolution bound**, not an interval-arithmetic proof. It is limited to the fixed symmetric positive linear network and real nonnegative Laplace samples. It does not certify complex-frequency, nonlinear Richards, hysteresis, multiphase or empirical soil behaviour. A useful certificate must also be reasonably tight; the report exposes bound-to-observed-error ratio for that reason.

## A posteriori response-error bound (real nonnegative Laplace axis)

For the normalized symmetric interior operator (A=C^{-1/2}L_{ii}C^{-1/2}), the certificate requires exact sparse symmetry, positive diagonal and nonpositive off-diagonals. It computes a residual-verified solution of (A\hat y=\mathbf1), then scales it until the floating-point recomputation satisfies (y>0) and (Ay\ge\mathbf1) componentwise.

For a symmetric Z-matrix, a positive vector with (Ay>0) is a nonsingular M-matrix characterization. Under that mathematical condition, (A^{-1}\ge0), so (A^{-1}\mathbf1\le y) and
[
\lambda_{\min}(A)\ge\frac{1}{\lVert y\rVert_\infty}.
]
This gives a conservative resolvent norm bound on the real (s\ge0) axis. For the reduced state (X_r(s)=V(A_r+sI)^{-1}B_r), the full-space residual is
[
R(s)=B-(A+sI)X_r(s),
]
and the report uses
[
\lVert\Lambda(s)-\Lambda_r(s)\rVert_F
\le \lVert D-D_r\rVert_F
+\frac{\lVert B\rVert_F\,\lVert R(s)\rVert_F}
{\lambda_{\min}^{LB}(A)+s}.
]

Each sample records observed absolute/relative response error and the corresponding conservative upper bounds. The report also records the supersolution digest, minimum/maximum components, minimum componentwise (Ay-\mathbf1) margin, and PCG residual. The workflow rejects a sample if its reported upper bound does not dominate the observed full sparse error.

**Claim boundary:** this is a floating-point verified supersolution prototype, not interval arithmetic or a formal proof about rounding. It applies only to fixed symmetric anchored conductance/storage graphs and real (s\ge0). It does not certify complex-frequency behaviour, nonlinear Richards dynamics, hysteresis, topology uncertainty or empirical soil prediction. Bound tightness must be reported alongside validity; a valid but very loose bound is not automatically useful.

## Collatz–Wielandt refinement of the eigenvalue lower bound

The base supersolution gives `lambda_min(A) >= 1 / ||y||_infinity`. The reference now optionally tightens this bound using positive-vector Collatz–Wielandt iterations for the nonnegative inverse `M=A^-1`.

For any positive vector `v`, solve `A*z_hat=v` and recompute the true residual `r=v-A*z_hat`. Since `A^-1*1 <= y`, inverse positivity gives:

    A^-1*v = z_hat + A^-1*r
           <= z_hat + ||r||_infinity * y

componentwise. After a small floating-point inflation, this gives:

    rho(A^-1) <= max_i (z_hat_i + ||r||_infinity*y_i) / v_i

and therefore a refined `lambda_min(A)` lower bound. The algorithm retains the strongest valid bound seen, never replaces the original supersolution fallback with a weaker value, and stops after at most four additional PCG solves. A failed refinement records a reason and keeps the original bound.

The report exposes both initial and refined lower bounds, the number of refinement steps, maximum true infinity-norm residual, fallback reason, and total certificate PCG work. The workflow asserts the refined lower bound is no weaker and the added solve budget remains bounded.

This is still a floating-point residual-corrected Collatz prototype, not interval arithmetic. Its useful performance is determined by the measured lower-bound gain and the resulting response-error-bound tightness; merely returning a valid positive bound is not enough.

## Portwise componentwise response-error bound

The global spectral-norm bound can be very conservative because it multiplies global `||B||_F` and `||R||_F` norms. The M-matrix supersolution also gives a componentwise envelope that preserves which boundary ports actually couple to the residual support.

For output port (i) and input port (j), let (B_i) denote column (i) of the interior-to-boundary coupling matrix and (R_j(s)) the full-space residual column of the reduced state. Since ((A+sI)^{-1}) is nonnegative and no larger than (A^{-1}) elementwise for real (s\ge0), and (A^{-1}\mathbf1\le y),

[
\left|B_i^T(A+sI)^{-1}R_j\right|
\le \left(|B_i|^T y\right)\,\lVert R_j\rVert_\infty.
]

The response-error envelope is

[
U_{ij}(s)=|D-D_r|_{ij}
+\left(|B_i|^T y\right)\,\lVert R_j(s)\rVert_\infty,
\qquad
\lVert\Lambda-\Lambda_r\rVert_F\le\lVert U\rVert_F.
]

The implementation independently checks every response-matrix entry against this componentwise envelope before emitting the portwise Frobenius bound. It reports the original global spectral bound, the portwise bound, and the minimum of the two independently valid bounds. The same check is applied separately to the adaptive projection and the zero-shift block-Arnoldi baseline.

This method requires the exact symmetric anchored M-matrix admission checks and the verified supersolution. It applies only to real nonnegative Laplace samples. It is not a substitute for interval arithmetic and does not validate the pore graph or underlying soil model.

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

The current test suite contains twenty-seven tests, including an analytic 2×2 M-matrix bound, 24-order rescaling, positive/asymmetric off-diagonal rejection, Collatz monotonicity/solve-budget controls, and spectral plus portwise response-error bound dominance over every audit sample. Synthetic fixtures include a 128-node chain, a four-node branched network, and a high-contrast three-port network. Conductance/storage rescaling by 24 orders of magnitude tests scale invariance. Tests also verify that a zero-shift basis preserves both DC and high-frequency limits, while a nonzero-only basis reports its feedthrough-limit mismatch. None of these fixtures is empirical soil data.

## Claim boundary

This package includes a floating-point a-posteriori response-error upper-bound prototype on the real nonnegative Laplace axis; it is not interval-certified. It does not implement complex-frequency certification, adaptive globally optimal shift selection, nonlinear Richards flow, state-dependent conductivity, hysteresis, multiphase flow, reactive transport, root uptake or empirical soil validation. A measured speed advantage is not claimed. The current purpose is to establish sparse structure and a passivity-preserving projection baseline against the dense exact reference.
