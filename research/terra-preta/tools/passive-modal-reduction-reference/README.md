# Passive modal boundary-response reference v0.1

Status: executable mathematical reference for fixed positive linear conductance graphs with positive diagonal interior storage and ideal prescribed-potential boundary ports. This is not a nonlinear soil-flow solver or empirical validation.

## Manufactured chain

The fixture has three conductances in series: 2, 3 and 4. The two interior storage values are 1 and 2. The normalized interior operator A = C^(-1/2) L_ii C^(-1/2) has exact eigenvalues 2 and 6.5. The DC effective conductance is 12/13.

The package computes the exact dynamic Schur complement directly at complex Laplace samples and independently from its modal expansion. It uses a deterministic symmetric Jacobi eigensolver and reports eigenpair residual and orthogonality error.

## Modal realization

For the restricted linear model:

C xdot + L_ii x + L_ib u = 0
q = L_bb u + L_bi x

define A = C^(-1/2) L_ii C^(-1/2), B = C^(-1/2) L_ib, and D = L_bb. Then:

zdot = -A z - B u
q = D u + B^T z
Lambda(s) = D - B^T (s I + A)^(-1) B

After A = U diag(lambda) U^T, the transfer is represented as a sum of rank-one positive-semidefinite modal residues:

Lambda(s) = D - sum_k (b_k b_k^T)/(s + lambda_k)

## Exact-DC passive truncation

For retained modes R, correct the feedthrough by all omitted modes at DC:

D_R = D - sum_(k not in R) (b_k b_k^T)/lambda_k

Lambda_R(s) = D_R - sum_(k in R) (b_k b_k^T)/(s + lambda_k)

This preserves the full DC boundary operator. The realization passivity matrix is:

Q_R = [[D_R, B_R^T], [B_R, diag(lambda_R)]]

Its Schur complement with respect to the positive modal block is the exact DC operator. The report includes the minimum eigenvalue of this matrix, retained pole stability, complex-frequency positive-real sample checks, conjugate symmetry, direct-vs-modal error and reduced-vs-full error.

The positive-real checks are finite numerical samples; the state-space block matrix is the structural passivity certificate for the declared model. The certificate applies only to the declared realization and assumptions.

## Run

    cargo fmt --manifest-path research/terra-preta/tools/passive-modal-reduction-reference/Cargo.toml -- --check
    cargo test --manifest-path research/terra-preta/tools/passive-modal-reduction-reference/Cargo.toml
    cargo clippy --manifest-path research/terra-preta/tools/passive-modal-reduction-reference/Cargo.toml --all-targets -- -D warnings
    cargo run --manifest-path research/terra-preta/tools/passive-modal-reduction-reference/Cargo.toml -- research/terra-preta/tools/passive-modal-reduction-reference/fixtures/three-edge-chain.json /tmp/passive-modal-report.json

## Qualification coverage

- exact spectrum [2, 6.5] and DC conductance 12/13;
- eigenpair residual and orthogonality;
- full-modal transfer against direct dynamic Schur evaluation at complex frequencies;
- exact-DC truncation;
- full and reduced passivity-matrix positive semidefiniteness;
- positive-real checks over declared samples and conjugate symmetry;
- slower-mode retention improves low-frequency fit against faster-mode retention;
- fail-closed storage, anchoring, mode-index and Laplace-domain checks;
- input SHA-256 and no-overwrite output.

## Claim boundary

All values are synthetic. The implementation covers fixed positive conductances, positive diagonal interior storage and ideal Dirichlet ports. It does not qualify nonlinear Richards flow, state-dependent unsaturated conductivity, wetting/drying hysteresis, multiphase flow, reactive transport, root uptake or empirical soil behaviour. Sampled frequency checks alone are not a proof of passivity; the reported block certificate applies to this realization and these assumptions.
