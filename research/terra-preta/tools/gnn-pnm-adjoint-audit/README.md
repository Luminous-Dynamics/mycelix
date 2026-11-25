# Terra Preta — differentiable GNN-PNM adjoint audit

This dependency-free Rust reference audits the hand-coded conductance adjoint in the public repository:

- https://github.com/ITLR-DDSim/gnn-pnm-permeability
- Paper: Zhao & Xiao (2026), https://doi.org/10.1016/j.jhydrol.2026.135602
- Dataset: https://zenodo.org/records/16610986

It reproduces the relevant mathematical structure on a four-pore series network:
- conductance-weighted graph Laplacian;
- inlet/outlet Dirichlet rows replaced with identity, matching the external source;
- scalar inlet flow and permeability objective;
- transposed adjoint solve;
- edgewise conductance derivative;
- centered finite differences.

It transcribes the public branch logic for its comparison gradient without importing the external Python stack or changing that repository.

## Findings under the manufactured chain

For conductances [2, 3, 4], inlet pressure 20, outlet pressure 10 and target K 5, the current source pattern leaves the interior–interior edge derivative at zero because delta_lambda is initialized to zero and only the one-boundary/one-interior masks are assigned. The finite-difference derivative for that edge is nonzero.

The public formula also collapses inlet orientation into a single inlet-membership boolean and uses the same direct-flow sign for p1-inlet and p2-inlet cases. The scalar inlet flow itself is orientation-aware (adds flows for p1 inlet and subtracts for p2 inlet), so the direct derivative must retain that orientation too.

The corrected reference uses:
- p1 boundary, p2 interior: state term lambda[p2] * dp;
- p1 interior, p2 boundary: state term -lambda[p1] * dp;
- both interior: state term -(lambda[p1] - lambda[p2]) * dp;
- direct inlet term: (K - K_star) * conversion * (is_p1_inlet - is_p2_inlet) * dp.

Every corrected derivative is checked against centered finite differences. Tests also reverse the inlet-edge orientation and require the corrected gradient to remain invariant.

## Run

    cargo fmt --manifest-path research/terra-preta/tools/gnn-pnm-adjoint-audit/Cargo.toml -- --check
    cargo test --manifest-path research/terra-preta/tools/gnn-pnm-adjoint-audit/Cargo.toml
    cargo clippy --manifest-path research/terra-preta/tools/gnn-pnm-adjoint-audit/Cargo.toml --all-targets -- -D warnings
    cargo run --manifest-path research/terra-preta/tools/gnn-pnm-adjoint-audit/Cargo.toml

## Scope

This is an independent mathematical audit of the publicly visible derivative pattern. It is not a reproduction of the full PyTorch training, does not modify the external repository, and does not prove the reported paper results invalid. Before drawing conclusions about trained model performance, compare against the exact external commit, data release, complete parameter settings, and any supplementary adjoint implementations.
