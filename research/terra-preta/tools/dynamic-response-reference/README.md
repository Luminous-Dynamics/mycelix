# Dynamic response reference v0.1

**Status:** executable reference for fixed positive linear conductance networks with diagonal lumped storage at interior nodes. All checked-in values are manufactured. This is not an unsaturated soil-flow solver or empirical validation.

## Model and exact boundary reduction

Let the graph Laplacian be `L = B diag(g) B^T`, with positive edge conductances. Interior nodes have storage capacity matrix `C_i = diag(c_i)`; boundary ports are ideal prescribed-potential ports with zero lumped storage in this v0.1 formulation.

The interior state satisfies:

`C_i dot(u_i) + L_ii u_i = -L_ib u_b`.

For a Laplace parameter `s >= 0`, eliminate the interior state:

`u_i(s) = -(L_ii + s C_i)^(-1) L_ib u_b(s)`.

The exact dynamic boundary response is:

`Lambda(s) = L_bb - L_bi (L_ii + s C_i)^(-1) L_ib`.

This preserves the complete linear input/output response at the named boundary ports for the declared model and real nonnegative Laplace parameters. At `s = 0`, it reduces to the steady Kron operator.

## Manufactured one-storage-node fixture

Two boundary ports connect through one interior storage node:

- inlet-to-storage conductance: 2;
- storage-to-outlet conductance: 3;
- interior storage capacity: 4;
- boundary potentials are prescribed; boundary storage is zero.

The dynamic operator has the analytic form:

`Lambda(s) = [[2 - 4/(5+4s), -6/(5+4s)], [-6/(5+4s), 3 - 9/(5+4s)]]`.

At `s=0`, the equivalent two-terminal conductance is `2*3/(2+3) = 1.2`. The outlet step response to a unit inlet potential step is:

`q_out(t) = 1.2 * (1 - exp(-5t/4))`.

This makes storage delay explicit: the steady permeability alone cannot describe the transient response.

## Validation

Run:

```bash
cargo test --manifest-path research/terra-preta/tools/dynamic-response-reference/Cargo.toml
cargo fmt --manifest-path research/terra-preta/tools/dynamic-response-reference/Cargo.toml -- --check
cargo clippy --manifest-path research/terra-preta/tools/dynamic-response-reference/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/dynamic-response-reference/Cargo.toml -- \
  research/terra-preta/tools/dynamic-response-reference/fixtures/single-storage-node.json \
  /tmp/dynamic-response-report.json
```

The CLI hashes exact input bytes and refuses to overwrite a report.

Tests cover:
- analytic dynamic Schur complement at multiple real Laplace samples;
- DC limit against the steady equivalent conductance;
- analytic step response initial delay and steady limit;
- symmetry, minimum symmetric response eigenvalue and nonnegative sampled quadratic forms for real `s >= 0`;
- edge reversal and node-storage permutation invariance;
- repeated-input scaling negative control;
- fail-closed validation of boundary storage, negative storage and unanchored components.

## Important qualification limits

The sampled quadratic forms establish a finite manufactured diagnostic on the positive real Laplace axis. They are **not** a formal complex-frequency positive-real / passivity certificate. The first implementation does not solve a nonlinear time-domain system or compute a full complex-frequency spectrum.

The exact formula assumes:
- fixed conductances;
- linear time-invariant dynamics;
- diagonal lumped storage only on interior nodes;
- ideal prescribed-potential boundary ports;
- no hysteresis, advection-dispersion state, reactions, capillary nonlinearities or moving geometry.

The next increments are complex-frequency transfer evaluation, generalized symmetric storage matrices where physically justified, full transient state-space integration, passive reduced-order realization, and comparison against nonlinear Richards/DualPerm and pore-scale transient solvers. A local tangent operator around a nonlinear operating state must remain separately labelled as a local approximation.

All results remain synthetic algebraic software qualification until independent specimen and field observations are admitted.
