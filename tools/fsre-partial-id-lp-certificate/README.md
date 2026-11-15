# FSRE sparse LP certificate E0

A dependency-free Rust verifier for exact primal/dual certificates of continuous linear programs and lossless reduction of identical LP columns.

## LP contract

The input declares:

\[
\max c^\top x \quad \text{subject to } Ax \le b,\quad x\ge0.
\]

A primal candidate is a valid lower bound only when \(Ax\le b\) and \(x\ge0\). A dual candidate is a valid upper bound only when \(A^\top y\ge c\) and \(y\ge0\). Exact weak duality requires \(c^\top x\le b^\top y\). If both candidates are feasible and the objectives are equal, the optimum of the **declared LP** is proven.

The verifier uses normalized rational arithmetic, not floating-point tolerances. Overflow or malformed rationals fail closed.

## Exact canonical-column reduction

Columns are equivalent only when their objective coefficient and every row coefficient are identical as exact rationals. For nonnegative continuous variables, equal columns can be replaced by a single variable equal to the sum of the original variables. The reduced primal is formed by summing original primal values in each class; the dual is unchanged.

This is an exact algebraic reduction of the declared LP. It does not apply automatically to integer/binary variables or models with omitted variable-specific constraints. Every constraint that distinguishes two variables must be present in the coefficient signature.

## Run

    cargo fmt --manifest-path tools/fsre-partial-id-lp-certificate/Cargo.toml -- --check
    cargo test --locked --offline --manifest-path tools/fsre-partial-id-lp-certificate/Cargo.toml
    cargo clippy --locked --offline --manifest-path tools/fsre-partial-id-lp-certificate/Cargo.toml --all-targets -- -D warnings
    cargo run --quiet --locked --offline --manifest-path tools/fsre-partial-id-lp-certificate/Cargo.toml -- tools/fsre-partial-id-lp-certificate/fixtures/reduced-columns-optimal.psv

## Claim ceiling

This crate does not solve LPs. It independently verifies supplied candidates/certificates for a supplied continuous LP. It does not authenticate the LP model, its causal semantics, population frame, source data or assumption set. Solver execution is explicitly NOT_PERFORMED. A primal/dual proof establishes an optimum only for the exact LP encoded in the input.

## Exact finite-world bridge

The library also builds a simplex LP for one action's finite-world query values. For world values \(v_w\), introduce nonnegative masses \(p_w\) with \(\sum_w p_w=1\), then optimize \(\sum_w p_w v_w\). Its exact maximum is \(\max_w v_w\); negating the objective gives the exact minimum. The builder supplies a one-hot primal witness and an exact dual certificate.

This makes a direct mathematical bridge to #6197's finite-world bounds. Feed the action-specific values from its world table into `analyze_finite_world_extremum` for minimum and maximum. The result includes the extremal world ID and exact primal/dual proof for the generated LP. Safety-floor qualification remains a separate per-world gate; it is not inferred from an expected-value LP.
