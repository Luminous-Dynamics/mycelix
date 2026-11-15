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

## Constrained partial-identification LPs

The library builds a probability-distribution LP over a declared finite world set. It supports moment and aggregate facts:

\[
\ell_k \le \sum_w a_{kw}p_w \le u_k,\quad
\sum_w p_w=1,\quad p_w\ge0.
\]

Each lower and upper endpoint is represented by a separate row. A synthetic known-answer case uses four query values (0, 10, 20, 30) with half the probability mass constrained to the first two worlds; exact primal/dual certificates prove sharp bounds [10, 20]. Without that aggregate fact, the unconstrained upper bound is 30.

## Infeasibility certificates

A Farkas verifier accepts an exact multiplier vector only when the multipliers are nonnegative, every combined variable coefficient is nonnegative, and the combined RHS is strictly negative. The synthetic contradictory aggregate facts (same mass at least 3/4 and at most 1/2) have a combined RHS of -1/4 and zero combined variable coefficients. That proves infeasibility of the declared LP, not of an unmodelled real-world population.

## Exact HiGHS interchange

The CLI can emit exact deterministic MPS for coefficients with finite decimal representations:

    cargo run --quiet --locked --offline --manifest-path tools/fsre-partial-id-lp-certificate/Cargo.toml -- --export-mps tools/fsre-partial-id-lp-certificate/fixtures/reduced-columns-optimal.psv

Non-terminating rationals such as 1/3 are rejected; the exporter does not round. An MPS golden is checked by the dedicated CI job. HiGHS floating-point outputs remain proposals until reconstructed and checked by the exact verifier.
