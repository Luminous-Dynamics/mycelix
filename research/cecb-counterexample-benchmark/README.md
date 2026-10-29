# CECB Counterexample Benchmark v0.1

**Status:** deterministic synthetic reference landscapes and a scoring oracle. This is not an economic simulator, policy ranking, real-world evidence, or a claim that any discovery method is superior.

The dependency-free Rust crate provides known-truth landscapes and a scoring API for comparing counterexample/scenario-discovery methods. It intentionally does **not** implement PRIM, CART, an oblique tree, or a Symthaea-guided search. First establish trustworthy known-answer evaluation, then compare candidate algorithms under matched budgets.

## Run

```sh
cargo fmt --manifest-path research/cecb-counterexample-benchmark/Cargo.toml -- --check
cargo test --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml
cargo run --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml
```

The CLI emits deterministic JSON Lines with one score record per case/rule/label source. Each record includes exact confusion counts and precision/recall numerator-denominator pairs. Undefined precision/recall remains JSON `null`, never silently zero-filled.

## Landscapes

| Case | Shape / property | Purpose |
|---|---|---|
| `axis_aligned_2d` | `x0 >= 2` | Exact axis-rule recovery baseline |
| `oblique_2d` | `x0 + x1 >= 3` | Measures loss from an axis-aligned shortcut on a diagonal boundary |
| `nonlinear_interaction_2d` | `x0 * x1 >= 6` | Detects single-feature shortcuts that miss interactions |
| `noisy_boundary_2d` | Latent diagonal with deterministic label flips | Keeps latent truth distinct from observed labels |
| `irrelevant_dimensions_8d` | Failure depends on two of eight dimensions | Tests sensitivity to six irrelevant coordinates |
| `correlated_feasible_2d` | `abs(x0 - x1) <= 1`, failure if `x0 >= 2` | Tests a constrained feasible scenario set |
| `structural_family_a` | Failure if `x >= 2` | Development structural family |
| `structural_family_b` | Failure if `x <= -2` | Deliberate structural reversal; the same rule does not transfer |

The grid cases enumerate all 81 integer pairs in [-4, 4], except the constrained landscape, which retains 25 feasible points. The eight-dimensional case is a deterministic sample of 1,024 points. It is a software fixture, not a probability model or claim about real-world frequencies.

## Scoring API

- `score_predictions` accepts one Boolean prediction per point, so arbitrary discovery algorithms can be scored without putting their implementation inside the oracle.
- `score_rule` is a convenience baseline for axis-aligned interval conjunctions.
- Only feasible points contribute to confusion counts; infeasible exclusions are counted separately.
- Latent truth and observed labels remain distinct.
- Malformed rules, out-of-range features, duplicate feature clauses, inverted intervals, empty datasets, inconsistent dimensions and mismatched prediction lengths fail closed.
- The latent-oracle score is an evaluator-only upper bound. It must never be passed to a candidate process as an observation. These public landscapes are for development, not a hidden-test/security boundary.

## Initial qualification targets

These are targets encoded by deterministic tests, not achieved algorithm performance:

- Exact confusion counts on axis-aligned, oblique and nonlinear cases.
- Deterministic labels and score rows for identical inputs.
- A rule that works in structural family A fails in family B.
- Infeasible points cannot improve a score and are accounted for.
- Undefined precision or recall is preserved rather than zero-filled.
- Candidate methods are later compared on matched query, compute and timeout budgets. No algorithm is assumed to win.

## Next steps

1. Add a versioned candidate-result contract for rule/region sets and evaluator-visible predictions.
2. Add exhaustive enumeration and random/stratified sampling baselines; then compare reviewed PRIM/CART-compatible implementations.
3. Report coverage-density-complexity-runtime trade-offs and recall-versus-query-budget curves.
4. Add more physically admissible correlated scenarios and held-out model-family splits.
5. Connect this benchmark to CECB-ROBUST-001 only after the reference suite and candidate/evaluator boundary receive review.

Related work: [CECB-ROBUST-002 / #5512](https://github.com/Luminous-Dynamics/mycelix/issues/5512); [CECB robust tournament PR #5511](https://github.com/Luminous-Dynamics/mycelix/pull/5511).
