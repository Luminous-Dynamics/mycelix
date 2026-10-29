# CECB Counterexample Benchmark v0.1

**Status:** synthetic known-truth reference fixture and scorer. It is not a policy simulator, policy ranking, or real-world evaluation.

This dependency-free Rust crate provides deterministic landscapes and a scoring API for evaluating counterexample/scenario-discovery candidates against exact synthetic labels. It intentionally does **not** implement PRIM, CART, an oblique tree, or a Symthaea-guided search. The first goal is to establish a trustworthy scoring oracle before comparing methods.

## Run

```sh
cargo fmt --manifest-path research/cecb-counterexample-benchmark/Cargo.toml -- --check
cargo test --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml
cargo run --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml
```

The CLI emits deterministic JSON Lines: one score row per case/rule/label source. Each record includes exact confusion counts and precision/recall numerator-denominator pairs. Undefined precision/recall stays JSON `null`; it is never silently replaced with zero.

## Included landscapes

| Case | Shape / property | Known-truth purpose |
|---|---|---|
| `axis_aligned_2d` | `x0 >= 2` | A simple axis rule should recover the failure set exactly. |
| `oblique_2d` | `x0 + x1 >= 3` | Measures the loss of an axis-only shortcut on a diagonal boundary. |
| `nonlinear_interaction_2d` | `x0 * x1 >= 6` | Catches single-feature shortcuts that miss interactions. |
| `noisy_boundary_2d` | Latent diagonal with deterministic label flips | Keeps latent truth separate from observed labels. |
| `irrelevant_dimensions_8d` | Failure depends on two of eight dimensions | Tests whether a method is distracted by six irrelevant coordinates. |
| `correlated_feasible_2d` | `abs(x0 - x1) <= 1`, failure if `x0 >= 2` | Makes the valid scenario set explicit. Infeasible evaluation points are skipped and counted. |
| `structural_family_a` | Failure if `x >= 2` | Development structural family. |
| `structural_family_b` | Failure if `x <= -2` | Deliberate structural reversal: the same rule does not transfer. |

The grid cases enumerate all 81 integer pairs in ([-4,4]^2), except the constrained landscape, which contains 25 feasible points. The 8-dimensional case is a fixed deterministic sample of 1,024 points; it is a software test fixture, not a probability model.

## Scoring contract

- `score_predictions` accepts one Boolean prediction per point, allowing arbitrary discovery methods to be scored without putting their implementation inside the oracle.
- `score_rule` is a convenience baseline for axis-aligned interval conjunctions.
- Only feasible points contribute to the confusion matrix; every excluded infeasible point increments `infeasible_skipped`.
- Latent truth and observed labels are explicit and distinct.
- Malformed rules (feature out of range, duplicate feature clauses, inverted interval), empty datasets, inconsistent feature dimensions and mismatched prediction lengths fail closed.
- The evaluator-only latent oracle row is an upper bound for reference/debugging. It must never be passed to a candidate process as an observation. These public synthetic landscapes are for development and are not a hidden-test/security boundary.

## Initial qualification targets

These are targets encoded by deterministic unit tests, not a claim about a discovery algorithm:

- Exact confusion counts on axis-aligned, oblique and nonlinear examples.
- Same inputs produce identical labels and score rows.
- A rule that works in structural family A is shown to fail in family B.
- Infeasible points cannot improve scores and are counted separately.
- Unknown/undefined precision or recall is preserved rather than zero-filled.
- Candidate methods are later compared on matched query, compute and timeout budgets; no algorithm is assumed to win.

## Next extensions

1. Add a versioned candidate-result contract for rule/region sets, including feature references and evaluator-visible predictions.
2. Add simple exhaustive and random/stratified discovery baselines, then independently implement PRIM/CART-compatible candidate generation or use reviewed implementations with pinned versions.
3. Add coverage-density-complexity-runtime plots and query-budget curves.
4. Add valid/invalid correlated-dimension generators and holdout family/scenario splits.
5. Integrate with CECB-ROBUST-001 only after the reference suite and evaluation boundaries are reviewed.

Related work: [CECB-ROBUST-002 / #5512](https://github.com/Luminous-Dynamics/mycelix/issues/5512); [CECB robust tournament PR #5511](https://github.com/Luminous-Dynamics/mycelix/pull/5511).
