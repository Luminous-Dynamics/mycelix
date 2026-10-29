# CECB Counterexample Benchmark v0.1

**Status:** deterministic synthetic reference landscapes and a scoring oracle. This is not an economic simulator, policy ranking, real-world evidence, or a claim that any discovery method is superior.

The dependency-free Rust crate provides known-truth landscapes and a scoring API for comparing counterexample/scenario-discovery methods. It intentionally does **not** implement PRIM, CART, an oblique tree, or a Symthaea-guided search. First establish trustworthy known-answer evaluation, then compare candidate algorithms under matched budgets.

## Run

```sh
cargo fmt --manifest-path research/cecb-counterexample-benchmark/Cargo.toml -- --check
cargo test --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml
cargo run --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml
```

The CLI emits deterministic JSON Lines: 18 per-rule score records plus three exhaustive box-search summaries. Score records include exact confusion counts and precision/recall numerator-denominator pairs. Search summaries expose the number of distinct prediction regions, Pareto-front size, maximum true positives, minimum false positives at that coverage, and whether exact recovery exists. Undefined precision/recall remains JSON `null`, never silently zero-filled.

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

## Exhaustive box baseline

`exhaustive_axis_aligned_boxes` enumerates the distinct prediction regions representable by axis-aligned boxes over the observed feature values for one- and two-dimensional datasets. Equivalent prediction masks are deduplicated; the least-complex representative is retained. The function refuses dimensions above two rather than silently exploding the search space.

`pareto_box_front` retains candidates not dominated simultaneously on three separate criteria: maximize true positives, minimize false positives, and minimize rule complexity. It does not create a weighted sum. For the axis-aligned landscape the box class can recover the known failure region exactly; for the oblique and nonlinear landscapes it cannot, which exposes a structural limitation of the hypothesis class rather than a scoring error.

## Scoring API

- `score_predictions` accepts one Boolean prediction per point, so arbitrary discovery algorithms can be scored without putting their implementation inside the oracle.
- `score_rule` is a convenience baseline for axis-aligned interval conjunctions.
- Only feasible points contribute to confusion counts; infeasible exclusions are counted separately.
- Latent truth and observed labels remain distinct.
- Malformed rules, out-of-range features, duplicate feature clauses, inverted intervals, empty datasets, inconsistent dimensions and mismatched prediction lengths fail closed. Exhaustive enumeration above two dimensions fails closed.
- The exhaustive axis-aligned frontier is a bounded hypothesis-class reference, not a universal optimum. The latent-oracle score is an evaluator-only upper bound. It must never be passed to a candidate process as an observation. These public landscapes are for development, not a hidden-test/security boundary.

## Candidate/evaluator protocol v0.1

The new Rust protocol module introduces a typed boundary for candidate evaluation:

- Development rows expose features, feasibility, and observed labels only. They do not expose latent truth.
- Holdout inputs expose sample IDs, features, and feasibility only. Per-sample holdout labels live in private evaluator state and aggregate scores are the only evaluation output.
- The candidate submission must identify its method and version and predict every holdout sample exactly once. Duplicate, missing, or unknown IDs; empty method metadata; and over-budget query receipts fail closed.
- The candidate query count is passed separately by the trusted harness. It must be instrumented by that harness; a count reported by candidate code is not evidence of compliance.
- The protocol split is deterministic and documented as `alternating_input_index_v0.1`. It is intended for API smoke tests and is **not** an independent generalization estimate: neighboring points may share structure. For robust policy comparisons, use held-out structural model families (CECB-ROBUST-001 / issue #5510).

The evaluator field is private on the split object, and its derived-debug output is deliberately redacted so formatting the whole split does not accidentally print per-sample targets. This remains an API-level leakage barrier, not a process sandbox: candidate implementations that are untrusted must run separately, and the trusted harness must retain the evaluator and measure query usage externally. The current PR adds the contract and tests but does not yet include PRIM/CART or a Symthaea search method.

## Initial qualification targets

These are targets encoded by deterministic tests, not achieved algorithm performance:

- Exact confusion counts on axis-aligned, oblique and nonlinear cases.
- Deterministic labels and score rows for identical inputs.
- Exhaustive box search is deterministic and finds an exact box for axis-aligned truth, but not for oblique or nonlinear truth.
- A rule that works in structural family A fails in family B.
- Infeasible points cannot improve a score and are accounted for.
- Undefined precision or recall is preserved rather than zero-filled.
- Candidate methods are later compared on matched query, compute and timeout budgets. No algorithm is assumed to win.

## Next steps

1. Review and qualify the candidate/evaluator API on exact-head CI; then add a harness that instruments actual query usage and emits immutable evaluation receipts.
2. Add exhaustive enumeration and random/stratified sampling baselines; then compare reviewed PRIM/CART-compatible implementations.
3. Report coverage-density-complexity-runtime trade-offs, calibration where probabilities exist, and recall-versus-query-budget curves with repeated seeds and uncertainty intervals.
4. Add more physically admissible correlated scenarios and held-out model-family splits.
5. Connect this benchmark to CECB-ROBUST-001 only after the reference suite, split design, and candidate/evaluator boundary receive review.

Related work: [CECB-ROBUST-002 / #5512](https://github.com/Luminous-Dynamics/mycelix/issues/5512); [CECB robust tournament PR #5511](https://github.com/Luminous-Dynamics/mycelix/pull/5511).
