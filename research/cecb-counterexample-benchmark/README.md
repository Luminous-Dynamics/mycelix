# CECB Counterexample Benchmark v0.1

**Status:** deterministic synthetic reference landscapes and a scoring oracle. This is not an economic simulator, policy ranking, real-world evidence, or a claim that any discovery method is superior.

The dependency-free Rust crate provides known-truth landscapes, a scorer, a candidate/evaluator protocol, a budgeted univariate-threshold learner, a bounded exhaustive axis-aligned-box learner, and a depth-limited CART-style tree. It does **not** implement PRIM, claim equivalence to a vendor's CART implementation/defaults, provide oblique split predicates, or include Symthaea-guided search. These simple, auditable baselines establish a reproducible floor before more sophisticated methods are added.

## Run

```sh
cargo fmt --manifest-path research/cecb-counterexample-benchmark/Cargo.toml -- --check
cargo test --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml
cargo run --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml
cargo run --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml --bin cecb-tournament
cargo run --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml --bin cecb-box-tournament
cargo run --locked --manifest-path research/cecb-counterexample-benchmark/Cargo.toml --bin cecb-cart-tournament
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
- `discover_univariate_threshold`, `discover_exhaustive_axis_box`, and `discover_cart_tree` are separate candidate learners that consume observed development labels only via the budgeted oracle.
- Only feasible points contribute to confusion counts; infeasible exclusions are counted separately.
- Latent truth and observed labels remain distinct.
- Malformed rules, out-of-range features, duplicate feature clauses, inverted intervals, empty datasets, inconsistent dimensions and mismatched prediction lengths fail closed. Exhaustive enumeration above two dimensions fails closed.
- The exhaustive axis-aligned frontier is a bounded hypothesis-class reference, not a universal optimum. The latent-oracle score is an evaluator-only upper bound. It must never be passed to a candidate process as an observation. These public landscapes are for development, not a hidden-test/security boundary.

## First discovery baseline and matched observation budgets

The `discovery` module fits a one-feature threshold by requesting observed labels only for an evenly spaced subset of feature-only development rows. It considers both `x >= threshold` and `x <= threshold`, minimizes development classification errors, and uses deterministic tie-breaking. The non-cloneable development oracle enforces each label request and records a trusted query trace; the runner derives the receipt count from that trace, not from the method's claimed count. Wall-clock time, peak memory, and any external simulator calls must be measured separately by the trusted runner.

The `cecb-tournament` binary emits three JSONL receipts for budgets 3, 5, and 9 on the reversed structural-family fixture. The known-answer behavior is intentionally revealing:

| Development observation budget | Fitted threshold | Holdout TP | FP | FN | TN |
|---:|---:|---:|---:|---:|---:|
| 3 | `x >= 4` | 0 | 1 | 3 | 5 |
| 5 | `x >= 2` | 0 | 3 | 3 | 3 |
| 9 | `x >= 2` | 0 | 3 | 3 | 3 |

All three fits are evaluated on family B while discovery sees family A's observed labels. More training examples recover the family-A boundary but do not resolve the structural reversal. The receipts bind each run to the exact checkout SHA, the pull-request head SHA, and the SHA-256 of the committed split manifest; they record evaluator and method versions, declare that no random seed is used, include the trusted query trace, and expose the confusion counts. CI verifies the manifest digest format and contents, reruns the binary, and byte-compares the receipts.

This is one simple baseline and one tiny negative control, **not** a competition against SOTA. The labelled-row budget is enforced by the in-process oracle for this reference baseline; it does not meter arbitrary external simulator calls. A real tournament must isolate each candidate, give it only the candidate view and oracle capability, instrument all calls in a trusted harness, and record wall time and peak memory independently.

## Two-dimensional box hypothesis tournament

The `discover_exhaustive_axis_box` baseline fits an axis-aligned conjunction of one/two feature intervals by enumerating the bounded box hypothesis class over the labelled development observations. It minimizes training classification errors, then rule complexity, false positives, and predicted-positive count with deterministic tie-breaking. It requests each label through the budgeted oracle. This is deliberately called an **exhaustive box baseline**, not PRIM or CART, and it refuses dimensions greater than two.

The `cecb-box-tournament` binary compares three known-truth shapes with the same 41 development-label queries and 40-point holdout. The expected known-answer receipts are:

| Truth shape | Development errors | Rule complexity | Holdout TP | FP | FN | TN |
|---|---:|---:|---:|---:|---:|---:|
| Axis-aligned | 0 | 1 | 13 | 0 | 0 | 27 |
| Oblique | 3 | 2 | 6 | 0 | 6 | 28 |
| Nonlinear interaction | 5 | 2 | 3 | 0 | 5 | 32 |

The axis-aligned case is exactly representable by one interval. The oblique and interaction cases expose a hypothesis-class limitation: this box learner chooses high-purity regions and therefore avoids false positives on this holdout, but it misses failures that the single box cannot cover. The reported values are deterministic known-answer expectations encoded in CI; they are not claimed as Rust-run results until that workflow completes. The fixture and all targets are public, so this is a regression/qualification suite rather than a blind generalization benchmark.

Every receipt binds the exact checkout SHA, pull-request head SHA, and SHA-256 of the committed box-tournament manifest. It also records evaluator version, observation policy, query trace, development error, rule complexity, and held-out confusion counts. CI reruns the binary and byte-compares the output before validating the known answers.

## CART-style depth/complexity tournament

The `discover_cart_tree` baseline uses axis-aligned binary splits, Gini impurity reduction, fixed deterministic tie-breaking, majority-class leaves, and a maximum tree depth. The depth is part of the protocol: a depth-1 stump is compared with depth 3 on the same 41 development-label queries and 40-point holdout. The implementation is intentionally described as CART-style rather than claimed to match scikit-learn, R, SAS, or other vendor CART implementations.

The `cecb-cart-tournament` binary emits seven JSONL receipts: the three truth shapes at depths 1 and 3, plus the family-A-to-B transfer negative control. The deterministic known-answer expectations are:

| Truth shape | Max depth | Development errors | Tree splits | Holdout TP | FP | FN | TN |
|---|---:|---:|---:|---:|---:|---:|---:|
| Axis-aligned | 1 | 0 | 1 | 13 | 0 | 0 | 27 |
| Axis-aligned | 3 | 0 | 1 | 13 | 0 | 0 | 27 |
| Oblique | 1 | 9 | 1 | 0 | 0 | 12 | 28 |
| Oblique | 3 | 1 | 5 | 8 | 0 | 4 | 28 |
| Nonlinear interaction | 1 | 8 | 1 | 0 | 0 | 8 | 32 |
| Nonlinear interaction | 3 | 3 | 5 | 5 | 0 | 3 | 32 |
| Reversed structural family | 3 | 0 | 1 | 0 | 3 | 3 | 3 |

The oblique and nonlinear cases show why method capacity and complexity should be measured together: depth 3 recovers more held-out failures than depth 1, but requires more splits. The reversed-family control shows that this added capacity does not make the learned relationship transfer to every structural model. These figures are deterministic expectations derived independently and encoded in CI; they are **not Rust-run results until the workflow actually finishes**. The fixtures are public known-answer tests, not blind evaluation evidence.

Receipts bind the exact checkout SHA, PR head SHA and SHA-256 of the committed CART tournament manifest; they also include evaluator/method versions, query traces, depth, development errors, tree complexity and held-out confusion counts. CI checks all seven known-answer rows and byte-compares repeated runs.

## Candidate/evaluator protocol v0.1

The new Rust protocol module introduces a typed boundary for candidate evaluation:

- Development inputs expose features, feasibility, and partition-local IDs, but no labels. Candidate methods obtain observed development labels one at a time through a non-cloneable oracle that enforces the query budget and records every request.
- Holdout inputs expose partition-local sample IDs, features, and feasibility only. Per-sample holdout labels live in private evaluator state; aggregate scores are the only evaluation output.
- Original source IDs and structural-family identities stay in the trusted manifest. The candidate-safe view does not expose source IDs or family names, and debug formatting redacts both the development-label ledger and holdout evaluator.
- The candidate submission must identify its method and version and predict every holdout sample exactly once. Duplicate, missing, or unknown IDs; empty method metadata; and over-budget query attempts fail closed.
- The evaluator derives the receipt query count from private oracle state; its public scoring method accepts no caller-supplied count. The trusted harness also reads the query trace from oracle state rather than trusting candidate-reported metadata. This API-level control still needs process isolation for untrusted native code; safe external methods should receive only the view and the query capability, never the evaluator-bearing wrapper.
- The protocol split is deterministic and documented as `alternating_input_index_v0.1`. It is intended for API smoke tests and is **not** an independent generalization estimate: neighboring points may share structure. For robust policy comparisons, use held-out structural model families (CECB-ROBUST-001 / issue #5510).

The protocol also has a structural holdout constructor, `build_model_family_split`. It leaves every sample from the named model family out of development. Source IDs and family identity stay in the trusted manifest; candidate records use partition-local IDs and do not reveal family names. Its known-answer fixture combines the two reversed families: a threshold `x >= 2` that detects family A's failures produces TP=0, FP=3, FN=3, TN=3 on held-out family B. This is a deliberately small falsification case showing why random/interleaved splits alone cannot justify transfer claims. It is not evidence about real simulator families; broader conclusions require more independently authored families and repeated leave-family-out evaluations.

The candidate-facing value is a separate CandidateView containing only feature-only development and holdout inputs, plus a candidate-safe manifest. Labels are available only through the query capability. It excludes source row IDs and model-family identity. The evaluator-bearing ProtocolSplit and its trusted manifest remain with the harness; custom debug output redacts both so formatting the whole split does not print per-sample targets or holdout metadata. This remains an API-level leakage barrier, not a process sandbox: untrusted candidate implementations must run separately, and the trusted harness must measure query usage externally. The PR includes a small CART-style Gini-split tree, but not a PRIM implementation, a vendor-matched CART configuration, an oblique tree, or a Symthaea-guided search method.

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

1. Qualify the scorer, protocol and new threshold baseline on exact-head CI; then build an isolated harness that instruments actual oracle/simulator usage and peak memory.
2. Add random/stratified sampling and reviewed PRIM/CART-compatible baselines; then add an oblique hypothesis class and Symthaea-guided discovery only behind the same protocol.
3. Report coverage-density-complexity-runtime trade-offs, calibration where probabilities exist, and recall-versus-query-budget curves with repeated seeds and uncertainty intervals.
4. Expand beyond the two reversed fixtures to more independently authored and physically admissible model families, with repeated leave-family-out evaluation.
5. Connect this benchmark to CECB-ROBUST-001 only after the reference suite, split design, and candidate/evaluator boundary receive review.

Related work: [CECB scorer and discovery PR #5515](https://github.com/Luminous-Dynamics/mycelix/pull/5515); [CECB-ROBUST-001 / #5510](https://github.com/Luminous-Dynamics/mycelix/issues/5510); [CECB-ROBUST-002 / #5512](https://github.com/Luminous-Dynamics/mycelix/issues/5512); [CECB-ROBUST-003 / #5517](https://github.com/Luminous-Dynamics/mycelix/issues/5517); [CECB-WITNESS-001 / #5519](https://github.com/Luminous-Dynamics/mycelix/issues/5519); [related Mycelix robust tournament PR #5511](https://github.com/Luminous-Dynamics/mycelix/pull/5511).
