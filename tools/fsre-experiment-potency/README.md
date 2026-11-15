# FSRE experiment potency E0 reference

A dependency-free Rust reference engine for selecting a cost-constrained subset of safe experiment designs over a finite declared set of worlds.

## Objective

For each selected experiment set, the per-world outcomes induce a deterministic partition of the declared world set. The residual identified-set width is the maximum query-value range among the partition cells:

\`R(S) = max_G (max(q(w) for w in G) - min(q(w) for w in G))\`.

Guaranteed potency is \`R(∅) - R(S)\`. The engine maximizes this quantity under the declared cost budget, subject to hard evidence/authority/rights/safety gates and schedule compatibility.

This is an E0 partition oracle, not a general SCM polynomial-programming implementation of the NeurIPS 2026 max-potency method.

## Run

\`\`\`sh
cargo fmt --manifest-path tools/fsre-experiment-potency/Cargo.toml -- --check
cargo test --locked --offline --manifest-path tools/fsre-experiment-potency/Cargo.toml
cargo clippy --locked --offline --manifest-path tools/fsre-experiment-potency/Cargo.toml --all-targets -- -D warnings
cargo run --quiet --locked --offline --manifest-path tools/fsre-experiment-potency/Cargo.toml -- tools/fsre-experiment-potency/fixtures/complementary-probes.psv
\`\`\`

## E0 fixture

The four query values are arranged as:

\`[[0, 1], [1, 2]]\`.

The row and column probes each reduce worst-case width from 2 to 1; together they uniquely partition the four worlds and reduce it to 0. The constant-output probe is exactly inert and can be pruned. The unpruned and pruned exhaustive searches must produce identical optimum, cost and tie sets.

Important mathematical boundary: with deterministic observation partitions and this max-over-cells width objective, two experiments that each have exactly zero individual potency cannot jointly have positive potency. If each leaves the global minimum and maximum worlds in one observation class, their joint signature also leaves those extremes together. Complementarity is therefore demonstrated with positive-but-modest individual potency, not the impossible zero-potency claim.

## Hard gates and schedule semantics

Authority, source-use, rights/ethics, safety/service-floor and physical-realizability states are non-compensatory candidate gates. \`UNKNOWN\` never becomes \`CLEARED\` or \`ESTABLISHED\`.

Intervals are half-open \`[start, end)\`. Mutually exclusive regimes on the same physical unit may not overlap. Sequential different regimes on one unit require \`NO_CARRYOVER\` or \`WASHOUT_VERIFIED\` declarations. This is a consistency check over declared schedule metadata, not physical certification.

## Claim ceiling

- Query values and observation outcomes are supplied finite-world inputs.
- The fixture is synthetic.
- Authority, source use, rights, safety and realizability labels are assertions, not authenticated evidence.
- Constant-output pruning is sound only relative to the declared world set.
- A successful E0 result does not prove the world set is complete, the causal model is true, the experiment will physically work or field outcomes will improve.
