# FSRE decision defeater oracle v0.1

**Status:** Bounded, dependency-free E0 reference implementation for a declared finite perturbation universe.

This crate is a small-instance correctness oracle for #6210 and the protocol in `research/fsre/DECISION_DEFEATER_ANALYSIS_BENCHMARK_V0_1.md`. It is not a scalable optimizer, causal discovery engine, physical model, source authenticator or deployment authorization service.

## Supported model

- A complete finite world × action grid with integer decision and safety values.
- A fixed MAXIMIZE objective and a fixed service-safety floor.
- Four decision properties:
  - `ROBUST_SAFE_ACTION_RETAINED`
  - `NONEMPTY_SAFE_ACTION_SET`
  - `INVARIANT_UNIQUE_OPTIMUM`
  - `MINIMAX_REGRET_SET_UNCHANGED`
- Typed, additive `DECISION_VALUE_DELTA` and `SAFETY_VALUE_DELTA` perturbations against existing world/action cells.
- Explicit mutation cost, evidence reference and admissibility disposition.
- Exhaustive enumeration of every subset up to 22 admissible perturbations.
- Deterministic minimum-cost witness set, tie counting, truncation disclosure and a separate replay operation.
- A test-only independent exhaustive reference for the frozen pump-station fixture; this is not a general independent verifier.

Identity, service-boundary allocation, source-dependency, source-rights, temporal-lineage and physical-parameter mutations are not yet executable in v0.1. They must be added as typed mutations in later increments; this version returns an explicit unsupported/indeterminate disposition rather than treating them as searched.

## Run

    cargo fmt --manifest-path tools/fsre-decision-defeater/Cargo.toml -- --check
    cargo test --locked --offline --manifest-path tools/fsre-decision-defeater/Cargo.toml
    cargo run --quiet --locked --offline --manifest-path tools/fsre-decision-defeater/Cargo.toml -- tools/fsre-decision-defeater/examples/pump-station.psv

To replay a selected witness, use the library API `replay_witness` with the original input and the witness mutation IDs. This recomputes the selected subset against the frozen input; it is a deterministic self-replay, not an independent implementation.

## Input format

Metadata is strict `#key=value` text, followed by a world/action table, the literal `MUTATIONS` separator and a mutation table.

World header:

    world_id|action_id|decision_value|safety_value

Mutation header:

    mutation_id|mutation_kind|world_id|action_id|delta|cost|evidence_ref|admissibility

Supported mutation kinds are `DECISION_VALUE_DELTA` and `SAFETY_VALUE_DELTA`. Multiple deltas to the same cell compose additively using wide-integer accumulation; the v0.1 mutation cap bounds the maximum accumulated value. Mutation IDs are unique and canonicalized lexically before enumeration.

Required metadata:

- `analysis_id`, `input_snapshot_id`, `decision_cutoff`
- `decision_direction=MAXIMIZE`
- `decision_property`
- `safety_floor`
- `source_use_status=AUTHORIZED`
- `authority_status=ESTABLISHED`
- `world_set_completeness=DECLARED_COMPLETE`
- `perturbation_universe_completeness=DECLARED_COMPLETE`
- `cost_unit`

`target_action` is required for the retained-safe-action and unique-optimum properties. `max_subsets` and `max_witnesses` are optional bounded resource controls.

Admissibility is not a cost: `BLOCKED` mutations are excluded and counted; any `UNKNOWN` mutation makes the perturbation universe not identified and prevents numerical search. A global rights/authority gate blocks before world or mutation rows are parsed. These are caller-supplied assertions and are explicitly reported as unauthenticated.

## Output semantics

- `DEFEATER_FOUND` with `PROVEN_MINIMAL`: complete exhaustive search found the minimum declared mutation cost.
- `NO_DEFEATER_FOUND_IN_DECLARED_UNIVERSE`: complete exhaustive search found no defeater in this finite declared universe.
- `SEARCH_LIMIT_EXCEEDED`: search incomplete; any returned witness is only `WITNESS_ONLY`, with its cost an upper bound, not a proven margin.
- `BLOCKED_SOURCE_USE`, `BLOCKED_AUTHORITY`, `NOT_IDENTIFIED_SOURCE_USE`, `NOT_IDENTIFIED_AUTHORITY`, `NOT_IDENTIFIED_WORLD_SET`, `NOT_IDENTIFIED_PERTURBATION_UNIVERSE`, `UNSUPPORTED_MUTATION`: no numeric margin.
- A finite negative search is never represented as universal robustness.

The engine stores every minimum-cost witness up to the declared cap, reports the total tie count and marks truncation explicitly. It does not convert timeout, unsupported mutation or unknown admissibility into a negative search result.

## Claim boundary

The result is exact only for the supplied finite grid and additive perturbations declared admissible by the caller. It does not prove the perturbation universe is complete in reality, authenticate evidence or permissions, establish causal truth, prove an outcome change is physically realizable, or make an infrastructure decision on behalf of an authority.
