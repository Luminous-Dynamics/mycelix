# FSRE finite-world partial-identification benchmark v0.1

## Purpose

Provide a small-instance exact oracle for partial-identification implementations. The oracle exhaustively evaluates an explicitly enumerated finite set of admissible worlds. It is not a generic causal estimator and is not intended to scale to exponentially large canonical state spaces.

## Estimand and input contract

Each row contains one potential decision-value and one safety value for an action in an admissible world:

    world_id|action_id|decision_value|safety_value

All quantities are integer fixed-point units. The input metadata declares the estimand, unit, decision-time cutoff, maximization direction, safety floor, assumption-set ID and frozen input-snapshot ID.

Every world must contain exactly one row for every action. The complete Cartesian grid is required; missing potential outcomes are not imputed. The declared world-set completeness is an assumption assertion and is never authenticated by the crate.

## Exact outputs

- Per-action sharp minimum and maximum over the declared finite world set.
- Point versus partial identification within that declared set.
- Per-action safety bounds and robust safety-floor qualification.
- Worst-case regret across the same aligned worlds.
- Robust dominance only when an action is no worse in every world and strictly better in at least one.
- Whether the maximizing action set is invariant across all worlds.
- Minimax-regret actions, restricted to actions satisfying the hard safety floor in every world.

The engine reports no numerical bounds when source-use permission is not authorized, authority is not established, or world-set completeness is unknown. A source-use or authority blocker is not converted into a wider interval or a penalty.

## What this does not establish

- The engine does not construct or validate the admissible-world set.
- It does not establish causal identification from observational or interventional data.
- It does not authenticate source owners, evidence, assumptions, or authority.
- It does not prove the declared world set contains every real-world possibility.
- It does not estimate population coverage or infer unobserved potential outcomes.
- A solver/oracle result is conditional on its declared inputs and model.

## Exact small-world benchmark

Use exhaustive enumeration as the oracle for later LP/MILP or canonical-domain reduction implementations. Generate small binary and integer outcome models and enumerate every admissible world independently in the test harness.

Acceptance targets:
- 100% exact equality with independent exhaustive extrema on all frozen E0 fixtures.
- 100% truth inclusion when the declared finite world set is known to contain the generated truth.
- 0 numerical output when source-use, authority, or world-set completeness gates fail.
- 0 incomplete world/action grids silently accepted.
- 0 recommendations where no action passes the service floor in every world.
- 100% byte-identical output under row permutation.
- 100% exact agreement for worst-case regret and invariant action-rank classification.

## Scale and algorithmic roadmap

The reference oracle is bounded to one million input rows and is intended for small finite worlds. Benchmark input sizes at 10^1, 10^2, 10^3, 10^4, 10^5 and the declared row cap, recording p50/p95 runtime and peak memory on fixed hardware.

For larger implicit world sets, compare:
1. Canonical-domain reduction and constraint-aware state quotienting.
2. LP formulations over compatible structural causal models.
3. MILP formulations where discrete identity, split/merge or allocation constraints require integrality.
4. Robust optimisation with explicit uncertainty sets.
5. Conservative Manski-style bounds as a baseline.

The LP/MILP backend must be checked against this exact oracle on overlapping small instances. Report solver status, primal/dual residuals or certificates as available, integrality gap, runtime, memory and independent verification. A solver certificate verifies the encoded optimisation problem, not the causal assumptions or the source evidence.

## Matched comparison protocol

Freeze the estimand, units, action set, safety floor, evidence cutoff, admissible-world construction, assumptions and hardware. Compare bound validity, interval width, decision-rank invariance, worst-case regret, false-safe rate, abstention and runtime. Do not claim superiority from unmatched world sets or synthetic cases alone.

## Next increment

Add multiple outcome metrics and a separate boundary/identity allocation model. Compose with #6184 service-universe qualification, #6185 temporal lineage, #6190 frame-dependence qualification and #6178 realised regret. Do not collapse rights, authority, safety, identity and statistical uncertainty into one scalar confidence score.
