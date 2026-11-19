# Mycelix Temporal Compute Placement Oracle v0.1

**Status:** Dependency-free Rust reference kernel with authored tests. Rust compilation and hosted workflow validation remain pending.

## Purpose

This package extends the static small-instance placement oracle with fixed task intervals. It chooses worker assignments but does not choose start times or implement a queueing policy.

Each task occupies its assigned worker over the half-open interval [start_ms, end_ms):
- start is inclusive;
- end is exclusive;
- end events are processed before start events at the same timestamp;
- adjacent intervals can reuse the same resource capacity;
- overlapping intervals consume aggregate CPU, memory and GPU capacity simultaneously.

An exact sweep-line check evaluates every relevant interval boundary. A placement is feasible only when every intermediate resource state fits the worker's declared capacity. The reported peak CPU, memory and GPU pressures are computed independently across every worker and time segment. They are not collapsed into one maximum, because a CPU-bound placement and a memory-bound placement are different trade-offs.

## Optimization and descriptive fields

The exact small-instance Pareto frontier uses five objective dimensions:
1. estimated cost from declared CPU/GPU hourly rates multiplied by fixed task interval duration;
2. remote input bytes;
3. distinct declared failure-domain IDs;
4. distinct declared operator IDs;
5. peak CPU pressure over time;
6. peak memory pressure over time;
7. peak GPU-slot pressure over time.

The number of regions used is reported as descriptive metadata but excluded from Pareto dominance and preference ordering. Hard permitted-region policy is enforced independently. Diversity labels do not prove independent operators, physical failure isolation or legal residency.

Search limits are explicit. If the node or frontier bound is reached, the result is Incomplete, never Optimal.

## Known-answer and adversarial suite

The ten authored unit tests cover adjacent interval reuse, overlapping overcommit rejection, nested intervals with simultaneous end/start events, hard region constraints, invalid windows, search budget exhaustion, frontier limits, deterministic input order, duration-sensitive cost, and a CPU-vs-memory bottleneck counterexample that preserves both non-dominated alternatives.

The JSON fixture is synthetic and demonstrates that two adjacent tasks each requiring the worker's full capacity can be placed on the same worker without overlap.

## Industry comparison

This is a reference subproblem, not a scheduler replacement.

- Ray: task/actor placement and data-locality-aware strategy.
- Kueue: quota reservation, AdmissionChecks and Topology-Aware Scheduling.
- Slurm: reservations, time limits and backfill scheduling.

Any comparison must freeze the same worker inventory, task windows, CPU/RAM/GPU requirements, locality policy and failure schedule. Unsupported semantics (fair-share, priority, preemption, dynamic start times, DAGs, elasticity) are recorded as incomparable rather than approximated silently.

## Exclusions

No dynamic start-time selection, deadlines, queue fairness, task DAG, priority, preemption, migration, elastic scaling, provider discovery/authentication, leases, runtime dispatch, output verification, measured energy, network-failure protocol or physical provider benchmark is implemented.

The result is placement feasibility over caller-supplied resource and time declarations. It does not authenticate inventory or authorize execution.

## Run

    cargo fmt --manifest-path mycelix-workspace/tools/temporal-compute-placement-v0.1/Cargo.toml -- --check
    cargo test --manifest-path mycelix-workspace/tools/temporal-compute-placement-v0.1/Cargo.toml --locked
    cargo clippy --manifest-path mycelix-workspace/tools/temporal-compute-placement-v0.1/Cargo.toml --locked --all-targets -- -D warnings
