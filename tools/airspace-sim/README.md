# Airspace Replay

A small Rust-first, deterministic simulator/replay harness for the research contract in [AIRSPACE_COORDINATION_ARCHITECTURE_V1.md](../../docs/aerocommons/AIRSPACE_COORDINATION_ARCHITECTURE_V1.md).

## What it does

- Replays 16 versioned synthetic flight-trajectory fixtures, 13 lifecycle-transition fixtures, 14 reservation-snapshot fixtures, 16 reservation-event-stream fixtures, and 21 dynamic airspace-constraint fixtures.
- Uses explicit millimetre / millimetre-per-second integer units and a fixed simulation window.
- Refuses to compare coordinates across unqualified clocks, incompatible reference frames, or vertical datums.
- Rejects expired, withdrawn, ended, malformed, out-of-bounds, or over-budget flight-intent fixtures.
- Rejects duplicate flight identifiers before pairwise conflict work, preserving unambiguous attribution in the synthetic report.
- Validates lifecycle transitions against stable intent/issuer/trajectory/time-basis identity, exact successor sequence numbers, monotonic event time, and explicit state-transition rules.
- Compares horizontal and vertical separation independently.
- Adds conservative relative-motion margins to the caller-supplied threshold so a potential encounter between sample instants is less likely to be missed.
- Requires an authority review for an emergency-priority scenario if no geometric conflict has already been found.
- Checks half-open reservation intervals across providers without selecting a winner or applying last-writer-wins to duplicate reservation IDs.
- Treats expired, withdrawn, or ended claims as non-current; flags duplicate versions and incompatible time bases for reconciliation.
- Never reports a no-conflict result from a partitioned or stale/incomplete provider snapshot.
- Keeps emergency claims from automatically preempting conflicting claims; emergency cases require the competent operational process.
- Replays provider-local reservation updates by explicit sequence, detecting gaps, forks, broken predecessor links, time reversal, changed reservation semantics, and illegal state transitions.
- Reconciles versioned airspace-constraint updates, including activation, changed volumes, withdrawal, exact retransmission, contradictory digest reuse, source identity changes, and stale/partitioned views.
- Screens current flight-intent envelopes against active 4D axis-aligned restriction volumes with explicit coordinate frame, vertical datum, time basis and validity.
- Documents a proposed geographic interchange boundary in [AIRSPACE_GEOMETRY_INTEROPERABILITY_PROFILE_V1.md](../../docs/aerocommons/AIRSPACE_GEOMETRY_INTEROPERABILITY_PROFILE_V1.md); AIXM/OGC mappings are future adapters, not implemented or qualified by this crate.
- Uses continuous-time slab intersection for constant-velocity trajectories against expanded axis-aligned 4D restriction boxes; no temporal point samples are used to discover intersections.
- Adds a separate synthetic local-planar polygon kernel with exact rational segment-boundary events, polygon-ring/self-intersection checks, hole validation, vertical-slab intersection and half-open time windows. The 34-case fixture corpus in `fixtures/polygon_cases.json` is independently replayed by Rust unit tests; it does not change the existing axis-aligned-box evaluator.
- Uses closed spatial boundaries for conservative screening, including tangency; excludes polygon-hole interiors but treats hole boundaries as occupied for impact screening.
- Returns `unresolved` for nonzero flight uncertainty, incompatible frame/datum/time basis, or unqualified time rather than silently discarding uncertainty or transforming coordinates.
- Uses a bounded synthetic uncertainty horizon to expand the box by velocity-scaled guard margins. This reduces dependence on sample rate but is not a validated flight uncertainty or reachable-set model.
- Reports known impacts as `reassessment_required` even when the input view is stale, and uses `degraded` rather than no-impact when no hit is observed in an incomplete view.
- Treats input arrival order as non-semantic when a complete sequence chain is available; idempotently deduplicates only exact full-event retransmissions and counts them in the report.
- Projects only accepted/activated, unexpired reservation streams into the conflict screen; proposals are not accepted slots.
- Rejects snapshots above the bounded claim/pairwise-work budget and received event histories above 1,000 records.
- Produces machine-readable JSON with expected-vs-actual classifications and explicit authority scope.
- Maintains a machine-checked traffic-diversity catalogue with 15 real-world archetypes and 12 planned interaction cases. These are explicitly design metadata, not executable traffic simulations; real-world classes are marked unsupported by the current toy kernel.
- Requires every replay and polygon-screen flight to carry explicit traffic and model profile IDs. The executable applicability registry admits only the synthetic-fixture / constant-velocity-toy pair; missing, unknown, or real-world traffic profiles fail closed as unresolved before trajectory, dynamic-restriction, or polygon-intersection work. This gate constrains model selection; it does not validate the toy model or establish aviation safety.
- Defines a 12-case paradigm-shift research manifest at `fixtures/paradigm_shift_benchmarks.json` covering independent checking, model-envelope falsification, corridor optionality, time-dependent capacity, federation partitions, non-compensatory hard gates, source-byte/freshness/rights separation, unknown traffic, community externalities, counterfactual recovery and cross-project reproducibility. The manifest is deliberately marked as research design, not executable or passed benchmarks; a unit test enforces unique IDs, falsifiers, evidence requirements and honest status vocabulary.
- Documents the proposed research programme in [BEYOND_TRAJECTORY_CLEARANCE_RESEARCH_V1.md](../../docs/aerocommons/BEYOND_TRAJECTORY_CLEARANCE_RESEARCH_V1.md). It positions Mycelix as a coordination/evidence fabric complementary to existing UTM/ATM work, not as a replacement for competent aviation authorities.

## Run

From the repository root:

```bash
cargo test --manifest-path tools/airspace-sim/Cargo.toml
cargo run --manifest-path tools/airspace-sim/Cargo.toml -- --verify
```

Or pass a specific scenario-set JSON path after `--verify`.

## Classification semantics

- `accept_for_planning_only`: no conflict was found within the synthetic model and declared scope.
- `conflict`: at least one pair meets the conservative synthetic conflict predicate.
- `degraded`: the supplied provider snapshot is partitioned, stale, or incomplete, so absence of a conflict cannot be established.
- `reject`: malformed, expired, or out-of-bounds fixture inputs.
- `unresolved`: missing time qualification or incompatible coordinate frame / vertical datum.
- `requires_authority_review`: an emergency-priority intent/claim remains subject to competent operational review.
- `no_conflict_in_provided_snapshot`: no overlap was found in the supplied complete snapshot; this is not a slot grant, proof of globally conflict-free airspace, or flight authorization.
- `reassessment_required`: the continuous synthetic screen found a trajectory/constraint intersection; this requests competent-service reassessment, not a maneuver or authorization decision.

## Reservation snapshot semantics

Reservation intervals use half-open semantics: `[start_s, end_s)`, so `[100, 200)` and `[200, 300)` are adjacent but do not overlap. Two current claims conflict only when they name the same resource, share a qualified time basis, and their intervals overlap. Expiry and terminal state are considered before pairwise conflict checking.

The snapshot evaluator accepts one asserted snapshot. The separate event evaluator sorts provider-local streams by sequence and checks contiguous numbering, predecessor linkage, monotonic issue time, stable reservation semantics, and legal lifecycle transitions. Gaps, forks, or contradictory payloads are unresolved; exact full-event retransmissions are deduplicated and counted. Opaque digest fields are fixture linkage labels, not computed hashes, signatures, or authenticated evidence. Cross-provider claims are screened for conflict, but this does not implement global causal ordering, authenticated exchange, partition merge, supersession, or winner selection. A `complete_fresh` marker is a synthetic input, not independently verified network evidence. Incomplete views cannot yield a no-conflict result. Emergency priority does not automatically preempt another reservation.

## Important limitations

This is **not** an aircraft simulator, flight dynamics model, UTM/ATM service, collision-avoidance system, regulator, or authorization service. The new planar polygon kernel is restricted to synthetic local Cartesian millimetres and zero-uncertainty centerlines. Geographic polygon/polyhedron intersection, CRS transformation, uncertainty-envelope intersection, and authoritative AIXM / OGC Moving Features adapters remain future work in the linked interoperability profile, not implemented or qualified here. It does not model provider-network replication, signatures/identity authentication, cross-provider causal ordering or partition merge, reservation supersession, three-dimensional polygon/polyhedron geometry, time-varying wind/weather, terrain, sensor error distributions, non-cooperative traffic, or actual certified operating procedures.

The trajectories are straight and constant-velocity. The conservative margin is based on relative velocity and the maximum sampling interval; it is useful for testing the contract but is not a formally validated reachability analysis. The coordinate frame and separation thresholds are synthetic fixture values. They are not official separation minima and must never be reused for live operations.

A passing replay means only that the implementation matched these fixture expectations. It does not establish safety, airworthiness, certification, or permission to fly. Production evolution would require qualified aerospace expertise, independent verification, validated dynamics and uncertainty models, and competent-authority engagement.
