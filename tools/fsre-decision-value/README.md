# FSRE decision-value reference evaluator

A dependency-free Rust library that computes finite-state expected value of sample information (EVSI) using exact integer weights and loss units.

## Scope

The reference evaluator accepts:
- competing hypotheses with positive integer prior weights;
- candidate actions with a loss for each hypothesis and an explicit admissibility flag;
- a required admissible `NO_ACTION` alternative;
- measurements with authorised/safe flags, acquisition and delay cost;
- outcome likelihood weights for every hypothesis.

For every measurement, outcome likelihood weights for each hypothesis must sum exactly to the declared likelihood scale. This makes the finite synthetic probability model auditable and avoids floating-point drift in the reference calculation.

The evaluator returns:
- the best admissible baseline action;
- the best admissible action conditional on each possible outcome;
- baseline and expected post-measurement loss as exact rationals;
- gross EVSI and net EVSI after acquisition and delay cost;
- a deterministic positive/non-positive/blocked disposition.

## Hard gates

- An unauthorised measurement is blocked without an EVSI calculation.
- An unsafe measurement is blocked without an EVSI calculation.
- Inadmissible actions never participate in the loss minimisation.
- Every case must include an admissible `NO_ACTION` option.
- A measurement with a zero-probability outcome, malformed likelihood vector, duplicate identity or likelihood weights that do not sum to the declared scale is rejected.
- Integer overflow returns an error; values are never silently saturated in EVSI arithmetic.

This v0.1 evaluator keeps the action admissibility set fixed before measurement. An observation cannot create authority or make a previously inadmissible action eligible. More detailed outcome-conditioned physical feasibility and authority remain owner-native qualification results and are a future extension.

## Exact calculation

For prior weights (w_h), likelihood weights (q_{y,h}), likelihood scale (Q), and loss (L(a,h)):

`baseline = min_a [Σ_h w_h L(a,h)] / Σ_h w_h`

`post = [Σ_y min_a Σ_h w_h q_{y,h} L(a,h)] / [Q Σ_h w_h]`

`gross_EVSI = baseline - post`

`net_EVSI = gross_EVSI - acquisition_cost - delay_cost`

All quantities are returned as exact rational numerators and denominators. Costs and losses must use the same declared units. The caller must pre-register that loss unit and cost model; this crate does not invent them.

## Run

```sh
cargo test --locked --offline --manifest-path tools/fsre-decision-value/Cargo.toml
```

The v0.1 library currently has 16 unit tests covering exact positive EVSI, high-information/zero-decision-value observations, an uninformative signal, a 101-point signal-quality sweep, a 50-case prior/signal matrix, authorisation and safety blocks, malformed likelihoods, excluded actions, duplicate identities, deterministic ordering, scale invariance and acquisition/delay cost thresholds. The 50-case matrix is a generated development property test, not an independently authored or sealed benchmark corpus. The focused workflow runs formatting, tests and Clippy; queued or skipped is not a pass.

The v0.1 tool is a deterministic finite-case reference, not a calibrated model, sensor scheduler, causal engine or operational policy. Its unit tests use declared synthetic values only. The API accepts already qualified case inputs; it does not gather data, authorize measurements or actuate systems.
