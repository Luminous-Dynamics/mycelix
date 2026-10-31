# PRAXIS-NRES-015-BT1: Bounded bulk-tail credal reference

**Status:** synthetic public-development reference implementation; not calibrated, confirmatory or deployable.  
**Parent protocol:** PRAXIS-NRES-015 metric-transfer preregistration.  
**Evaluator:** benchmarks/nres-capability-loop/src/bulk_tail.rs.  
**CLI:** cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --bulk-tail.

## Purpose

BT1 supplies a small exact comparator for the bulk-tail method family registered in NRES-015. It isolates one question before a more expressive credal learner is attempted: what decision-risk upper bound follows when (a) the bulk distribution may be perturbed within a declared total-variation ball, and (b) total tail mass and per-outcome tail loss are separately bounded?

It is intentionally not presented as an implementation or reproduction of Chen et al. (ICML 2026), nor as a calibrated ambiguity set. It is a transparent reference model against which later variants can be tested.

## Model

Let B be the predeclared finite bulk support. Let p_B be a normalized conditional distribution over that set, L_B(s) the loss for each bulk state, and eta_B in [0,1] the total-variation radius. Total variation is defined as half the L1 distance:

U_B = sup over q with TV(q,p_B) <= eta_B of sum_s q(s) L_B(s).

For finite support, this upper expectation is exact by moving at most eta_B mass from the lowest-loss donor states to a maximum-loss receiver. Donors are exhausted in ascending loss order. The implementation reports both nominal and worst-case bulk expectation, along with the mass moved.

Let total tail mass t lie in a declared interval [t_low,t_high], and let M_T be a finite uniform upper bound on tail loss. The two-band worst-case upper expectation is:

U_BT = sup over t in [t_low,t_high] of ((1-t) U_B + t M_T).

Because the expression is affine in t, the exact maximum is at an interval endpoint: use t_high when M_T >= U_B, otherwise use t_low.

**Unbounded-tail rule:** when M_T is explicitly null and t_high > 0, the robust upper expectation is infinite. The JSON report uses a null numeric value and infinite-unbounded-tail status; it does not substitute a sample maximum, clip the tail, or silently return a finite number. If t_high = 0, the tail cannot contribute and the bulk bound remains finite.

## Input and governance contract

The fixture freezes the bulk state IDs and treats the complement of the finite support as the tail. Each profile declares:
- a normalized conditional bulk probability vector and aligned state losses;
- the bulk TV radius;
- lower and upper bounds on tail mass (the tail upper bound is at most 0.5, so the designated bulk remains the high-mass band);
- either a finite tail loss upper bound or explicit JSON null;
- a text statement identifying the basis of that tail bound or its absence.

The finite cap is only an input assumption. The evaluator cannot establish that an operational bound is physically, legally or empirically defensible. In a real study, the bulk partition, tail-mass interval and cap must be selected and frozen using only permitted training/validation evidence, with independent custody and a documented derivation. No confirmatory outcome may determine any of them.

## Public synthetic fixture arithmetic

The public fixture uses a 16-state support, eight frozen bulk state IDs and the eight-state complement as tail. It includes three illustrative cost profiles. Independent arithmetic gives:

| Profile | Nominal bulk loss | Worst bulk loss | TV mass moved | Tail interval | Tail cap | Overall upper risk |
|---|---:|---:|---:|---:|---:|---:|
| Low-cost | 5.32 | 6.42 | 0.10 | [0.05, 0.12] | 40 | 10.4496 |
| Moderate-cost | 9.10 | 11.80 | 0.18 | [0.10, 0.22] | 30 | 15.8040 |
| High-cost | 16.44 | 23.46 | 0.25 | [0.15, 0.35] | unbounded | infinite |

These numbers are arithmetic checks of an invented fixture only. They are not observed preparedness data, calibrated risks, forecast accuracy, realized regret, or evidence of comparative performance.

## Tests and controls

The module authors 15 Rust unit tests covering:
- hand-solvable two-state and multi-state TV transfers;
- monotonicity in the TV radius and the radius-one maximum;
- both tail-mass endpoint choices;
- positive admissible mass with an unbounded tail;
- zero tail mass with an unbounded cap;
- invalid probability mass, negative loss, active-curriculum flag, duplicate bulk IDs and invalid tail intervals.

The conformance artifact separately records 16 invalid-input controls, six boundary cases and six round-trip requirements. Rust compilation and test execution must be reported from an actual toolchain/CI run; authored tests are not passing tests.

## Known limitations and next research increments

1. **No calibration:** BT1 assumes the TV radius and tail-mass interval are supplied. It does not infer or calibrate them.
2. **No dependence model:** source-family clustering, temporal dependence and repeated measurements are outside this minimal evaluator.
3. **No geometry inside the bulk:** total variation is a transparent baseline, not a learned transport metric. The bulk state partition is fixed.
4. **Tail uniformity is strong:** a single cap must bound all admissible tail losses. A missing defensible cap correctly makes risk infinite whenever positive tail mass is possible.
5. **No decision optimizer:** BT1 evaluates a declared profile and loss vector; it does not yet select actions, learn loss-aware transport costs, or optimize bulk/tail partitions.
6. **No SOTA comparison:** future comparisons must use identical source-family splits, forecasts, actions and held-out loss profiles against NRES-013 fixed W1, NRES-014 conformal W1, CVaR and the preregistered loss-aware OT candidates.

The next substantive step is to connect BT1 to the NRES-013 action loss matrices, then add a train/validation-only bulk-selection procedure and held-out profile transfer evaluation. That must precede any confirmatory performance claim.
