# TCDM Transition Contract V1 — Monotonic Time and Evidence-Weighted Updates

**Status:** candidate transition contract; research-only, not production behavior  
**Baseline:** `Luminous-Dynamics/mycelix@3a7833513144e90c230a65488c78e7c31bf685a3`  
**Follow-up issue:** [#5222 — make TCDM time and evidence-weight updates fail-closed](https://github.com/Luminous-Dynamics/mycelix/issues/5222)  
**Inspected source:** `crates/mycelix-core-types/src/trust.rs`, blob `cb2d5c18dd58be7d237cc2050d4df1b55fd4d62a`

## What this work establishes

The inspected `TCDMState` implementation calculates elapsed time with direct `i64` subtraction. A timestamp regression may produce a negative duration and a decay factor above one; extreme timestamps may overflow the subtraction. Interaction updates cap weight from above but do not explicitly reject negative or non-finite inputs; the confidence increment is fixed rather than proportionate to evidence weight. Interaction-count overflow and public/deserialized invalid state also need explicit contracts.

These observations motivate a testable candidate policy. They are not proof that every caller can reach the invalid states, and this PR does not change production behavior.

## Candidate transition rules

### State domains

- `trust_level` and `confidence` must be finite and in ([0,1]).
- `decay_rate` must be finite and in ([0,1]). Rate (0) means immediate zero after positive elapsed time; rate (1) means no decay.
- `interaction_count` must fit `u32`.
- Invalid state or inputs return an error without committing partial changes.

### Time decay

Preserve the existing zero timestamp as a first-observation sentinel: a first valid decay call sets the timestamp watermark but does not apply decay. After initialization:

- timestamps are monotone nondecreasing;
- backward time rejects and never increases trust/confidence or rolls the watermark backward;
- elapsed-time arithmetic uses checked subtraction, rejecting `i64` overflow;
- equal timestamps cause no decay;
- for (Delta tge0), trust is multiplied by (r^{Delta t/86400}) and confidence by the square root of that factor.

The monotonic rule is a safety contract, not a claim that wall-clock timestamps are globally trustworthy. A production integration should prefer a trusted time source, define restart/clock-correction behavior, and distinguish event time from processing time.

### Evidence-weight update

- Weight must be finite and nonnegative; the existing upper cap of 0.3 is preserved.
- Zero weight means **no evidence**: no score, confidence or count change.
- For a positive weight, (alpha=min(weight,0.3)), (x=1) for success or (x=0) for failure, and the candidate update is (T'=T(1-alpha)+xalpha).
- The candidate confidence increment is (0.05\cdot(\alpha/0.3)), capped at 1.0, so zero-weight evidence cannot increase confidence and smaller weights contribute proportionately less.
- Interaction count increments only for positive-weight evidence and uses checked arithmetic. Overflow rejects before state mutation.

These confidence semantics are a proposed policy to test, not a theorem or an empirically calibrated confidence model. They should be compared against the existing policy using held-out streams and reviewed before production adoption.

## Frozen fixture and Rust oracle

The v1 corpus has **30 cases: 17 valid transitions and 13 invalid inputs**. It includes first timestamp initialization, same-time and multi-day decay, rate boundaries, backward time, `i64` subtraction overflow, invalid/non-finite state, success/failure evidence, capped and zero/subnormal weights, `f32` maximum finite weight, non-finite weights and `u32` count overflow.

The separate Rust integration oracle validates those examples without invoking `TCDMState`. It also runs a deterministic **100,000-step** sequence of candidate interactions and daily decay, asserting finite bounded trust/confidence throughout. Five mutation controls target backward-time score boosts, negative-weight acceptance, fixed confidence increments for zero weight, unchecked arithmetic and invalid state propagation.

It uses only the existing test `serde_json` dependency. The exact-head workflow pins Rust 1.96.0 and the checkout action, runs formatting and locked metadata checks, mechanically verifies equality of all three trust source copies and both MATL bridge copies, executes the oracle and core-type library tests, and records exact source identity plus clean postflight. Any mirror divergence fails closed. This workflow qualifies a research reference only.

## Mirror and runtime constraint

At the inspected baseline, these three trust source files are byte-identical (blob `cb2d5c18dd58be7d237cc2050d4df1b55fd4d62a`):

- `crates/mycelix-core-types/src/trust.rs`
- `mycelix-core/libs/mycelix-core-types/src/trust.rs`
- `mycelix-workspace/mycelix-core/libs/mycelix-core-types/src/trust.rs`

The two MATL bridge files `mycelix-core/libs/matl-bridge/src/tcdm.rs` and `mycelix-workspace/mycelix-core/libs/matl-bridge/src/tcdm.rs` were also byte-identical (blob `3c216bea351d4fd9cfc026963f99ba794b4add80`). The nested workspace bridge directly consumes `mycelix_core_types::trust::TCDMState`.

Therefore a standalone-path patch alone cannot support a claim that the full workspace runtime is hardened. The owning manifest, live bridge consumer and mirror synchronization path must be established before any production adapter is approved.

## Proposed acceptance gates

| Measure | Initial target |
|---|---:|
| Frozen corpus | 17/17 valid transitions accepted; 13/13 invalid inputs rejected |
| Generated invariant run | 100,000 steps; zero NaN/Inf or out-of-range states |
| Time monotonicity | zero backwards-time boosts and zero watermark regressions |
| Input mutation | zero negative/non-finite weights accepted |
| Zero-weight semantics | zero state/count/confidence changes |
| Counter overflow | zero unchecked wraps |
| Mirror consistency | exact source parity or a single authoritative shared implementation |
| Compatibility | existing MATL/TCDM behavior compared on frozen workloads before adoption |
| Production correspondence | **not claimed by this PR** |
| Sybil/correlation resistance | **not claimed by this primitive** |

## Recommended next experiments

1. Run old and candidate transitions on identical deterministic and held-out interaction histories; report score delta, confidence growth, zero-weight handling and time-regression outcomes.
2. Use an adversarial sweep over failure runs, success runs, source concentration, missing interactions and timestamp correction scenarios.
3. Measure calibration separately; confidence increment is not automatically a probability of correctness.
4. Resolve workspace ownership, then port only the reviewed rule set to the authoritative type and keep all mirrors mechanically synchronized.
5. Add a compatibility migration that exposes rejected input to callers rather than silently swallowing it; never weaken gates to get a green run.

No MATL threshold, governance eligibility policy, identity rule or production trust score changes in this research PR.

## Sources

- [TCDM source at inspected baseline](https://github.com/Luminous-Dynamics/mycelix/blob/3a7833513144e90c230a65488c78e7c31bf685a3/crates/mycelix-core-types/src/trust.rs)
- [MATL bridge source at inspected baseline](https://github.com/Luminous-Dynamics/mycelix/blob/3a7833513144e90c230a65488c78e7c31bf685a3/mycelix-workspace/mycelix-core/libs/matl-bridge/src/tcdm.rs)
- [Issue #5222](https://github.com/Luminous-Dynamics/mycelix/issues/5222)
- [Posterior reference PR #5221](https://github.com/Luminous-Dynamics/mycelix/pull/5221)
