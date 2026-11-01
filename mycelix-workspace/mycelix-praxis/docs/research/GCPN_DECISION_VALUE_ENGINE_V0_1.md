# GCPN Decision-Value Verification Engine v0.1 — Stage B

**Status:** Rust source authored; Cargo build, Rust tests and CI remain outstanding.  
**Parent issue:** #5665.  
**Specification PR:** #5666.  
**Package:** `benchmarks/gcpn-decision-value-verification`.  
**Dependencies:** none; isolated Cargo package.

## 1. Implemented cost/loss core

The first package increment evaluates binary probabilistic forecasts under one frozen synthetic decision profile.

The profile declares:
- action cost;
- avoidable event loss when no action is taken;
- residual event loss under action;
- minimum action lead time;
- a stable profile identifier.

For the simple loss matrix, the action/no-action indifference threshold is computed analytically as:

`p* = action_cost / (avoidable_event_loss - residual_event_loss_under_action)`.

The evaluator validates the profile and refuses invalid or infeasible cost/loss combinations. It does not tune this threshold against the evaluation outcomes.

Each synthetic case binds candidate/reference probability forecasts, climatology probability, shared outcome, issue times, decision cutoff, event-valid time, action feasibility, independent-system identifier and verification source/version.

## 2. Forecast quality and decision value remain distinct

The evaluator calculates candidate/reference Brier scores on identical cases where both forecasts were issued before the event valid time.

Separately, it calculates realized losses under:
- candidate forecast policy;
- reference forecast policy;
- climatology policy;
- never act;
- always act when feasible;
- perfect-information lower-bound oracle (infeasible reference only).

It then reports net benefit relative to each named baseline and candidate regret relative to the perfect-information oracle. A late forecast can still be a valid pre-event forecast score while being unusable at the decision cutoff. The engine keeps those two properties separate.

The policy engine records false actions, missed events, late forecasts, insufficient action lead, infeasible cases and usable decision cases separately. The result report carries the complete frozen profile values (action cost, avoidable loss, residual loss and minimum action lead), the analytic threshold, and verification source/version so that a policy-loss number is not detached from the assumptions that produced it.

## 3. Ranking-reversal oracle

The frozen synthetic profile uses:
- action cost: 20;
- avoidable event loss: 100;
- residual event loss under action: 0;
- minimum action lead: 30 seconds;
- analytic threshold: 0.20.

The four-case DGP contains two events followed by two non-events.

| Model | Probabilities | Mean Brier | Realized policy loss |
|---|---|---:|---:|
| Candidate | 0.90, 0.19, 0.19, 0.19 | 0.184575 | 120 |
| Reference | 0.21, 0.21, 0.21, 0.21 | 0.334100 | 80 |

The candidate has the better Brier score but higher realized loss under this particular frozen decision profile. Its net benefit versus reference is −40 synthetic cost units. Never-act loss is 200, always-act loss is 80, and perfect-information loss is 40.

This is an intentionally constructed ranking reversal demonstrating context dependence. It does not establish that Brier score is inferior, that the reference is universally better, or that the synthetic threshold should be used in the real world.

## 4. Threshold sensitivity without test-set tuning

The package exposes `evaluate_threshold_grid`, which evaluates a caller-supplied, strictly increasing grid of thresholds and reports candidate loss, reference loss, net benefit, false actions and missed events at every point.

It validates grid values and does not search for, optimize or select a threshold. The profile threshold remains the analytic cost/loss value; a sensitivity grid is a separate, predeclared diagnostic.

## 5. Matched verification source

Each decision case records a verification source and version. The evaluator rejects a panel that mixes verification sources or versions. It also rejects duplicate case IDs, invalid probabilities, invalid issue/valid/cutoff chronology and missing independent-system identity.

Candidate and reference Brier scores use the exact same paired subset: cases where both forecast vintages existed before the event valid time. A forecast that arrives after the decision cutoff but before the event remains eligible for forecast scoring, while its decision policy falls back to no action and increments the late-forecast count. A forecast issued after the event valid time is excluded from the paired score denominator.

## 6. Authored tests

Twelve Rust tests are included:

1. analytic threshold is action cost divided by avoidable event loss;
2. proper-score and decision-value rankings can reverse, with exact Brier and realized-loss oracles;
3. perfect information is an explicitly infeasible lower-bound comparator;
4. never-act and always-act baselines remain separate;
5. late forecast is unusable for action but remains in a pre-event score panel;
6. forecasts issued after valid time are excluded from the paired score denominator;
7. insufficient action lead and infeasible cases are reported separately;
8. invalid cost/loss profile fails closed;
9. duplicate case IDs and invalid probabilities are rejected;
10. independent-system identity is required;
11. a frozen threshold grid reports each threshold point and rejects unordered/duplicate values;
12. mixed verification sources are rejected.

These are authored source tests, not executed results.

## 7. Limitations and next steps

Not implemented in this increment:
- full DGP families for rare events, profile heterogeneity, action effectiveness and informative cluster sizes;
- exact population expected-loss oracles across a full cost/loss grid;
- bootstrap uncertainty over independent systems for score and utility differences;
- multiple-profile fairness/dispersion report without implicit averaging;
- action-effectiveness uncertainty and sensitivity;
- serialized profile fingerprint and output schema;
- real WeatherBench/WMO public forecast adapter;
- human-subject or operational validation.

The package currently reports realized sample costs under one synthetic profile. It is not a general economic valuation engine or a causal estimate of avoided impact.

## 8. Validation status

- Isolated Cargo manifest, Rust library and CLI are on the implementation branch.
- Source-level review checks the analytic threshold, ranking-reversal arithmetic, paired denominator and decision cutoff treatment.
- No Cargo build, Rust test execution, CLI run, statistical uncertainty qualification or CI pass is claimed because this authoring environment has no Cargo or rustc.

## 9. Research alignment and non-claims

The direction is grounded in the 30 August 2026 decision-relevant forecast-evaluation literature and user-focused verification research. This package is a synthetic evaluator only.

No operational warning threshold, real user preference model, action recommendation, funding release, emergency decision, lives-saved estimate or causal impact claim is produced.
