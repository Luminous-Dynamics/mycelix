# Craft sequential EVSI oracle V0.1

**Status:** independent test-only exact finite-state sequential acquisition reference. No production planner, sensor calibration, legal authority, field acquisition or autonomous action is implemented.

## Paradigm shift: plan evidence sequences, not isolated observations

One-step EVSI can miss complementary evidence. The frozen XOR counterexample has two equally informative latent bits (X) and (Y), and a terminal decision that depends on (X \oplus Y). Each perfect observation alone leaves terminal expected loss at 5, so one-step EVSI is 0 and greedy one-step EVSI stops. Acquiring both reveals parity exactly: terminal loss 0, gross EVSI 5, total acquisition cost 0.6, net decision value 4.4.

This demonstrates an algorithmic failure mode for greedy one-step acquisition, not a universal empirical result. A finite-horizon dynamic-programming policy can choose an observation, condition on its result, select a second observation, and stop with a terminal action. The budget applies per realized branch; the cost objective is expected acquisition cost.

## Exact model and assumptions

- Finite latent-state distribution and declared action-loss table.
- Explicit safe-action set; terminal choice is minimum expected loss among those actions.
- Finite categorical observation likelihoods.
- Bounded horizon and hard per-path acquisition budget.
- No repeated acquisition of the same candidate in one path.
- Mandatory evidence preempts optional value optimization.
- Unavailable, out-of-domain, inadmissible or unconditioned observation models are excluded; if no admissible observation remains while observations were declared, the oracle abstains.
- For multiple observations, the case must declare `conditional_independence_given_complete_latent_state_v0.1`. The observations are then assumed conditionally independent given the complete latent state. Unknown dependencies or shared causes that are not represented in that latent state require abstention or a richer joint/history-conditioned model.

The DP is cross-checked on the frozen tiny case against a separate, deliberately unmemoized policy-tree enumerator.

## Baselines and benchmark metrics

- Mandatory-only / stop-now.
- Random-safe and expert-priority (to be added in the comparative harness).
- One-step EIG greedy.
- One-step EVSI greedy, constrained by the same budget and mandatory-evidence gate.
- Exact bounded sequential DP.
- Full-horizon clairvoyant hindsight optimum (future benchmark upper bound only).

Report gross EVSI and EIG separately; net decision value after cost; exact expected terminal loss; acquisition cost; regret against the exact constrained oracle; policy depth; budget violations; mandatory-evidence compliance; abstentions; state-space size; p50/p95/p99 runtime and memory by horizon/candidate/state count. No aggregate score can hide a mandatory-evidence or safety violation.

## Local validation

Go 1.23.2: frozen fixture tests, deterministic policy-tree tests, race detector and `go vet` passed. Statement coverage: 93.2% for this small finite-state reference package. These local tests validate the declared finite model and fixture only; they are not hosted exact-head CI or field/empirical performance.

## Next research steps

1. Generalize from the complete-latent-state factorization to explicit joint likelihoods and history-conditioned dependencies.
2. Add three-to-five-observation complementary and redundant modality fixtures.
3. Compare sequential DP against one-step EVSI, EIG, fixed, random-safe and expert policies over a preregistered synthetic-world suite.
4. Add brute-force/oracle agreement and property-based small-domain sweeps.
5. Add calibration, misspecification and selection-bias cases; never treat assumed likelihoods as calibrated.
