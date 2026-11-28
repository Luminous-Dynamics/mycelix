# Craft sequential EVSI oracle V0.1

Test-only exact finite-state, finite-horizon acquisition planner. It performs adaptive recourse: after each observation result, it may select another observation or stop and choose the minimum expected-loss safe action.

## Frozen counterexample

Four equally likely states encode two latent bits X and Y. The optimal action depends on X XOR Y, with wrong-action loss 10. The prior expected decision loss is 5. Observation X alone and Y alone each have one-step EVSI 0, so one-step greedy EVSI stops. Acquiring both at cost 0.3 each reveals parity, produces terminal loss 0, gross EVSI 5, and net decision value 4.4 under budget 0.6.

The planner refuses to combine multiple observation models unless the case declares `conditional_independence_given_complete_latent_state_v0.1`. Shared causes must be represented in the latent state or with richer history-conditioned likelihoods; unknown dependency causes abstention. This finite model is an oracle, not a calibration claim.

## Local validation

Go 1.23.2: package tests, race detector, vet passed; 93.2% statement coverage. Frozen tests cover complementary information, one-step greedy failure, no-new-information, budget infeasibility, mandatory evidence, unknown dependency, out-of-domain models and malformed likelihoods.

No field acquisition, sensor calibration, legal authority, production planning or autonomous action is implemented.
