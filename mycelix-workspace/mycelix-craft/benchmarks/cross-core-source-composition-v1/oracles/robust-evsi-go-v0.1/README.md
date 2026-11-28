# Craft robust EVSI under model ambiguity V0.1

Test-only finite scenario-robust evidence-acquisition oracle. It evaluates a single observation across a declared finite set of plausible prior/likelihood models. It does not establish that the model set covers the real world or that any scenario is calibrated.

## Criterion

For model set M, latent state theta, safe action a, observation y and a result-conditioned action policy pi:

- Baseline robust risk = min_a max_m E_m[L(a,theta)].
- Acquisition robust risk = min_pi max_m [cost + E_m[L(pi(y),theta)]].
- Robust gross EVSI = baseline robust risk minus minimum worst-case terminal loss.
- Robust net value = baseline robust risk minus acquisition robust risk.

The same result-to-action policy is evaluated under every scenario. The policy never observes the hidden model label. The finite policy space is capped at 100,000 deterministic mappings.

## Frozen model-ambiguity counterexample

Prior unsafe/safe = 0.05/0.95. Protect costs 2 in either state; release costs 100 in unsafe and 0 in safe. Baseline robust risk is 2.

At observation cost 0.2:
- Model A is perfectly informative. Nominal gross EVSI is 1.9, nominal net value 1.7.
- Model B is uninformative. It yields no decision value.
- Under the two-model ambiguity set, the minimax policy protects for both results. Worst-case terminal risk remains 2; acquisition risk is 2.2; robust gross EVSI is 0 and robust net value is -0.2. The robust oracle rejects the acquisition.

This illustrates why a nominal recommendation must not be reported as robust when an uninformative but admissible model remains possible.

## Controls

- Mandatory evidence preempts optional optimization.
- Missing model-set admissibility or coverage, unavailable/out-of-domain/inadmissible observations abstain.
- Probability and likelihood normalization, unique identifiers and common state universe are validated.
- A single shared policy is selected across all model scenarios (no model-label leakage).
- Nominal per-model EVSI is reported beside robust EVSI.
- The output says `EstablishedWithinDeclaredFiniteSet` only for the supplied set and leaves real-world calibration `NotEvaluated`.

## Local validation

Go 1.23.2: frozen fixture, model-ambiguity, nominal-vs-robust, coverage abstention, mandatory evidence, invalid distributions, duplicate model IDs and policy-space cap tests pass; race detector and vet pass; statement coverage 90.2%.

No field calibration, production planner, legal authority, physical outcome or autonomous action is claimed.
