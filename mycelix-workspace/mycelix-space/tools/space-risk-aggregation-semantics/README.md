# Space risk aggregation semantics reference

A dependency-free Rust reference for keeping event-level Pc, joint mission/fleet path risk, long-term collision frequency, detection probability and observation significance distinct.

## Why separate the quantities?

An event-level Pc is conditional on a particular conjunction, orbit-state/covariance model, event definition and conditioning context. It is not automatically:
- the probability of at least one collision across a mission or constellation;
- a long-term collision frequency per object-year;
- the probability an actual collision would be detected by a mitigation policy;
- a p-value testing consistency of observations with a collision null.

NASA's 2025 paper *Issues with Satellite Collision Risk Aggregation* argues that aggregate Pc is best interpreted as an aggregate Bayesian degree of belief rather than automatically as long-term collision frequency, and discusses P_d and p_obs as distinct quantities:
https://ntrs.nasa.gov/citations/20250008868

A 2024 NASA / SpaceX / Aerospace extended abstract further discusses residual risk below mitigation thresholds, sensitivity to the maneuver commitment point (MCP), and product-formula saturation for large constellations:
https://ntrs.nasa.gov/citations/20240014277

NASA CARA's own publications include grouped conjunction risk, Bayesian longitudinal trends, collision consequence, covariance realism and survival-probability remediation thresholds:
https://www.nasa.gov/cara/cara-publications/

## Aggregation from marginals with unknown dependence

For event probabilities p_i that share the same probability space and conditioning context, the union probability obeys the sharp Frechet bounds:

    max_i(p_i) <= P(union_i A_i) <= min(1, sum_i p_i)

The API returns those bounds and deliberately gives no point estimate. If probability space, conditioning, event definition, horizon or source model revision differs, it rejects the aggregation.

## Product formula only with evidence

For mutually independent events:

    P(any event) = 1 - product_i(1-p_i)

The API requires explicit event-family, independence-model and evidence references, and the mutual-independence assertion must be true. Product survival is computed using sum(log1p(-p_i)) and -expm1(log_survival) to preserve precision for rare events. Saturation diagnostics reveal when large event families make the union probability nearly one.

A caller assertion is not itself proof of independence. Shared OD, atmospheric, sensor, event-overlap, maneuver or latent environmental uncertainty can invalidate it.

## Preferred approach: joint mission/fleet paths

For complete independent joint scenarios, each scenario contains all event outcomes over the declared horizon. One scenario contributes one binary path outcome: did any event occur?

The estimator counts scenarios with any collision and applies a one-sided Hoeffding bound using the number of independent scenarios. Event outcomes within one scenario may be arbitrarily dependent; they are not counted as independent trials.

This composes with the pair-reuse and shared-latent cluster estimators, but does not replace their sampling-model requirements.

## Separate typed quantities

The crate uses separate types for:
- EventLevelPc
- FrechetUnionBounds
- IndependentProductRisk
- JointPathRiskReport
- LongTermCollisionFrequency
- ProbabilityOfDetection
- ObservationSignificance

Long-term frequency is computed from a distinct collision count / exposure quantity and declared population/source model; it is never inferred from Pc. P_d binds to policy and calibration revision. p_obs binds to a null hypothesis, test statistic and measurement-model revision.

## MCP and residual risk

Aggregate risk at a maneuver commitment point uses the evidence available at that time. Later revised CDMs may produce a different estimate; those data must not be leaked into the MCP-time decision metric. Residual risk after mitigation includes events that were below the threshold, events missed by the process, and non-zero post-maneuver risk. A benchmark should report the distribution and timing of decisions, not one retrospective scalar.

## Claim ceiling

This reference qualifies mathematical semantics and evidence requirements. It does not prove event independence, calibrate local Pc, validate flux models, estimate actual collision frequency from aggregate belief values, or establish maneuver safety.
