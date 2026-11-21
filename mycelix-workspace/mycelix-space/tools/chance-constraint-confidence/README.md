# Chance-constraint confidence reference

A dependency-free Rust reference implementation of a one-sided exact Clopper-Pearson upper confidence bound for the probability of a Bernoulli violation.

This is a statistical calculation only. It does not validate the physical/orbital model, calibrate observation likelihoods, prove trial independence, establish covariance realism, or certify collision-avoidance safety.

## Important model requirement

The exact Clopper-Pearson guarantee assumes independent, identically distributed Bernoulli trials with a common violation probability. The API requires both a sampling-design identifier and an evidence reference for that assumption. These are caller attestations, not automatically verified facts.

MCTS rollouts sharing a search tree, belief model, random seed lineage, or common uncertain parameters may be dependent. Repeated rollouts must not be counted as independent trials without a defensible design argument. This crate intentionally does not convert an effective sample size into an exact binomial bound.

## Example

    cargo run --locked -- 1 27 0.95 frozen-iid-seed-design-v1 fixture://sampling-design/independent-seeds-v1

The exact one-sided 95% upper confidence bound for 1 violation in 27 IID Bernoulli trials is approximately 0.1639743947, despite the empirical fraction being 0.0370370370.

For zero violations in 100 IID trials, the one-sided 95% upper bound is approximately 0.0295130496, not zero.

## Decision interpretation

Let k be observed terminal chance-constraint violations, n be IID Bernoulli trials, 1-alpha_c be the statistical confidence level, and alpha_s be the separate operator/policy chance-constraint threshold.

The report gives U = BetaInverse(1-alpha_c; k+1, n-k) for k<n, and U=1 for k=n. The zero-violation case uses U=1-alpha_c^(1/n).

The observed sample supports the declared statistical threshold only when the justified upper bound U is no greater than the policy threshold alpha_s, and only within the declared sampling/model domain. The two alpha values are not interchangeable.

The output claim ceiling is conditional statistical evidence only. It is not a maneuver authorization or an operational-safety certificate.
