# Space weighted anytime confidence sequence

A dependency-free Rust reference for an anytime-valid confidence sequence for bounded observations, intended for weighted rare-event importance-sampling contributions.

## Betting construction

For an observation Y_t in a known range [a,b], normalize Z_t=(Y_t-a)/(b-a) in [0,1]. For a candidate mean m in (0,1) and fixed bet fraction r in (0,1], define:

    F_t(m,r) = 1 - r * (Z_t-m)/(1-m)

Under the conditional null E[Z_t | F_(t-1)] >= m:
- F_t is nonnegative for all Z_t in [0,1];
- E[F_t | F_(t-1)] <= 1;
- each product over time is a nonnegative supermartingale;
- a fixed mixture across bet fractions is also a nonnegative supermartingale.

Ville's inequality gives P(sup_t M_t(m) >= 1/delta) <= delta. Inverting across m produces an anytime-valid upper confidence sequence for the largest uniform lower cap on conditional mean. Reflecting Z to 1-Z gives the lower sequence. A two-sided sequence allocates delta/2 to each side.

For IID observations, the upper and lower sequences form a simultaneous confidence sequence for the common mean. Under dependence, they bound uniform conditional-mean caps; they are **not** a generic confidence interval for an arbitrary heterogeneous average.

## Intended rare-event application

With a defensive importance proposal q(z)=epsilon*p(z)+(1-epsilon)*q_adapt(z), the collision contribution is:

    Y_i = I[high-fidelity collision_i] * p(z_i)/q(z_i)

and lies in [0,1/epsilon]. The same construction can be used for:
- direct weighted collision contributions in [0,1/epsilon];
- paired surrogate residuals in [-1/epsilon,1/epsilon];
- paired candidate-minus-baseline risk contributions in [-1/epsilon,1/epsilon].

This inference layer does not generate samples or validate the proposal. The caller must bind the exact support interval, fixed proposal, sample-order, evidence for IID/conditional-mean assumptions and the event-oracle revision. The conditional-mean evidence reference is an attestation, not an automated proof.

## Relation to existing tools

- PR #6460: exact fixed-sample Clopper-Pearson bound for IID Bernoulli indicators.
- PR #6462: Bernoulli betting confidence sequence for binary violation observations.
- PR #6465: fixed-sample empirical-Bernstein inference for bounded weighted importance-sampling contributions.
- This crate: time-uniform betting confidence sequence for bounded weighted observations.

The binary PR #6462 method must not be applied directly to nonbinary weighted contributions. This crate addresses the bounded-value domain under its own conditional-mean assumptions.

## Example

    cargo run --locked -- 0 20 0.05 0,0,1,0,2,0 iid-fixed-defensive-q-v1 bounded-weighted-mean-v1 fixture://mean-model/evidence-v1 0123456789abcdef

## Claim ceiling

This is time-uniform statistical inference for bounded observations under the declared process assumptions. It does not validate covariance realism, orbital dynamics, the event oracle, model calibration, physical Pc or maneuver safety.
