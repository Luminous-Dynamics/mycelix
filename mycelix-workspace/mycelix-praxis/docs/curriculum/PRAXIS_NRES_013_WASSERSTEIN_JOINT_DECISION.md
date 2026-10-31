# PRAXIS-NRES-013: Wasserstein ambiguity over joint service-failure states

**Status:** synthetic benchmark proposal and Rust reference evaluator for review. This is an exact finite-support distributionally robust decision evaluator for the declared finite support; it is not a real-world risk model or operational recommendation.

**Source:** NRES-009 joint-service forecast fixture, Git blob SHA-1 799799f8a315d4c0f06a43b5dce5ab2e15ccefb5.  
**Fixture:** civic-resilience-wasserstein-joint-decision-fixture-001.json.  
**Conformance corpus:** civic-resilience-wasserstein-joint-decision-conformance-001.json.  
**Schema:** nres-wasserstein-joint-decision-v1.schema.json.  
**Implementation:** benchmarks/nres-capability-loop/src/wasserstein.rs.

## 1. Research question

NRES-009 evaluates candidate distributions over all joint binary service-breach states. NRES-012 evaluates ambiguity over a single Bernoulli event probability. NRES-013 connects them: it asks whether a decision remains defensible when uncertainty concerns the entire joint distribution, including dependence among service failures.

For state s in {0,1}^d and nominal distribution p over all 2^d states, define:

    P_epsilon(p) = {q in Delta_(2^d): W_(1,d)(q,p) <= epsilon}.

Probability mass can move between complete joint service states. This preserves the multivariate structure of the forecast rather than independently widening each service marginal.

## 2. Exact finite-support Wasserstein evaluator

The frozen ground metric is weighted Hamming distance:

    d(s,t) = sum_i w_i * 1[s_i != t_i],  w_i > 0.

For action loss vector ell(a,s), the finite-support Kantorovich dual is:

    sup_q E_q[ell(a,S)]
      = inf_(lambda >= 0) [
          lambda*epsilon
          + sum_i p_i * max_j(ell(a,s_j)-lambda*d(s_i,s_j))
        ].

The dual objective is convex and piecewise linear. The Rust evaluator constructs each source state's upper envelope of affine functions, enumerates all non-negative breakpoints, and evaluates the dual at this finite candidate set. It does not sample adversarial distributions or rely on a heuristic optimizer.

At epsilon=0 the ambiguity set is the singleton nominal distribution; the exact nominal expectation is returned directly (the dual infimum is attained only as lambda tends to infinity). At epsilon at least the support diameter, every distribution over the support is reachable and worst-case loss equals the largest support loss.

## 3. Actions, resources and cost profiles

Three abstract synthetic action bundles are compared: no_action, targeted and comprehensive. Each declares resource units and a prepared-service bitmask. Profiles declare resource caps, resource-unit costs, service breach losses and the residual fraction for a prepared service.

    loss(a,s) = resource_units(a)*resource_unit_cost
                + sum over breached services i:
                    service_breach_cost_i * (residual_fraction if prepared else 1).

Infeasible actions are excluded before policy selection and before the perfect-information reference is computed. All profile values are fictional planning-loss units, not actual municipal or infrastructure costs.

## 4. Policies

1. **Nominal:** minimize expected loss under p.
2. **Wasserstein worst-case expected loss:** minimize the exact worst-case expectation over P_epsilon(p).
3. **Wasserstein minimax regret:** first compute statewise regret against the best feasible action after the state is known, then minimize its exact worst-case expectation over P_epsilon(p).

The statewise comparator is a lower bound, not a deployable policy. Worst-case expected loss and minimax regret are distinct objectives. Ties retain the first feasible action in the declared catalog order.

## 5. Fixture and lineage

The fixture preserves all eight NRES-009 source cases, each evaluator-owned observed joint-state mask and every candidate state probability. Each case is crossed with four radius tiers: 0, 0.15, 0.40 and 0.90 weighted-Hamming units. The resulting 32 rows are eight cases times four sensitivity settings, not 32 independent observations.

The source file is pinned by Git blob SHA-1 799799f8a315d4c0f06a43b5dce5ab2e15ccefb5. The evaluator receives NRES-009 separately and verifies source identity, service catalog, case IDs, copied observations and copied forecast distributions.

The synthetic cost profiles are designed to expose different decision regimes. At moderate preparation cost, nominal and minimax-regret decisions remain targeted on some cases while worst-case expected-loss decisions shift toward comprehensive action as ambiguity grows. At high cost, no_action can remain optimal. These are fixture-design choices, not empirical findings.

## 6. Metrics and reporting

For each profile and radius tier, report mean, nearest-rank p90 and maximum realised cost; mean realised statewise regret; mean selected nominal expected loss; mean exact worst-case expected-loss and worst-case regret bounds; selected action counts; evaluation row count; unique source-case count; and perfect-information reference cost.

No profile, radius or objective is collapsed into a composite score.

## 7. Research comparison and next increments

Yamao et al. (UAI 2026) study robust decision-focused learning with observation uncertainty and Wasserstein ambiguity around conditional empirical distributions: https://proceedings.mlr.press/v337/yamao26a.html. Ohnemus et al. (ICML 2026) study loss-aware trainable optimal-transport ambiguity sets, where downstream loss informs ambiguity-set construction: https://proceedings.mlr.press/v306/ohnemus26a.html.

NRES-013 does not train predictors, learn transport metrics or learn ambiguity sets. Its contribution is a reproducible benchmark substrate that exactly solves a finite-support Wasserstein decision problem over complete joint service states, preserving the same forecasts and source outcomes across policies, with explicit resource feasibility and statewise regret.

The next research competition should add:
- radii calibrated on a separate calibration split and frozen before confirmatory testing;
- a learned loss-aware transport cost compared with fixed weighted Hamming distance;
- independently generated common-mode, component-specific and evidence-degradation shifts;
- CVaR and other tail objectives alongside expected loss and regret;
- trajectory-level joint states linked to NRES-007, resource depletion and multi-stage recourse;
- independently custodied challenge cases with paired uncertainty intervals.

## 8. Limitations and non-claims

This first fixture has four services and 16 joint states. The reference evaluator is limited to six services until complexity and numerical stability are profiled. Radii and transport weights are fixed sensitivity settings, not calibrated confidence sets or learned loss-aware ambiguity. The source fixture is public development data, not a hidden challenge set. This is a 24-hour single-horizon decision evaluator; it does not propagate cascades or model recovery. Action bundles and costs are fictional. No SOTA superiority, real-world effect, causal result, legal authority or operational readiness is claimed.

## 9. Reproducibility

From the mycelix-praxis package root:

~~~bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --wasserstein-joint
~~~

The CLI consumes the pinned NRES-009 source fixture and the NRES-013 evaluator fixture. A passing run establishes code and synthetic-fixture conformance only.
