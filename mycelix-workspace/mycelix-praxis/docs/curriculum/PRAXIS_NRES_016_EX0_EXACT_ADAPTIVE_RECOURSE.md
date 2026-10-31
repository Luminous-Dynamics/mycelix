# PRAXIS-NRES-016-EX0: Exact adaptive-recourse micro-oracle

**Status:** synthetic public-development exact reference; not a production controller, DRO solver or confirmatory result.  
**Parent protocol:** PRAXIS-NRES-016-ADAPTIVE-RECOURSE-PROTOCOL.md.  
**Evaluator:** benchmarks/nres-capability-loop/src/adaptive_recourse.rs.  
**CLI:** cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --adaptive-recourse-exact.

## Purpose

EX0 is the smallest executable correctness anchor for the NRES-016 decision model. It verifies that a finite-horizon policy can be optimized over observable information states while keeping action-conditioned transition laws, resources and service floors explicit.

It is an exact dynamic-programming solver over a finite information-state DAG, minimizing expected total cost under the probabilities supplied by the fixture. It is not a distributionally robust optimizer, does not learn transition probabilities, and does not use nested conditional CVaR.

## State and transition contract

Each information state contains:
- a unique state ID and unique canonical observable-history key;
- a stage index;
- currently available resource units;
- current service levels keyed by service;
- feasible actions.

Each action has a fixed cost, resource use, and a conditional outcome list. Each outcome has a positive probability, stage cost, resource replenishment and either a next information-state ID or explicit null at the terminal stage.

For current resource balance b_t, action use u_t and realized replenishment r_t, the next information state must satisfy:

b_(t+1) = b_t - u_t + r_t.

Every action's conditional outcome probabilities must sum to one. A non-terminal transition must point to a state in the next stage; terminal transitions must explicitly use a null next-state ID. All declared states must be reachable from the initial state under at least one declared action/outcome.

## Non-anticipativity by construction

The policy is a mapping from unique observable information-state IDs to actions. The fixture does not provide a latent scenario label as a policy input. A duplicate observable-history key is rejected: two copies of the same observed history cannot become separate decision variables and cannot increase the sample count.

This is a deliberately explicit information-state DAG rather than a scenario-indexed policy file. When a future stochastic-programming backend is added, it must reproduce this policy's non-anticipativity constraints and pass negative controls that permute hidden future outcomes without changing current actions.

## Exact objective

For information state i, action a, and each action-conditioned outcome o with probability p(o|i,a), the finite-horizon recursion is:

V(i) = min over feasible a of [ fixed_cost(i,a) + sum_o p(o|i,a) * ( stage_cost(i,a,o) + V(next(i,a,o)) ) ].

An action is feasible only if:
- its resource use does not exceed current available resources;
- every declared transition obeys stage and resource conservation;
- every positive-probability successor admits a feasible continuation;
- the current and all successor information states remain at or above every declared service floor.

If no policy satisfies all floors, the evaluator returns status=infeasible with a null objective. It does not silently soften or relax service floors.

Tie-breaking uses the lexicographically smallest root action ID when expected costs are equal within the declared numeric tolerance. The finite DAG and fixed transition law make this a deterministic exact oracle for this fixture.

## Fixture and hand solution

The public synthetic instance has three stages, two service floors (water 70, power 60), one conserved resource with two opening units, five stage-0/1 information states and seven terminal states.

At the root:
- PREVENT costs 1, uses one resource, and transitions to quiet/storm with probabilities 0.7/0.3.
- DEFER costs 0, uses no resource, and transitions to quiet/storm with probabilities 0.6/0.4. Its storm successor has water level 40, below the hard floor 70, so DEFER is infeasible pathwise.

After PREVENT:
- the quiet branch chooses USE-SPARE at cost 1 instead of HOLD at cost 4;
- the storm branch chooses REPAIR at cost 2 instead of HOLD at cost 5.

The expected total cost is therefore:

1 + 0.7 * 1 + 0.3 * 2 = 2.3.

The optimal policy is PREVENT at ROOT, USE-SPARE at P-QUIET and REPAIR at P-STORM, followed by terminal END actions. Each transition conserves resource units, and every selected-policy state meets both service floors.

The different transition probabilities for PREVENT and DEFER are synthetic action-conditioned assumptions included to exercise the interface. They do not establish that any real intervention causes such an effect.

## Unit tests and controls

The module authors 10 Rust tests covering:
- exact expected cost against the hand solution;
- selection of actions at multiple stages;
- pruning an action whose future branch violates a service floor;
- one action per observable information state;
- duplicate observable-history rejection;
- resource conservation and resource-overdraw rejection;
- conditional probability normalization;
- explicit infeasibility;
- stage-transition validation.

The conformance corpus separately records 23 invalid-input controls, six boundary cases and eight round-trip requirements. These tests are authored, not confirmed passing; compilation and execution require an actual Rust toolchain/CI run.

## Scope boundaries and next work

EX0 proves only the finite expected-cost recursion for the supplied finite model. It does not solve:
- ambiguity over transition kernels;
- nested conditional CVaR or other dynamic risk measures;
- partial observation filtering or belief-state updates;
- vector resources, procurement lead-time pipelines or workforce rest rules;
- continuous state/action spaces;
- multi-agent decomposition or large scenario trees;
- calibrated shift alarms or causal identification of action effects.

Next, use EX0 as the oracle to add risk recursion on the same tiny information structure, then add a scenario-tree non-anticipativity formulation and independently compare their objective and policy. Only after agreement should rolling-horizon, multistage stochastic, two-stage robust and multistage adaptive-robust baselines be scaled up.
