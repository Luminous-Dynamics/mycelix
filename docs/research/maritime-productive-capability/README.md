# Maritime Productive Capability Benchmark (MPCB) v0.1

Research seed for [Mycelix #6773](https://github.com/Luminous-Dynamics/mycelix/issues/6773).

## Scope

MPCB tests whether an evidenced regional production network can deliver the exact work packages needed to build, maintain, repair and upgrade maritime vessels and port equipment.

It consumes Mycelix #4814 production/dependency facts and composes the engineering, scheduling and geographic owners. It does not create another supplier registry, inventory ledger, generic optimizer, shipyard planner, GIS or economic accounting system.

The benchmark is intentionally separate from a local-content percentage. A material or machine is not usable merely because it appears in a regional inventory or supplier directory. It must be accessible in the required period, match the exact specification/revision, have a qualified process and available capacity, satisfy inspection requirements, and be deliverable to the worksite.

## Owner boundaries

- **Symthaea** supplies configuration-bound technical requirements and engineering suitability/qualification context. It does not qualify its own generated design.
- **Symtropy** supplies deterministic work-package, resource, schedule, scenario and accepted-output evidence. Simulation does not become physical acceptance.
- **Mycelix** supplies owner-native production, supplier, inventory, capacity, skill, commitment, custody, provenance and revocation facts. This benchmark owns only the maritime comparison contract and tiny exact reference.
- **Sol-Atlas** supplies rights-aware geography, facility/corridor context and read-only spatial projections. Proximity is not evidence of deliverability or capacity.

Existing owners to compose:
- Mycelix [CIV-ECON-005 #4814](https://github.com/Luminous-Dynamics/mycelix/issues/4814)
- Symtropy [SHIPBUILD-CO-DESIGN-001 #2048](https://github.com/Luminous-Dynamics/symtropy/issues/2048)
- Symtropy [MULTIPURPOSE-SHIPYARD-001 #4294](https://github.com/Luminous-Dynamics/symtropy/issues/4294)
- Symtropy [MARITIME-SERVICE-CHAIN-001 #3683](https://github.com/Luminous-Dynamics/symtropy/issues/3683)
- Sol-Atlas [MARITIME-FOUNDRY-001 #370](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/370)

## Paradigm: capability closure, not local-content accounting

Keep these propositions distinct:

1. **Accessible input:** material/equipment can be delivered to the actual worksite before its required-by time.
2. **Qualified process:** the exact facility, machine, process, grade, tolerance, procedure and revision are suitable.
3. **Available capacity:** time-windowed capacity remains after all other work packages and commitments are allocated.
4. **Accepted output:** inspection and acceptance requirements have been satisfied by the appropriate owner.
5. **Sustained capability:** the region can maintain, repair, replace and requalify the system throughout its service life.

A missing/unknown property is not a zero, a pass or an optimistic substitute. A supplier offer is not an order; an order is not dispatch; dispatch is not receipt; receipt is not acceptance; acceptance is not payment settlement.

## v0.1 tiny exact fixture

The fixture at docs/research/maritime-productive-capability/fixtures/maritime-capability-tiny-v0.tsv is synthetic and deliberately hand-checkable.

It contains two required work packages:
- WP-A: fabricated deck panel;
- WP-B: crane support bracket.

Each has one qualified regional option and one qualified imported option. The regional options each require 2 t of the same plate stock and 8 welder-hours in the same period. Regional capacity is 3 t and 12 welder-hours; the NDT limit is 2 jobs. Each regional option is individually feasible, but using both together is not. Imported options have no use of these regional constrained resources.

The four possible selections are therefore:
- local A + local B: **infeasible** (4 t > 3 t and 16 h > 12 h);
- local A + imported B: feasible;
- imported A + local B: feasible;
- imported A + imported B: feasible.

The three feasible candidates trade cost, maximum procurement lead time, embodied emissions and regional value added. These are independent objective dimensions, not one weighted score. The expected feasible and Pareto sets are recorded here as the fixture contract; the future Rust oracle must recompute them from the input rows rather than trusting this prose.

Expected result:
- candidate count: 4;
- feasible count: 3;
- infeasible count: 1;
- Pareto count: 3 (all three feasible candidates are nondominated under the declared objective directions).

The independently enumerated expected outcomes are:

| Portfolio | Feasibility | Cost units | Max lead days | Embodied-CO2 units | Regional-value units |
|---|---|---:|---:|---:|---:|
| A-LOCAL + B-LOCAL | Reject: plate and welding over capacity | 210 | 6 | 14 | 130 |
| A-LOCAL + B-IMPORT | Feasible / Pareto | 190 | 16 | 20 | 83 |
| A-IMPORT + B-LOCAL | Feasible / Pareto | 185 | 18 | 20 | 65 |
| A-IMPORT + B-IMPORT | Feasible / Pareto | 165 | 18 | 26 | 18 |

All three feasible portfolios are nondominated when minimizing cost, lead time and embodied emissions while maximizing regional value added. The all-local plan is rejected before objective ranking: it reserves 4 t against 3 t of plate capacity and 16 h against 12 h of welding capacity. Its NDT reservation exactly meets the 2-job limit.

This tiny instance is a resource-feasibility regression only. It is not a regional shipbuilding cost model, a real procurement quote, an LCA result or proof of industrial competitiveness.

## Result contract

For each candidate portfolio, preserve:
- selected option for every required work package;
- all source/offer/commitment and capability references;
- exact product/specification/configuration revision;
- time-indexed input reservations by resource and site;
- feasibility result and every blocking witness;
- total cash cost, lead-time outcome, energy/emissions boundary and regional value-added estimate as separate fields;
- sensitivity/uncertainty metadata and evidence cut-off;
- excluded or unsupported dimensions and claim ceiling.

Hard gates are non-compensatory: unqualified material, inaccessible stock, unavailable process, resource overbooking, missing inspection, revoked commitment, unauthorized substitution or impossible delivery cannot be offset by lower cost or higher local value.

## Benchmark ladder

1. Freeze typed input semantics and the synthetic tiny fixture.
2. Implement a dependency-light Rust exact Cartesian enumerator / independent verifier for tiny fixed option sets.
3. Add negative controls: stale inventory, shared upstream supplier hidden behind two vendors, wrong material revision, unqualified process, expired commitment, unit mismatch, missing NDT, and capacity valid individually but overbooked jointly.
4. Add at least 30 synthetic cases and at least 10 tiny independently adjudicated cases.
5. Compare import-turnkey, local-assembly, regional-module, distributed-supplier, brownfield-upgrade and repair/remanufacture-first portfolios on identical work packages and shocks.
6. Only after synthetic correctness, admit rights-cleared supplier, facility, workforce, logistics, price, quality and delivery evidence.
7. Hold out complete shock families and facilities; do not tune and score on the same suppliers.

## Metrics

Report separately:
- accepted work-package / vessel / equipment throughput;
- on-time accepted output and missed service obligations;
- first-pass yield, inspection delay, rework and repair turnaround;
- regional value added, imported intermediate inputs and gross local expenditure;
- cash-flow/bankability, fiscal exposure and economic opportunity cost;
- skill coverage and durable competency formation;
- qualified substitute depth, supplier/upstream concentration and common-cause cut sets;
- inventory coverage, working capital, obsolescence and maintenance continuity;
- energy, water, material yield, scrap and lifecycle emissions at declared boundaries;
- worst-case lost output and recovery time under compound disruptions;
- false-available, false-substitute and false-accepted rates;
- exact-oracle agreement, replay determinism and compute/runtime cost.

Employment and training are important distributional/capability outcomes; gross payroll or transferred employment is not automatically a net social benefit. Local value-added is not gross procurement spend. Supplier count is not independent redundancy when suppliers share an upstream mill, sub-tier, port or power source.

## Baselines and SOTA discipline

Compare against:
- current/import-dominant or no-build baseline;
- lowest-price and shortest-lead-time sourcing heuristics;
- local-content quota heuristic;
- lean/minimum-inventory policy;
- diversified/buffered sourcing;
- capability-aware greedy allocation;
- MILP/CP-SAT or robust optimization only when their exact formulation, solver and bounds are reproducible;
- exhaustive enumeration on tiny finite cases.

A timeout, heuristic incumbent or solver-reported feasible flag is not an optimality certificate. Publish all objective vectors, feasible sets, Pareto fronts, bound gaps, seeds, solver versions and negative controls.

The World Bank/S&P CPPI is a vessel-time-in-port comparator, not a substitute for receiver-accepted cargo or regional production capability. Reuse IAPH/IHMA Port Call Optimization and DCSA Port Call 2.0 semantics rather than creating a competing port-call standard. Align industrial-intelligence evaluation with NIST's 2026 SI for Manufacturing programme and its focus on trustworthy intelligence, contextualized data, integration benchmarks, human-machine teaming and digital-twin VVUQ.

## Claim ceiling

A passing synthetic fixture establishes only the stated resource and objective semantics. It does not prove a supplier exists, inventory is real, a process is qualified, local production is economical, a vessel is compliant/seaworthy, a port is feasible, a job will be created, a project is bankable, or any physical work is authorized.

No live procurement, payment, inventory mutation, contract award, production dispatch or worksite instruction is in scope.

## Standalone Rust reference

The dependency-free research crate lives at research/maritime-productive-capability and is deliberately outside the Holochain production workspace.

From the repository root:

```sh
cargo fmt --manifest-path research/maritime-productive-capability/Cargo.toml -- --check
cargo test --locked --manifest-path research/maritime-productive-capability/Cargo.toml
cargo clippy --locked --manifest-path research/maritime-productive-capability/Cargo.toml --all-targets -- -D warnings
cargo run --locked --manifest-path research/maritime-productive-capability/Cargo.toml -- docs/research/maritime-productive-capability/fixtures/maritime-capability-tiny-v0.tsv
```

The current reference enumerates the finite Cartesian product of sourcing options, rejects unqualified/missing/mis-unit/mis-period/over-capacity reservations, computes the objective vector, and calculates the nondominated feasible set. Its Rust unit tests include the tiny known-answer case, single-local-option controls, unit and period mismatch negative controls, and input-order-invariant receipt rendering.

The available authoring session did not have Cargo/rustc installed, so this Rust crate has not been locally compiled here. Treat hosted exact-head test and strict Clippy results as pending until observed; the independent fixture enumeration is not a substitute for a Rust build.
