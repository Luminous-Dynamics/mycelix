# PRAXIS-NRES-007: Cascading disruption and service-recovery trajectories

**Status:** synthetic benchmark proposal and Rust reference evaluator for review. No real municipal, infrastructure, learner or emergency-response performance has been measured.  
**Fixture:** `examples/courses/civic-resilience-cascade-recovery-fixture-001.json`.  
**Conformance corpus:** `examples/courses/civic-resilience-cascade-recovery-conformance-001.json`.  
**Schema:** `schemas/nres/nres-cascade-recovery-v1.schema.json`.  
**Implementation:** `benchmarks/nres-capability-loop/src/trajectory.rs`.

## 1. Research shift

NRES-005 connected exercise findings to verified corrective actions. NRES-006 made the resulting evidence portable. NRES-007 changes the unit of analysis from a one-time exercise result to a **time-indexed service trajectory under a frozen disruption sequence**.

A readiness percentage or final pass/fail can hide the most consequential information: how far a critical service fell below its minimum floor, for how long, whether recovery was sustained, and which upstream dependencies were stressed at the same time.

NRES-007 therefore reports per-service trajectories and measures. It does not create a total resilience score, rank communities, or score people.

## 2. What it measures

For every service and every scenario, report:

1. **Observed minimum-floor status at each sample.** Keep all samples and thresholds.
2. **Maximum shortfall.** The largest positive difference between the service floor and observed availability, in percentage points.
3. **Breach sample count.** Number of observed samples below the frozen floor.
4. **Deficit area.** Exact integral of the positive part of floor minus a piecewise-linear trajectory, in percentage-point-hours. This captures both depth and duration; it is not a service-volume or health-impact estimate.
5. **Observed sustained-recovery hour.** Earliest sampled hour at which the service is at/above its floor and remains so for every later sample through the declared horizon. Null means not observed by the horizon.
6. **Horizon floor status.** Whether the final observed sample meets the floor. This is distinct from sustained recovery.
7. **Resource use.** Planning points and responder-hours are reconciled against their own limits.

Every service remains an independent row. Do not aggregate deficit area, recovery time or service floors across services.

## 3. Dependency graph semantics

An edge `prerequisite_service_id → dependent_service_id` means the benchmark declares the first service a required input to the second. The graph is validated for unique, resolvable service IDs and acyclicity. This is a scenario assumption used to make dependency coverage inspectable; it is not an inferred model of any real utility, municipality or provider.

The fixture includes electricity, communications, water, public information, welfare-check coordination, health logistics and transport access. It uses a directed acyclic graph to keep the reference case deterministic and interpretable. Future profiles may represent redundancy or substitutable inputs explicitly, but must version their semantics rather than overload the edge meaning.

## 4. Matched stress tests

The public fixture contains two fictional profiles under exactly the same frozen 72-hour shock sequence: a reference profile and a comparison profile with illustrative continuity measures. Both have the same service catalog, time grid, dependency graph, shock IDs and resource limits.

A machine-readable expected-output vector (with an absolute numeric tolerance) is published in the conformance corpus so independent implementations can check exact metrics. The calculated profile contrast is **descriptive only**. Different resource commitments and scenario assumptions mean the comparison is not a causal treatment effect, procurement recommendation or proof that a real intervention works. Future confirmatory comparisons require preregistration, independently controlled scenarios, blinded evaluation where feasible, repeated seeds and independent data custody under NRES-004.

## 5. Comparison with existing practice

| Practice | Existing strength | NRES-007 composition |
|---|---|---|
| FEMA HSEEP | Exercise objectives, evaluator observations, AAR/IP and improvement planning | Add machine-checkable time-series measures and matched stress-sequence consistency to the exercise evidence |
| IFRC Preparedness for Effective Response | Preparedness assessment, prioritisation, work planning, accountability and locally led learning | Add an inspectable service-dependency model and explicit recovery trajectories without replacing local assessment |
| WHO simulation-exercise programmes (2026) | Move from ad-hoc drills to institutionalised, progressively complex system testing with continuity and impact | Add a deterministic public fixture, precise per-service recovery arithmetic and portable conformance cases |
| UNDRR Recovery Readiness Framework | Governance, finance, capacity and data-system domains with structured indicators | Complement readiness assessment with time-dependent synthetic exercise observations; do not reinterpret the framework's indicators |

References:
- FEMA HSEEP: https://training.fema.gov/programs/nsec/hseep/
- IFRC National Society Preparedness Framework: https://www.ifrc.org/document/ifrc-national-society-preparedness-framework
- WHO national health simulation exercise programme guidance (24 March 2026): https://www.who.int/news/item/24-03-2026-new-who-guidance-helps-countries-to-institutionalize-simulation-exercises-to-strengthen-health-emergency-readiness
- UNDRR Recovery Readiness Framework: https://www.undrr.org/publication/documents-and-publications/recovery-readiness-framework-and-assessment-tool

NRES-007 does not replace these programmes, certify alignment, or claim superior real-world performance. Its proposed contribution is a small, open, executable test object that can be adapted independently and compared against institutional exercise outputs.

## 6. Numerical definitions

For service floor (F), observed availability (A(t)), and time interval ([t_i,t_{i+1}]), define the deficit (D(t)=\max(0,F-A(t))). The reported deficit area is:

\[
\operatorname{DeficitArea}=\int_0^T D(t)\,dt
\]

The implementation integrates the positive part of each piecewise-linear segment exactly, including threshold crossings. The unit is percentage-point-hours. The interpolation is only a declared numerical convention for the area measure; raw observed samples remain authoritative.

The sustained-recovery time is the earliest sample (t_i) for which all observations (A(t_j)\geq F) for every (j\geq i). Recovery after (T) is unknown and is never inferred.

## 7. Hard integrity gates

- 100% of scenarios reference the same frozen shock-sequence ID for a matched comparison.
- 100% of service trajectories match the declared sample grid exactly.
- 100% of service IDs and dependency endpoints resolve; dependency graph is acyclic under this profile.
- 100% of services, including every critical floor, are reported separately.
- 100% of resource pools are checked against their own limits.
- 100% of listed negative conformance cases are rejected.
- 100% of null/unknown recovery states remain distinct from zero-hour recovery.
- 0 composite resilience scores, community leaderboards, political fields or personal-suitability fields are accepted.

These are data-integrity targets, not public-service performance targets. Service thresholds must come from an authorised local standard or be explicitly labelled as scenario assumptions before any confirmatory use.

## 8. Research benchmarks and path to SOTA

Do not define a single winner. Compare systems on a per-service Pareto surface:

- lower deficit area for each critical service;
- lower maximum shortfall and fewer breach samples;
- earlier sustained recovery, when observed;
- no hidden service-floor failures;
- resource use within each separately declared constraint;
- complete evidence and reproducible calculations.

A profile only dominates another when it is no worse on all predeclared critical-service and resource constraints and strictly better on at least one; otherwise report the trade-off rather than collapsing it to a scalar.

Next research gates:
1. Two independently written Rust/other-language adapters produce identical service metrics from the same fixture.
2. A third-party validator accepts the valid case and rejects all negative controls.
3. Property-based tests cover irregular grids, crossings, no recovery, immediate recovery, duplicated IDs, graph cycles and resource boundaries.
4. Evaluator reliability is tested with blinded human observations; pre-register the agreement statistic and target at least 0.80 before confirmatory deployment.
5. A field study, only with institutional authority and appropriate data governance, compares repeated exercises and delayed recovery outcomes. Learner outcomes, exercise performance and actual service outcomes remain separate.
6. Any claim of improvement over an industry system requires independent held-out scenarios, equal shock sequences, disclosed resource budgets, a reproducible evaluator, uncertainty intervals and a named comparator implementation.

A provisional engineering benchmark may measure evaluator latency, peak memory and bytes per sample at 10², 10³, 10⁴ and 10⁵ service-samples on a named machine. Publish the hardware, compiler, workload and repeated-run distribution; do not claim speed superiority without those measurements.

## 9. Reproducibility and boundaries

Run from the `mycelix-praxis` directory:

```bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --trajectory
```

The default evaluator and NRES-006 `--envelope` mode remain available. The trajectory fixture is public synthetic development data, inactive in the curriculum and not a confirmatory test set. A successful run means fixture consistency only. No legal determination, real-world readiness estimate, operational instruction, causal effect or SOTA claim is produced.
