# Joint Capacity Allocation v1 — benchmark and claim contract

**Status:** experimental feasibility checker; target metrics are proposed, not measured.  
**Tracking issue:** [PROC-MULTIREQ-001 #5144](https://github.com/Luminous-Dynamics/mycelix/issues/5144).  
**Implementation:** `src/joint_capacity.rs`.

## 1. What this slice does

The reference checker validates a proposed assignment table against a single frozen input manifest. It checks per-request demand and budget, per-offer quantity and order-lot constraints, explicit versioned request-to-offer capability mappings, offer arrival deadlines, and shared capacity pool ceilings. All offers referencing the same capacity-pool ID draw against one ceiling, rather than each receiving a separate copy.

Its report reconciles each request, offer, capacity pool, quantity family and currency deterministically. Quantity summaries are grouped by capability ID, profile digest and unit; costs are grouped by currency. The request priority rank is carried through for the next policy layer, but this checker does not optimize or make priority decisions.

This is not a solver, eligibility authority, cryptographic evidence verifier, inventory system, procurement award, contract, purchase order or payment path. Verification state, live-domain origin, event times and digests are caller-supplied assertions. The supplied manifest digest is not recomputed here.

## 2. Explicit boundaries in v1

- The checker accepts only current evidence marked Verified + LiveDomain. Verified synthetic evidence is not live supply.
- Every assigned request/offer pair needs a matching, current, explicitly versioned capability mapping.
- Unit and currency labels must match exactly; there is no inferred conversion.
- Each request has its own cost budget. Outputs never sum different currencies into one scalar.
- Unit cost is assumed to be normalized within the declared currency. Fixed order charges, freight, tax, FX conversion, contract minimums beyond the order-lot rule, and landed-cost modeling remain future work.
- Capacity is shared by one pool ID and its declared time bucket. Cross-bucket cumulative capacity coupling and recourse across disruption scenarios are not implemented in this slice.
- Partial allocation is valid output, but never silently reported as full service.
- Any hard constraint violation or checked-arithmetic overflow rejects the complete allocation. There is no repair, silent clipping or “best effort” passing report.
- Canonical outputs are sorted by stable IDs; candidate input order must not alter the report. New allocations to requests whose deadline is already before the evaluation cutoff are rejected; this reference is for prospective allocation, not retrospective settlement.
- The checker does not calculate an optimality gap because it does not optimize.

## 3. Hand-authored adversarial corpus

The registry currently contains 46 stable fixture IDs (the minimum target is 40). The live registry is [JOINT_CAPACITY_GOLDEN_CORPUS_V1.md](JOINT_CAPACITY_GOLDEN_CORPUS_V1.md). The current implementation has 19 authored module tests. The registry does not mean all 46 cases are implemented, nor does authorship mean a test has passed. A test-local independent oracle is authored for 48 binary assignment matrices across three shared-pool ceilings; it is not yet qualified by executed tests.

Every fixture should freeze the request, offer, mapping, pool and assignment inputs, expected decision/error, and input-manifest digest. Expected outcomes should be literal or produced by a separately implemented test oracle; invoking the checker twice is not independent verification. Retain negative controls as permanent regression cases.

## 4. Correctness gates

| Property | Target |
|---|---:|
| Accepted allocation violates a declared demand limit | 0 |
| Accepted allocation exceeds a per-offer ceiling | 0 |
| Accepted allocation double-counts a shared pool | 0 |
| Accepted allocation violates a pool unit, request currency, or deadline | 0 |
| Assigned offer lacks a current verified live capability mapping | 0 |
| Stale, unknown, contested, or synthetic capacity becomes live capacity | 0 |
| Request budget or order-lot violation accepted | 0 |
| Arithmetic overflow wraps or produces a passing report | 0 |
| Per-request and per-pool reconciliation mismatch | 0 |
| Identical valid input manifest with permuted vectors changes canonical report | 0 |
| Independent checker/oracle disagreement on supported tractable fixtures | 0 |
| Authored adversarial fixtures after qualification | 40 minimum |
| Deterministic generated micro-instances after independent oracle qualification | 10,000 minimum |

Generated tests must pin generator version, seed, case count, fixture digest, code revision and expected outcomes. Use property-based mutations to vary request/offer order, pool ceilings, duplicate lines, mapping freshness, deadlines, order increments, per-request budgets, currencies and integer boundaries. Shrink every counterexample into a named permanent fixture. Do not generate the 10,000-case corpus and then treat agreement with the same implementation as an oracle.

## 5. Performance protocol

Only after all correctness gates pass, measure validation latency (p50, p95, maximum), throughput, peak memory, assignment count, and sizes of request/offer/pool/mapping tables for pinned machine and Rust toolchain profiles. Start with small cases and scale the matrix at 10, 100, 1,000 and 10,000 assignment lines. Record exact revision, hardware, toolchain, corpus hash, seed and run command. These are measurement targets, not current performance claims.

Do not pick a universal millisecond threshold before observing a reproducible reference environment. Correctness, no false-feasible allocation, fail-closed bounds and determinism are hard gates; speed is a later optimization objective.

## 6. Next optimizer layer

Only after this checker has an independent qualified oracle should a solver be evaluated. It should compare request-by-request optimization (diagnostic only), cheapest-first greedy, declared-priority greedy, static joint allocation, shared-capacity joint optimization, and—later—joint first-stage reservation plus scenario recourse. A separate exhaustive allocator on tiny instances should certify the optimum and Pareto frontier. Baselines must consume identical frozen manifests, and a request-wise result that double-pledges a pool must be labeled infeasible, never deployable.

Keep policy trade-offs explicit: aggregate cost, worst unfilled request, priority-policy misses, concentration and reserved resilience should not be hidden inside an uncalibrated universal score. Report a certified optimality gap only when the optimizer can prove it; a search bound or interrupted run means incomplete/unknown, not optimal.

## 7. Interoperability and evidence references

- The Open Contracting Data Standard joins planning, tender, award, contract and implementation stages using a contracting-process identifier. This model is not an OCDS implementation; future adapters must map IDs and data semantics explicitly: https://standard.open-contracting.org/latest/en/primer/how/
- GS1 EPCIS treats corrections as later events that preserve the earlier history rather than silently rewriting it. That supports append-only provenance, but syntactic conformity does not prove a physical event true: https://www.gs1.org/standards/epcis-and-cbv-implementation-guideline/current-standardd
- Multi-request sourcing with contingent supply, multi-period lead-time uncertainty, and distributionally robust disruption planning are established research directions. The project should claim value from its evidence gates, exact shared-pool accounting, independently checked outputs and auditability—not from inventing joint procurement optimization: https://doi.org/10.1016/j.cor.2024.106593 ; https://doi.org/10.1016/j.ejor.2022.04.025 ; https://doi.org/10.1016/j.ijpe.2026.110160


Each accepted row is also emitted as a canonical allocation witness that names the mapping ID/digest and the exact capacity-pool snapshot digest/time bucket, alongside quantity, normalized unit cost, currency and deadline fields. Witness rows are sorted by request/offer ID; no content hash is fabricated because this dependency-free module does not verify or recompute digests.
