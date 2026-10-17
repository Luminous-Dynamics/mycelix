# Ex-ante Procurement Strategy v1 — benchmark and claim contract

**Status:** experimental proposal for `src/strategy.rs`; not evidence of compilation, correctness, purchasing performance, or production readiness.
**Tracking issue:** [PROC-STRATEGY-001 #5109](https://github.com/Luminous-Dynamics/mycelix/issues/5109).
**Second-stage dependency:** [adaptive recourse PR #5108](https://github.com/Luminous-Dynamics/mycelix/pull/5108).

## Scope of the first implementation

The strategy layer chooses which already-described flexible/backup options to commit before a disruption. For each subset it evaluates the supplied frozen scenarios with the current recourse evaluator, then returns the non-dominated set. It does not authorize a contract or purchasing action.

The initial model is deliberately bounded:
- At most 8 candidate options / 256 commitment subsets.
- A selected option is reserved at its declared maximum activation quantity; partial reservation sizing is not implemented.
- Total search-node budget covers each subset plus the recourse search granted to that subset.
- Any recourse search-bound exhaustion aborts the whole call; no partial frontier is labelled complete.
- Inconsistent/shared capacity pools and over-budget commitments make that candidate subset infeasible.
- Scenarios are unweighted. Their shortage sum is a declared Pareto axis and corpus diagnostic, **not expected shortage** and not probability-weighted. Duplicate scenarios can change this axis, so the scenario manifest must be frozen.

The new module currently calls the reference recourse implementation to score candidate subsets. The test suite includes one test-local tiny two-option oracle that independently enumerates commitment subsets and integer activation quantities, but it is only one hand-authored case—not a generalized independent recourse oracle. A wider independent brute-force corpus and exact-head CI qualification remain mandatory before claiming solver correctness.

## Decision surface

Return a Pareto frontier minimizing these explicit dimensions:
1. Upfront commitment cost: base cost plus all option-retainer fees, including unused options.
2. Worst-case unfilled quantity across the declared scenario set.
3. Sum of unfilled quantities across the declared scenarios as a Pareto axis and corpus diagnostic; it is not an expected shortage. Duplicate scenarios can change this axis, so the scenario manifest must be frozen.
4. Worst-case all-in scenario cost, including activation cost and fees.
5. Total reserved maximum activation quantity, as a physical-capacity diagnostic.

There is no synthetic scenario probability and no weighted sum that hides trade-offs. A buyer's authorized policy may later select from the frontier; the reference engine does not choose a winner merely because it is convenient to rank.

## Required baselines on identical manifests

For each frozen request, capacity, evidence, option and scenario manifest, compare:
- No contingency options.
- Cheapest single backup by retainer fee (and separately cheapest total cost where this differs).
- Flexible-only candidates.
- Backup-only candidates.
- Greedy reserve-cheapest-first.
- Full subset enumeration, with independently implemented second-stage recourse oracle on tractable cases.

Report both the chosen set and the full frontier. Report ties; do not silently collapse different supplier/failure-domain topologies into one winner. All strategies must consume the same evidence snapshot and scenario manifest.

## Independent verification corpus

Start with at least 40 hand-authored fixtures and stable identifiers. Include:
- no-disruption control and single supplier outage;
- shared supplier parent, site, critical sub-tier, utility, route, port and regional failures;
- identical supplier offers with shared capacity pool;
- option retainer cost when unused;
- base order plus reserved options exceeding a pool ceiling;
- activation + transit missing the deadline;
- option evidence valid at decision time but stale/expired by scenario observation;
- budget sufficient for retainers but insufficient for activation;
- cheapest option loses its capacity to an overlapping commitment;
- equivalent options whose input order differs;
- infeasible subsets mixed with feasible alternatives;
- tiny integer cases where an independent oracle can enumerate every legal commitment subset and every allowed activation quantity;
- forced search-bound exhaustion, which must not return an incomplete frontier.

After oracle qualification, generate at least 10,000 deterministic micro-instances with pinned seed generator, source revision, input digest and expected results. Preserve negative controls and shrink counterexamples into named fixtures.

## Acceptance targets (proposed, not measured)

| Gate | Target |
|---|---:|
| Independent exhaustive oracle agreement on tractable cases | 100% |
| Demand conservation / capacity pool ceiling violations | 0 |
| Stale or ineligible capacity promoted to feasible recovery | 0 |
| Scenario arriving-after-deadline represented as on-time | 0 |
| Budget/arithmetic overflows or over-budget strategy reports | 0 |
| Candidate-order dependent frontier for same manifest | 0 |
| Scenario-accounting completeness | 100% |
| Generated micro-instances after oracle qualification | At least 10,000 |
| Runtime and memory | Measure p50/p95/max on named pinned host/toolchain after correctness gates |

Also report frontier size, infeasible-subset count, search nodes, largest candidate set completed, and regret against the independent optimum on tractable cases. For larger cases, report bounded/incomplete status and a certified gap only if the solver can actually prove one; otherwise report unknown, not optimal.

These are software/reference-model gates. They do not prove supplier availability, savings, disruption probabilities, fairness, legal conformity or actual-world resilience.

## Cross-project boundaries

- **Symthaea:** may forecast need, substitutions, lead-time scenarios and uncertainty; those outputs are advisory and versioned, not authority or ground-truth inventory.
- **Mycelix:** binds evidence, authorized policy, commitments and review receipts; labels and digest references still require upstream cryptographic/provenance verification.
- **Sol-Atlas:** contributes evidence-labelled geography, route, port, utility and supplier-dependency layers; an inferred route must not be mistaken for current dispatch capacity.
- **Symtropy:** runs deterministic and stochastic operations experiments against a frozen scenario corpus; simulation does not certify supplier quality, clinical safety or real capacity.

## Interoperability and research context

OCDS links procurement planning, tendering, awards, contracts and implementation around a process identifier; Mycelix should preserve this lifecycle linkage at adapters rather than inventing an incompatible export model. GS1 EPCIS/CBV gives an interoperable event vocabulary for product status, location, movement and chain of custody; event adapters can refresh evidence, but an event's existence is not proof that its content is truthful.

Recent research on flexible/backup procurement and robust supplier allocation motivates comparison against static and robust baselines. Findings from a particular model or data set must not be reported as expected gains for this implementation.

- Open Contracting Data Standard: https://standard.open-contracting.org/latest/en/primer/how/
- GS1 EPCIS/CBV: https://www.gs1.org/standards/epcis
- Dai, Zou & Zhang (2026), resilient supplier selection/order allocation: https://doi.org/10.1016/j.ijpe.2026.110060
- Hosseini et al. (2019), supplier selection and order allocation under disruption risks: https://doi.org/10.1016/j.ijpe.2019.03.018
