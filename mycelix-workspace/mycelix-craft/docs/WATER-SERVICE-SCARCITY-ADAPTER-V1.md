# Water Service Scarcity Adapter V1

- **Tracking issue:** [CRAFT-WATER-SCARCITY-ARBITRATION-001 / #6791](https://github.com/Luminous-Dynamics/mycelix/issues/6791)
- **Existing arbitration owner:** [NRVCA-010 / #5427](https://github.com/Luminous-Dynamics/mycelix/issues/5427), implementation draft [PR #5428](https://github.com/Luminous-Dynamics/mycelix/pull/5428)
- **Constraint-analysis owner:** [NRVCA-007 / #5419](https://github.com/Luminous-Dynamics/mycelix/issues/5419)
- **Correlated-stress owner:** [NRVCA-005 / #5409](https://github.com/Luminous-Dynamics/mycelix/issues/5409)
- **Status:** adapter contract proposal; no executable water adapter or field qualification
- **Data classification:** synthetic-only until authority/community permissions and source/data rights are approved

## 1. Ownership rule

The water-service profile must not implement a second shared-budget allocator.

It supplies a water-domain-qualified candidate set to the NRVCA-010 owner, consumes its per-candidate receipts, and independently checks that its own domain prerequisites have not been collapsed into a generic Boolean.

The NRVCA owner arbitrates declared candidate costs, shared budget, resource-pool capacities, policy-authored priority tiers and deterministic tie-breaks. It does not establish that:
- an alternative drinking-water source is safe;
- delivery reaches the required population or a safe/convenient point;
- a quantity satisfies litres/person/day;
- a repair crew is competent, a permit is current or an isolation state is safe;
- a work package will restore a physical service outcome;
- a municipal policy or priority tier is legitimate;
- a selected recommendation has authority to dispatch, reserve, procure or spend.

Those remain owner-qualified evidence and authority decisions. The arbiter's recommendation receipt is not a service delivery or effectuation receipt.

## 2. Domain input: protected service obligations

Each water-domain candidate must reference a separately qualified obligation packet:

```text
WaterServiceFloorRefV1
  obligation_id
  jurisdiction_ref
  service_area_cohort_ref
  source_instrument_ref
  source_instrument_revision
  applicability_decision_ref
  population_measurement_ref
  population_value
  population_unit
  quantity_floor_value
  quantity_floor_unit
  interruption_start_event_ref
  interruption_duration_as_of
  water_quality_qualification_ref
  delivery_point_accessibility_ref
  local_authority_policy_ref
  exact_context_digest
```

This is a proposed adapter record, not a new universal statutory schema. The responsible authority must define which source instrument, service cohort, population estimate, clock and criterion apply.

For the South African interruption-over-48-hours scenario, the published compulsory standards provide a test anchor of at least 15 litres of drinking water per person per day through safe and reasonably convenient delivery points. The adapter must not mark that floor feasible from tanker dispatch, a provider assertion, or an unqualified source. Quantity, drinking-water quality, delivery and accessibility are separate prerequisites. Exact local applicability and professional interpretation remain with the competent authority.

## 3. Candidate translation

The adapter may translate domain-qualified work into one or more NRVCA candidate recommendations.

| Water-domain candidate | Candidate class | Resource claims | Required domain preconditions |
|---|---|---|---|
| Alternative supply for a protected cohort | Protected service floor | potable litres/day, tanker capacity, delivery window, approved delivery point | population and quantity validated; source quality qualified; point safe and accessible; authority plan valid |
| Repair of a critical interruption | Protected service floor only where local policy designates it | qualified crew-hours, material inventory, isolation/access window | permit and isolation valid; crew qualification current; scope and work package approved |
| Independent pressure restoration observation | Protected verification / monitoring commitment | qualified observer-hours, calibrated instrument slot | exact service zone; instrument calibration; observation window/profile approved |
| Water-quality sampling and qualified analysis | Protected verification / quality commitment | sampler-hours, sample transport, laboratory capacity | sampling plan current; sample identity and custody; laboratory/method qualification |
| Discretionary network inspection | Discretionary recovery | crew-hours, vehicle, access window | safe access, authorized scope and current work context |
| Storage/tanker repositioning | Discretionary or protected, according to local authority policy | tanker-hours, vehicle capacity, storage capacity | route/access, water-source and quality qualifications |

A candidate's cost and resources must be authoritative inputs from the relevant owner or declared synthetic assumptions. The adapter may not derive a plausible cost, resource capacity or crew competency from a language-model estimate.

## 4. Capacity and unit semantics

The adapter must keep unlike units separate.

Examples of resource pools include:
- qualified crew-hours in an explicit shift/time window;
- tanker capacity in litres per delivery window;
- verified potable water volume available at a named source and time;
- repair-material quantities;
- qualified laboratory throughput;
- calibrated instrument/observer capacity;
- budget in a declared currency and price vintage.

The existing NRVCA-010 reference uses integer cost and resource-capacity units in its current synthetic contract. A production or pilot adapter must either use a declared exact integer base unit with overflow/rounding rules or request a separately reviewed owner-contract extension. It must not convert unlike quantities into a hidden scalar score.

A time-window resource is a distinct capacity pool. A tanker available on one shift is not automatically available in another. Late or incomplete inventory observations must remain unknown/indeterminate, not be treated as spare capacity.

## 5. Protected bundle semantics

The generic arbitration prototype models protected actions as an atomic service-floor bundle: if required protected candidates cannot all be satisfied, it selects nothing from lower tiers and requests manual review.

The water adapter must define bundle membership from a versioned, locally adopted policy. It must not silently decide that every repair is protected or that one cohort's protected need can be substituted by another cohort's discretionary work.

Before a water action enters the protected bundle, the domain adapter must establish:
1. exact affected-service cohort and scope;
2. applicable service-floor obligation;
3. safe and quality-qualified supply/work preconditions;
4. evidence-backed resource feasibility;
5. current competent authority and policy references;
6. a clear failure/appeal/escalation path.

If a service floor is infeasible, the system reports the shortfall, the limiting resources, missing evidence and required human escalation. It does not relax the floor, select lower-tier work as a substitute, or claim compliance.

## 6. Decision output and receipts

For each candidate, preserve:
- water episode, obligation, cohort and exact context references;
- candidate and action identity;
- owner-authoritative cost and unit;
- resource pool claims and capacities with source/observation references;
- precondition dispositions and owner receipts;
- policy class and priority-rule revision;
- selected/deferred/rejected/manual-review disposition;
- explicit reason codes and resource/budget arithmetic;
- arbitration implementation/schema/commit identity;
- replay input and output digests.

Preserve water-domain facts separately from arbitration results. A selected emergency-supply candidate does not prove the water was delivered; a selected repair candidate does not prove repair completion; a selected sampling action does not prove the sample passed.

## 7. Pareto and policy analysis

Use NRVCA-007/#5419 to report non-dominated alternatives and constraint-violation vectors. Keep at least these dimensions distinct:
- people-days below the protected supply floor;
- customer-minutes lost;
- pressure-deficit exposure;
- repeat-incident burden;
- water-quality and worker-safety gate status;
- resource/budget consumption;
- distribution of service burden across approved cohorts.

Do not add litres/person/day, customer-minutes, safety violations, Rand and worker-hours into a single weighted total. If a competent local authority adopts a policy priority, pin its exact version and show the resulting feasible set and Pareto trade-offs. Do not present the policy as a universal social-welfare ranking.

## 8. Required synthetic cases

The first frozen water-adapter panel should include at least:
1. One protected alternative-supply candidate exactly meets the floor; all other requirements remain independently checked.
2. Candidate volume is 1 litre/person/day below the floor.
3. Aggregate tanker volume exists, but the delivery point is unsafe or inaccessible.
4. Quantity is adequate, but source quality is pending or contradicted.
5. Two cohorts compete for one tanker fleet; both protected obligations are feasible only under a larger declared capacity.
6. Protected bundle exceeds the available budget; lower-tier work cannot substitute.
7. A critical repair is infeasible because the crew qualification or permit is stale/revoked.
8. A repair uses a shared crew pool; input order must not determine selection.
9. Sampling and laboratory capacity conflict with a repair/material bottleneck.
10. Prolonged alternative supply requires the extended monitoring and source-origin prerequisites.
11. Repeated incidents compete with a new interruption under a fixed contingency budget.
12. All protected candidates are jointly infeasible; the receipt must expose the exact conflict and require manual review.

Every case needs a negative control where an unrelated candidate or independent evidence remains unaffected.

## 9. Qualification gates — targets, not results

- 0 cost-budget or resource-capacity violations.
- 100% deterministic candidate receipts across all permutations in bounded cases.
- Exactly one terminal disposition per candidate.
- 0 silent protected-floor relaxation or lower-tier substitution after infeasibility.
- 0 alternative-supply action eligible without independently qualified quantity, water quality and accessible delivery point.
- 0 repair action eligible without current permit, isolation and crew prerequisites where required.
- 100% agreement with an independent exhaustive oracle on bounded synthetic cases.
- 0 cross-unit hidden weighted sums.
- False holds, unmet service floors, opportunity costs, resource utilization, distributional effects and review time reported separately.

Passing synthetic conformance does not establish optimality, legal compliance, fairness, service improvement or readiness for dispatch.

## 10. Pilot gate

A future pilot requires written authorization from the responsible Water Services Authority/Institution and affected communities; approved service-floor and resource-allocation policy; data-sharing and source-owner permissions; privacy/security and worker-safety review; professional approval of water-quality, pressure and repair criteria; an appeal/contestability process; and independent measurement.

No live dispatch, procurement, resource reservation, municipal-system access or field deployment is in scope for this proposal.
