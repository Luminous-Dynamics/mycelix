# FSRE equity-aware service resilience benchmark v0.1

**Date:** 2026-11-14  
**Status:** Research and evaluation contract. No real population-level equity result, policy approval, or operational rationing authority is claimed.

## 1. Research thesis

A service-resilience portfolio must not be called successful merely because aggregate delivered volume, average restoration time, or system-wide reliability improves. It must also disclose who received the service, who experienced the deficit, who participated in the decision, and which liabilities were shifted into the future.

The benchmark therefore treats authorised service floors for declared population cohorts as hard constraints before comparing aggregate efficiency, cost, or restoration quality. A surplus in one cohort cannot silently compensate for a shortfall in another cohort whose floor is protected.

This is not a claim that FSRE invented equity-aware resilience. It is a qualification and benchmarking profile that makes existing equity methods auditable within the same service-outcome and evidence lineage as recovery-option qualification.

## 2. Prior art and terminology

Two recent reviews/studies should shape this work:

- Coleman, Li, Comes et al., *Weaving equity into infrastructure resilience research: a decadal review and future directions*, npj Natural Hazards 1, 25 (published 2 September 2024), reviews 99 studies and distinguishes distributional-demographic, distributional-spatial, procedural, and capacity equity. It identifies limitations in geographic coverage and quantitative methods, and calls for improved monitoring data, computational models, and decision-support integration. It also stresses that procedural and capacity questions are less developed than distributional analyses. DOI: https://doi.org/10.1038/s44304-024-00022-x
- Beck, Cha & Peacock, *Development of an Equity Metric for Infrastructure Service Performance Assessment*, Risk Analysis 45(11), 3899–3914 (published online 9 October 2025), DOI: https://doi.org/10.1111/risa.70126. It distinguishes five equity dimensions—recognitional, distributional, restorative, procedural, and transgenerational—and develops Theil-T-derived measures for distributional and restorative service-performance inequality. Its demonstration uses a limited hypothetical electric distribution network. It is a relevant method comparator, not a water-sector field validation or universal equity standard.

The two taxonomies overlap but are not interchangeable. FSRE should preserve their source definitions and record which dimension each benchmark claim addresses rather than inventing a universal equity classification.

| Dimension | Practical question | Example FSRE evidence |
|---|---|---|
| Recognitional | Who is visible in the population boundary and data, and whose needs or impacts are missing? | Cohort definitions, denominator coverage, missingness, accessible service-quality requirements |
| Distributional | How are service, outage burden, costs and recovery benefits distributed across the whole population? | Per-cohort service-hours, outage duration, service deficit, whole-population inequality metrics |
| Restorative | Do interventions reduce disparities affecting historically or structurally disadvantaged groups? | Between-group gaps and decomposition of service-performance inequality against a declared reference |
| Procedural | Who participates, can contest assumptions, and receives a reasoned response? | Consultation/decision records, representation, objections, response and amendment lineage |
| Capacity | Can people and communities actually cope with or adapt to service disruption? | Access to safe alternatives, household storage/backup, mobility/access needs, communications and support capacity |
| Transgenerational | Are present decisions shifting financial, environmental, maintenance or hazard burdens to future generations? | Lifecycle liabilities, deferred maintenance, environmental effects, future hazard exposure and asset flexibility |

## 3. Non-compensable service-floor contract

For each declared service (s), cohort (k), scenario (w), and time interval (t), the service owner and authorised public/community process must specify:

- cohort identity and the lawful, minimum-necessary population boundary;
- numerator, denominator, geography, time basis and observation window;
- minimum safe service quantity and quality (F_{s,k,w,t});
- permitted continuity/degradation envelope and quality limits;
- measurement source, missingness, uncertainty, source owner and validation state;
- the approving authority, decision date, revision and applicable emergency policy.

For an allocation plan to pass a declared hard-floor test:

[
D_{s,k,w,t} geq F_{s,k,w,t}
]

for every cohort/time/scenario cell for which the floor is declared applicable and estimable.

The condition is evaluated per cohort and per interval. Aggregate service across cohorts is never a substitute for a missed floor. The executable portfolio reference already tests this principle: a synthetic plan can deliver total volume equal to the sum of all cohort floors while missing the household floor and exceeding the hospital floor. It remains NOT_FEASIBLE.

The floor itself is not selected by FSRE, Symthaea, an optimiser, or a hidden weighting function. It must come from an authorised service owner, regulator, rights framework, or legitimate participatory process. FSRE records and tests the declared floor; it does not decide whose service is expendable.

If a required cohort denominator, floor, population mapping, quality measurement or authority is unknown, the affected claim is NOT_ESTIMABLE. Missing cohort records must not be interpreted as zero population, zero demand, or successful service.

Emergency derogations are possible only where an applicable lawful policy explicitly defines the decision authority, scope, duration, affected cohorts, notice, review and remedy. A benchmark may test conformity to such a policy; it cannot create or grant the derogation.

## 4. Measure a vector, not a universal equity score

Publish the following separately. Do not add them into a single resilience/equity score without a declared policy model and sensitivity analysis.

### 4.1 Distributional service performance

For each cohort, report service level, safe service-hours, outage duration, time below floor, quality violations, recovery latency, population/household coverage, and uncertainty. Include population-weighted totals and per-cohort values; publish denominators and missingness for every rate.

Whole-population measures may include Theil-T or other distributional indices when their resource/service variable, zero handling, population coverage, and unit are declared. Such indices answer an inequality question; they do not establish that any minimum safe service floor was met.

### 4.2 Restorative disparity

Report gaps between the pre-registered groups of concern and the declared comparison population, before and after the intervention. Where data and assumptions support it, use a documented decomposition of distributional inequality (including a Theil-T-derived between-group component) to distinguish within-group from between-group contributions.

Do not infer group membership from names, addresses, device identifiers, or sensitive proxies without an approved lawful basis and validated methodology. A missing or unclassifiable record remains unresolved; it is not silently assigned to the comparison group.

### 4.3 Recognitional and procedural evidence

Report cohort coverage, under-representation, data omissions, disaggregation limits, and whose needs are not observable. For procedural equity, record who was invited, who participated, accessible participation routes, which concerns were raised, how they were answered, and whether decisions changed. Attendance counts alone do not prove influence or legitimacy.

### 4.4 Capacity and transgenerational consequences

Record whether cohorts have safe substitutes, access to information, mobility, storage, financial buffers, or other coping capacity, with strict privacy boundaries. Evaluate lifecycle costs, deferred maintenance, environmental impacts, future hazard exposure and options left available to future decision-makers.

## 5. Benchmark challenge families

Every family needs positive controls, negative controls, unknown/missing-data controls, and independently adjudicated expected results.

1. **Aggregate compensation trap.** Total delivered service equals or exceeds the sum of cohort floors, but one cohort is below its own floor while another receives surplus. Expected: NOT_FEASIBLE, never “equity satisfied.”
2. **Small-cohort masking.** A small hospital, remote settlement, care facility, or isolated neighbourhood experiences a large deficit while the system-wide average appears healthy. Expected: local floor violation remains visible.
3. **Denominator drift.** A cohort is renamed, merged, split, or omitted between reporting periods. Expected: no fixed-cohort trend or equity improvement is claimed until identity and population boundaries are reconciled.
4. **Unrecognised population.** A group has incomplete records or a service-quality need absent from the inventory. Expected: missingness and recognition gap are reported; absence of evidence is not evidence of zero need.
5. **Intersectional disparity.** Separate marginal cohorts each appear acceptable while their intersection experiences severe shortfall. Expected: intersections are assessed only where valid population data, minimum cell size and governance permit; sparse cells trigger privacy-safe abstention, not fabricated certainty.
6. **Restorative intervention.** An intervention improves total performance but widens the declared group-of-concern gap. Expected: total gain and between-group change are reported separately; no restorative improvement is claimed.
7. **Procedural exclusion.** A decision is optimised with no accessible participation, no response to objections, or an unrecorded change to service floors. Expected: procedural evidence is incomplete; the algorithm cannot assert that the resulting floor was legitimately authorised.
8. **Capacity disparity.** Two cohorts receive the same nominal service quantity but have different ability to access, store, transport, or safely use it. Expected: nominal equality is not described as equitable outcome without the capacity evidence.
9. **Transgenerational burden shift.** A short-term recovery portfolio reduces current cost by deferring maintenance or environmental remediation. Expected: present benefits and future liabilities are separated; lifecycle cost and future option loss remain visible.
10. **Simpson's-paradox / aggregate reversal.** Aggregate cohort rates improve after population mix changes while every stable cohort is unchanged or worse. Expected: no unqualified aggregate trend claim; the cohort and population-mix decomposition is published.

## 6. Matched baselines and ablations

Use the same frozen population boundary, service floors, hazard scenarios, evidence snapshot, cost horizon, and permissible data for:

- B0: current operator procedures and actual rationing/restoration policy;
- B1: aggregate-average or cost-minimising plan with no explicit cohort floors;
- B2: floor-constrained allocation with no disparity decomposition;
- B3: distributional inequality metric only (e.g. whole-population Theil-T where its assumptions are satisfied);
- B4: restorative/between-group decomposition (including a Theil-T-derived method where supported);
- B5: max-min / lexicographic floor-preserving allocation;
- B6: qualified human/operator/community panel;
- B7: FSRE composition with claim envelope, cohort floors, owner-native evidence, independent verification and distributional reporting;
- ablations: remove floor constraints, cohort lineage, uncertainty, procedural record, restorative metric, or independent outcome verification one at a time.

A comparator must be evaluated within the method's purpose and assumptions. The Theil-T-derived study was demonstrated on a hypothetical electric distribution network; porting it to South African water/wastewater requires a defined service variable, denominator, zero policy, time window, and independent validation. Do not treat its reported sensitivity in an electric toy network as water-sector performance.

## 7. Proposed qualification gates

For the declared deterministic benchmark suite:

- zero false “all service floors met” results when any required cohort floor is violated;
- 100% explicit reporting of the denominator, scope, time window and missingness of every cohort metric;
- zero promotion of missing, unresolved or identity-mismatched cohorts into an accepted trend;
- 100% traceability from each floor to its authorised issuer, revision, scope and effective period;
- zero unapproved changes to a cohort definition or floor during evaluation;
- no single aggregate score may override a hard floor or hide a cohort result;
- uncertainty or privacy constraints that prevent disaggregation must produce an explicit claim ceiling, not an invented cohort estimate.

These are conformance targets for the declared test pack, not a guarantee of real-world equity. For operational studies, pre-register practical effect thresholds, minimum cell sizes, statistical power, data governance, uncertainty, and non-inferiority margins before accessing outcome data. Publish both improvement and harm for every pre-registered cohort.

## 8. Data governance and safeguards

- Use population groups only at a lawful, necessary and approved level of granularity.
- Do not infer or expose sensitive identities for individual residents.
- Use aggregation thresholds, privacy-preserving disclosure review and suppression where small cells can re-identify people.
- Preserve a separate “unknown/not classifiable” category and report its size and trend.
- Do not let model confidence, vendor data, or unverified social proxies establish group status.
- Ensure communities and service owners can inspect, challenge and correct the declared cohort boundary and service-floor evidence.
- Keep allocation, dispatch and emergency rationing decisions with accountable authorities and operators.

## 9. Integration boundary

This benchmark reuses, rather than replaces:
- Mycelix for authority, evidence provenance, temporal records, commitments and completed-effect receipts;
- Sol-Atlas for spatial boundaries, population reachability and identity transition evidence;
- Symtropy for physical service capacity and hazard-domain model receipts;
- Symthaea for competing explanations and value-of-information investigations;
- operator and community authorities for service-floor approval, operating rules and control;
- an independent evaluator for service outcomes and distributional reproduction.

No separate population register, identity graph, authority service, decision engine or universal equity score is introduced.

## 10. Falsification

Narrow or reject the FSRE equity claim if:
- the declared service floors are not legitimate, measurable, or meaningful to affected cohorts;
- cohort data are too sparse or biased to support the stated claim;
- floor constraints systematically reduce another cohort below its own authorised minimum;
- apparent disparity reduction is caused by denominator changes, missing records or reclassification;
- procedural evidence is merely attendance without influence, response or accountability;
- the added controls delay essential response without improving safety or fairness;
- incumbent operator systems already deliver equivalent or better outcomes at lower cost and burden.

The strongest claim is not that FSRE discovers the morally correct allocation. It is that an independently reviewable engineering workflow can expose distributional trade-offs, enforce authorised minimum service boundaries, preserve uncertainty, and make the decision and its consequences reproducible.
