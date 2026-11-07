# GCPN — Civilian protection portfolio cost-effectiveness benchmark v0.1

**Parent:** [GCPN #5587](https://github.com/Luminous-Dynamics/mycelix/issues/5587)  
**Child issue:** [GCPN-CIVILIAN-PORTFOLIO-COST-001 #5930](https://github.com/Luminous-Dynamics/mycelix/issues/5930)  
**Status:** synthetic comparison contract; no cost inputs, field outcomes or comparative results admitted  
**Review date:** 7 November 2026  
**Claim ceiling:** comparable synthetic cost/outcome contract only

## 1. Purpose

Compare the full-lifecycle cost and independently measured civilian outcomes of **non-weapon protection and essential-service resilience portfolios**. The research question is:

> Under identical population, hazard scenario, geography, service floors, accessibility needs, time horizon and staffing assumptions, which lawful civilian portfolio delivers the strongest defensible outcome vector for its lifecycle cost?

This is not a weapon procurement, interceptor design, engagement-performance or cost-exchange-ratio benchmark.

## 2. Explicit non-goals and prohibited scope

This benchmark does not design, engineer, manufacture, optimize, rank or cost-model weapon interceptors, kinetic counter-UAS systems, guidance, tracking/fire-control, target selection, engagement performance or kill probabilities. It contains no tactical threat tracks, military operational data or classified information.

Only civilian public-warning, shelter, evacuation, passive building protection, essential-service continuity, rescue coordination, maintenance, community training and independent outcome-verification portfolios are in scope.

## 3. Paradigm shift: compare outcomes, not equipment prices

A device purchase price is not the cost of protection. The full system includes people, training, integration, communications, energy, inspection, maintenance, replacement, downtime, approvals, rescue/response arrangements and end-of-life. A nominally low-cost device may perform poorly when it is unavailable, inaccessible, unmaintained or disconnected from the rest of the emergency system.

Conversely, a costly intervention may not be justified if a combination of warning, passive protection, evacuation, service continuity and local recovery produces better outcomes.

The evaluation therefore:
- compares portfolios under an identical case frame;
- includes the full lifecycle resource burden;
- preserves hard safety/accessibility constraints;
- reports outcomes as a vector, not an opaque score;
- treats missingness and uncertainty as first-class;
- measures distribution of costs and benefits;
- does not monetize every human outcome;
- separates delivery, response, effect and causal attribution.

## 4. Portfolio families

The frozen synthetic benchmark declares the following portfolios. A real run must specify exact inclusions, quantities, coverage, service levels and operating responsibilities for each.

| ID | Portfolio | Contents |
|---|---|---|
| B0 | Existing baseline | Existing public-service and site arrangements, fully documented |
| B1 | Warning and communications | Accessible multilingual alerts, redundant delivery, local fallback and acknowledgement |
| B2 | Warning + assisted evacuation | B1 plus evacuation planning, transport assistance, shelter access and rescue coordination |
| B3 | Passive building / shelter resilience | Professionally reviewed passive protection, fire/life-safety, safe shelter and accessible egress |
| B4 | Essential-service continuity | Water, sanitation, power, cooling, communications and medical-support continuity |
| B5 | Integrated civilian portfolio | B1–B4 elements composed with explicit ownership and dependencies |
| B6 | Best-of-breed incumbent workflow | Existing qualified provider systems and human-led response, with exact feature and licence boundaries |

A portfolio must not be counted as providing an outcome merely because a product has a feature label. Service coverage, condition, usability, staffing and operation must be evidenced.

## 5. Common comparison frame

Every portfolio result binds the same:
- hazard/scenario family and version;
- geographic boundary and source spatial resolution;
- population/cohort definition, eligible denominator and accessibility requirements;
- exposure and vulnerability assumptions, where lawfully and ethically obtained;
- service floors and emergency-planning assumptions;
- time horizon, baseline window and evaluation window;
- staffing and operating schedule;
- source/observation eligibility and missingness policy;
- outcome definitions, units, ascertainment process and cutoff;
- currency, price year, discounting, inflation and sensitivity assumptions.

If a portfolio does not share the same eligible case frame, label the comparison **INCOMPARABLE**. Do not silently drop unavailable records or compare different denominators.

## 6. Full lifecycle cost taxonomy

Every cost line must have a currency, price/base year, geography, scope, quantity or calculation basis, source, uncertainty and owner.

Required cost dimensions:
1. Capital and site preparation.
2. Professional design, review, testing and statutory approvals.
3. Procurement, taxes, installation and commissioning.
4. Integration, software licences, data access and communications.
5. Staff, operator time, community coordination and training.
6. Exercises, inspection, certification and calibration.
7. Energy, connectivity, consumables and recurring operations.
8. Maintenance, spares, repairs, replacement and downtime.
9. Insurance, financing, compliance and contingency.
10. Decommissioning, disposal, residual value and post-project obligations.

No assumption of free volunteer labour, public infrastructure, donated assets, free data, cloud, network access, maintenance or professional review. Report both cash expenditure and total resource cost where they differ; do not hide unpaid or displaced work.

## 7. Outcome vector

Report, at minimum:
- authenticated warning reach and successful delivery;
- acknowledgement and accessible comprehension under user testing;
- people who can reach and use an accessible shelter/evacuation option;
- person-hours with essential services above each predeclared service floor;
- time to notification, assistance handoff and independently verified stable recovery;
- false, missed, delayed and unresolved alerts;
- operator workload and intervention burden;
- privacy leakage, unauthorized access and protected disclosure;
- adverse effects, exclusion, displacement and distribution of burdens;
- independently observed effects and their evidence tier.

Cost-per-outcome ratios are permitted only when the numerator, denominator, scope, time horizon, uncertainty and source lineage are declared. An alert receipt is not evidence that an alert was true or acted upon. A reported incident is not a measured harm outcome. A lower report count is not proof of greater safety.

## 8. Causal and economic interpretation

Do not claim “cost per life saved” or causal harm reduction from descriptive incident counts, synthetic outcomes, or before/after comparisons without a defensible design.

Where causal estimation is proposed, preregister:
- estimand and counterfactual;
- eligible population and inclusion/exclusion rules;
- baseline, intervention and follow-up windows;
- confounders, spillovers and displacement;
- outcome ascertainment and missingness;
- adverse outcomes and uncertainty;
- privacy/release profile and reviewer scope.

Use multiple scenarios and sensitivity analysis. Show affordability and maintenance sustainability under plausible funding, staffing and replacement scenarios. Preserve non-monetized outcomes instead of forcing them into a false total score.

## 9. Frozen synthetic cases and required dispositions

The companion JSON file freezes ten adversarial cases. It includes:
- full matched case frames;
- price-year/currency mismatches;
- hidden volunteer labour and assumed-free public infrastructure;
- missing outcome denominators;
- alert delivery misrepresented as response;
- incident-report counts misrepresented as actual harm;
- hard accessibility/life-safety failures masked by low cost;
- unsupported “cost per life saved” claims;
- privacy/disclosure violations;
- attempts to introduce prohibited weapon/interceptor scope.

Expected dispositions are one of:
- `ELIGIBLE_FOR_SYNTHETIC_COMPARISON`;
- `INCOMPARABLE`;
- `BLOCKED`;
- `UNKNOWN`;
- `CLAIM_LIMITED`.

No disposition is a real-world safety or effectiveness conclusion.

## 10. Professional review

Before a real portfolio comparison:
- an emergency-management/public-warning professional reviews warning and response workflows;
- registered civil/building/electrical/mechanical professionals review infrastructure and service continuity within their scope;
- a fire-safety professional and accessibility specialist review shelter and egress claims;
- a public-health/epidemiology reviewer reviews health-related outcomes and causal design;
- a maritime/port safety professional reviews maritime service portfolios;
- a quantity surveyor/economist reviews cost lines, assumptions, discounting and opportunity costs;
- privacy/data-protection specialists and community representatives review lawful purpose, data minimization, accessibility and distributional impacts.

Each review receipt binds exact inputs, reviewer role/credentials, scope, applicable references, findings, limitations, disposition and date. No reviewer approves beyond their competency. A software pass is not professional sign-off.

## 11. Acceptance gates

Proposed gates for a future implementation:
- 100% of portfolios bind the same frozen case/population/denominator/time-horizon contract before a paired comparison.
- 100% of cost lines identify currency, price year, scope, source and lifecycle category.
- Zero hidden labour or silently free infrastructure/data/maintenance.
- Zero silent deletion of missing outcomes or failed records.
- Zero causal claim without preregistered counterfactual and appropriate review.
- Zero hard safety/accessibility failure compensated by a lower price.
- Zero protected-data disclosure in public projections.
- Zero prohibited weapon/interceptor design or performance content.
- Exact reproducibility of the comparison manifest and independent evaluator result.

## 12. Reuse and ownership

- Mycelix #5587 owns federated warnings, civil-protection governance and cross-border coordination.
- Mycelix #2006 owns the Johannesburg Civic Resilience pilot profile.
- Mycelix #2942 owns shared evidence-frontier, projection, authority and outcome semantics.
- Existing emergency service, public-warning, procurement, provenance and outcome-ascertainment owners retain their respective authorities.
- Symthaea may analyse scenarios and report uncertainty; it does not confer civic authority.
- Symtropy may run effects-disabled service-continuity simulations; it does not certify real infrastructure.
- Sol-Atlas may present source-bounded geographic coverage; it does not infer household-level exposure.

Do not create a second authority, event-provenance or warning schema.

## 13. Sources and limitations

The protocol is an evaluation contract, not a cost database. No current product prices, local response costs, lives-saved estimates or portfolio performance values are asserted. Inputs must be sourced and professionally reviewed for each jurisdiction and study.

The benchmark is a civilian-protection alternative programme. It is not a pathway to weapon-interceptor design, engineering, manufacture or optimization.
