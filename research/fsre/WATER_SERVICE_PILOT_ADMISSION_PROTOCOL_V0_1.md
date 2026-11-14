# FSRE water-service pilot admission protocol v0.1

**Date:** 2026-11-14  
**Status:** Research governance and preregistration protocol. It does not grant access, certify a utility, approve a public intervention or qualify a deployment.

## 1. Purpose

The FSRE programme now has synthetic recovery qualification, differential references, point-in-time evidence replay, single-interval portfolio feasibility, stock-flow accounting, cohort service floors and a read-only as-of portfolio composer. These are meaningful software/conformance increments. They do **not** establish a real-world utility baseline, an operator partnership, causal improvement, or operational decision-support maturity.

This protocol defines the gates required to move from synthetic conformance to a lawful retrospective water/wastewater study, connected shadow evaluation and routine decision support. It prevents maturity labels from being inferred from CI status or technical integration alone.

The machine-readable pre-registration contract is [water-service-study-manifest-v0.1.schema.json](schemas/water-service-study-manifest-v0.1.schema.json). JSON Schema validates structure only; reviewers must separately assess permission, legitimacy, evidence adequacy, independence and maturity.

## 2. Maturity and admission gates

### G0 — Synthetic software conformance (M1a-like)

Required:
- frozen synthetic fixture and expected-output register;
- production/reference separation and exact differential results;
- adversarial negative controls, boundary tests and explicit unknown handling;
- exact code revision, toolchain, workflow and runner conclusion;
- all claims labelled synthetic/software-only.

No operator data or real service benefit may be implied. CI success is a code-revision result, not a field validation.

### G1 — Retrospective real-world replay (M1b)

Admission requires all of:
1. **Lawful access:** source licences or written owner permission cover the exact purpose, fields, retention and publication plan. "Public URL" is not a licence.
2. **Historical source frontier:** immutable source snapshots, schema revisions, release/observation periods, decision-log availability and extraction timestamps. A current database export alone cannot establish what was knowable at a historical cutoff.
3. **Identity integrity:** system/facility/WSA/municipal identity mappings across renames, mergers, splits, transfers and demarcation changes; unresolved transitions remain unresolved.
4. **Denominator and population:** explicit service/geography/cohort boundaries, numerator and denominator, missingness and suppressed/unclassifiable categories.
5. **Outcome observations:** operator or independently maintained service observations (e.g. hours of safe supply, pressure/quantity, quality, wastewater collection/treatment/discharge) with units, sensor/method calibration, time basis and uncertainty.
6. **Actual decision records:** work orders, incident timelines, crew/stock/replenishment evidence, permits/authority and the incumbent decision/runbook outputs for the historical events selected.
7. **Independent adjudication:** a reviewer who did not author the evaluator or curate its challenge answers adjudicates the source bindings, scenario labels and outcome definitions.
8. **Held-out design:** chronological and/or catchment/WSA holdouts. Random-row splits alone are not accepted for spatially or temporally dependent infrastructure data.

A national aggregate report or municipal financial cube alone cannot pass G1 as a system-level service trajectory. It may support a bounded public-summary replay and denominator/missingness analysis.

### G2 — Connected shadow study (M2)

Additional requirements:
- owner-authorised, read-only connector and documented data minimisation;
- source latency, availability, missingness, calibration and clock uncertainty monitored;
- frozen daily/weekly evidence snapshots and reproducible as-of cutoffs;
- named operator review workflow; every recommendation or warning is logged as advisory;
- no command, write-back, rationing, switching, procurement or dispatch path;
- incident rehearsal for stale data, connector compromise, source revocation, clock drift, network partition and conflicting owner records;
- operator can pause or withdraw the study without losing control of the source system.

Shadow mode measures whether the evidence and portfolio output is timely, understandable and useful. It does not prove decision influence or improved service.

### G3 — Routine operational decision support (M3)

Additional requirements:
- repeated prospective use within a defined operator workflow;
- documented operator acceptance, rejection, override and reasons;
- pre-registered task-success and service outcomes, not model accuracy alone;
- matched incumbent workflow and domain-model baselines using identical events, resources, permissions and information cutoffs;
- independently reviewed safety, cybersecurity, privacy, data governance and maintenance arrangements;
- service and distributional outcomes measured over adequate follow-up, including adverse effects and no-effect cases.

A routine dashboard or connected digital twin does not automatically qualify as M3. The system must be shown to participate in a real decision workflow with appropriate governance and measured consequences.

### G4 — Bounded closed-loop operation (M4)

M4 is **out of scope for the present FSRE programme**. It requires a separate control-authority and safety case, engineering hazard analysis, fail-safe design, independent interlocks, verified human override, cybersecurity and drift monitoring, incident response, audit, change control and prospective outcome validation. The read-only FSRE composer does not provide those controls. No M4 claim may be made from an M1a/M1b/M2/M3 study.

## 3. Minimum water/wastewater evidence pack

For drinking water:
- source availability, treatment, storage, pumping, pressure zones, distribution reachability and customer delivery;
- water quality and laboratory results with sampling point, collection/result time and method;
- outage/pressure events, work orders, repairs, energy/fuel, crew and spare-part availability;
- authorised minimum safe service floors by cohort, plus protected critical facilities where owner policy supports them.

For wastewater/sanitation:
- generation, reticulation/collection, conveyance, pumping, treatment, bypass, discharge/reuse and receiving environment;
- flow meters, rainfall/infiltration context, bypass/overflow reports, plant inlet/outlet and quality sampling;
- sludge handling and disposal where in the declared service boundary;
- treatment-plant condition must not be substituted for end-to-end sanitation service.

For every source:
- owner, schema/version, source revision/digest, unit, denominator, geography, observation time, availability time, valid interval, access basis and missingness;
- clock basis and uncertainty;
- exact evidence snapshot and extraction/replay procedure.

## 4. Matched benchmark design

At minimum compare:
- B0: incumbent operator runbook and configured SCADA/GIS/EAM/APM workflow;
- B1: public domain methods used for the task (EPANET/WNTR for hydraulic/uncertainty; REWET for water-distribution restoration; appropriate wastewater methods for sewer/I&I);
- B2: published DMDU/real-options or adaptation-pathway method where the question is long-horizon investment under deep uncertainty;
- B3: qualified human operator/engineer panel;
- B4: FSRE as-of evidence and portfolio qualification composition;
- ablations removing temporal replay, common-cause receipts, resource stock-flow, cohort floors, independent reference or service-outcome verification.

Freeze the event set, network/catchment, information cutoff, source snapshot, budget, available crews/stock, authority, service floors and model versions. Use event-level and temporal holdouts; stratify by hazard and service cohort. Compare identical service outcomes such as customer-days without safe service, cumulative deficit, duration below quality/floor, time to first and stable restoration, environmental exceedance, lifecycle cost and operator effort.

REWET/DMDU reported results are not directly comparable to FSRE conformance counts. Each method is evaluated on its intended task. Publish unavailable comparisons rather than fabricate a common score.

## 5. Primary and secondary outcomes

Before data access, choose one primary operational outcome. Candidate choices:
- cumulative safe-service deficit per customer/household-day;
- time to restore the minimum safe service floor;
- wastewater service-chain completeness or unaccounted-flow residual with uncertainty;
- decision regret against an adjudicated hindsight reference in a historical planning problem.

Secondary outcomes:
- cohort-specific floor breaches and longest below-floor episode;
- false-qualified / false-safe decisions in sealed negative controls;
- evidence latency, stale/missing data, abstention and claim ceiling;
- lifecycle and emergency procurement cost;
- common-cause/resource overcommitment exposure;
- operator time, comprehension, override and integration burden;
- sustainability measures only where the system boundary and measurement are explicit.

Select minimum detectable effect, power, non-inferiority/harm margins, censoring/missingness, subgroup definitions and analysis code before looking at holdout outcomes. Do not select the primary endpoint after inspecting which one favours FSRE.

## 6. Independence and adversarial controls

- Challenge authors are distinct from evaluator developers for the sealed set.
- Outcome adjudicators are distinct from the operator and FSRE implementation team where feasible.
- Freeze a second data snapshot for independent reproduction.
- Preserve source-level owner assertions separately from evaluator-derived status.
- Include delayed publication, future observations, retrospective revocation, stale telemetry, clock uncertainty, denominator drift, identity transitions, common-cause failures, resource contention and protected-cohort shortfalls.
- Report disagreement and adjudication rather than resolving ambiguity in FSRE's favour.

## 7. Stop conditions

Do not proceed to a higher maturity gate when:
- permission or purpose limitation is unresolved;
- source licensing or privacy basis is insufficient;
- system/cohort identity or denominator cannot be defended for the claim;
- the primary outcome cannot be independently measured;
- no valid incumbent/operator baseline exists;
- clock uncertainty makes decision-time ordering boundary-sensitive and no bounded method is evaluated;
- the study would expose vulnerable residents or critical infrastructure to unnecessary risk;
- the operator cannot retain control or withdraw access;
- independent review cannot reproduce the result.

An honest NOT_ESTIMABLE outcome is a valid research result. It is not permission to infer a positive claim.

## 8. Initial pilot recommendation

The first real pilot should be a **retrospective historical replay on one willing water-services authority's bounded service area**, not a national dashboard and not live automation. Select a period with documented incident/maintenance decisions, source revisions, service interruptions and a defensible cohort denominator. Include both ordinary and adverse periods. Use the current public Green Drop/Treasury sources only as contextual metadata unless owner-authorised system-level operational records are available.

A successful G1 study would establish that FSRE can faithfully reproduce what was knowable at historical decision cutoffs, qualify the proposed portfolio against the source frontier and independently report observed service trajectories and cohort effects. A failed or inconclusive study should narrow the claim, expose the missing evidence, or stop the pilot.

## 9. Current programme maturity statement

As of this protocol's date:
- FSRE synthetic Rust references and dedicated workflows represent software/conformance work (M1a-like).
- No real utility source authentication, system-level Green Drop extraction, owner-approved operational data connector, retrospective operator decision replay, connected shadow trial or measured utility service improvement is established.
- M1b, M2, M3 and M4 remain admission targets, not programme achievements.

The distinction is intentional: reproducible software is a prerequisite for trustworthy infrastructure engineering, not a substitute for evidence that the infrastructure or service improved.
