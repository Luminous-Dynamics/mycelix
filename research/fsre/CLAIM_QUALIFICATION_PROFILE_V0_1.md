# FSRE claim qualification profile v0.1

**Date:** 2026-11-13  
**Status:** Research profile for claim scope and evidence admissibility.  
**Claim ceiling:** This document defines how FSRE should qualify statements. It does not certify any current FSRE claim, source, model, operator, system or intervention.

## 1. Purpose

FSRE is intended to produce claims about essential services, dependencies, resilience options and interventions. Those claims can influence public investment, operator response, procurement and trust. The programme must therefore control the boundary between:
- what was directly observed;
- what a source reported;
- what a deterministic evaluator computed;
- what a domain model predicted;
- what a human/operator judged;
- what an intervention changed;
- what an independent evaluator could reproduce.

These are distinct epistemic categories. They do not form a universal ladder where a higher label automatically dominates a lower one. A technically valid synthetic test does not become field evidence; a field observation does not prove causality; a model result does not prove the underlying asset state.

This profile composes existing Mycelix provenance and completed-effect evidence, Sol-Atlas benchmark manifests/provenance, Symtropy model-adequacy receipts, Symthaea causal-investigation receipts and operator records. It does not create a second evidence ledger, trust root, benchmark manifest or universal confidence score.

## 2. The claim is the unit of qualification

A report should not label an entire system "verified" because one test passed. Qualify each material claim within a declared scope.

A claim record should reference, not duplicate, the owner-native evidence:
- stable claim identifier and schema version;
- exact statement and claim type;
- service, operator, cohort and geographic scope;
- unit, denominator and measurement method;
- observation time, validity interval, time basis and uncertainty;
- source record IDs, schema versions, digests and owner;
- data/model/scenario/evaluator versions;
- method and analysis plan;
- missingness, staleness, contradictions and known counterevidence;
- assumptions and exclusions;
- verifier identity/role and verification method;
- claim ceiling and expiry/revalidation trigger.

The exact record should be versioned through Mycelix's canonical evidence/provenance owners. FSRE may render a derived claim report, but it must not become the authoritative copy of underlying evidence or authority.

## 3. Claim classes — keep these separate

| Class | What it establishes | What it does not establish | Example FSRE language |
|---|---|---|---|
| Synthetic fixture semantics | A frozen synthetic input produces the declared deterministic output in the named evaluator/version | Real-world frequency, physical accuracy, live source authenticity or field effectiveness | "The v0.1 synthetic cascade fixture matched its six expected failed-service-sink sets." |
| Software conformance | The named code passed tests, reference agreement, static analysis or a golden output at an exact revision | Correctness outside the tested contract, production reliability or external data truth | "The recovery-option evaluator passed 20 deterministic tests and a CLI golden comparison at exact commit X." |
| Published public-source report | The named publisher reported a value for the stated population, unit, denominator, geography and period | Current operational state, unreported areas, causal explanation or a different geographic/time grain | "Stats SA's 2025 survey reported household piped-water access at the published national estimate." |
| Derived public-data analysis | A documented computation over frozen public-source rows is reproducible | Accuracy beyond source limitations, current service delivery or causal impact | "This analysis links Treasury and DWS source records for historical context, with unresolved mappings disclosed." |
| Operator-observed condition | A named operator/source measured or recorded a condition within an explicit time, asset/service and method boundary | Independent truth merely because it was reported; future validity beyond the evidence window | "The operator's telemetry source reported the specified flow at the named time, subject to its calibration and freshness status." |
| Model-supported result | A qualified model predicted or simulated an outcome within its documented validity domain and uncertainty | That the physical outcome occurred, that the model is valid outside its domain, or that a proposed intervention is safe to execute | "Symtropy's qualified model estimated the declared capacity range under scenario revision Y." |
| Expert/operator assessment | A responsible expert or operator made a bounded judgment with documented method and authority | Independent measurement, proof of physical effect or generalisation to unreviewed contexts | "The authorised engineering review judged the fallback procedure feasible under the stated conditions." |
| Intervention-associated observation | A measured change followed an intervention or operational change | Causation where counterfactual, confounding, spillover and measurement changes are not addressed | "Service deficit was lower in the observed post-intervention period; causal attribution remains unresolved." |
| Causally identified intervention effect | A study design with defensible identification assumptions estimates an effect against a specified counterfactual | Universal transfer, unmeasured outcomes, different service settings or guarantees of future effect | "The preregistered stepped-wedge study estimated the declared effect for the named population/period, with uncertainty and limitations." |
| Independently verified service outcome | A party independent of the intervention proposer/operator can reproduce the outcome from retained owner-native evidence under a declared protocol | Automatic causality, universal safety, or regulatory/certification status | "The independent evaluator reproduced the declared service outcome from the retained evidence package." |

A claim may have more than one class, but the classes must be stated separately. For example, a field outcome may be independently reproduced while its causal attribution remains unresolved.

## 4. Three orthogonal questions

Every important claim should answer three separate questions.

### 4.1 Provenance: where did the evidence come from?
- Was the source authenticated under its owning system's trust model?
- Is the exact source record/schema/version retained?
- Is the record bound to the correct service, scenario, time basis and scope?
- Is the source within its valid-time interval?
- Can a reviewer retrieve and re-evaluate the owner-native record?

A local string reference or a signature over a message does not establish that the claimed physical fact is true. The evaluator must not authenticate records by naming convention.

### 4.2 Adequacy: is the evidence fit for this claim?
- Does the source measure the stated unit and denominator?
- Does the geographic and temporal grain match the claim?
- Does the model cover the relevant operating envelope and population?
- Are calibration, sampling and sensor uncertainties disclosed?
- Are missing and contradictory observations retained?
- Is the source fresh enough for the asserted decision?

A national survey percentage cannot substantiate neighbourhood uptime. A wastewater risk classification cannot substantiate current drinking-water quality. Municipal repairs expenditure cannot prove a particular work order was completed.

### 4.3 Independence: who verifies the claim?
- Who proposed the intervention?
- Who operated or executed it?
- Who owned the source data?
- Who independently evaluates the claimed outcome?
- Did the evaluator have access to the same frozen inputs and enough information to identify missing evidence?
- Is there a conflict of interest or shared failure domain that weakens the independence claim?

A separate organisation is not automatically independent if it relies on the same unaudited source, model, vendor, controller or economic incentive.

## 5. Intervention claim protocol

An intervention-effect claim should be qualified only after the following are frozen before outcome inspection:

1. Service/cohort and operator boundary.
2. Primary estimand and measurement units.
3. Baseline period, comparator and counterfactual strategy.
4. Treatment, timing, exposure and deviations.
5. Guardrails for quality, safety, affordability, accessibility, environment and rights.
6. Confounders, concurrent interventions and seasonality.
7. Spillover and displacement hypotheses.
8. Missingness and observation-validity rules.
9. Practical-effect threshold, sample size/power analysis and uncertainty method.
10. Independent evaluator and reproducibility package.

Use randomised or stepped-wedge designs where operationally and ethically feasible. Where not feasible, a matched control, interrupted time series or difference-in-differences analysis must state and test its identifying assumptions as far as possible. Correlation, temporal order, a model's prediction or a signed execution receipt alone cannot establish causality.

Publish negative and inconclusive results. If the outcome is not identifiable with available evidence, the correct claim is "not identified" or "unknown," not a positive narrative.

## 6. Service trajectory claims

A service-trajectory claim must name:
- service and cohort;
- target, minimum safe floor and quality thresholds;
- exact measurement unit and denominator;
- observation interval and maximum hold duration;
- time-basis and uncertainty rule;
- known/unknown coverage;
- safe-degraded/full/unsafe duration;
- service deficit and integration rule;
- first known breach and stable recovery definition.

The current Rust trajectory evaluator computes deterministic metrics from supplied observations. It does not authenticate those observations, establish water quality, model physical hydraulics or prove an intervention effect. Its output qualifies as software-computed metrics over provided input, not an independent service measurement.

Unknown intervals remain unknown. Do not forward-fill stale data or assume missing telemetry indicates healthy operation.

## 7. Recovery-option claims

"Qualified for declared scenario" means only that the seven required owner/operator dispositions were present, current, scope-bound and qualified under the evaluator's declared rules:
- physical feasibility;
- spatial access;
- dependency independence;
- staffing;
- resources;
- authority;
- safety.

It does not mean:
- a real asset or supplier is available now;
- the evidence itself has been authenticated by the reference evaluator;
- the option is authorised for dispatch;
- the action is safe outside the tested scenario/time;
- the service will certainly recover;
- capital or procurement should be approved.

The report explicitly states `execution_authority=NONE` and `operational_actuation=NOT_AUTHORISED`. Consequential actions require separate authority and operator-side revalidation.

## 8. Public-data claims and the South African water study

The source register currently records 10 public source entries, all `DISCOVERED_NOT_EXTRACTED`. That status means only that the source has been identified; it does not mean that raw data has been downloaded, parsed, licensed, reconciled or qualified.

Keep these claim domains separate:
- Stats SA household access survey estimates;
- DWS Blue Drop drinking-water risk categories;
- DWS Green Drop wastewater-system assessments;
- DWS No Drop distribution-efficiency indicators;
- Treasury municipal financial submissions.

Each has its own denominator, unit, period, geographic grain, revision and rights status. None is a current Johannesburg service-hours, water-quality, pressure, leakage or work-order baseline. The first public-data study may legitimately conclude that the evidence is insufficient for a service-outcome claim.

## 9. Revalidation and claim expiry

A claim must be reconsidered when:
- source schema, source record, scenario or evaluator version changes;
- validity interval expires or clock uncertainty changes;
- service, jurisdiction, cohort or operating boundary changes;
- a new common-cause, model-adequacy or contradictory observation appears;
- authority is revoked or purpose changes;
- the physical model leaves its qualified domain;
- a source is corrected, restated or withdrawn.

Revalidation is a new evaluation referencing the new input set, not mutation of the old claim. Preserve the prior claim and its exact provenance for audit.

## 10. No scalar evidence or resilience score

Do not combine provenance, model adequacy, independence, service performance, rights and uncertainty into one number. Do not let a high availability estimate compensate for failed water quality or expired authority. Do not let many model outputs compensate for absent observations.

Present each claim's scope, source frontier, uncertainty, independent verification state and unresolved gaps directly. If a dashboard needs an aggregate indicator, it must be clearly labelled as a navigation aid and preserve the underlying claims and veto conditions; it cannot be treated as a decision authority.

## 11. Existing owner systems

- Mycelix evidence/provenance, time, authority and completed-effect evidence (#338) own source lineage and execution-effect envelopes.
- Sol-Atlas #338 owns canonical benchmark preregistration; spatial/provenance owners qualify location and alternative evidence.
- Symtropy #4392/#3787 own model adequacy, domain validity, physical trajectories and model/runtime comparisons.
- Symthaea #10366 owns competing hypotheses, discriminating experiments, belief updates and causal-investigation receipts.
- The responsible operator owns staffing/resource/safety attestations and operational decisions.
- The independent evaluator owns its reproduction and claim statement.

FSRE only composes claim contracts and service-level evaluation across those owners. It does not create another ledger, trust root, clock, model, causal engine or capital-allocation authority.

## 12. Review and falsification

The profile is useful only if it reduces unsupported claims and makes independent review cheaper without making valid claims unusably difficult to express.

Measure:
- percentage of material claims with exact scope, denominator, source references and claim ceiling;
- rate of stale, missing, conflicting and wrong-scope evidence detected;
- unsupported claim acceptance rate on blinded negative controls;
- evaluator agreement on frozen reference cases;
- reviewer time to reproduce a claim;
- cost to produce a fully qualified service-outcome package;
- over-abstention rate and percentage of decisions blocked only by unavailable data;
- number of claims narrowed or withdrawn after new evidence.

If the profile adds administrative burden without reducing false confidence, duplicated evidence or review cost, simplify it. Do not weaken non-negotiable authority, safety, provenance or unknown-state invariants to improve apparent benchmark scores.
