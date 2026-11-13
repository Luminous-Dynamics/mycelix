# FSRE investment portfolio evaluation protocol v0.1

**Date:** 2026-11-13  
**Status:** Research protocol proposal; no municipal investment result or capital-allocation recommendation is asserted.

## 1. Research question

Does a portfolio selected using service trajectories and qualified recovery options preserve more safe, executable recovery capability under a frozen scenario family than the operator's existing maintenance/capital plan, at an acceptable lifecycle cost and without worsening protected service outcomes?

This is not an automated public-capital allocation system. The operator, legitimate authority and affected communities retain their decision rights.

## 2. Unit of evaluation

The experimental unit is a **portfolio × service lane × scenario set × observation horizon**, not a country, ministry, municipality-wide composite or universal resilience score.

Each study declares:
- service and operator boundary;
- affected geographies/cohorts;
- service unit, target and minimum safe floor;
- quality, safety, affordability, accessibility, environmental and rights guardrails;
- baseline portfolio and feasible alternatives;
- budget/staffing/resource constraints;
- scenario family and disruption horizon;
- source-data frontier, validity and missingness rules;
- total lifecycle-cost boundary;
- independent evaluation method.

## 3. Portfolio arms

At minimum compare:
1. Status quo / no-new-investment control.
2. Existing operator maintenance and capital plan.
3. Asset-condition / risk-priority plan from the incumbent EAM/APM.
4. Standard N-1 or conventional redundancy plan.
5. Human multidisciplinary panel under the same evidence and budget.
6. Strong AI/optimisation agent under the same permitted information, compute and decision boundary.
7. FSRE option-preserving portfolio, using all qualified source-owner evidence.
8. FSRE ablations removing spatial qualification, physical-model qualification, dependency/common-cause qualification, workforce/resource checks, authority checks or independent outcome evaluation.

Treat each vendor comparison as an actual configured deployment where possible. A product brochure is not a benchmark run. If proprietary software cannot be independently run, document the comparator limitation and do not infer a win.

## 4. Outcomes — report separately

### 4.1 Service outcomes
- service deficit over the declared horizon;
- time below minimum safe service floor;
- quality/safety non-compliance exposure;
- safe-degraded service duration;
- time to stable safe recovery;
- time to full restoration;
- observation coverage and unknown duration.

All outcomes are reported by service area/cohort. Unknown intervals are not silently imputed as healthy. No aggregate improvement may mask a protected service lane's severe deterioration.

### 4.2 Recovery-option outcomes
- count of owner-qualified options by service and scenario;
- option capacity and uncertainty range;
- independent failure-domain classes;
- mobilisation time and feasibility window;
- staff, spares, resource and route constraints;
- authority status;
- options gained/lost by portfolio;
- common-cause and unknown evidence frontier.

A graph path or different identifier is not a qualified recovery option.

### 4.3 Economic outcomes
- capital expenditure;
- recurring operating cost;
- maintenance and spares;
- staffing, training, retention and overtime;
- supplier and logistics costs;
- energy/water/chemical consumption;
- integration, licence, compute and data-stewardship cost;
- independent audit and assurance cost;
- migration, transition and vendor-exit cost;
- cost of service disruption and recovery where credible data exist.

Report total lifecycle cost and its uncertainty. Do not conflate budgeted, actual and audited actual Treasury records.

### 4.4 Distribution and spillovers
- service effect by geography/cohort;
- worst-affected lane;
- lower-tail continuity and recovery;
- displaced resource use, outage or environmental burden;
- effects on neighbouring/connected services;
- affordability and access consequences;
- local employment and skills outcomes only where reliable, lawful and within the study purpose.

## 5. Decision under deep uncertainty

Do not collapse all objectives into a hand-tuned weighted score.

Produce a Pareto set of feasible portfolios under declared constraints. For uncertain scenario families, report:
- expected outcomes when a justified probability model exists;
- worst-case outcome over the declared ambiguity set;
- tail risk/CVaR at predeclared levels;
- regret relative to an explicitly identified hindsight oracle;
- robustness and rank stability under plausible parameter/model changes;
- sensitivity to missing observations, demand, cost, lead time and compound failure;
- value of information for additional measurements.

The hindsight oracle is an evaluation reference, not information available to the deployable decision-maker. Do not use an oracle's future knowledge to make the decision-maker appear clairvoyant.

## 6. Causal and statistical design

Pre-register the primary estimand, practical-effect threshold, analysis window, baseline, comparison arm, subgroup plan and stopping rules in Sol-Atlas #338's canonical run manifest.

Where field conditions permit, use randomised or stepped-wedge introduction. Where randomisation is not feasible, consider matched controls, interrupted time series or difference-in-differences only when their identifying assumptions are defensible. Account for:
- seasonality and rainfall;
- demand and population change;
- tariffs, grant flows and electricity availability;
- planned maintenance and capital works;
- concurrent interventions and policy changes;
- changes in telemetry, sampling or reporting;
- spillovers between connected service areas.

Use intention-to-treat where appropriate and report deviations. Independent evaluators should reproduce the outcome from retained source records, not infer causality from model output or correlation alone.

## 7. Benchmark integrity

- Freeze source snapshots, models, scenario manifests and metric code before outcome inspection.
- Bind every portfolio to exact data, model and dependency versions.
- Match staff effort, information, compute, tuning and budget among comparison arms.
- Separate training/dev tasks from sealed holdouts.
- Keep planted challenge truth and grader keys outside agent-visible workspaces.
- Retain failed, aborted and unidentifiable cases.
- Publish uncertainty intervals, missingness, subgroup outcomes and all prespecified metrics.
- Report cost per verified service-improvement unit only when the numerator and denominator are valid and independently observed.

## 8. Proposed qualification targets

These are research objectives, not achieved results or universal standards:
- zero accepted unauthorised consequential actions in the declared intervention suite;
- zero false-independent recovery options in frozen common-cause and identifier-inflation controls;
- zero false-safe outcomes on quality, unknown-observation and stale-evidence controls;
- exact deterministic agreement with independent reference calculations;
- no protected service lane below its minimum safety/quality constraint;
- at least 10% improvement in a preregistered primary service-deficit estimand over the strongest matched baseline, with a paired uncertainty interval excluding the predeclared no-practical-effect region;
- lower lifecycle cost per verified service improvement than the strongest incumbent plan, with full cost and uncertainty disclosed;
- option-set gains remain robust under hidden common-cause, staffing, supply, spatial and authority challenges.

Targets must be adjusted after power analysis and operator consultation. A small synthetic suite cannot establish a production failure probability.

## 9. Falsification

Reject or narrow the FSRE investment thesis if:
- it cannot outperform the incumbent operator workflow on matched, service-relevant tasks;
- option qualification does not reduce false redundancy or unsafe recovery decisions enough to justify cost and delay;
- existing deployed products already provide equivalent independent cross-operator verification at lower lifecycle cost;
- the apparent portfolio improvement depends on missing or stale data;
- benefits accrue to one cohort by transferring unacceptable harm to another;
- portfolio choices reduce local autonomy or create new provider, supplier or data concentration;
- the outcome cannot be independently reproduced.

## 10. Existing implementation owners

- Mycelix #2006 / #4816 / #5015: authority, study boundaries, multi-scale system context and service portfolio.
- Mycelix #5923 / #5981: canonical dependencies and hidden common causes.
- Sol-Atlas #21 / #338: spatial alternatives and benchmark preregistration.
- Symtropy #4392 / #3787: physical adequacy and matched runtime/semantic comparison.
- Symthaea #10366: causal diagnosis and experiment selection.
- Mycelix #338: completed-effect evidence envelope, as an input to post-action evidence lineage, not as a substitute for independent service-outcome verification.
