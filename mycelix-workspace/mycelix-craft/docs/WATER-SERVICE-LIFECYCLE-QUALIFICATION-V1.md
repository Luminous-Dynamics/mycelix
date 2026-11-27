# Water Service Lifecycle Qualification V1

- **Benchmark:** CRAFT-WATER-SERVICE-REPAIR-V1 (#6771)
- **Status:** Proposed measurement and integration protocol
- **Version:** 1.0-draft
- **Date:** 2026-11-27
- **Data boundary:** Public source metadata plus synthetic conformance fixtures only

## 1. Research question

Can an evidence-governed service workflow reduce false restoration claims, unsafe or premature closure, repeated visits and avoidable review effort while improving the service outcomes experienced by affected people?

The unit of evaluation is not a ticket or a model answer. It is a **service episode** with a declared scope, event chronology, evidence history, responsible authorities, affected-service population and independently measured outcome.

The benchmark does not assume that a well-formed record means a repaired network, that a completed work order means restored service, or that an eligible closure packet authorizes closure.

## 2. Measurement discovery: do not compare unlike 48-hour KPIs

Johannesburg Water's 2025/26 Business Plan labels its water-burst indicator as restoration within 48 hours and provides a 67.46% baseline (labelled 2023/24) and 90% FY2025/26 target. Its detailed operational definition says the 48-hour milestone is that someone is on site and has initiated resolution; it explicitly says this does not necessarily mean the callout was resolved.

The Johannesburg Water FY2024/25 Annual Report separately reports 70.35% of bursts repaired within 48 hours against an 85% target. The two published figures must not be compared as though the numerator, event, denominator and reporting period were equivalent. The annual-report achievement statement and business-plan operational definition must first be reconciled with the metric owner.

References:
- Johannesburg Water, Business Plan 2025/26, printed pp. 83-84 and 95-96: https://www.johannesburgwater.co.za/wp-content/uploads/2025/07/JHB-Water_Business-Plan-2025_For-Upload_FINAL_070725.pdf
- Johannesburg Water, Integrated Annual Report 2024/25: https://joburg.org.za/work_/Documents/Annual-Integrated-Report-2025/City-of-Johannesburg-Annual-Intergrated-Report-2025.pdf

Every reported KPI must carry a versioned measurement definition containing:
- event start and stop semantics;
- reporting period and time zone;
- numerator, denominator and eligibility rules;
- source system and extraction time;
- treatment of reopened, cancelled, duplicate and censored episodes;
- scope, units, exclusions and known data limitations;
- owner and approval/revision history.

A label is not a definition. If these elements cannot be reconciled, the output is `not_established`, not an inferred comparison.

## 3. Lifecycle event model

Preserve, at minimum, these independently timestamped milestones:

1. Need/incident notification received.
2. Acknowledgement and triage.
3. Dispatch.
4. Crew arrival and first response.
5. Permit/isolation and work authorization.
6. Repair work started.
7. Repair work completed.
8. Independent pressure/flow restoration observed.
9. Required water-quality sample collected.
10. Qualified laboratory result and applicable quality release.
11. Alternative supply started, delivered and measured where interruption persists.
12. Local authority closure or explicit continued monitoring.

Each event has both **valid/event time** (when it happened) and **recorded/knowledge time** (when the system learned or admitted it). Corrections append new events; they do not rewrite history. Late evidence must not be backdated into a timely operational action.

A service episode may have more than one work order, observation, source correction, outage interval or closure attempt. Repeat incidents are new episodes linked to historical records, not edits to the prior event.

## 4. Outcome vector — do not collapse into one score

Report every dimension separately. A composite score can conceal a dangerous trade-off and is not an acceptance criterion.

| Dimension | Operational definition | Evidence boundary |
| --- | --- | --- |
| First response time | Notification to crew arrival / first qualified on-site response | Dispatch telemetry or independently time-stamped arrival; not ticket status |
| Repair completion time | Notification to verified work completion | Work-step evidence, permits and accountable completion record |
| Verified restoration time | Notification to sustained pressure/flow recovery against a preregistered local profile | Independent, in-scope measurements over the authority-approved observation window |
| Quality release time | Sampling to qualified result and release decision | Sample identity, sampling point/time, applicable parameters, laboratory method and source authority |
| Customer-minutes lost | Sum across affected service points of interruption duration, weighted by the affected customer count | Declared population denominator and service-point observation window |
| Pressure deficit exposure | Time integral of the positive pressure shortfall below the local target, reported by service point/zone | Time-series coverage, sensor calibration/provenance and profile revision |
| Alternative-supply adequacy | Qualifying drinking-water volume delivered per person per day during interruption | Actual delivery, source quality and accessible delivery point; dispatch is not delivery |
| Repeat failure | Same asset/zone recurrence within preregistered 7/30/90-day windows | Stable asset/zone identity and explicit recurrence rule |
| Safety and quality | Unresolved hazard, permit, isolation, worker-safety or water-quality states | Competent professional / statutory authority evidence |
| Distributional service burden | Outcome vector stratified by approved, privacy-protecting service-area cohorts | Minimum cohort sizes, no household-level publication and disclosed missingness |
| Operational burden | Review minutes, false holds, repeat visits, emergency procurement and rework | Time/cost capture method and common scope across baselines |

### Calculation notes

For service point (i), interruption duration (d_i) and affected population (n_i):

[
CML = \sum_i n_i d_i
]

Report units explicitly (for example, customer-minutes). Do not infer population from ticket count.

For pressure (p_i(t)) and the locally approved minimum profile (p_{min,i}(t)), pressure deficit exposure may be reported as:

[
PDE_i = \int_{t_0}^{t_1} \max(0, p_{min,i}(t)-p_i(t))\,dt
]

The integral is a diagnostic measure, not a universal statutory threshold. Declare units, sampling frequency, interpolation, calibration, missing-data policy and profile version. Also report deficit duration and worst affected service-point cohort so a mean cannot hide local deprivation.

Water quality is a separate safety gate. A passed pressure observation cannot satisfy a pending, expired, superseded, contradictory or high-risk laboratory result.

## 5. Industry and standards alignment

The water-service benchmark must align with existing utility-performance frameworks rather than claim to invent their measures.

- **ISO 24516-1:2016/Amd 1:2025** was published in September 2025. Its scope explicitly adds performance indicators and requirements for monitoring and review for drinking-water distribution networks. Use it as a standards reference for indicator governance and monitoring/review; do not claim Mycelix certification or reproduce protected standard text. Official catalogue: https://www.iso.org/standard/87404.html
- The **World Bank Water Utility Turnaround Framework (2018)** identifies ten customer-oriented dimensions of utility success: accessibility, safety, sufficiency, reliability, convenience, cost-effectiveness, financial sustainability, affordability, responsiveness and transparency. Its Appendix F also presents a weighted seven-indicator success index. These dimensions and composite approaches pre-exist this work. Craft's contribution to test is the evidence and authority chain linking service events to separate outcome measures, not a novel set of utility KPIs. Reference: https://documents1.worldbank.org/curated/en/515931542315166330/pdf/Water-Utility-Turnaround-Framework-A-Guide-for-Improving-Performance.pdf (Appendix F, Tables F.1–F.2).
- South Africa National Treasury's **MTSR Guidance Note 4, Annexure 1 (February 2026)** defines W8, water complaints response within 48 hours, as a response event: complaint logged, someone is on site, matter appraised and a resolution process is initiated. It explicitly states that the callout is not necessarily resolved. The calculation notes also say multiple complaints about the same fault, when logged separately, count as one callout. The terminal-event rule and duplicate-callout inclusion rule must both be versioned; label/deadline matching alone is insufficient. Reference: https://lg.treasury.gov.za/approot/common/guiding_notes/GN4-Annexure%20A%20Technical%20Indicator%20Description%20and%20Minimum%20Commitments%20Definition%20Sheets%20updated%20February%202026.pdf (W8, printed page 31).
- Johannesburg Water's FY2024/25 Annual Report separately states its percentage of bursts restored within 48 hours. The annual-report statement and business-plan/municipal response definition must be reconciled with the metric owner before headline percentages are compared. See NRVCA-013/#5434, NRVCA-014/#5437 and the reusable dual-axis comparator extension in PR #5442.

This leads to an important implementation boundary: use the existing NRVCA dual-axis comparator (now with nine water-specific synthetic test pairs, including duplicate-callout denominator drift) for declared measurement-definition and target-policy comparability. WSR-016 should become a caller of that owner, not a second KPI-comparability implementation inside Craft. Craft remains responsible for its service lifecycle, owner-native claim/closure evidence and the mapping of each metric observation to the correct event context.

## 6. Statutory anchors and scope boundaries

The revised Compulsory National Water and Sanitation Services Standards, Government Gazette 52814 (6 June 2025), provide test anchors including:
- alternative drinking-water supply of at least 15 litres per person per day where interruption exceeds 48 hours, through safe and reasonably convenient delivery points;
- monitoring-programme and source-origin requirements for alternative supply continuing beyond one week;
- SANS 241-based drinking-water quality management and specified sample points, frequency and parameters;
- ISO 17025-accredited laboratory methods or a credible proficiency-tested alternative;
- notification to the DWS regional office and provincial Health Department district office within 12 hours after confirmation of a health-risk incident, with applicable drinking-water quality advisory communication.

Official source: https://www.gov.za/sites/default/files/gcis_document/202506/52814gon6292.pdf

These are scenario anchors for qualified human/professional interpretation. Synthetic conformance results are **not legal-compliance determinations**. The benchmark must pin the exact applicable instrument and revision; no general-purpose rule engine may infer jurisdiction or applicability from a label.

## 6. Metamorphic qualification

The companion `metamorphic-cases.json` defines 20 proposed counterfactual mutations. They test whether an evaluator changes only the conclusion elements actually affected by a change.

Required properties include:
- provider response or completion cannot substitute for independent physical restoration;
- evidence with changed context, stale status or a shared common cause cannot remain admissible as before;
- repair scope and action denominators cannot silently shrink;
- a correction/revocation creates targeted requalification while preserving the historical decision;
- the 48-hour response/repair semantic conflict remains unresolved until definition and period are reconciled;
- an improved arrival time cannot be reported as an improved restoration time;
- customer-minutes lost scale with the affected population when duration is held constant;
- a transient pressure spike or network average cannot conceal a sustained/local service deficit;
- regulatory boundary values are tested explicitly, without rounding away a failed threshold.

The current shell/JSON validation only checks corpus structure and referential integrity. It does **not** execute the mutations against the Rust cores. Executable differential replay requires adapters that invoke the accepted versions of the semantic owners and compare their outputs to an independent oracle. Do not call the mutation vectors "passed" until that runner exists and has executed at the recorded source heads.

## 7. Cross-core composition contract

The integration path is a sequence of typed handoffs, not a new generic authority or currentness service:

- Need Intake (#6750): scope, community need, dissent, assumptions and source references.
- Project Formation (#6733): project/package/capability envelope and participant acceptance.
- Claim Support (#6754): exact semantic support for each material claim.
- Decision Closure (#6757): material-element coverage against approved, versioned mappings.
- Evidence Lineage (#6762): correction, revocation, expiry and withdrawal propagation.
- Impact Qualification (#6769): event/context-bound change impact and surviving evidence sufficiency.
- Existing observation/provenance and temporal owners (#6140, #181, #1087, #2631/#1207, #4835): observation independence, leases, applicability, epochs and independent evaluation.

The pilot adapter must:
1. preserve each owner's schema version, exact artifact/context digest, authority revision, validity interval and disposition;
2. reject missing, stale, out-of-scope or unverifiable owner receipts;
3. preserve owner-specific uncertainty and reason codes instead of flattening them into a Boolean;
4. block on required safety/quality/authority states;
5. record the exact input/output digest for deterministic replay;
6. distinguish "eligible for local review" from the competent authority's closure decision.

No downstream adapter may manufacture a missing owner receipt, reuse a digest across distinct contexts, widen a lease or elevate machine output into human authority.

## 8. Fair industry baselines

Benchmark against configured, documented workflows rather than product names as strawmen:

1. Current authority work-order process, with its real event/status semantics documented.
2. Structured manual checklist with independent close-out review.
3. Plain traceability graph / requirements-management workflow.
4. Configured IBM DOORS Next or Jama Connect workflow where the evaluator has lawful access and equivalent configuration.
5. Single-model LLM and retrieval-grounded LLM assistance.
6. Multi-agent workflow with the same source packet and review budget.
7. Evidence-governed cross-core workflow.

Utility service-performance measures (including interruption duration, pressure, customer impact and losses) remain the appropriate outcome baselines. Mycelix's hypothesis is not a new generic water KPI; it is the auditable linkage of need, authority, claim support, material closure, event lineage, scoped impact and measured consequence.

All baselines receive the same source packet, permitted data, service scope, intervention opportunity and time budget. Record configuration, human review time, model/version, prompts, retrieval snapshot and cost.

### Cohort matching and censored-event handling

Raw 48-hour percentages must not be used as a cross-utility league table. The World Bank finds that water-utility performance varies substantially across and within countries, with population scale, connection patterns and local conditions influencing outcomes. The AWWA Utility Benchmarking Program also organizes comparisons by utility type, size and region.

For matched comparisons, stratify or match at least:
- utility/service-area size and population served;
- household connections versus shared connections/public standposts;
- baseline continuity of supply and planned versus intermittent supply;
- outage cause, incident severity and affected asset class;
- pressure zone/topography and relevant network characteristics;
- reporting-period and metric-definition vintage.

Report unmatched and excluded cohorts instead of silently dropping them. Do not infer a causal efficiency difference from a raw difference between unmatched utilities.

**Open episodes must remain in the denominator.** A report that computes time-to-repair only over closed work orders can omit long-running unresolved cases and create survivorship bias. At each frozen extraction cutoff:
- include all eligible notifications in the declared cohort;
- record whether each episode has reached the named terminal event;
- represent still-open episodes as right-censored at the knowledge-time cutoff, not as completed and not as absent;
- preserve duplicate, cancelled, reopened and repeat-event rules explicitly;
- report number at risk / still open, observed event counts and missing timestamps alongside completion percentages.

For larger prospective datasets, evaluate time-to-event distributions (including median/p90 where estimable) and survival estimates with stated censoring assumptions, rather than only threshold pass rates. Do not apply survival estimators when their censoring assumptions are unsupported; publish the raw counts and uncertainty instead. A 48-hour indicator remains one view of the distribution, not a complete service-quality measure.

## 9. Qualification targets — targets, not results

| Dimension | Target |
| --- | --- |
| False safe/closed outcomes on frozen high-consequence cases | 0 |
| Provider-only completion accepted as restoration | 0 |
| Stale, revoked, wrong-zone or wrong-context evidence silently accepted | 0 |
| False `NotAffected` on high-consequence mutations | 0 |
| Shared-cause sources counted as independent | 0 |
| Frozen known-answer scenario correctness | 100% |
| Deterministic replay of frozen inputs and receipts | 100% |
| Material-element recall | At least 99%, reported by domain and adjudicated mapping |
| Unnecessary invalidation of unaffected independent evidence | 0 on adjudicated fixtures |
| KPI comparisons with unreconciled semantics/period/denominator | 0 |

Report false holds and requalification latency separately; fail-closed behavior is not free. Report confidence intervals and counts, not percentages alone, for small corpora.

## 10. Prospective pilot design

The synthetic benchmark is only the first qualification tier.

A prospective pilot requires written authorization from the responsible Water Services Authority/Institution, community participation, data-sharing and privacy agreements, information-security review, worker-safety protocols, professional approval of pressure/quality criteria, and a documented incident/escalation process.

Before outcome comparison:
- preregister cohort selection, control/matched-area design, outcome definitions and analysis;
- obtain a new baseline using the same metric definition on both sides;
- use a prospective holdout and independent measurement/adjudication;
- report missing data, adverse events, null effects and differential service burden;
- separate process improvement from causal service improvement.

Where ethically and operationally acceptable, use a controlled or stepped-wedge rollout with a predeclared analysis. Otherwise describe the comparison as observational and do not attribute causality to Mycelix.

No partnership, system access, field deployment, regulatory certification or service improvement is claimed by this protocol.
