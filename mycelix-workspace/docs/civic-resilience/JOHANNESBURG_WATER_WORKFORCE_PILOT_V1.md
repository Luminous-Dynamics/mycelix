# Johannesburg water reliability and paid-work pilot
## CIV-RES research specification v1

**Status:** proposal for partner discovery and synthetic/shadow-mode evaluation  
**Research snapshot:** 14 October 2026  
**Parent tracking issue:** [#5015 — Civic automation portfolio](https://github.com/Luminous-Dynamics/mycelix/issues/5015)  
**Related source-contract work:** [PR #5008 — preserve service-standard semantics](https://github.com/Luminous-Dynamics/mycelix/pull/5008)  
**Authority:** no municipal or utility endorsement, data-sharing permission, API access, live dispatch authority, or funding commitment is established by this document.

## 1. Recommendation

Start with a **water-fault work-coordination shadow pilot**. Do not start by building a replacement municipal portal or connecting directly to utility telemetry.

The first deliverable should let an authorized operator (or a research team using synthetic records) see whether reports, public notices, inspection visits, repair claims, and completion evidence can be represented without losing source identity, timing, or responsibility. It should help prioritize human attention and reduce avoidable repeat visits, but must not silently submit, assign, close, or verify real utility work.

Water is the best initial candidate because Johannesburg Water has publicly described active investment in pressure management and acoustic leak detection. Its 23 July 2026 statement said that a further 306 smart pressure controllers were being installed, targeting completion by the end of September 2026; it also reported 225 noise loggers installed and another 125 planned by October. These are the statements and schedule in that release—not independently verified completion status as of this research snapshot. No pilot may assume sensor data is accessible to Mycelix or that rollout is complete.

The software opportunity is the operational bridge between observation, authorized inspection, work coordination, evidence, repair, and feedback—not duplication of those sensors.

## 2. Evidence register and limits

| ID | Source | What it supports | What it does not establish |
|---|---|---|---|
| JW-1 | [Johannesburg Water smart-technology release, 23 July 2026](https://www.johannesburgwater.co.za/blog/2026/07/23/johannesburg-water-accelerates-smart-technology-rollout-to-reduce-water-losses/) | Utility-reported pressure-controller and leak-logger programme; prospective maintenance workflow | Current installation completion, telemetry access, exact pilot zone, measurable pilot effect, or Mycelix authority |
| COJ-1 | [City SDBIP, IDP and Service-Level Standards document index](https://joburg.org.za/documents_/Pages/Key%20Documents/SDBIP-IDP-and-Budget/SDBIP-IDP-And-Budget.aspx) | The City lists its 2026/27 SDBIP, final IDP and 2025/26 service-level standards charter | The current effectiveness of any specific indicator or SLA in a case. The linked 2026/27 SDBIP PDF could not be retrieved during this research pass; its table contents are not asserted here. |
| JW-2 | [Johannesburg Water service standards](https://www.johannesburgwater.co.za/services/) and [PR #5008](https://github.com/Luminous-Dynamics/mycelix/pull/5008) | Published target language needs distinct action, subject, clock and version semantics | Effective date/version of the web table, adopted production threshold, exact clock start/exclusions, or case-specific status |
| SA-1 | [Stats SA, Victims of Crime 2025/26 key findings](https://www.statssa.gov.za/?PPN=P0341&SCH=74591&page_id=1856) | National victimization estimates show that reported crime is incomplete: 37.3% of people who experienced personal-property theft, 41.7% of people who experienced street robbery, and 48.2% of households experiencing housebreaking reported some or all incidents | Johannesburg-specific incidence or a causal estimate of any proposed intervention |
| JOBS-1 | [Department of Public Works and Infrastructure, 2025/26 budget speech](https://www.gov.za/news/speeches/minister-dean-macpherson-public-works-and-infrastructure-dept-budget-vote-202526-09) | Government's stated EPWP reform direction includes tackling gatekeeping/exploitation and linking participants to skills, permanent work or entrepreneurship | Eligibility, available funding, local slots, or procurement approval for this pilot |
| SA-JOBS-2 | [Stats SA, South Africa’s Youth and the Labour Market in Q1 2026](https://www.statssa.gov.za/?p=19526) | National Q1 2026 unemployment was 60.9% for ages 15–24 and 40.6% for ages 25–34; NEET rates were 37.6% and 45.6% for ages 15–24 and 15–34 respectively | These are national statistics, not Johannesburg estimates, recruitment criteria, funded posts, or a forecast of pilot impact |

Source-currentness rule: retain issuer, exact URL, retrieval timestamp, visible publication/effective dates, observation status, and an immutable digest where bytes can lawfully be captured. A fetch failure is a missing observation, not evidence that a target or programme does not exist.

## 3. Intended workflow

1. **Observe.** Accept synthetic incidents in development. In a later authorized trial, use only approved manual observations or explicitly permitted exports. Keep original source and source timestamp.
2. **Normalize without merging identities.** Give the Mycelix record its own stable identity. Preserve external issuer, channel, reference number, and source-currentness separately. Equal-looking IDs from the City, utility, contractor, or public bulletin are not automatically the same case.
3. **Qualify.** Classify reports by issue type, location precision, urgency evidence, source reliability, possible duplicates, and missing fields. Uncertain or conflicting cases remain unresolved for a human.
4. **Recommend.** Show candidate clusters, repeat faults, likely trade/parts needs, and travel-efficient work batches. Recommendations are advisory and explain the inputs and uncertainty.
5. **Authorize.** A designated operator decides whether to inspect, assign, escalate, or submit through an existing authorized channel. No automated dispatch or external write access in the initial pilot.
6. **Record work.** Capture attendance, time, materials, inspection observations, repair claimed, exceptions, and links to authorized external work references. Do not expose precise sensitive infrastructure details publicly.
7. **Verify separately.** A repair claim is not verified completion. The verifier should be independent of the person/contractor whose work is being checked, subject to feasible safety and operational constraints. Record the verifier, method, timestamp, evidence class, and unresolved conflicts.
8. **Close the feedback loop.** Only an authorized source can establish its own current case status. Mycelix can report its evidence state, not rewrite the utility's state. Corrections and reopening preserve history.

Required state distinctions:

- Reported ≠ source accepted ≠ assigned ≠ inspected ≠ repair claimed ≠ independently verified ≠ resident-confirmed.
- Response ≠ restoration ≠ repair/replacement ≠ notification ≠ water-quality result.
- A working-day target is not silently converted to a calendar-hour target.
- A public bulletin is not per-case status; a portal link is not API authority; submission is not acceptance.
- Unknown, stale, conflicted or unverifiable information must remain visibly unknown.

## 4. Deliberately narrow pilot boundary

### Included in the initial prototype

- Synthetic water-fault corpus with duplicates, contradictory observations, stale notices, missing timestamps, re-opened cases, false completion claims and unavailable sources.
- Manual import of public notices and standards with exact source references and retrieval metadata.
- Mock work queue with advisory prioritization and human-authored decisions.
- Evidence lineage and separate completion-verification record.
- Baseline definitions, privacy threat model, job-task specification and cost model.
- Manual fallback that works even if a data feed or model is unavailable.

### Excluded until an authorized partner and contract exist

- Direct utility telemetry, portal scraping, authenticated portal automation or undocumented API use.
- Writing to a City/utility system, automatic crew dispatch, changes to bills/accounts, or automatic entitlement/penalty decisions.
- Production SLA breach claims based on unversioned web text.
- Public maps of vulnerable pipe segments, sensor placement, repair schedules, residences or precise incident locations.
- AI-based determination of fraud, worker misconduct, resident credibility or criminal suspicion.

### Site/area selection

Do not select a neighborhood from public anecdotes or model inference. An authorized operational partner should nominate a bounded, non-sensitive service zone using aggregated repeat-fault and workload data. Selection requires a written data-use basis, privacy assessment, safety review and baseline availability. The public repository must not contain exact vulnerable-asset coordinates or operationally sensitive details.

## 5. Jobs and delivery model

The pilot must name the payer before describing an opportunity as funded employment. Candidate roles are not yet funded positions.

| Role/pathway | Concrete work | Progression |
|---|---|---|
| Field asset/data-quality assistant | Verify permitted asset labels, normalize records, document observation provenance and flag duplicates | GIS/data-quality technician |
| Maintenance logistics coordinator | Group authorized inspection work, track parts requests, surface repeat faults and keep manual fallback current | Operations planner / service coordinator |
| Qualified trade worker or apprentice | Perform inspections and repairs only within qualifications, supervision, safety rules and authorized work orders | Recognized trade/contractor employment |
| Independent quality/evidence reviewer | Check work evidence, sample records, document exceptions and prevent unsupported closure | Quality assurance / compliance technician |
| Software and integration engineer | Maintain contracts, validators, adapters, reliability tests and data minimization | Systems engineering / civic-tech work |
| Reuse and materials-custody worker | Where repair work legitimately generates recoverable material, document safe custody and lawful disposition | Repair/reuse or compliant recycling enterprise |

### Potential paying routes (unconfirmed)

- Existing authorized utility maintenance or operations procurement.
- An already appointed service contractor that has budget and contractual permission to adopt a workflow tool.
- An approved public-work or skills programme, subject to its current rules and real funding.
- A clearly funded grant or philanthropic pilot where the grant covers labour, supervision, safety, evaluation and reporting.

Do not assume savings will finance wages before savings are measured, realized, and legally available. Do not promise City procurement, EPWP placements, jobs, or partner access. Verify procurement route, budget holder, role classification, minimum wages, worker safety, insurance, qualifications, employment status and payment terms before field work starts.

Avoid reproducing gatekeeping or patronage in recruitment: publish eligibility criteria, selection rules, available paid roles and grievance route; record selection decisions; prohibit placement fees and retaliation. Ensure the programme does not substitute unsafe, unsupervised or lower-paid temporary work for qualified work.

### Job-quality measures

Record separately: funded posts; people engaged; paid hours; full-time equivalents using a declared method; gross/net pay as appropriate and lawfully collected; payment delay; training completed; recognized credentials; safety incidents; transitions to continuing employment or enterprise revenue; and 6-/12-month retention. A short placement must not be counted as a durable job without qualification.

## 6. Making infrastructure crime less profitable

The initial threat model should focus on two bounded mechanisms: theft of public infrastructure for resale, and fraudulent payment for work not delivered.

### Infrastructure theft / illegal resale

- Keep non-public asset identity and work evidence available to authorized operators.
- Record removed/replaced components and materials custody where legally and operationally appropriate.
- Preserve traceable handoff and purchase records for authorized investigation.
- Coordinate with competent authorities and compliant buyers; do not instruct residents or workers to confront suspects.
- Do not publish exploitable locations, response windows, security gaps or precise live asset status.

### False work / procurement leakage

- Bind work evidence to an authorized work order, contract version, scope and responsible role.
- Preserve approval, inspection, variation and payment events as distinct records.
- Flag duplicate invoice identifiers, impossible quantities, missing acceptance evidence and unexplained variation for human review.
- Treat a flag as a lead, not a finding of fraud; retain explanations, correction and appeal paths.
- No automatic blacklisting, prosecution referral, adverse employment decision or accusation by a model.

The Second-Hand Goods Act is a relevant legal starting point for the stolen-goods trade boundary: https://www.gov.za/documents/second-hand-goods-act. Local legal/procurement review is still required. No claim is made that the proposed record model alone deters theft or proves criminal intent.

## 7. Evaluation design

First collect a usable baseline with consistent definitions. Where the partner permits, compare the pilot zone against its own pre-period and a similar non-pilot zone or staggered rollout. Record weather, planned outages, major projects, staffing, spare-part availability, demand spikes and changed reporting channels as confounders. Do not attribute a citywide change to a small pilot.

### Operational outcomes

- Report-to-triage and triage-to-authorized-inspection times.
- Inspection-to-repair and repair-claim-to-independent-verification times.
- Repeat fault rate per defined asset or service zone.
- Backlog age and proportion with a known next action.
- Cost per independently verified valid resolution.
- Duplicate work, avoidable repeat trips and missing-evidence rates.
- Service availability or water-loss measure only when defined and supplied by the authorized operator.

### Workforce outcomes

- Total paid work and full-time-equivalent method.
- Actual payment timeliness and unpaid/contested hours.
- Safety events, training completion and credential attainment.
- 6-/12-month work continuity, repeat contracting and enterprise revenue where applicable.

### Trust, rights and integrity outcomes

- Number of decisions made from stale, conflicting or unauthorized inputs (target: zero).
- Unsupported automatic status changes or closure (target: zero).
- Sensitive-data disclosures and substantiated privacy complaints.
- False-positive workload and percentage of anomaly flags overturned.
- Worker/resident correction requests resolved within an agreed, published process.
- Distribution of service access and outcomes across areas, without using protected traits or poverty as proxies for risk.

Thresholds are intentionally **not invented here**. Agree them with the operational partner and independent evaluator after baseline collection and before looking at post-intervention results. Report denominators, missing data, uncertainty, and negative outcomes, not only aggregate wins.

## 8. Phase gates

| Gate | Deliverable | Required to pass |
|---|---|---|
| G0 — Desk research | Versioned source and ownership matrix | Publisher, observation date, effective date/version state, authority limit and missing data explicit |
| G1 — Synthetic | Deterministic corpus and fail-closed tests | Duplicates, stale/conflicting sources, false completion, replay and missing source all produce expected outcomes |
| G2 — Partner readiness | Named operational owner, payer, approval and data-use terms | Written permission, security/privacy assessment, worker-safety plan, baseline availability and manual fallback |
| G3 — Shadow operation | Human-reviewed recommendations beside existing workflow | Exact-head qualification, complete audit trail, no external writes, agreed safety and quality thresholds met |
| G4 — Controlled field trial | Authorized operator uses the pilot under a written protocol | Independent evaluation, funded paid roles, incident response and correction routes available |
| G5 — Scale decision | Published evaluation and cost model | Demonstrable benefit without unacceptable privacy, service, labour or fairness harm; recurring payer identified |

Any unauthorized data access, material privacy incident, unsafe work, unsupported production SLA claim or inability to reconstruct a decision stops progression until resolved.

## 9. Architecture fit with Mycelix

- Reuse the existing Commons Support operational family after its identity, mutability, transition, indexing and authority boundaries are repaired and qualified; do not create a second general ticket core by default.
- Keep CIV-RES service-issue identity separate from a City/utility reference number.
- Keep observations, source claims, operational work, external statuses and independent verification as separate linked types.
- Reuse CIV-RES source/version/currentness qualifications; do not infer currentness from successful retrieval alone.
- Treat Symthaea forecasts and priority recommendations as advisory, with disclosed uncertainty and no authority to dispatch, pay, close or accuse.
- Make adapters read-only by default and capability-gated when permissions exist.
- Keep personal data off public ledgers by default; publish aggregate outcomes only where privacy and re-identification risk are reviewed.
- Run on synthetic and manually authorized data until G2; an architecture draft is not a production integration.

## 10. Go/no-go questions before field work

- Who owns and operates the workflow today?
- Who can authorize the proposed data and the exact permitted uses?
- Who pays each role, on what contractual basis and within what payment period?
- What is the source of ground truth for repair completion?
- How will safety, privacy, complaints, corrections and independent review work?
- Is the workload problem a data/orchestration gap, or principally a staffing, spare-parts, procurement, energy or infrastructure-capacity constraint that software cannot solve?
- What baseline and comparison make a benefit attributable rather than anecdotal?

If these questions cannot be answered, remain in synthetic/shadow research. Do not turn a missing partner, missing data, or missing funding into a fictitious implementation status.

## References

1. Johannesburg Water, 23 July 2026, smart technology and water-loss programme: https://www.johannesburgwater.co.za/blog/2026/07/23/johannesburg-water-accelerates-smart-technology-rollout-to-reduce-water-losses/
2. City of Johannesburg, 2026/27 SDBIP, IDP and service-level-standards index: https://joburg.org.za/documents_/Pages/Key%20Documents/SDBIP-IDP-and-Budget/SDBIP-IDP-And-Budget.aspx
3. Johannesburg Water published customer service standards: https://www.johannesburgwater.co.za/services/
4. Stats SA, Victims of Crime 2025/26 key findings: https://www.statssa.gov.za/?PPN=P0341&SCH=74591&page_id=1856
5. Department of Public Works and Infrastructure, 2025/26 budget speech: https://www.gov.za/news/speeches/minister-dean-macpherson-public-works-and-infrastructure-dept-budget-vote-202526-09
6. South African Government, Second-Hand Goods Act: https://www.gov.za/documents/second-hand-goods-act
7. Mycelix CIV-RES ownership/adapter census: https://github.com/Luminous-Dynamics/mycelix/issues/2028
8. Mycelix service-standard semantics: https://github.com/Luminous-Dynamics/mycelix/pull/5008
