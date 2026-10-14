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

Water is the best initial candidate because Johannesburg Water has publicly described active investment in pressure management and acoustic leak detection. Its 23 July 2026 statement said that a further 306 smart pressure controllers were being installed, targeting completion by the end of September 2026; it also reported 225 noise loggers installed and another 125 planned by October. I also checked its 6 October 2026 system-update notice, which lists operational notices but does not explicitly confirm the installation programme's final counts. That missing confirmation in one notice is not proof the rollout is incomplete; completion remains independently unverified in this research record. No pilot may assume sensor data is accessible to Mycelix or that rollout is complete.

The software opportunity is the operational bridge between observation, authorized inspection, work coordination, evidence, repair, and feedback—not duplication of those sensors.

## 2. Evidence register and limits

| ID | Source | What it supports | What it does not establish |
|---|---|---|---|
| JW-1 | [Johannesburg Water smart-technology release, 23 July 2026](https://www.johannesburgwater.co.za/blog/2026/07/23/johannesburg-water-accelerates-smart-technology-rollout-to-reduce-water-losses/) | Utility-reported pressure-controller and leak-logger programme; prospective maintenance workflow | Current installation completion, telemetry access, exact pilot zone, measurable pilot effect, or Mycelix authority |
| JW-3 | [Johannesburg Water system updates, 6 October 2026](https://www.johannesburgwater.co.za/blog/2026/10/06/system-updates-6-october-2026/) | A later official notice page was checked; its extracted contents are operational interruption notices | The specific notice does not confirm smart-controller/noise-logger completion. No observed confirmation is not evidence of non-completion. |
| COJ-1 | [City SDBIP, IDP and Service-Level Standards document index](https://joburg.org.za/documents_/Pages/Key%20Documents/SDBIP-IDP-and-Budget/SDBIP-IDP-And-Budget.aspx) | Official index lists the 2026/27 SDBIP, final IDP and 2025/26 service-level standards charter; a subsequent retry retrieved the linked SDBIP PDF | An index or annual indicator is not a case-level service SLA, current provider status, or permission to access City systems |
| COJ-SDBIP-2026-27 | [City of Johannesburg 2026/27 SDBIP PDF](https://joburg.org.za/documents_/Documents/2026-27_SDBIP.pdf), KPI 17 pages 28/58; KPI 15 page 27; KPI 24 page 32; KPI 55 page 47; output KPI 52/WS5.2 page 90 | Water-loss unit/denominator ambiguity plus a second same-title water-loss target series; local co-op service-contract intervention; City KPI 24 target of 12,500 EPWP opportunities versus national target apportioned to COJ of 9,996; KPI 55 target of six green-economy projects | Keep KPI 17 and KPI 52/WS5.2 distinct until definition, denominator, period and target mapping are confirmed. KPI 15 is not a contract award; KPI 55 counts projects, not jobs. Pages 27, 32–33, 47 and 90 are text-layer-only/not visually reviewed in this pass. |
| JW-2 | [Johannesburg Water service standards](https://www.johannesburgwater.co.za/services/) and [PR #5008](https://github.com/Luminous-Dynamics/mycelix/pull/5008) | Published target language needs distinct action, subject, clock and version semantics | Effective date/version of the web table, adopted production threshold, exact clock start/exclusions, or case-specific status |
| SA-1 | [Stats SA, Victims of Crime 2025/26 key findings](https://www.statssa.gov.za/?PPN=P0341&SCH=74591&page_id=1856) | National victimization estimates show that reported crime is incomplete: 37.3% of people who experienced personal-property theft, 41.7% of people who experienced street robbery, and 48.2% of households experiencing housebreaking reported some or all incidents | Johannesburg-specific incidence or a causal estimate of any proposed intervention |
| JOBS-1 | [Department of Public Works and Infrastructure, 2025/26 budget speech](https://www.gov.za/news/speeches/minister-dean-macpherson-public-works-and-infrastructure-dept-budget-vote-202526-09) | Government's stated EPWP reform direction includes tackling gatekeeping/exploitation and linking participants to skills, permanent work or entrepreneurship | Eligibility, available funding, local slots, or procurement approval for this pilot |
| SA-JOBS-2 | [Stats SA, South Africa’s Youth and the Labour Market in Q1 2026](https://www.statssa.gov.za/?p=19526) | National Q1 2026 unemployment was 60.9% for ages 15–24 and 40.6% for ages 25–34; NEET rates were 37.6% and 45.6% for ages 15–24 and 15–34 respectively | These are national statistics, not Johannesburg estimates, recruitment criteria, funded posts, or a forecast of pilot impact |
| SA-JOBS-3 | [DPWI, Working on Infrastructure pilot launch, 4 May 2026](https://www.gov.za/news/speeches/minister-dean-macpherson-launch-working-infrastructure-pilot-programme-04-may-2026) | Official speech describes an 8–10 month structured pilot with training, mentoring and employment transition; its first launch is in KwaZulu-Natal and Limpopo and recruitment is through SAYouth supported by Harambee | Gauteng/Joburg inclusion, local slots and funding are not confirmed. This is a design/partner-discovery lead only; Mycelix must not duplicate SAYouth or store unnecessary applicant identity data |

Source-currentness rule: retain issuer, exact URL, retrieval timestamp, visible publication/effective dates, observation status, and an immutable digest where bytes can lawfully be captured. A fetch failure is a missing observation, not evidence that a target or programme does not exist.

## 1A. Rollout currentness check

The July schedule's planned completion date has passed, so the pilot must not treat that date as an installation receipt. The [6 October 2026 system-update notice](https://www.johannesburgwater.co.za/blog/2026/10/06/system-updates-6-october-2026/) was checked on 14 October. Its extracted content describes operational notices, and no explicit rollout completion confirmation was observed in that specific notice. This is bounded negative evidence only: it neither proves completion nor proves non-completion. The software should retain the original plan date, the later source checked, its scope, and an `NotObserved` completion state until the responsible utility publishes or directly confirms the final count.

## 2A. What the current-year SDBIP adds

A retry retrieved the City's official [2026/27 SDBIP PDF](https://joburg.org.za/documents_/Documents/2026-27_SDBIP.pdf) (151 pages). I visually reviewed the summary table on printed page 28 and the technical indicator definition on printed page 58. This supersedes the earlier retrieval-timeout note: a timeout was temporary, not evidence the document was absent.

Three useful but importantly different planning signals appear in the document:

1. **Water-loss KPI 17.** The summary table gives the 2026/27 target as `200 293Ml`, with quarterly values `204 270Ml`, `203 054Ml`, `201 839Ml`, and `200 293Ml`. The technical definition separately says losses are expressed as annual volume lost per service connection per day and gives a formula dividing by (365 \times\) number of service connections. The summary's `Ml` label and the technical definition/formula appear semantically inconsistent. I have recorded both verbatim instead of silently reconciling them. **No derived value, unit conversion, production calculation or SLA claim is authorized from this record** until the responsible metric owner clarifies it.
2. **Green-economy KPI 55.** The SDBIP text layer reports a 2026/27 target of six green-economy projects with measurable job creation, with quarterly targets 1, 1, 2 and 2. It measures *projects*, not jobs, paid placements, or durable employment. The page-47 table could not be visually reviewed because the screenshot request timed out, so this remains a text-layer-only observation.
3. **EPWP Phase 5: distinguish two target series.** The SDBIP footnote states a nationally set five-year target of 50,030 work opportunities for the City across 2024/25–2028/29, with 9,996 apportioned to COJ for 2026/27. Separately, City's operational KPI 24 table lists 12,500 EPWP work opportunities for 2026/27, with quarterly targets 3,500, 7,000, 9,800 and 12,500; the footnote describes this as a 25% increment over the national target. Do not overwrite one target with the other. The pages containing these details were text-extracted but not visually reviewed. The document also warns that projects range from one day to 24 months and that not all provide an exit strategy or skills development. Neither target series is a count of durable jobs or funding for Mycelix.

4. **A second same-title water-loss series exists.** The separate Water and Sanitation outcome table on printed page 90 lists KPI 52 / Circular 88 reference WS5.2, also titled “Total water losses”, with baseline text `838.77` and a four-value sequence `196 099Ml`, `188 163Ml`, `180 274Ml`, `173 495Ml`. The extracted table header distinguishes the 2024/25 baseline, a three-year MTREF target and the 2029/30 end-of-term target, but the exact year-to-value mapping was not fully bound from the text layer and this page was not visually reviewed. The same title is not enough to prove this is identical to KPI 17. Record it as a separate series; do not infer a year mapping, average values, select one as canonical, or calculate a case deadline without an authoritative crosswalk.

This changes the pilot design: it should be able to **crosswalk to the City's existing outcome indicators without claiming equivalence**. Keep three measures distinct: water-loss indicator result, number of green-economy projects, and number/quality/retention of paid jobs. A service-coordination prototype might support existing programmes, but must not invent ownership of these KPIs, infer local budget availability, or double-count EPWP work opportunities as lasting jobs. The source owner must resolve the water-loss unit/denominator ambiguity before any numerical comparison.

## 2B. Programme fit: do not assume the national pilot includes Joburg

DPWI's [4 May 2026 Working on Infrastructure pilot launch speech](https://www.gov.za/news/speeches/minister-dean-macpherson-launch-working-infrastructure-pilot-programme-04-may-2026) provides a useful design pattern: an 8–10 month structured opportunity, recruitment through SAYouth supported by Harambee, TVET/accredited training, mentoring, workplace learning, and transition support. The speech says the initial pilot begins in **KwaZulu-Natal and Limpopo**. It does not establish that Johannesburg/Gauteng is included, or that positions/funding are available here.

Therefore:
- Treat this programme as a *partner-discovery lead*, not a confirmed Joburg funding route. Before citing it as a local pathway, find a dated official expansion announcement or obtain written confirmation from the programme owner.
- Prefer connecting eligible participants through the existing authorized recruitment mechanism rather than building a duplicate public-sector applicant database. Store only the minimum consented evidence needed for this pilot, and do not put applicant identity, contact details, eligibility evidence or work histories on a public DHT.
- Reuse the useful programme design principles—published eligibility, transparent selection, no placement fees, documented training, mentorship, workplace evidence and a named exit pathway—but do not claim programme affiliation or eligibility from general policy text.
- For a Joburg pilot, first identify a City department/entity or an already-appointed contractor with a real maintenance work package, budget, supervisor and written permission. If no payer or authorized work package exists, remain in synthetic/shadow mode rather than describing candidate tasks as funded positions.
- Audit for gatekeeping without building an opaque scoring system: publish selection rules and complaint routes; log access/change history; keep a human-reviewed, privacy-protecting process for duplicate or inconsistent records; prohibit political-affiliation, poverty or neighborhood proxies; and track whether participants receive the wages, training and progression actually promised.

The intended improvement is not a second recruitment platform. It is a better link from an authorized real work package to transparent selection, paid work, skills evidence, and measurable transitions into continuing work.

## 2C. Compare water and waste-cooperative pilots before choosing a live path

The 2026/27 SDBIP reveals a second candidate worth investigating before treating the water pathway as the only jobs vehicle. On printed page 27, KPI 15 is the percentage of known informal settlements receiving basic removal services. Its intervention text includes a decentralized fleet/depot model and incorporating local cooperatives into basic removal services through new service-level contracts; the plan shows a 100% target. The page-27 table was only observed in the PDF text layer during this pass and was not visually reviewed.

This is a **procurement-discovery lead**, not proof a tender is open, a budget remains available, a cooperative has been awarded a contract, or the target is achieved. The water and Pikitup candidates should therefore be compared rather than silently selecting one:

| Decision dimension | Water-fault coordination | Pikitup basic-removal/cooperative services |
|---|---|---|
| Existing operational signal | Utility reports leak-detection and pressure-management work | SDBIP explicitly mentions local cooperatives in basic-removal service contracts |
| Primary prototype value | Preserve source identity, coordinate authorized inspections, reduce repeat handling, evidence verified repairs | Support transparent work-packaging, lawful route/collection records, service verification and material recovery |
| Potential paid roles | Field data quality, maintenance logistics, qualified trades, independent QA, integration engineering | Cooperative route operations, waste collection, sorting/reuse, logistics, field QA and enterprise management |
| Biggest dependency | Written utility/contractor access and a clarified water-loss metric | An actual authorized Pikitup work package, procurement route, payer and eligible operator |
| What is presently confirmed | Publicly documented utility initiative, not Mycelix access or a funded pilot | Publicly documented planning intervention, not an open/awarded contract or Mycelix funding |
| Ready for live deployment? | No | No |

Choose a live candidate only after identifying the responsible operator, written permissions, actual funded scope, lawful procurement/employment route, usable baseline and independent completion evidence. If neither passes that gate, stay in synthetic/shadow mode.

For the Pikitup option, the desired software must not displace existing municipal procurement, payment or safety controls. A future prototype could make work-package eligibility, route/task assignment, quantities collected, lawful material custody, quality checks, payment milestones and participant progression auditable. It must not publish individual worker identity, sensitive route schedules, high-value material pickup locations or unverified allegations of theft.

KPI 15 is a service-coverage target, KPI 55 is a green-economy project count, and employment quality is a third measure. Keep all three separate. For any work opportunity, track paid hours, actual on-time pay, safety, credentials, work-record quality and transition/retention at six and twelve months. Neither the word "cooperative" nor an indicator in a planning document establishes that a specific business has a right to work or funding.

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

### Paid-work integrity gate

The wage floor is a legal classification question, not a model optimization. The Department of Employment and Labour's 3 February 2026 announcement states that, from 1 March 2026, the ordinary national minimum wage is **R30.23 per hour**, while workers employed under the separate EPWP dispensation have a minimum of **R16.62 per hour**. These figures are source pins, not a decision about which regime covers a particular cooperative, contractor, learner or technical role. A qualified human must confirm the applicable arrangement and recheck the rule in force before an offer or contract; software must never pick EPWP because it is cheaper.

Before announcing a position, record the legal employer/contracting entity, written work scope, lawful programme/worker classification, wage basis and effective date, paid hours, pay period and date, funding committed for the full term, payroll/payment owner, and an accessible grievance/non-retaliation route. Budget the full cost—not only the headline hourly rate—including any applicable employer costs, supervision, tools and PPE, transport, training/credentials, safety and insurance, administration, and evaluation. Do not use unpaid “trials,” speculative savings, an unawarded grant or a work-opportunity count as substitutes for a funded job.

### Procurement discovery is not an opportunity guarantee

The City procurement hub links current bids, RFQs, canceled bids, awards and entity procurement. Pikitup's tender index, checked 14 October 2026, displays mixed lifecycle stages: the visible examples include a June 2026 cancellation and April/May 2026 awards alongside older advertised/closed notices. No tender-specific open cooperative/basic-removal work package was captured in the pages inspected for this pass. That bounded result is **not** proof that no opportunity exists and is not a complete procurement audit.

For every candidate opportunity, follow the index to the individual notice and capture the tender ID, issuer, latest amendments/Q&A, lifecycle status, closing date and timezone, scope/geography, cooperative/supplier eligibility, budget/funding authority, award or signed contract evidence, named payer and payment terms. An annual SDBIP target, a page labeled “Current Tenders,” or an older award is not a currently open, eligible or funded work package. Keep the Pikitup candidate at partner-discovery-only until authoritative notice and payer evidence exist.


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
9. South African Department of Employment and Labour, 3 February 2026 wage-rate announcement (effective 1 March 2026): https://www.labour.gov.za/Media-Desk/Media-Statements/Pages/Minister-of-Employment-and-Labour%2C-Meth-increases-the-statutory-National-Minimum-Wage-to-R30%2C23-per-hour.aspx
10. City of Johannesburg, Tenders and Quotations procurement hub (checked 14 October 2026): https://joburg.org.za/work_/Pages/Work%20in%20Joburg/Tenders%20and%20Quotations/Tenders-and-Quotations.aspx
11. Pikitup, Tenders index (checked 14 October 2026): https://pikitup.co.za/tenders/
