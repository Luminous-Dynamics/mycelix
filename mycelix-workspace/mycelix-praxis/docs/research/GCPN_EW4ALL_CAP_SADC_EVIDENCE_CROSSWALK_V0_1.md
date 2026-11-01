# GCPN ↔ EW4All / CAP / SADC MHEWS Evidence Crosswalk v0.1

**Status:** source-grounded architecture and evaluation crosswalk; research only.  
**Date:** 1 November 2026.  
**Parent:** issue #5641.  
**Related integrated baseline:** GCPN evaluation v0.6 (eight synthetic evidence modules).

## 1. Executive boundary

GCPN should complement the existing civilian early-warning ecosystem. It is not a new warning authority, national MHEWS, regional command centre, forecasting agency, mobile network, emergency-response organization or replacement for the Common Alerting Protocol (CAP).

ITU adopted CAP as Recommendation X.1303. CAP provides a common all-hazard alert format so consistent messages can be exchanged over different information and communications networks. That is an interoperability foundation; it does not, by itself, prove that a recipient received a usable representation, understood the warning, interpreted uncertainty correctly or carried out a protective action.

GCPN's proposed contribution is a federated, auditable evidence layer around existing authority-owned systems: provenance and conformance, continuity and delivery denominators, human usability, action-chain evidence, forecast verification, sampling uncertainty and lifecycle cost. Its current examples remain synthetic and non-operational.

## 2. Why the ecosystem alignment matters now

The international and regional agenda is already explicit:

- WMO/UNDRR's Global Status of MHEWS 2025 reports 119 countries (60%) reporting an MHEWS. It notes progress, but persistent gaps remain, especially in Africa. Africa's comprehensiveness improved by 72% since 2015, while remaining the lowest-scoring region. This is a reason to prioritize sustainable local/national capacity, not to invent a parallel authority.
- SADC's 2026 capacity-strengthening programme is designed to strengthen national MHEWS and SHOC, improve information exchange between national systems, SHOC and AMHEWAS, emphasize CAP/open standards, co-produce understandable and actionable alerts with end users, and promote responsible AI/ML with data governance, transparency and human oversight.
- SADC's related national-roadmap consultancy (issued 22 October 2026; closing 6 November 2026) seeks nationally owned, technically robust, investment-ready five-year MHEWS roadmaps across all four EW4All pillars, including consultation, capacity assessment, interoperability and investment priorities.
- WMO's 29 October 2026 analysis of 35 EW4All roadmaps shows how national priorities are translated into costed, time-bound implementation. Among 20 roadmaps with clear budget allocations, the identified total was approximately USD 1.38 billion (about USD 69 million per country on average). This is not a universal price tag or a procurement estimate for GCPN; it demonstrates why local scope, institutional capacity and lifecycle costs matter.

These sources support a national-ownership and ecosystem-integration strategy. They do not validate GCPN's synthetic performance.

## 3. Four-pillar crosswalk

| EW4All pillar | Existing mandate / ecosystem role | Relevant GCPN evidence package | What GCPN does not provide; evidence gap |
|---|---|---|---|
| 1. Disaster risk knowledge | National risk governance, risk knowledge, exposure/vulnerability assessments and local knowledge remain with competent national and local institutions. | Civil record provenance can preserve source/authority metadata; topology and cost modules can test evidence continuity and resource assumptions. | No authoritative hazard maps, exposure models, vulnerability assessment, population registry or national risk assessment. Those must be supplied and governed by the responsible institutions. |
| 2. Detection, observations, monitoring, analysis and forecasting | National meteorological, hydrological, geological, health and other competent services produce authoritative observations and forecasts. | Forecast calibration v0.2 evaluates synthetic probabilities, proper scores, Murphy decomposition, subgroup failures and drift. Forecast uncertainty v0.1 evaluates paired score differences. | GCPN does not operate sensors, observe hazards, generate forecasts, issue model products or determine hazard thresholds. Real performance requires permissioned data, verified outcomes and valid dependence clusters. |
| 3. Warning dissemination and communication | National warning authorities approve authoritative messages; CAP and network/channel providers support consistent exchange and dissemination. | Civil conformance tests authority binding; last-mile tests unique/accessibly-reached denominators; human outcomes test comprehension, action-intent identification, uncertainty and false certainty. | No live CAP gateway, mobile-broadcast service, telecom distribution infrastructure, language service or proof of population coverage. Real recipients and field outcomes are not measured in synthetic fixtures. |
| 4. Preparedness and response capabilities | National and local emergency-management authorities, services and communities own preparedness plans, response procedures and action. | Warning-to-action benchmark audits synthetic authority-decision, conditional finance, action-start, service outcome and grievance evidence. Human outcomes only test understanding of the action stated in the message. | No emergency dispatch, responder direction, evacuation order, fund release, resource allocation, protective-behaviour measurement or causal impact evaluation. Those remain separate, authority-led functions. |

**Cross-cutting capabilities:** topology resilience, lifecycle cost and the evidence-integrity approach can support governance, coordination, continuity, monitoring and evaluation. They cannot replace institutional capacity, training, peer learning, locally led planning or funded maintenance.

## 4. Responsibility and data-flow boundaries

| Actor / standard | Responsibility retained | Safe GCPN relationship |
|---|---|---|
| National warning authority | Owns warning authority, approval, thresholds, message content and national public-warning procedures. | Supplies signed/traceable authority assertions for offline conformance; GCPN never invents or substitutes authority. |
| National technical services | Own observations, validated forecasts, event definitions and authoritative technical advice. | Supplies forecast/outcome evidence for verification; GCPN reports metrics and uncertainty without issuing forecasts. |
| SADC SHOC | Regional humanitarian/emergency coordination and harmonized information exchange under its mandate. | May exchange provenance-bearing evidence if invited and authorized; no GCPN regional command role. |
| AMHEWAS | African multi-hazard early warning and early-action coordination within its mandate. | Interoperability target only; national and continental governance remains with its owners. |
| CAP / ITU-T X.1303 | Common structured format for exchanging all-hazard alerts over ICT networks. | GCPN can validate conformance and add auditable evidence fields/sidecars without forking CAP semantics or claiming CAP proves comprehension. |
| Local government, community and civil-society partners | Local context, preparedness, last-mile feedback, accessibility co-design and grievance pathways. | Co-design and independently evaluate usability; no recipient profiling or unconsented personal-data capture. |

A safe architecture is federated: evidence may be exchanged according to national permission and information-sharing agreements, while the authority and source institution remain identifiable. No global command authority or single universal operational database is proposed.

## 5. Mapping the eight GCPN modules to bounded evidence roles

| GCPN module | Evidence role | Primary independent limitation |
|---|---|---|
| Civil conformance | Validate record schema, authority assertion, chronology, uncertainty-preserving fields and negative controls. | Schema correctness does not prove the originating authority or physical event is true. |
| Topology resilience | Compare continuity and failure behavior across centralized, regional and local-first synthetic patterns. | No real network availability, carrier performance or national topology validated. |
| Last mile | Preserve intended, unique, duplicate, accessible and uncovered receiver counts under synthetic channel failures. | A receipt or accessible rendering does not prove comprehension or action. |
| Warning-to-action | Check authorization, decision chronology, conditional finance, action start, evidence of completion and grievance accounting. | Synthetic chain evidence does not prove real protective behavior or causal harm reduction. |
| Forecast calibration | Score supplied probabilities against supplied outcomes; report Brier/log loss, calibration, resolution, uncertainty, sharpness, strata and drift. | Does not produce forecasts; small synthetic event samples are not operational evidence. |
| Lifecycle cost | Account for capital, setup, staff, maintenance, replacement and scenario restoration against consistent reach denominators. | Cost units are synthetic; local prices, staffing rates, procurement and budgets must be independently obtained. |
| Human outcomes | Measure accessible representation, comprehension, stated-action identification, uncertainty understanding and false certainty per intended trial and per accessible trial. | Synthetic task vectors are not human-subject evidence, actual action or population outcomes. |
| Forecast uncertainty | Paired resampling of matched model losses and reporting intervals instead of point estimates alone. | The current fixture uses event_id as an assumed unit; real meteorological dependence clusters have not been validated. |

The integration predicate is a strict conjunction of independent module fixture oracles. It is deliberately not a weighted aggregate. A good score in one layer must not compensate for a critical failure in another.

## 6. What existing standards already solve—and what they do not

**CAP interoperability:** CAP standardizes the structure and exchange of alerts. GCPN should consume/validate CAP-compatible records and measure downstream evidence; it should not create a competing message protocol.

**National and regional coordination:** SADC is explicitly strengthening national MHEWS, SHOC and AMHEWAS linkages. GCPN should support agreed evidence exchange and local capacity transfer, not duplicate SHOC or continental coordination.

**People-centred warning:** SADC calls for co-production of understandable, trusted and actionable products with end users. GCPN's human-outcomes benchmark is a candidate measurement layer for later co-designed evaluation. It is not a substitute for community participation or human-subject research.

**Forecast science:** ECMWF and WMO use proper scoring and verification frameworks; ECMWF's work also emphasizes separating reliability/calibration from intrinsic information/resolution. GCPN should compare against appropriate climatology, persistence/lead references and operational baselines on matched authoritative data. A synthetic Brier score or 12-event bootstrap is not a victory over operational NWP.

**Cost and sustainability:** WMO's roadmap analysis and SADC national-roadmap work emphasize costed, time-bound, nationally owned implementation. GCPN's lifecycle-cost code is only an accounting harness until populated with locally measured costs, staffing, service obligations and maintenance assumptions.

## 7. Field-validation sequence

No operational pilot should be attempted on the strength of these synthetic benchmarks alone. A defensible progression is:

0. **Authority and governance gate.** Obtain documented permission from the national authority and relevant institutions. Agree who owns messages, observations, data, retention, security, incident response and stop authority.
1. **Synthetic conformance.** Reproduce source hashes, parser negative controls, chronology, authority fields, failure scenarios, denominator accounting and strict module gates offline.
2. **Authorized historical replay.** Replay permissioned historical CAP messages and suitably de-identified delivery logs in a sandbox. Keep source authority and original messages intact. No live dispatch.
3. **Community co-design.** Work with local-language communities, disability organizations, local broadcasters and user representatives to design accessible message variants and comprehension tasks. Obtain independent ethics review before any human-subject study.
4. **Controlled continuity exercise.** Test pre-agreed degraded-connectivity scenarios in a closed exercise that cannot issue public warnings or direct responders.
5. **Prospective shadow evaluation.** Only with formal authority approval, compare evidence in shadow mode. The existing authority remains the sole issuer and decision-maker; GCPN does not change thresholds or trigger action.
6. **National roadmap alignment.** Use independently measured infrastructure, staff, maintenance, replacement and training costs to inform the relevant national investment plan. Do not substitute synthetic cost units for a national budget.

Stop or do not advance if authority provenance cannot be established, recipient data cannot be governed, accessibility cannot be evaluated without intrusive profiling, a component hides a denominator, independent outcomes are unavailable, or a test could inadvertently dispatch a public warning.

## 8. Proposed scorecard—research hypotheses, not official standards

| Dimension | Measurement | Proposed research target / gate |
|---|---|---|
| Authority integrity | Valid authority assertions; unsigned/unbound record controls | No authority-binding failure in a declared conformance corpus; national authority defines acceptable evidence. |
| Interoperability | CAP parse/round-trip semantic consistency; cross-system provenance | No semantic loss in agreed test corpus; no change to authority, time, urgency, certainty or area semantics. |
| Continuity | Unique coverage under named partition/failure scenarios | Publish every failure case and uncovered denominator; no aggregate availability claim hides a failed region. |
| Accessibility | Accessible-format availability for each co-designed interaction stratum | ≥95% is an aspirational field-study target, not a current standard. |
| Human comprehension | Correct core-message responses per intended trial | ≥90% is an aspirational target; also report conditional rate among accessible trials. |
| Uncertainty comprehension | Correct reading of probabilistic/conditional language and false certainty | ≤5% false certainty is a hypothesis; it must be validated with communities and human-factors researchers. |
| Equity / profile gap | Worst-profile effective comprehension and gap to best profile | ≤10 percentage-point gap is a research target; never hide the worst profile in an average. |
| Forecast quality | Proper scores, reliability, resolution, sharpness, subgroup/drift diagnostics | Compare against climatology, lead reference and competent operational baselines on held-out authoritative outcomes. |
| Statistical uncertainty | Empirical coverage of nominal intervals under known synthetic DGPs and valid event clusters | For a nominal 95% interval, proposed qualification band 93–97%, with Monte Carlo uncertainty and adequate independent clusters reported. |
| Sustainability | Local currency cost per accessible unique recipient, staff hours, maintenance and replacement | No fixed universal cost target; nationally owned service scope and local rates required. |

These targets are research hypotheses and must not be represented as WMO, ITU, UNDRR or SADC standards.

## 9. How to make GCPN materially better without claiming to replace SOTA

1. **Conformance is necessary, not sufficient.** Pass CAP/schema and signature tests, then independently test authority provenance, chronology, semantic preservation, last-mile reach and human comprehension.
2. **Preserve denominator lineage.** Track intended → attempted → uniquely reached → accessible → comprehended → stated-action understood. Measure actual behavior and impact only in a separately approved study. Never infer a later stage from an earlier one.
3. **Expose worst-case strata.** Report subgroup and failure-case results even when the aggregate improves. No weighted score may conceal an inaccessible channel, forecast subgroup regression or unauthorized action.
4. **Quantify uncertainty honestly.** Use matched baselines and dependence-aware resampling. With few independent event clusters, report wide intervals and withhold ranking.
5. **Design for degraded operation, not just ideal uptime.** Test partitioned and local continuity while preserving authority and provenance. A locally available message must not become an unauthorized message.
6. **Make maintenance and people first-class costs.** Include training, handover, staff time, replacement, community co-design and long-term support—not just initial software or capital cost.
7. **Keep AI subordinate to accountable institutions.** Any future AI use requires transparent limitations, data governance, evaluation under shift, human oversight and an explicit non-AI fallback. The current GCPN benchmarks do not use AI to generate warnings.

A credible claim of superiority requires pre-registered comparisons, representative permissioned data, independent observations, validated dependence clusters, credible baselines, enough sample size, uncertainty intervals, adverse subgroup reporting and independent reproduction. Until then, the claim is narrower: GCPN is building an auditable evaluation architecture, not a superior operational MHEWS.

## 10. Governance and data minimization

- National warning authorities retain message and action authority.
- Data sovereignty, sharing purpose, retention, access and audit responsibilities are agreed before any non-synthetic data exchange.
- Prototype evaluation avoids person-level records and recipient profiling. Use only the minimum aggregated counts needed for evaluation.
- Suppress small cells where disclosure risk exists. Do not infer protected traits or disability labels from communication behavior.
- Community organizations participate in co-design and grievance/appeal processes.
- Independent evaluators should be able to reproduce results from source-controlled fixtures and authorized evidence without gaining unnecessary access to personal data.
- NRVDEF remains a separate research programme with different legal, physical-verification and treaty boundaries.

## 11. Sources

1. SADC, *Consultancy to undertake capacity building arising from identified MHEWS gaps and needs from Member States* (2026): https://sadc.int/procurement-opportunities/consultancy-undertake-capacity-building-arising-identified-mhews-gaps-and
2. SADC, *Individual Consultancy for the Development of National MHEWS Roadmaps* (issued 22 October 2026; closing 6 November 2026): https://sadc.int/procurement-opportunities/individual-consultancy-development-national-multi-hazard-early-warning
3. WMO/UNDRR, *Global Status of Multi-Hazard Early Warning Systems 2025* (12 November 2025): https://wmo.int/resources/publication-series/global-status-multi-hazard-early-warning-systems/global-status-multi-hazard-early-warning-systems-2025
4. WMO, *From Global Ambition into National Action: Take Aways from 35 EW4All Roadmaps* (29 October 2026): https://wmo.int/media/update/from-global-ambition-national-action-take-aways-from-35-ew4all-roadmaps
5. ITU, *Common Alerting Protocol and call to action* (CAP adopted as ITU-T X.1303): https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Pages/Common-Alerting-Protocol-and-Call-to-Action.aspx
6. ECMWF, *Forecast User Guide: Statistical Concepts — Probabilistic Data* (updated May 2026): https://confluence.ecmwf.int/spaces/FUG/pages/673551875/Section%2B12.B%2BStatistical%2BConcepts%2B-%2BProbabilistic%2BData
7. Bonavita & Geer, *Forecast verification using information and noise* (ECMWF Technical Memorandum 927, May 2025): https://www.ecmwf.int/en/elibrary/81662-forecast-verification-using-information-and-noise

## 12. Non-claims

This crosswalk is not an endorsement by SADC, WMO, UNDRR, ITU, SHOC, AMHEWAS, a national authority or any service provider. It is not a tender response, national roadmap or funding application. It does not create alerts, forecast hazards, authorize decisions, release funds, direct responders, establish real population coverage, prove lives saved or certify operational readiness. All GCPN values cited from v0.6 are synthetic test-vector results.
