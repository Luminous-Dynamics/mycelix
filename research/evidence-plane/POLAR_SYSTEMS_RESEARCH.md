# Polar Infrastructure, Living Systems and Resource Evidence — Research Plan v0

**Status:** research and benchmark specification, not a certified engineering design, medical protocol, field validation or extraction recommendation.

This plan coordinates three new work items:
- [Symtropy #2726 — climate-resilient polar infrastructure digital twin](https://github.com/Luminous-Dynamics/symtropy/issues/2726)
- [Mycelix #5463 — voluntary, community-designed polar living routines](https://github.com/Luminous-Dynamics/mycelix/issues/5463)
- [Symthaea #8504 — evidence-audited Arctic hydrocarbon resource and trade-off atlas](https://github.com/Luminous-Dynamics/symthaea/issues/8504)

Use the shared [Evidence Plane contract](https://github.com/Luminous-Dynamics/mycelix/pull/5452) when it is mature; this plan does not claim that runtime adapters or field tools already exist.

## 1. Do not collapse three environments into one

- **Arctic land communities:** people live in settlements with local governments, Indigenous rights and knowledge, supply chains and infrastructure built on varied geology. Permafrost thaw, erosion, freeze/thaw, snow/wind loading, water/wastewater continuity, health access, and long supply lines can interact.
- **Antarctic stations:** remote research facilities on continental ice/rock, with severe resupply and evacuation constraints and a different legal/environmental context. Lessons transfer only after an explicit assumptions review.
- **Geographic North Pole:** the pole is over the Arctic Ocean and mobile sea ice, not a permanent grounded land site. A concept located there must address moving ice, drift, ocean, access/evacuation and platform/mobility hazards; it must not copy a land-foundation design.

For each target location define the design basis: measured and projected temperature ranges, wind and gust, snow drift and load, icing, solar/darkness cycle, precipitation/freezing rain, salt, freeze/thaw cycles, permafrost/ground movement or ice drift, access windows, resupply, and design life. Extreme minima alone are not a complete climatic design basis. Use applicable local codes, site investigation and licensed engineering review.

## 2. Infrastructure paradigm: graceful degradation and repairability

Reference engineering material:
- [UFC 3-130-05 Arctic and Subarctic Utilities (16 Jan 2025)](https://www.wbdg.org/FFC/DOD/UFC/ufc_3_130_05_2025.pdf) covers cold-region water supply, wastewater, fire protection, utility distribution and thermal calculations.
- [UFC Arctic/subarctic criteria index](https://legacy.wbdg.org/dod/ufc) includes site assessment/selection, foundations and building design.
- [Northwest Territories Good Building Practice for Northern Facilities, 4th edition](https://www.gov.nt.ca/en/newsroom/gnwt-releases-4th-edition-good-building-practice-northern-facilities-design-guide) incorporates climate change, geotechnical, environmental and hazardous-material concerns.
- The [Arctic Council EPPR programme](https://www.admin.arctic-council.org/about/working-groups/eppr/) identifies cold, darkness, remoteness and limited infrastructure as emergency-preparedness constraints.

Design requirements to investigate:
1. **Passive-first thermal survival:** envelope, air tightness, heat recovery, thermal zoning, moisture management, condensation and mould prevention, and critical-space survival during an outage before simply adding larger heaters.
2. **No single common-mode failure:** trace shared feeders, pipes, control systems, communications, snow-obstructed access and one-point spares dependencies. Two pumps behind one frozen power supply do not provide independent redundancy.
3. **Serviceability under real human conditions:** field-replaceable modules, accessible shutoffs, standardized fittings, clear diagnosis, glove-usable controls, low-tool repairs, spares sized by resupply delay, and safe operation when one subsystem is unavailable.
4. **Water and sanitation as critical services:** site-appropriate intake, freeze protection, storage, wastewater treatment/disposal, alarms, safe shutdown and recovery. Never treat water/wastewater design as a generic accessory to the energy model.
5. **Multi-vector energy resilience:** heat/electricity coupling; fuel backup; renewables according to local resource; batteries and thermal storage; demand response; cold-related battery power/capacity derating; wind icing; polar-night solar deficit; safe generator/fuel storage; and logistics/resupply.
6. **Climate-change adaptive ground design:** geotechnical investigation, foundation alternatives, monitoring of ground temperature/movement, drainage/erosion, and repair/relocation options. Do not assume frozen ground stays frozen for the infrastructure's service life.
7. **Recoverable operations:** backup communications, local/manual control, black-start and staged restart, offline procedures, and drills for delayed resupply, evacuation and extended outage.
8. **Commissioning and lifecycle:** design to an explicit maintenance regime, failure response plan, decommissioning plan and total life-cycle cost/emissions—not only design-day efficiency or first cost.

### Infrastructure tests

Define a site-appropriate envelope with a qualified engineer; no universal test temperature is proposed here. Test cold soak and start, thermal cycling, icing/snow obstruction, wind/load cases, humidity/condensation, battery power/capacity under cold conditions, generator/inverter transitions, islanding/black-start, water/wastewater freeze and recovery, sensor drift, offline control, communication loss, and cascading failures. Test component limits and full-system interactions; vendor component ratings do not prove system survival.

Minimum comparator set: existing/design incumbent; transparent rules; conventional LP/MILP/MPC or accepted engineering method; robust/stochastic optimization where supported; candidate design; ablations; deliberate fault cases.

### Proposed scorecard

Keep metrics separate:
- hours critical services are delivered / unserved, and severity;
- minimum indoor critical-zone temperature and time-to-unsafe conditions during outages;
- potable water and sanitation continuity;
- outage frequency/duration, black-start and recovery time;
- fuel and resupply mass/volume, lifetime cost and lifecycle emissions;
- worst-case and tail outcomes over frozen stress scenarios;
- diagnosis/repair time, special tools, spare parts and hands required;
- runtime, peak memory, sensor/model error and common-mode failure count;
- hard safety/health/environmental/rights constraints, with zero permitted regression for superiority.

A single composite score must not compensate for a life-safety, water safety, fire safety, or rights failure.

## 3. Habit design: supportive, adaptive, and co-designed

Research basis and limitation:
- [Nowozin et al., 2025, “Living in Biological Darkness II”](https://pmc.ncbi.nlm.nih.gov/articles/PMC11744753/) studied habitual winter daytime light and sleep in 11 healthy participants in Berlin, not a polar settlement. It is a source of hypotheses, not direct evidence of Arctic intervention effectiveness.
- [Arctic solstice light-environment research](https://pubmed.ncbi.nlm.nih.gov/39723449/) reports associations between seasonal light patterns, circadian phase and health-related markers; association is not proof that a specific intervention causes better health.
- The [Arctic Council emergency-preparedness work](https://www.admin.arctic-council.org/about/working-groups/eppr/) supports treating darkness, weather and remoteness as operational constraints.

Potential voluntary habit primitives to co-design:
- stable sleep/wake opportunities adapted to individual chronotype, shift patterns and work duties;
- access to appropriate daytime light when available and configurable evening lighting, while preserving safe work visibility, culture and personal choice;
- accessible indoor/outdoor movement during safe conditions;
- protected breaks, regular meals, hydration and social connection;
- buddy/check-in and weather/route/communication/PPE checks before outdoor tasks;
- handover and fatigue checks for safety-critical shifts;
- small maintenance routines for moisture, frost, filters, water, energy reserves and alarms;
- frequent low-burden drills with a learning-focused review rather than blame.

Do not prescribe clinical light therapy, supplements or medical treatments. Offer options and signpost qualified care where needed. Do not design habit support as employee surveillance or use routine adherence for punitive ranking. Co-design with residents, workers, local leadership and Indigenous communities where applicable. Participation and personal data sharing should be opt-in, revocable and privacy-preserving.

### Habit evaluation

Preregister a participatory pilot with a baseline period and a co-designed intervention; record season, role, shift schedule, workload, indoor conditions and other confounders. Use aggregate reporting and voluntary self-report by default. Evaluate:
- usefulness/acceptability and perceived autonomy;
- participation burden, withdrawals and accessibility across roles;
- self-reported sleep regularity/wellbeing only where voluntarily shared;
- operational checklist omissions and near misses, accounting for exposure and reporting changes;
- routine maintenance faults, time-to-diagnosis, and drill learning;
- privacy incidents and adverse or negative results.

Success is improved usefulness and operational reliability without coercion, privacy breach or inequitable burdens—not perfect adherence to one universal timetable.

## 4. Arctic hydrocarbon evidence: resource is not reserve

The [USGS Circum-Arctic Resource Appraisal (2008)](https://pubs.usgs.gov/fs/2008/3049/) reported mean estimates of 90 billion barrels of undiscovered conventional oil, 1,669 trillion cubic feet of natural gas and 44 billion barrels of natural gas liquids across 33 geological provinces north of the Arctic Circle; about 84% was expected offshore. The estimate is probabilistic, dates to 2008, covers the wider Arctic rather than the geographic North Pole, and is not a statement of proved reserves, economic reserves, current production or authorized projects. See the [USGS catalog record](https://www.usgs.gov/publications/circum-arctic-resource-appraisal-estimates-undiscovered-oil-and-gas-north-arctic).

The [UN Convention on the Law of the Sea, Articles 76–77](https://www.un.org/depts/los/convention_agreements/texts/unclos/part6.htm) distinguishes continental-shelf boundaries and coastal-state rights over seabed/subsoil resources. Specific geography and boundary status matter. In 2024, the United States and Canada announced a task force to negotiate the Beaufort Sea boundary, including overlap in the Central Arctic Ocean: [State Department statement](https://2021-2025.state.gov/joint-statement-on-creation-of-joint-task-force-to-negotiate-beaufort-sea-boundary/). Do not represent a generalized Arctic map as resolving jurisdiction.

Arctic oil-spill response has special constraints. The [Arctic Council](https://arctic-council.org/news/data-improve-oil-spill-response-in-arctic/) notes that cold water, sea ice, darkness, wind, remoteness and limited response infrastructure can make response operations difficult and impacts long-lasting. That increases the importance of prevention and credible response-capacity scenarios.

### What to build

An evidence-audited *resource and decision atlas*, not a “drill-here” optimizer:
- separate undiscovered-resource assessment, discovered accumulation, technically recoverable resource, proved reserves, production, project economics and legal authorization;
- map assessment units, source versions, probability/quantile semantics, spatial coverage, jurisdiction and unresolved boundaries;
- compare the no-new-development / declining-demand scenario, existing assets, proposed new development and feasible demand-reduction/renewable/efficiency alternatives;
- quantify uncertainties for commodity price, financing, infrastructure and ice/weather, methane leakage, lifecycle emissions, transport, decommissioning, remediation and demand;
- include spill-prevention and response readiness, ecosystem sensitivity, food/subsistence impacts, affected communities' rights and governance, and distributional costs as explicit constraints—not merely monetized penalties;
- retain uncertainty and missing data, and let independent reviewers challenge assumptions.

The EU's [20 October 2026 Arctic strategy](https://commission.europa.eu/news-and-media/news/eu-strategy-rapidly-changing-arctic-2026-10-20_en) frames the current regional agenda around people, security, climate, resilience, research and prosperity. This is a relevant current policy context, not a substitute for site-specific law or a reason to assume any single policy scenario.

### Resource-atlas benchmark

- exact transcription of official assessed figures, source citations, units and quantile semantics;
- known-wrong mutation tests: oil/gas unit swap, F95/F50/F5 swap, regional-to-pole spatial leakage, resource-to-reserve promotion, unsupported economic or legal conclusion;
- source and licensing completeness, source version/age and coverage gaps;
- uncertainty and sensitivity disclosure by assessment unit;
- independent-parser agreement and exact artifact hash verification;
- no ranking or single green/red decision score that conceals ecological, legal, rights, safety and climate constraints.

## 5. Evidence-plane and research governance

For all three threads:
- record exact commit, receipt/schema version, source byte hashes, transformations, parameters, units, CRS/time basis and output hashes;
- state data type (measured/reported/reanalysis/simulated/synthetic/inferred), scope and limitations;
- freeze benchmark protocol and holdouts before result inspection;
- match baseline information and compute/tuning budgets;
- report failures, uncertainty, tail outcomes and hard gates separately;
- never promote an S0 synthetic test into physical performance or an S1 table transcription into reproduced science;
- use qualified engineering, health, ecological and legal review wherever the use case requires it.

This specification is a starting point for experiments and requirements review. It is not a completed engineering design, health intervention validation, hydrocarbon reserve assessment or approval for deployment.
