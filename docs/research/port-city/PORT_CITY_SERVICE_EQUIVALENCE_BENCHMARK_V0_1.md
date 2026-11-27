# Port-City Service-Equivalent Golden Trace and Benchmark v0.1

**Issue:** PORT-CITY-SERVICE-GOLDEN-TRACE-001 (#6343)  
**Snapshot:** 19 November 2026  
**Evidence class:** synthetic known-answer integration trace  
**Status:** cross-owner benchmark contract; no site or physical plant qualified.

## Why this is the next research milestone

The 2026 industry frontier is not a simple port microgrid and not a simple flexible data centre. Recent literature now includes:
- integrated berth, quay/yard crane, onshore power supply and multi-source dispatch;
- bidirectional logistics/energy event-triggered re-planning;
- stochastic seaport virtual power plants with ship shore-power, crane-derived and truck battery-swap demand;
- robust/distributionally robust port energy scheduling;
- grid-interactive data-centre design with flexible workloads, cooling, storage and grid-facing power conversion;
- compute heat coupled to seawater reverse osmosis.

This creates a useful distinction: **coordination breadth is no longer sufficient as a claim of novelty**. The harder problem is ensuring every subsystem is optimized against the same service obligations, physical envelope, time frontier and evidence authority.

This golden trace tests the composition between owners. It deliberately does not reimplement their domain engines.

## Paradigm shifts

### 1. Optimize useful service, not device utilization or one energy metric

The benchmark's outcome vector is:
- vessel/berth/shore-power service;
- terminal throughput and handling completion;
- compute useful work and on-time completion;
- cooling and thermal reserve;
- qualified water service (only when treatment and lab evidence exists);
- PCC/feeder peak, energy, ramps and resilience;
- cost and emissions;
- marine/ecological boundaries;
- solver invocations, evidence freshness and plan churn.

A candidate that lowers peak by dropping jobs or delaying vessels is not better. A candidate that produces more modeled heat without a qualified sink is not better. A candidate that reduces RO SEC but produces off-spec water is not better.

### 2. Co-optimize three flexibility domains and two evidence domains

Physical flexibility:
1. **Logistics:** arrival, berth window, crane work, shore power readiness.
2. **Compute:** objective-specific workload flexibility, deadlines, useful-work and rebound.
3. **Thermal/water:** cooling mode, thermal-store state, feedwater heat demand and sink availability.

Evidence flexibility:
4. **Time frontier:** source/observation/first-available/decision time, freshness and revision.
5. **Authority frontier:** who may propose, verify, reserve, authorize, physically act and reconcile.

A weighted score is not allowed to offset a hard breach or missing authority. Use typed non-compensatory constraints and Pareto reporting.

### 3. Treat the point of common coupling as a scarce shared resource

The workload scheduler, port terminal, shore power, RO plant, cooling plant, battery, heat pump and renewable systems cannot each spend the same headroom independently. All proposed schedules must be joined against the canonical PCC/feeder/connection identities and time-varying import envelope owned by Symtropy #4582.

The integration oracle must verify owner outputs and handoffs, not implement a second aggregate power-flow engine.

### 4. Turn heat into a conditional service, not a free by-product

Compute heat has value only when:
- source temperature/flow/availability are suitable;
- a real downstream sink has a matching time window and capacity;
- transfer and pumping losses are explicit;
- the delivered heat is metered/observed;
- the heat is not counted again as cooling discharge.

RO energy or water-output credit remains unavailable until the process/quality owners qualify the response. Thermal heat closure is not water product qualification.

### 5. Optimize for resilience and restoration, not only a normal day

Evaluate grid, fibre, cooling, water and common-cause failures; measure unmet service and time-to-recover. Distinct route IDs do not prove cable independence. Telecom repair data do not set power-cable failure probabilities. Local protection and fail-safe control remain independent of Symthaea, Mycelix, network access or optimization.

## Hand-computable integrated case

The companion file `PORT_CITY_SERVICE_EQUIVALENCE_GOLDEN_TRACE_V0_1.json` includes 48 expected dispositions plus a four-interval 15-minute synthetic trace.

### Inputs

The shared PCC import is:

`PCC = fixed port + vessel shore power + terminal + IT + cooling + RO electricity - wind/PV`

All powers are kW. The PCC cap is 8,200 kW in every interval.

| Local fixture slot | Fixed port | Vessel shore | Terminal | IT compute | Cooling | RO | Wind/PV | PCC import |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 09:00 | 800 | 2,000 | 1,000 | 3,000 | 600 | 700 | 900 | 7,200 |
| 09:15 | 800 | 2,000 | 1,500 | 3,000 | 600 | 700 | 1,200 | 7,400 |
| 09:30 | 800 | 0 | 800 | 3,000 | 600 | 700 | 800 | 5,100 |
| 09:45 | 800 | 0 | 500 | 3,000 | 600 | 700 | 800 | 4,800 |

The vessel's shore-power service is 1.0 MWh. Terminal service is 1,000 synthetic work units and 0.95 MWh. Compute service consists of two fixed job IDs, 100 total useful-work units and 3.0 MWh IT energy. Cooling energy is 0.60 MWh. RO electrical demand is 0.70 MWh; this does **not** establish qualified water volume or quality. Fixed port demand is 0.80 MWh. Renewable generation is 0.925 MWh.

### Three matched policies

| Policy | 09:00 | 09:15 | 09:30 | 09:45 | Peak | Import energy | Result |
|---|---:|---:|---:|---:|---:|---:|---|
| Baseline | 7.2 | 7.4 | 5.1 | 4.8 | 7.4 MW | 6.125 MWh | Within cap |
| Naive objective-only shift | 5.4 | 9.2 | 5.1 | 4.8 | 9.2 MW | 6.125 MWh | Fails by 1.0 MW |
| No-new-peak admitted | 7.2 | 5.6 | 6.9 | 4.8 | 7.2 MW | 6.125 MWh | Within cap |

The naive policy moves 1.5 MW of compute plus 0.3 MW cooling from 09:00 into an already busy 09:15 interval. It preserves total compute energy and useful work, but raises import to 9.2 MW and violates the 8.2 MW PCC limit by 1.0 MW. The 9.2 MW peak is 24.32% above the baseline peak.

The guarded policy moves the same 1.5 MW compute and its associated 0.3 MW cooling from 09:15 to 09:30. It preserves:
- the same 100 compute work units and both job IDs;
- 3.0 MWh IT energy and 0.60 MWh cooling energy;
- 1.0 MWh vessel shore power;
- 0.95 MWh terminal energy and 1,000 handling units;
- 0.80 MWh fixed port energy and 0.70 MWh RO electric service;
- 6.125 MWh total PCC energy.

Its peak is 7.2 MW, a 2.70% reduction from the 7.4 MW baseline. This is a synthetic arithmetic control, not a real operational savings result.

### Synthetic heat interface

At 09:15, each policy has a synthetic named RO-feed heat sink requiring 250 kW useful heat. At 80% direct heat-exchanger efficiency, 312.5 kW is diverted from the IT heat stream and 62.5 kW is allocated as recovery loss to seawater. The direct IT heat still rejected to seawater therefore depends on that policy's IT load:
- baseline (3,000 kW IT): 2,687.5 kW direct IT heat to seawater;
- naive objective-only shift (4,500 kW IT): 4,187.5 kW direct IT heat to seawater;
- no-new-peak admitted shift (1,500 kW IT): 1,187.5 kW direct IT heat to seawater.

Each partition closes independently. RO SEC change, qualified product-water volume and environmental discharge limit remain null. The trace makes no potable-water or marine compliance claim.

## 2026 comparator programme

| Comparator | What it demonstrates | What we must add |
|---|---|---|
| Li et al. BETM, June 2026 | Logistics + energy event-triggered re-optimization; case-specific cost and solver-time gains | Compute, cooling, water-sink and time-qualified evidence without sacrificing service |
| Zhang et al. seaport VPP, Sep 2026 | Heterogeneous ship/crane/truck load uncertainty and intraday correction | Couple workload deadline/rebound and cooling state; preserve source's forecast assumptions |
| Li & Hu integrated OPS, Aug 2026 | Berth, QC/YC, shore power and multi-source dispatch | Cross-owner verifiable handoff and exact shared PCC constraints |
| IGDT-DRO port scheduling, May 2026 | Robust/temporal uncertainty treatment | Out-of-sample trace replay, forecast-vintage discipline and measured decision regret |
| ENTSO-E data-centre system report, Apr 2026 | Different flexibility by workload class | Objective-specific work/service profiles and no-new-peak admission |
| ASHRAE grid-interactive AI data centre guidance | Workload, cooling, storage, inverter support and grid coordination | Port-city coupling plus independent grid/service evidence |
| Yang et al. compute–RO coupling, Apr 2026 | Temperature-dependent thermo-electric coupling; 17% waste-heat synergy within one case | Independent heat and water boundary closure; no free-heat or water-quality inference |

### Benchmark target ladder (hypotheses, not achievements)

- Zero PCC/feeder hard-boundary breaches in accepted candidate schedules.
- Zero missed hard triggers in the declared trigger corpus.
- 100% service-floor preservation for admitted vessel, terminal and compute obligations.
- No unreported increase in rebound peak, total kWh, carbon or water burden.
- At least 50% fewer re-plan invocations than fixed 15-minute scheduling, but only if hard-miss rate and service outcomes do not worsen.
- 5–10% whole-system lifecycle-cost improvement versus the strongest matched baseline on held-out operational traces, with paired uncertainty intervals excluding zero and no hard-gate regression.
- 100% deterministic historical replay for the same owner artifacts, digest and decision-time frontier.
- Quantified value of information for additional sensors, forecasts, survey data and rights evidence.

A result that does not meet these targets is still useful if the Pareto trade-off, failure mode and uncertainty are measured honestly. Never infer that a 2.70% synthetic peak reduction is comparable to published cost reductions.

## 48 initial golden-trace dispositions

The JSON fixture lists positive, negative, unknown and conflict cases including:
- baseline and service-equivalent no-new-peak positive controls;
- naive objective-only new peak and deferred-work rebound;
- missing shore-power readiness;
- independent W versus Wh violations;
- duplicate PCC profile and future renewable forecast;
- stale PCC meter and feeder derating;
- thermal-store reserve and simultaneous charge/discharge;
- cooling hard margin;
- unavailable/missing heat sink, absent delivery observation, unbalanced recovery and heat double-count;
- stale feedwater demand, unknown water quality, unknown brine/outfall rights;
- shared cable landing, telecom-vs-power cable evidence category and stale common-cause inventory;
- concurrent grid/fibre outage and restoration;
- hard-event deduplication, new hard reason, hysteresis re-arm;
- owner source revision, idempotent snapshot digest and changed-digest conflict;
- independent Symthaea candidate rejection and missing-owner output.

## South African demand-response and reserve product fit

The same physical load may be technically flexible but not qualified for a particular grid product. The local product profiles must be treated as separate contracts:

| Product | Public profile | Evidence still required |
|---|---|---|
| Eskom Distribution IDR | 10 MW minimum; full response within 6 seconds; 10-minute sustain; up to 3 events/day; special metering | participant contract, baseline, telemetry/metrology, current eligibility and aggregation authority |
| SAWEM Instantaneous Reserve | Market Code Rev 2.3 defines demand-side response within 10 seconds, sustained at least 10 minutes | exact Trading Unit registration, current regulatory/market implementation status, qualification and performance evidence |
| NTCSA Supplemental DR | 30 min–6 h notice; minimum greater of 500 kW or 15% average plant demand; 2 h response; 30-minute metering | participant agreement, baseline, meter, current programme availability and settlement |
| NRS 048-9 emergency reduction | emergency load reduction / restoration arrangements | System Operator or utility instruction; not a voluntary/contracted DR offer |

Sources: [Eskom IDR programme](https://www.eskom.co.za/distribution/instantaneous-demand-response-programme/); [NTCSA SAWEM Market Code Rev 2.3](https://www.ntcsa.co.za/wp-content/uploads/2026/02/Market-Code-v2.3.pdf); [NTCSA Supplemental DR](https://www.ntcsa.co.za/supplemental-demand-response-programme-2/); [NRS 048-9 Edition 3](https://www.kaya959.co.za/wp-content/uploads/2024/04/NERSA-Approved-Version-of-NRS-048-9-2023-Edition-3.pdf).

Eskom's six-second product and the SAWEM ten-second definition are not the same product. A 30-minute meter cannot establish a six-second response. Published Ngqura/Durban spare substation capacity is not a qualified demand-response resource. The paired-site programme therefore leaves actual participant eligibility, flexible MW, aggregation and meter status unknown until direct owner/operator evidence exists.

NERSA's September 2026 tracker listed Market Code approval as finalisation, with an end-October target. That dated snapshot does not establish the later final decision; confirm subsequent approval and implementation status before treating a reserve product as available.

The companion fixture now includes eight cross-owner product-fit cases: six-second vs ten-second profile mismatch, supplemental sustain miss, insufficient metering, missing aggregation authority, spare-headroom overclaim, and emergency-curtailment misclassification.

## Adjacent private industrial energy assets: Coega Steels

A private industrial renewable/storage project in the Coega Special Economic Zone has been added as an adjacent node, not assumed to be part of the port or data-centre electrical portfolio.

Fedgroup's 30 October 2026 announcement says a 7 MWp solar PV plant supplies Coega Steels' rolling mill and reports nearly 6 million kWh generated by that date. It describes a 50 MWh BESS as nearing commissioning. The rolling mill is reported at 150,000 tonnes/year. A further 19.8 MWp solar expansion is planned, but subsequent reporting says its funding arrangement had not been finalized.

Sources: [Fedgroup release via Bizcommunity, 30 Oct 2026](https://www.bizcommunity.com/PressOffice/PressRelease.aspx?ai=272996&cid=752422); [Engineering News, 30 Oct 2026](https://www.engineeringnews.co.za/article/fedgroup-funds-renewable-energy-infrastructure-for-coega-steels-new-rolling-mill-2026-10-30); [The National, 6 Nov 2026](https://thenational.co.za/news/2026-11-06-fedgroup-backs-coega-steels-energy-expansion-amid-rising-power-demand/).

The golden trace now includes two negative/unknown controls:
- geographic proximity is not a resource join: private PV/BESS cannot be added to TNPA/data-centre dispatch merely because it is in Coega;
- a BESS described as nearing commissioning receives no reserve/dispatch credit without commissioning, MW power rating, usable MWh, SOC, meter, operational availability and owner/dispatch rights.

The trace leaves its exact PCC, metering boundary, wheeling/export/aggregation rights, BESS commissioning and power rating unknown. A shared industrial node requires an exact electrical boundary, owner-authorized contract, current operating state and independent meter evidence. Neither private generation nor storage is treated as port-owned, firm, exportable or dispatchable.

## Ownership and oracle design

The cross-stack harness consumes owner-native artifacts:
- Symtropy #4582 validates the port schedule / shared PCC W/Wh/readiness;
- #6211 validates workload IDs, useful work, deadlines and rebound;
- #6229 validates cooling/heat partition and named-sink delivery accounting;
- #6257 decides whether evidence-qualified deviations request re-planning;
- #5355/#2152/#4958 validate water process, mass/utility/residual closure and product quality;
- #2683/#1916 validate thermal state and temporal heat grade;
- Sol-Atlas supplies geospatial, marine, water, habitat, cable, rights and site evidence;
- Symthaea proposes candidates, requests information and abstains under uncertainty;
- Mycelix composes authority, exact artifact/digest, ownership, provenance and reconciliation;
- Mycelix #5762 remains the shared/transitive common-cause dependency owner.

The integration oracle checks that no handoff upgrades an upstream claim into an unsupported downstream claim. It does not duplicate any owner's physics, process ledger, schedule engine, spatial source registry or authority system.

## Verification status

- The four-slot PCC arithmetic was independently calculated when the fixture was authored: baseline and guarded policy each use 6.125 MWh; naive policy also uses 6.125 MWh but breaches by 1,000 kW.
- The 48-case fixture is synthetic and expected-disposition driven. It has not yet been executed through all owner-native verifiers.
- The related Rust implementations have authored tests but have not been compiled in this environment because Cargo/rustc are unavailable.
- This document makes no CI-pass or physical-validation claim.

## Claim ceiling

This is a synthetic integration benchmark and cross-owner claim-composition contract. It is not a measured port, a South African PCC capacity study, a water-production result, a cable reliability model, marine plume simulation, grid-protection qualification, environmental permit, construction approval or operational dispatch authority.
