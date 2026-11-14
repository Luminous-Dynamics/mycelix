# TRACE-Water, restoration planning and FSRE benchmark crosswalk v0.1

**Date:** 2026-11-14  
**Status:** Public-source research synthesis and benchmark proposal. No water-utility deployment, M3 decision support, M4 control authority or SOTA advantage is claimed.

## 1. Why this crosswalk changes the strategy

The 2026 water-AI and digital-twin literature has now articulated a very similar evidence-chain problem to the one FSRE is trying to solve. Jorge Alejandro Silva's *Explainable Artificial Intelligence and Digital Twins for Sustainable and Resilient Water Management—A Systematic Review* (Water, 1 October 2026, 18(19), 2443; DOI: https://doi.org/10.3390/w18192443) synthesised 48 records published from 2017 to 5 August 2026. It reports 14 M0 conceptual records, five M1a offline/simulated, 13 M1b retrospective real-world, 11 M2 connected pilots, five M3 operational decision-support deployments, and no M4 audited closed-loop deployment in that corpus. It also reports that 41 of 48 records had absent, asserted or proxy-based sustainability evidence (S0–S2), while seven reached counterfactual or prospective measurement (S3–S4).

The review proposes **TRACE-Water**:
- Traceable data;
- Robust validation;
- Actionable explanation;
- Controlled decision loop;
- Evaluated sustainability.

It explicitly states that TRACE-Water is a proposed synthesis requiring future empirical validation; no reviewed record satisfied all five dimensions at M4. M4 requires more than data connectivity: bounded control authority, fail-safe behavior, human override, drift/security monitoring, audit trail, and prospective operational/sustainability outcomes.

This is prior art and a useful external maturity rubric. FSRE must not claim to have invented this evidence-chain framing or imply that a 48-record review exhausts commercial or utility deployments. The right research question is whether FSRE's owner-preserving, decision-time evidence composition provides a distinct and useful *qualification interface* when applied to actual heterogeneous utility workflows.

## 2. TRACE-Water crosswalk

| TRACE-Water dimension | Established water-sector expectation | FSRE evidence currently available | Remaining gap before an operational claim |
|---|---|---|---|
| Traceable data | Provenance, calibration, coverage, representativeness, missingness, preprocessing, access and update/security controls | Claim envelope v0.2; owner/reference preservation; source register; as-of evidence replay; snapshot/cutoff/clock composition | Lawful owner-native source authentication, calibration/coverage evidence and real-source digests |
| Robust validation | Temporal/spatial leakage prevention, external-site tests, uncertainty, calibration, drift, rare-event and failure-mode tests | Independent Rust evaluators; frozen answers; negative controls; generated temporal/capacity sweeps; exact row-order behavior | Disjoint blind challenge authoring, cross-site holdouts, operator and domain-model baselines, calibration on real data |
| Actionable explanation | Audience- and decision-specific explanation evaluated for faithfulness, stability, usability and practitioner task success | Symthaea investigation and sequential EVSI are available as candidate explanation/information-acquisition components | No practitioner explanation study; no evidence that FSRE outputs improve operator task performance or appropriate reliance |
| Controlled decision loop | Explicit decision rights, oversight, escalation, override, audit, maintenance, accountability, incident response and safe recovery | Explicit no-authority/no-actuation outputs; owner-native authority boundary; read-only composition | No deployed operator loop, shadow-mode monitoring, override measurements, incident-response exercise or production safety case |
| Evaluated sustainability | Counterfactual, system boundary, time horizon, lifecycle costs, water/quality/energy/emissions/ecosystems/safety/access/equity outcomes | Service floors by cohort; stock-flow accounting; portfolio resource feasibility; equity and time-expanded protocols | No prospective counterfactual, verified real service gain, lifecycle study or independent field outcome |

### Maturity claims

The present FSRE synthetic work is **M1a-like software/conformance evidence**, not M1b, M2, M3 or M4. The categories should be applied to each deployed component and claim separately; a successful software workflow does not promote the whole programme to an operational maturity level.

Proposed progression:
1. **M1a — synthetic and benchmark conformance:** exact parsers, temporal boundaries, negative controls, independent references and frozen replay.
2. **M1b — retrospective real-world evidence:** lawful historical utility records, as-known-at-the-time reconstruction, cross-source identity reconciliation and independently adjudicated outcomes.
3. **M2 — connected shadow pilot:** owner-authorised data exchange, operator-visible outputs, no actuation, monitoring of data quality/latency, recorded overrides and incident simulation.
4. **M3 — routine operational decision support:** an identifiable utility workflow, repeated prospective use, documented decision influence, human authority and measured service outcomes.
5. **M4 — separately qualified bounded closed loop:** only after a distinct control-authority and safety case, fail-safe, human override, cybersecurity/drift monitoring, audit, recovery and prospective outcomes. FSRE does not currently approach this status.

The maturity rubric is a claim ceiling, not a marketing ladder. A water-distribution model may be M3 while a wastewater evidence integration remains M1a; report them independently.

## 3. The DMDU benchmark is a direct planning comparator

Liu, Pianosi and Mijic (2026), *A benchmarking framework to enhance cost-efficiency in water systems resilience planning under deep uncertainty*, Journal of Cleaner Production 548, 147850, DOI https://doi.org/10.1016/j.jclepro.2026.147850, proposes evaluating decision-making-under-deep-uncertainty approaches using historical climate to simulate decisions made before future conditions were known, then comparing outcomes with theoretically cost-optimal scenarios.

Its Luton fluvial-flood proof-of-concept reports that a refined real-options setup increased cost by 16.5% while reducing resilience deficit by 74.7% over the planning period. The authors also note simplifying assumptions, including a fixed resilience threshold and undiscounted cost estimation. Those numbers are specific to the study and cannot be treated as a water-utility target or directly compared with FSRE's synthetic conformance counts.

**Implication:** FSRE should not build another real-options or DMDU optimiser. It should make the underlying claims reproducible: what information was available at each decision, which options and service floors were declared, what cost/risk assumptions were owner-supplied, which future climate/hazard paths were held out, and whether the realised service trajectory matched the claim. The existing investment-portfolio protocol should compare DMDU methods on cost, resilience deficit, option preservation, distributional outcomes and decision regret using the same historical hindcast and known-future benchmark.

## 4. REWET is the water-restoration baseline—not a straw man

Naeimi and Davidson's 2025 paper, *Improving Restoration of Water Supply Service Following Disruptions*, Journal of Water Resources Planning and Management 151(7), DOI https://doi.org/10.1061/JWRMD5.WRENG-6452, evaluates 15 restoration plans with the Restoration of Water after an Event Tool (REWET). It studies the Anytown hypothetical network and the Los Angeles network under earthquake scenarios, varying crew management, task prioritisation, immediate damage discovery and rerouting. It reports average reductions in customer-days without service of 77% for Anytown and 67% for Los Angeles relative to the expected amount without restoration activity; the strongest prioritisation strategy depends on network, layout and damage pattern.

This is a direct domain baseline for water-distribution restoration, not an FSRE benchmark result. The fair experiment is:
- freeze a common network and disruption/damage pack;
- give REWET and FSRE the same physical/hydraulic/repair-capacity outputs where their interfaces permit;
- compare the same service trajectories and customer-days without service;
- include crew/task/rerouting assumptions, mobilisation time, model/solver effort and operator workflow;
- test whether FSRE's contribution changes the decision or merely restates the simulator output.

FSRE should consume REWET/EPANET/WNTR outputs where possible. It should not reproduce hydraulic simulation, damage generation or repair scheduling to claim novelty.

## 5. A matched end-to-end benchmark

### Frozen tasks

1. **Historical decision replay:** reconstruct a real prior restoration/maintenance decision using only evidence available then; separate retrospective corrections from original knowledge.
2. **Recovery portfolio feasibility:** evaluate owner-qualified alternatives against common-cause failures, resource contention and per-cohort service floors.
3. **Planning under climate uncertainty:** compare DMDU/real-options portfolios against a historical hindcast and a known-future theoretical reference.
4. **Post-event water restoration:** compare actual operator runbook, REWET/domain optimiser, qualified human team and FSRE evidence/portfolio qualification on identical event and service data.
5. **Wastewater service chain:** include generation, collection, conveyance, treatment, discharge/reuse and receiving environment; do not infer end-to-end sanitation service from plant condition alone.

### Baselines

- B0: operator's current runbook, EAM/APM, SCADA/GIS and procurement/inventory workflow as actually configured;
- B1: EPANET/WNTR hydraulic and resilience outputs;
- B2: REWET restoration plans and crew/rerouting alternatives;
- B3: DMDU real-options/adaptation-pathway method, with its own declared assumptions;
- B4: other configured enterprise platforms (e.g. Maximo, OpenFlows/WaterSight, Foundry) only where the operator actually uses them;
- B5: qualified human operator/expert panel;
- B6: FSRE composition consuming the same owner-native outputs;
- ablations removing as-of replay, common-cause receipts, resource stock-flow, cohort floors, independent reference or service-outcome verification.

### Outcome vector

Report separately:
- customer-days without safe service and cumulative service deficit;
- time to first floor restoration and time to stable recovery;
- unsafe-quality duration and environmental non-compliance;
- protected-cohort floor breaches and between-group service gaps;
- common-cause and shared-resource failure exposure;
- total lifecycle cost, mobilisation/maintenance cost and cost per verified safe service unit;
- decision regret against known-future/cost-optimal reference where the design permits it;
- time-to-evidence, latency, false/late/duplicate source handling and abstention;
- analyst/operator effort, integration cost, system runtime and solver optimality gap.

Never compare a 77% or 67% REWET customer-days reduction with a FSRE exact-test pass rate. They measure different things.

## 6. What would count as doing better than SOTA

### Hard qualification gates

- Zero future-available or future-observed records accepted in an earlier decision replay.
- Zero expired or known-and-effective revoked evidence accepted.
- Zero stale/unknown required evidence promoted to supported claim.
- Zero owner/reference ambiguity silently resolved.
- Zero portfolio-safe disposition with a missed protected cohort floor or known resource overcommit.
- 100% exact deterministic agreement with an independently authored reference on frozen cases.
- 100% reproducibility of scenario, snapshot, cutoff, clock-basis and source-owner/reference bindings.
- Zero unauthorised consequential actions.

### Operational hypotheses—not measured results

After an operator baseline and powered holdout exist, test whether FSRE:
- reduces false-qualified portfolio decisions by at least 50% relative to the incumbent workflow on adjudicated negative controls, without increasing unacceptable abstentions;
- reduces decision regret or lifecycle cost per verified safe service unit by at least 10% versus the strongest matched baseline, with no protected-cohort floor regression;
- improves time to safe service restoration or cumulative customer-days without service by at least 10% under the same physical/resource budget;
- maintains calibrated uncertainty and reliable abstention on held-out utility, catchment, hazard and time periods.

Select one primary outcome and preregister the effect margin, power, harm boundaries and subgroup analyses before looking at outcome results.

## 7. Key strategic conclusion

TRACE-Water already describes a broad evidence-to-decision chain. The DMDU literature already benchmarks planning under deep uncertainty. REWET and EPANET/WNTR already model water-network performance and restoration. Enterprise products already connect assets, telemetry, models and workflows.

FSRE's strongest defensible hypothesis is therefore **not a new water digital twin or a new resilience framework**. It is a portable, owner-preserving qualification and independent reproduction layer for the specific claims that connect these incumbents: "this evidence was available then", "this recovery option was qualified for this scenario", "this portfolio can preserve these service floors under shared constraints", and "the observed service outcome matches the declared claim."

The hypothesis is only valuable if actual operators, regulators or independent assurance bodies need it and will pay to integrate it. If a deployed incumbent already provides equivalent cross-system temporal qualification, independent reproduction and service-outcome reporting at lower total cost, FSRE should integrate or narrow its scope rather than manufacture a gap.

## 8. Primary references

1. Silva, J. A. (2026). *Explainable Artificial Intelligence and Digital Twins for Sustainable and Resilient Water Management—A Systematic Review*. Water 18(19), 2443. https://doi.org/10.3390/w18192443
2. Qureshi, M. et al. (2026). *A systematic literature review of digital twin applications for sustainable water systems in natural and built environment*. Discover Sustainability 7, 1386. https://doi.org/10.1007/s43621-026-03825-2
3. Liu, L., Pianosi, F. & Mijic, A. (2026). *A benchmarking framework to enhance cost-efficiency in water systems resilience planning under deep uncertainty*. Journal of Cleaner Production 548, 147850. https://doi.org/10.1016/j.jclepro.2026.147850
4. Naeimi, S. & Davidson, R. A. (2025). *Improving Restoration of Water Supply Service Following Disruptions*. Journal of Water Resources Planning and Management 151(7). https://doi.org/10.1061/JWRMD5.WRENG-6452
