# FSRE competitive landscape and differentiation hypotheses v0.1

**Reviewed:** 2026-11-13  
**Status:** Public-source desk research; not an exhaustive vendor evaluation or procurement assessment.  
**Claim ceiling:** Product capabilities below are described from vendor/standards documentation. No head-to-head benchmark has been run. The document does not establish a market gap, market share, customer outcomes or FSRE superiority.

## 1. Strategic conclusion

Do not position FSRE as a replacement for an enterprise operating system, asset-management suite, industrial digital twin, SCADA platform, government interoperability stack or resilience-management standard.

Several established products already connect data, models, operational workflows and actions:

- Palantir Foundry's Ontology explicitly integrates operational data, logic, actions and dynamic security. Current documentation describes shared ontologies for multiple organisations, human/agent collaboration, external-system write-back, model objectives and action/decision lineage. It is the strongest comparator for the broad integration/action-loop ambition; FSRE must not imply these capabilities are absent from enterprise platforms.
- IBM Maximo APM connects asset-health monitoring, reliability engineering, predictive analytics and AI decision support to maintenance planning and execution.
- Bentley iTwin combines engineering, reality, IoT, GIS and enterprise operational data to build infrastructure digital twins through open APIs and services, including operational use cases.
- Schneider Electric EcoStruxure Water spans water/wastewater treatment, supply and sanitation networks, including software, analytics, telemetry, SCADA and automation.
- Siemens Xcelerator provides an industrial partner ecosystem across industry, buildings, electrification, grids and transportation.
- GovStack, X-Road, the Digital Public Goods Standard and Open Contracting Data Standard provide public digital infrastructure, interoperability, open-product and procurement foundations.

These are substantial capabilities, not straw-man competitors. FSRE must be able to use them as components or comparators. There may be direct product overlap in a customer setting; discover that during procurement and operator interviews rather than assume the gap.

## 2. Product and platform comparison

| System / category | Documented strengths | FSRE treatment |
|---|---|---|
| Palantir Foundry / Ontology / AIP | Operational object model, links, logic, actions, dynamic security, workflow integration, AI tooling and governed operational write-back | A serious end-to-end enterprise comparator. Test whether a neutral federated qualification profile adds independently reproducible cross-operator service outcomes, or whether an existing deployment already does so |
| IBM Maximo Application Suite / APM | Asset health, condition monitoring, reliability-centred maintenance, predictive/prescriptive analytics and maintenance execution | Integrate with or compare against its asset and maintenance functions; FSRE must not recreate EAM/APM |
| Bentley iTwin Platform | Infrastructure digital-twin APIs, engineering/reality/IoT/GIS/schedule and operational data integration, change tracking and access control | Treat as spatial/engineering twin substrate and comparator for asset state, provenance and model interoperability |
| Schneider Electric EcoStruxure Water | Water/wastewater software, automation, telemetry/SCADA, asset and energy advice, analytics and water-cycle operations | Treat as a domain product and potential pilot/operational incumbent; compare using the same service area and approved data |
| Siemens Xcelerator ecosystem | Industrial software, IoT hardware, digital services, partner catalogue and cross-domain solutions | Evaluate the specific product/partner solution selected by an operator, not the marketplace as one monolithic comparator |
| IN-CORE | Integrated system-of-systems environment combining hazard, infrastructure-network, fragility and socioeconomic recovery models | Strong comparator for community-scale hazard-to-recovery studies; reuse or compare its domain outputs rather than rebuild a civilization simulator |
| InfraRisk | Modular simulation of coupled power, water and transport networks, hazard impacts, cascade failures and restoration dynamics | Compare physical/resource feasibility, recovery trajectories and service-level translation under matched scenarios |
| REFIT | Probabilistic cascading-failure modelling with domain-specific solver coupling, including EPANET/OpenDSS pathways | Compare probability handling, uncertainty propagation, option/portfolio evaluation and solver coupling |
| CIrcle | Participatory workshops, expert input and causal mapping for infrastructure interdependency pathways | Compare participatory knowledge capture, governance/organizational dependency coverage and traceability; keep qualitative and quantitative evidence distinct |
| PRIIA | Hypergraph dependency representations and optimisation-based restoration prioritisation | Compare post-disruption prioritisation and restoration planning; do not assume its graph semantics are interchangeable with Mycelix's |
| Kaleidoscope | Spatially enabled infrastructure analytics and co-location/vulnerability pattern analysis | Compare spatial vulnerability and investment prioritisation; distinguish correlation/co-location from explicit causal or operational dependencies |
| CISIA-Pro Simulator | MATLAB-based deterministic fault-propagation simulation over predefined dependency mappings | Compare deterministic cascade semantics and transparent challenge cases; document its runtime/environment requirements |
| GovStack / X-Road / DPI | Reusable government building blocks and secure inter-organisational data exchange | Reuse as exchange and service infrastructure; do not build replacement identity, payment, registry or data-exchange rails |
| ISO 22301 / 22316 / 22336; NIST cyber-resiliency engineering; EU CER; UNDRR | Business continuity, organisational resilience, policy/strategy, cyber-resiliency and critical-entity risk/response foundations | Map controls and evidence to the appropriate sector/jurisdiction; do not claim certification, legal compliance or standard replacement |



## 2.1 A newly published infrastructure interdependency tool comparison (7 November 2026)

Shamsi, Helmrich and Deegan's open-access 2026 comparative study identified 24 infrastructure tools across energy, water and transport; only seven explicitly model cross-sector interdependencies and cascading effects. Its seven-tool comparison includes IN-CORE, InfraRisk, REFIT, CIrcle, PRIIA, Kaleidoscope and CISIA-Pro. It found no single tool captures the full breadth of interdependency behaviour or dominates all assessed technical dimensions. It also reports that comparable runtime/hardware benchmark data were not available across the seven-tool set.

The article also explicitly notes that professional infrastructure operators use proprietary, multi-source tools that are poorly documented in academic literature and are not readily accessible to researchers. Thus, the seven-tool set is a transparent research comparator set, not an exhaustive representation of deployed industry capability. Any FSRE superiority claim must be tested against the actual configured products and runbooks used by a participating operator, including proprietary integrations where the owner permits evaluation.

The paper's four technical dimensions are ordinal assessments (interdependency breadth, data requirements, validation strength and integration potential), not a matched runtime/accuracy/service-outcome benchmark. IN-CORE's comparatively broad interdependency coverage is a useful method comparator, not a universal performance winner. The paper explicitly says comparable runtime and hardware data were unavailable across tools.

The study's comparison dimensions are useful benchmark-design inputs: interdependency breadth, data requirements, validation strength, integration potential, purpose fit, methodology variety, context suitability, application readiness and sectoral scope. These are tool-selection dimensions, not a common task-level performance score.

**FSRE implication:** build a shared, open service-level challenge protocol over owner-native outputs, not a competing infrastructure simulator. Select at least one accessible tool from different methodological families (integrated system-of-systems, probabilistic cascade simulation, participatory dependency mapping and hypergraph/optimisation restoration planning) and compare only claims each tool is designed to support. Keep methods and data budgets visible; do not pretend one input schema or one model class is suitable for all tools.

The source review is a literature comparison, not a head-to-head FSRE experiment. Tool access, licence, version, setup, calibration and operational maturity must be checked before a deployment-level comparison.

Reference: Shamsi, N., Helmrich, A. & Deegan, M. (published 7 November 2026), *Toward system-of-systems resilience: a comparative framework for infrastructure interdependency assessment tools*, Environment Systems and Decisions. https://doi.org/10.1007/s10669-026-10162-2

## 2.2 Water-sector domain-engine comparators

For the first water-service wedge, EPANET and WNTR are mandatory public baselines; configured Bentley OpenFlows Water/WaterGEMS/WaterSight or other operator-native systems are the commercial baseline when actually deployed. EPANET provides extended-period hydraulic and water-quality simulation, operational pump/tank controls and emergency/security analysis. WNTR 1.5 is an EPANET-compatible resilience package that can inject disruptive events, add response/repair strategies, simulate impacts and compute multiple resilience metrics. Bentley's OpenFlows Water suite documents hydraulic, transient, scenario, model calibration, SCADA/GIS integration, pump-scheduling and emergency-condition capabilities; WaterSight documents live SCADA/GIS/model/IoT analytics and emergency response simulation.

FSRE should consume outputs from these systems rather than replace their domain models. Its comparative claim is limited to cross-system evidence admissibility, owner-preserving resource/common-cause composition, cohort floors, stock-flow conservation, decision-time replay and independent outcome verification. See [the water restoration industry baseline crosswalk](WATER_RESTORATION_INDUSTRY_BASELINE_CROSSWALK_V0_1.md) for matched tasks, metrics, and falsification conditions.

Primary sources: EPA EPANET https://www.epa.gov/water-research/epanet; EPA WNTR https://usepa.github.io/WNTR/; Bentley OpenFlows Water https://www.bentley.com/products/openflows-water/; Bentley WaterSight https://www.bentley.com/en/products/openflows-watersight/.

## 2.4 Water-sector evidence-chain, maturity and restoration benchmarks (2026)

A further important comparator is Silva's 2026 systematic review of 48 water-AI/digital-twin records. It proposes TRACE-Water (Traceable data, Robust validation, Actionable explanation, Controlled decision loop, Evaluated sustainability), reports five M3 operational decision-support contributions and no M4 audited closed-loop contribution in its reviewed corpus, and says TRACE-Water itself needs prospective empirical validation. M4 requires bounded authority, fail-safe/override, drift and security monitoring, an audit trail and prospective outcomes—not merely a live data link. FSRE should map to this work rather than claim to have originated the evidence-to-decision framing. See [the TRACE-Water crosswalk](TRACE_WATER_AND_RESTORATION_BENCHMARK_CROSSWALK_V0_1.md).

Two direct domain comparators should shape the first water experiment:
- Liu, Pianosi & Mijic (2026) benchmark DMDU water-resilience planning with historical-climate hindcasts against a theoretical cost-optimal reference. Their Luton proof of concept reports a refined real-options setup with 16.5% higher cost and 74.7% lower resilience deficit under simplified assumptions. This is a planning-method result, not an FSRE target.
- Naeimi & Davidson (2025) use REWET to compare 15 restoration plans across Anytown and Los Angeles water networks and report customer-days-without-service reductions from restoration actions. This is the direct post-event water restoration baseline; FSRE should consume or compare its outputs, not recreate hydraulic or repair models.

These results require different metrics and matched tasks. A benchmark should not compare service-outcome improvements with software conformance counts or mix the DMDU known-future experiment with a post-event restoration trial.

Primary sources:
- Silva (2026), https://doi.org/10.3390/w18192443
- Qureshi et al. (2026), https://doi.org/10.1007/s43621-026-03825-2
- Liu, Pianosi & Mijic (2026), https://doi.org/10.1016/j.jclepro.2026.147850
- Naeimi & Davidson (2025), https://doi.org/10.1061/JWRMD5.WRENG-6452

## 2.3 Current water-monitoring research comparators (November 2026)

The sensor-placement frontier is moving quickly. These methods should be treated as specialist baselines, not dismissed as competitors FSRE has already surpassed.

| Method / study | Published capability or result | What the matched benchmark must still test |
|---|---|---|
| Structural-observability placement (van Gemert et al., arXiv, 9 Nov 2026) | Topology/interconnection-based placement demonstrated on real Brabant Water networks; reduces dependence on exact hydraulic parameters, provided topology is correct | Sensor count, coverage versus relaxed observability, performance under topology changes, and lifecycle cost |
| Continuous score-based pressure placement (Hu & Khu, 2 Nov 2026) | Replaces binary event thresholds with continuous sigmoid-style scoring of pressure residuals; reports robustness to spatially varying noise | Calibration, false alarms, event severity distribution, sensor degradation and cross-network generalisation |
| Dynamic Residual Masking probabilistic placement (Journal of Hydrology, Nov 2026) | Reports 91.4% detection accuracy with five sensors and 4.2% false positives in its evaluated configuration, using RF/SHAP-based sensitivities and pseudo-Jacobian transformation | Exact network/event split, external validation, hardware, calibration, false-negative severity, and comparability of labels and noise assumptions |
| Cascading-aware graph-signal placement (RESS, 2026) | Connects graph structure, hydraulic state, leakage factors and risk propagation | Whether risk propagation improves actionable detection and service outcomes over strong task-specific baselines |
| 2026 review of water and wastewater sensor/sampling placement | Finds trade-offs among computation, scalability, uncertainty and decision support; notes maintenance, sensor degradation, delayed response and operational constraints are underrepresented in many OSP models | Lifecycle maintenance, data latency, imperfect detection, dynamic reconfiguration and real operator workflow |

The reported 91.4%/4.2% figures are paper-specific results, not a universal SOTA score. They cannot be directly compared with REWET restoration results, another network, or FSRE's synthetic conformance counts. All candidate methods must be rerun on a common frozen dataset and held-out scenario pack.

FSRE should not implement another placement algorithm merely to claim novelty. It should qualify candidate sensor plans and their outputs across:
- acquisition and maintenance cost;
- observation-to-availability latency;
- data quality, freshness and clock semantics;
- correlation and common-cause exposure among sensors;
- sequential information value and decision regret;
- protected cohort service floors;
- independently observed service improvement.

The proposed [decision-grade monitoring benchmark](DECISION_GRADE_MONITORING_BENCHMARK_V0_1.md) formalises that comparison.

Primary sources:
- van Gemert et al. (9 Nov 2026), *Sensor placement for real-life water distribution networks, ensuring observability under uncertainties*: https://arxiv.org/abs/2611.12414
- Hu & Khu (2 Nov 2026), *Continuous score-based optimization of pressure sensor placement for burst detection in water distribution networks*, DOI: https://doi.org/10.1016/j.jclepro.2026.149780
- Damonti & Bernasconi (Nov 2026), *Probabilistic sensor placement in water distribution networks: a Dynamic Residual Masking framework using game-theoretic pseudo-Jacobians*, DOI: https://doi.org/10.1016/j.jhydrol.2026.136514
- Huang et al. (2026), *Cascading-aware sensor placement for leakage detection in water distribution networks*, DOI: https://doi.org/10.1016/j.ress.2026.112709
- *Optimal Sensor and Sampling Placement for Contaminant Detection: A Comprehensive Review Across Water Distribution and Wastewater Collection Systems* (2026), DOI: https://doi.org/10.3390/eng7030121

## 3. What is potentially distinctive about FSRE

### Prior-art boundary: resilience engineering is established

NIST SP 800-160 Vol. 2 Rev. 1 explicitly describes cyber-resiliency engineering as a specialty systems-engineering discipline used alongside systems security engineering and resilience engineering. It frames the capability around anticipating, withstanding, recovering from and adapting to adverse conditions. FSRE must not claim to invent resilience engineering, cyber resilience, or systems-of-systems thinking. Its differentiation hypothesis is narrower: a portable, owner-preserving qualification and independent benchmark layer that tests whether recovery options and service claims survive across heterogeneous models, operators, permissions and resource constraints.

The relevant comparator is therefore not just a simulator. It is the complete deployed workflow and assurance practice already available to an operator. Reference: https://csrc.nist.gov/pubs/sp/800/160/v2/r1/final.


The following are **differentiation hypotheses to test**, not claims that named vendors lack the capability.

### H1 — Service trajectory as the common evaluation unit

Join operator observations and qualified model outputs into a time-bounded service contract. Report service deficit, quality breaches, safe degradation, full service, unknown duration and stable recovery by service area/cohort. A dashboard's asset uptime is not a substitute for measured service delivery.

### H2 — Recovery options must be feasible, independent and executable

Do not count nominal redundant assets or graph paths as recovery options. Require domain-model capacity, spatial access, time-qualified dependency independence, staffed resources, current authority and safety constraints. Retain explicit unknowns.

### H3 — Portable qualification across heterogeneous incumbent systems

Do not claim that shared operational models, cross-organisation collaboration or governed action workflows are missing from enterprise platforms. Palantir documents shared ontologies across multiple organisations, granular permissions, human/agent action workflows, external-system write-back and decision lineage. Other vendors provide rich asset, spatial, physical and operational models.

The narrower FSRE hypothesis is that an operator, regulator, insurer or independent assurance body may benefit from a **portable, product-neutral qualification profile** that consumes owner-native records from more than one incumbent platform, checks exact service/scenario/time/scope bindings, preserves unknowns and produces an independently reproducible service-outcome report without making the assurance layer itself the operational system of record.

Test this as a head-to-head research question:
1. Select a real deployed service workflow using its incumbent platform(s).
2. Have the incumbent's own configured assurance/reporting path produce its service/recovery claim.
3. Independently re-evaluate the same source records with the FSRE profile.
4. Compare completeness, stale/unknown handling, cross-vendor portability, evaluator reproducibility, operator effort, lifecycle cost and service-outcome validity.
5. Reject the FSRE wedge if the incumbent already does this equivalently, more cheaply and with equal or better independent reproducibility.

This is a differentiation hypothesis, not an assertion that Palantir or another vendor cannot do it.

### H4 — Independent outcome verification as an architectural interface

Separate proposer, operator, data custodian and evaluator roles. Bind the evaluation to frozen source/model/authority/action versions, observations, missingness, baseline and uncertainty. A signed receipt is evidence of an assertion or event under a trust model, not proof of physical service improvement.

### H5 — Portability and lifecycle economics

Measure the total cost of a verified improvement, including licences, deployment, integration, data stewardship, training, staffing, audit, compute, maintenance, vendor transition and exit. An open architecture is only useful if another qualified operator can export, reproduce and continue the workflow at a defensible cost.



### H6 — Reproducible assurance beyond a vendor's own dashboard

An incumbent may already provide workflows, security controls, audit logs and model feedback. FSRE should earn a role only where a separate party needs to reproduce a narrowly defined claim from retained owner-native evidence, compare multiple incumbent systems without importing all their data into one control plane, and disclose the exact claim ceiling, source frontier and unresolved states. This may be useful to regulators, mutual-aid partners, insurers, auditors or procurement authorities—but only buyer/operator interviews and a paid pilot can establish demand.

## 4. How to benchmark against industry fairly

Run at least one matched service study with operator permission. Use the same:

- service boundary, observation window, data quality, source frontier and event set;
- permitted actions, authority scope, response-time budget and staffing assumptions;
- physical constraints, demand conditions, outage scenarios and resource limits;
- implementation/tuning budget, human effort, compute budget and evaluation method.

Compare these arms where available:

1. The incumbent operator runbook and current system.
2. The operator's deployed EAM/APM, SCADA, GIS, digital-twin or workflow product.
3. The best qualified domain simulator and existing common-cause/dependency analysis.
4. A strong AI agent with equivalent data and tool permissions.
5. Qualified human operators/engineers.
6. FSRE composition and preregistered layer ablations.

Do not compare a full FSRE stack against an unconfigured product brochure. Use deployed products and actual operational procedures wherever access permits. If vendor software cannot be run by an independent evaluator, report the comparator limitation rather than inferring a win.

Publish matched results by task and cohort. Keep safety and rights gates separate from performance scores.

## 5. Proposed FSRE differentiation benchmarks

These complement, rather than replace, canonical Mycelix, Sol-Atlas, Symtropy and Symthaea benchmark owners.

| Claim under test | Core measure | Qualification target / gate |
|---|---|---|
| Service trajectory calculation | Independent reference agreement for known/unknown intervals, service deficit, quality exposure and stable recovery | 100% exact agreement on frozen deterministic fixtures |
| Unknown handling | False-safe classification and abstention under stale/missing/contradictory observations | Zero false-safe classifications on declared negative controls; report coverage and sample size |
| Recovery-option integrity | False-independent option rate under hidden common causes, duplicate identifiers and alternative-path inflation | Zero false-independent options on frozen controls; full reason-code and evidence-frontier agreement |
| Physical feasibility | Capacity / quality / operating-envelope violations in model outputs | No out-of-domain result accepted as qualified; compare error and calibration against domain validation data |
| Authority | Revoked, expired, wrong-purpose and wrong-jurisdiction action acceptance | Zero accepted unauthorised consequential actions in declared adversarial suite |
| Federation | Local continuity, conflict reconciliation, revocation and exit/export under three or more nodes | 100% no-unauthorised-write conformance; explicit unresolved conflict retention |
| Outcome effect | Paired service effect and distributional non-regression | Preregister practical-effect margin after baseline and operator study design |
| Economics | Total lifecycle cost per independently verified service-improvement unit | Lower than the strongest incumbent workflow for the same service unit, with uncertainty interval and full cost disclosure |
| Portability | Independent export, re-host and reconstruction time/cost | A second operator can independently reproduce the same semantics and evidence lineage without central service dependency |

Targets are prospective and need sample-size/power analysis. Zero failures on a small fixture is not a production reliability estimate. Do not collapse these metrics into a single resilience or trust score.

## 6. Avoid these strategic mistakes

- Do not compete with Palantir by building another general-purpose enterprise ontology and workflow engine.
- Do not compete with IBM Maximo by building another asset-management or predictive-maintenance suite.
- Do not compete with Bentley by building another infrastructure digital-twin platform.
- Do not compete with Schneider by building another SCADA, water-process or utility automation product.
- Do not create a second dependency graph, evidence ledger, benchmark manifest, causal scheduler or physical simulator inside Luminous.
- Do not say "vendors do not verify outcomes" without testing their actual deployment, contractual assurance and independent audit trail.
- Do not claim regulatory compliance from a document crosswalk.
- Do not treat a benchmark speedup, more graph nodes, a larger scenario count or an integrated dashboard as societal impact.

## 7. Practical market discovery

For each prospective pilot, record:

1. Incumbent products, service providers, integration partners and operator runbooks.
2. The exact service-level problem left unresolved, with source evidence and stakeholder confirmation.
3. Existing contract/licensing and procurement constraints.
4. Data access, ownership, confidentiality, retention and lawful-purpose boundaries.
5. Operator authority, safety case, regulator and insurance requirements.
6. Baseline cost, integration cost, continuing stewardship and vendor exit cost.
7. Whether an adapter, qualification profile or independent audit is enough—or whether a new product is actually required.

The first market wedge should be a narrow paid service: independent resilience qualification and service-outcome assurance over systems the operator already owns. This is a hypothesis to validate with buyers, not a presumed procurement opportunity.

## 8. Primary references

- Palantir, The Ontology system: https://www.palantir.com/docs/foundry/architecture-center/ontology-system
- Palantir, Why create an Ontology?: https://www.palantir.com/docs/foundry/ontology/why-ontology
- IBM Maximo Asset Performance Management: https://www.ibm.com/products/maximo/asset-performance-management
- Bentley iTwin Platform: https://www.bentley.com/products/itwin-platform/
- Bentley iTwin IoT: https://www.bentley.com/products/itwin-iot/
- Schneider Electric, Water and Wastewater solutions: https://www.se.com/ww/en/work/solutions/water/
- Schneider Electric, Water Advisor deployment example: https://www.se.com/za/en/about-us/newsroom/news/press-releases/water-meet-your-digital-twin-68f20f7f90eeaab9b6094ce2/
- Siemens Xcelerator: https://www.siemens.com/en-us/company/digital-transformation/
- ISO 22301: https://www.iso.org/standard/75106.html
- ISO 22316 revision status: https://www.iso.org/standard/86493.html
- ISO 22336: https://www.iso.org/standard/50073.html
- NIST SP 800-160 Vol. 2 Rev. 1: https://csrc.nist.gov/pubs/sp/800/160/v2/r1/final
- Poulin, C. & Kane, M.B. (2021), *Infrastructure resilience curves: Performance measures and summary metrics*, Reliability Engineering & System Safety 216, 107926: https://doi.org/10.1016/j.ress.2021.107926
- EU Critical Entities Resilience Directive: https://eur-lex.europa.eu/eli/dir/2022/2557/oj/eng
- DORA Regulation 2022/2554: https://eur-lex.europa.eu/eli/reg/2022/2554/oj/eng
- UNDRR Disaster Resilience Scorecard for Cities: https://www.unisdr.org/campaign/resilientcities/assets/toolkit/documents/UNDRR_Disaster%20resilience%20scorecard%20for%20cities_Detailed_English_Jan2021.pdf
