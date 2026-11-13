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
| GovStack / X-Road / DPI | Reusable government building blocks and secure inter-organisational data exchange | Reuse as exchange and service infrastructure; do not build replacement identity, payment, registry or data-exchange rails |
| ISO 22301 / 22316 / 22336; NIST cyber-resiliency engineering; EU CER; UNDRR | Business continuity, organisational resilience, policy/strategy, cyber-resiliency and critical-entity risk/response foundations | Map controls and evidence to the appropriate sector/jurisdiction; do not claim certification, legal compliance or standard replacement |

## 3. What is potentially distinctive about FSRE

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
- EU Critical Entities Resilience Directive: https://eur-lex.europa.eu/eli/dir/2022/2557/oj/eng
- DORA Regulation 2022/2554: https://eur-lex.europa.eu/eli/reg/2022/2554/oj/eng
- UNDRR Disaster Resilience Scorecard for Cities: https://www.unisdr.org/campaign/resilientcities/assets/toolkit/documents/UNDRR_Disaster%20resilience%20scorecard%20for%20cities_Detailed_English_Jan2021.pdf
