# FSRE standards and industry crosswalk v0.1

**Status:** Desk-research crosswalk; not a compliance assessment.  
**Reviewed:** 2026-11-13  
**Purpose:** Position FSRE against established resilience, critical-infrastructure, digital-government and procurement frameworks without claiming to replace them.

## 1. Positioning

Resilience engineering is an established body of practice. FSRE should not claim to invent resilience engineering, business continuity, critical-infrastructure resilience, digital public infrastructure or systems assurance.

The proposed contribution is an **executable, service-centred integration and qualification profile**: a way to compose existing specialist systems across independent organisations, then test whether the composition improves a declared essential-service outcome while preserving local authority, rights, privacy and the ability to exit.

This is a research hypothesis about an implementation gap, not a proven claim that no existing vendor, government platform or standard does comparable work. A systematic product review and interviews with utilities, civil-protection agencies, municipalities, operators and assurance bodies remain necessary.

## 2. Standards and framework crosswalk

| Framework | Existing contribution | FSRE relationship | What FSRE must not claim |
|---|---|---|---|
| ISO 22301:2019 Business Continuity Management Systems | Requirements for an organisation's business-continuity management system, including preparation, response, continuity and recovery | Use its organisational continuity controls as inputs to service-level qualification and operator baselines | ISO certification, conformity or replacement of a BCMS |
| ISO 22316:2017 / ISO/FDIS 22316 (2026 revision process) | Organisational resilience guidance | Map organisational resilience capabilities to explicit service dependencies, recovery options and measured outcomes | A replacement for organisational resilience policy or guidance |
| ISO 22336:2024 | Guidance for resilience policy, strategy, implementation priorities and coordinated organisational capability | Use as a policy/strategy alignment reference; FSRE tests whether selected interventions work in bounded service contexts | That an executable profile alone establishes organisational resilience |
| EU Critical Entities Resilience Directive (EU) 2022/2557 | Requires covered critical entities to assess relevant risks, including cross-sector and cross-border dependencies, and implement proportionate resilience measures | Potential evidence and workflow support for covered entities operating under the Directive | Legal compliance, designation as a critical entity, or applicability outside the relevant legal scope |
| NIST SP 800-160 Vol. 2 Rev. 1 | Cyber-resiliency engineering for systems that anticipate, withstand, recover from and adapt to adverse cyber conditions | Reuse systems-engineering and cyber-resilience concepts; extend the service study across physical, institutional, economic and ecological dependencies | Replacement for NIST guidance or proof of cyber/physical safety |
| UNDRR Disaster Resilience Scorecard for Cities | City-level governance, risk, planning, infrastructure, response and recovery assessment structured around the Ten Essentials | Use as a city/planning assessment input; connect selected gaps to operational tests and measured service outcomes | Replacement for the Scorecard, disaster-risk assessment or local emergency authority |
| GovStack / GovSpecs | Composable, interoperable digital-government building blocks and functional specifications | Integrate with existing building blocks; use them for identity, payments, registries and other shared digital services where appropriate | A competing national digital-government platform or a new mandatory building-block stack |
| X-Road | Federated secure data exchange; organisational identity and exchange evidence remain with participating parties | Use as an exchange substrate or interoperability peer; add domain-specific evidence qualification and outcome evaluation above transport | Replacement for X-Road, or a claim that data exchange alone validates facts or authorises actions |
| Digital Public Goods Standard | Requirements covering open licensing, platform independence, privacy, documentation and standards | Design FSRE artefacts to meet relevant requirements and pursue assessment when appropriate | Digital Public Good status without an actual review against the Standard |
| Open Contracting Data Standard (OCDS) | Structured disclosure of public contracting from planning through implementation | Link contract milestones and delivery records to independent physical/service observations and outcome evaluation | That procurement disclosure proves delivery quality or service improvement |
| World Bank GovTech Maturity Index (GTMI) 2025 | 48 indicators across core government systems, online services, citizen engagement and GovTech enablers; 197 economies in the 2025 update | Treat as a macro-level maturity/context reference; evaluate operational service effects separately | A replacement GTMI score, country ranking or outcome measure |
| OECD Digital Government Outlook 2026 | Evidence on digital-government capability, interoperability adoption and AI governance/impact assessment | Use as evidence that infrastructure presence, institutional use and impact evaluation are different dimensions | A global estimate or a claim that the OECD figures describe every country |

## 3. Evidence corrections and dated context

The OECD Digital Government Outlook 2026 reports:

- National data-interoperability systems are in place in 30 of 36 OECD countries.
- On average, 63% of public institutions across OECD countries are connected to their national data-interoperability system.
- Only 28% of OECD countries report conducting **any financial or non-financial assessment** of the impact of government AI use cases.

The 28% result is not a finding that 28% systematically assess all government interventions. These figures describe OECD countries and should not be presented as global estimates.

The World Bank's 2025 GovTech Maturity Index covers 197 economies and uses 48 indicators across four GovTech areas. The World Bank explicitly describes the GTMI as a maturity snapshot, not a ranking of country performance or readiness. It is a useful context instrument, but not a substitute for service-level causal evaluation.

South African water indicators must also remain separate and dated:

- The 2025 Green Drop assessment covered 848 municipal wastewater treatment systems for the 2023/24 financial year and reported 396 systems (47%) in a critical state.
- The 2025 No Drop Progress Report was released on 31 March 2026. Parliament's 9 June 2026 briefing reported national non-revenue water at 47.3%, compared with 47.4% in the 2023 Full No Drop Report.

Green Drop concerns wastewater-treatment performance; No Drop concerns drinking-water distribution efficiency and losses. Neither percentage is a present-day Johannesburg utility baseline, and non-revenue water must not be treated as synonymous with physical leakage alone.

## 4. What FSRE can test that the components do not establish by themselves

FSRE should make a composed, bounded workflow answer questions that component standards and platforms leave to the implementation:

1. **Service truth:** What service was actually delivered, to whom, where and over what time interval? What is unknown?
2. **Dependency qualification:** Which dependencies and common failure domains are supported by the evidence frontier, and which remain unknown?
3. **Recovery feasibility:** Which alternatives remain technically feasible, resourced, reachable, staffed and authorised under the declared disruption?
4. **Decision quality:** Did causal investigation distinguish among plausible explanations and identify a useful next observation or experiment?
5. **Legitimate execution:** Was the action approved by the competent actor, within scope and still authorised at execution time?
6. **Outcome verification:** Did an independent evaluator observe the pre-registered service improvement, with uncertainty and distributional effects disclosed?
7. **Federated continuity:** Could each participating organisation continue safe local operation during partitions, revocation, provider outages and schema mismatch?
8. **Economic value:** Was the verified service improvement worth the full lifecycle cost, including integration, maintenance, training, audit and transition costs?

These are proposed FSRE benchmark questions, not capabilities already demonstrated by the current PR.

## 5. Competitive comparison method

Do not compare FSRE against a caricature of existing systems. Compare the full composition to the strongest realistic baseline available for the same service lane and frozen information:

- the actual operator workflow and runbook;
- the existing data-exchange and digital-government building blocks;
- the best domain-specific model or industrial digital twin;
- a strong AI agent with equivalent tool and data access;
- qualified human operators/analysts;
- the qualified FSRE composition;
- FSRE ablations that remove one integration layer at a time.

Keep benchmark tasks, evidence, permissions, human effort, compute budgets, tuning effort and evaluation rules matched. Publish cases where an existing system is equal or better. A missing comparator is not evidence of superiority.

## 6. Research and market diligence still required

Before making a product-market-gap claim:

- Review commercial critical-infrastructure risk, resilience-management, asset-management, digital-twin, incident-management and operational-assurance platforms.
- Review national and regional deployments built around GovStack, X-Road and comparable interoperability systems.
- Interview utilities, municipalities, grid operators, emergency services, regulators, insurers and procurement authorities.
- Map procurement requirements, data-access constraints, liability, standards, existing contracts and total cost of ownership.
- Identify which outcome/evidence/authority requirements are genuinely unmet, and which are already served by existing tools or institutional practice.

**Claim ceiling:** this document is a source-based standards and product-category crosswalk. It is not a legal opinion, certification assessment, exhaustive vendor review, procurement recommendation or proof of a market gap.

## 7. Primary references

- ISO 22301:2019: https://www.iso.org/standard/75106.html
- ISO 22316 current revision status: https://www.iso.org/standard/86493.html
- ISO 22336:2024: https://www.iso.org/standard/50073.html
- EU Critical Entities Resilience Directive: https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=CELEX:32022L2557
- NIST SP 800-160 Vol. 2 Rev. 1: https://csrc.nist.gov/pubs/sp/800/160/v2/r1/final
- UNDRR Disaster Resilience Scorecard for Cities: https://www.unisdr.org/campaign/resilientcities/assets/toolkit/documents/UNDRR_Disaster%20resilience%20scorecard%20for%20cities_Detailed_English_Jan2021.pdf
- GovStack / ITU GovSpecs: https://www.itu.int/en/ITU-D/ICT-Applications/Pages/GovStack-GovSpecs.aspx
- X-Road security architecture: https://x-road.global/security
- Digital Public Goods Standard: https://www.digitalpublicgoods.net/standard
- Open Contracting Data Standard: https://www.open-contracting.org/data-standard/
- World Bank GTMI 2025: https://www.worldbank.org/en/programs/govtech/gtmi-2025-update
- OECD Digital Government Outlook 2026 — interoperability: https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en/full-report/strengthening-digital-public-infrastructure-and-data-governance_2c7323c7.html
- OECD Digital Government Outlook 2026 — AI impact assessment: https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en/full-report/digital-governments-at-a-turning-point_0491aad4.html
- South African 2025 Green Drop results: https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
- South African 2026 parliamentary water progress briefing: https://www.parliament.gov.za/press-releases/media-statement-water-committee-warns-of-deepening-crisis-as-municipalities-fail-implement-corrective-water-plans
