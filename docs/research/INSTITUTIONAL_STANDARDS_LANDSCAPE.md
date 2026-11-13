# Institutional Standards Landscape and Gap Analysis

**Status:** Desk-research synthesis, 2026-11-13  
**Purpose:** Position Luminous Institutional Systems Engineering (LISE) against established governance, digital-government, risk-management and interoperability frameworks.  
**Caution:** This is a research map, not a certification, legal opinion or claim that LISE is superior to the referenced standards.

## 1. Executive finding

LISE should not attempt to replace existing governance standards. It should build a practical, testable engineering layer that can help organizations implement, connect and evaluate relevant principles.

Several mature frameworks already cover important parts of the problem:

- ISO 37000 provides organization-wide governance guidance.
- ISO 37004 provides a governance maturity model and measurement approach.
- OECD citizen-participation guidance provides process design and evaluation principles.
- OECD and World Bank digital-government work provides infrastructure and public-sector maturity benchmarks.
- NIST AI RMF provides a risk lifecycle for AI-related organizational risks.
- W3C Verifiable Credentials provides an interoperable data model for issuer/holder/verifier claims.

The opportunity is to connect institutional rules, scoped authority, evidence, execution, review, appeals, recovery and cross-organization coordination into reproducible workflows and comparative tests.

## 2. Standards and frameworks

| Framework | What it already contributes | LISE should do |
|---|---|---|
| ISO 37000:2021 | Purpose, value, strategy, oversight, accountability, stakeholder engagement, leadership, data and decisions, risk governance, social responsibility, viability | Map institutional workflows to its governance principles; do not invent a competing generic governance vocabulary |
| ISO 37004:2023 | Governance maturity model, measurement framework and maturity scale | Use as a maturity reference; add scenario-level behavioral and adversarial evidence |
| OECD Citizen Participation Guidelines (2022) | Ten-step participation process, eight participation methods and quality principles | Turn selected process obligations into test fixtures; measure inclusion, feedback closure, privacy and participant burden |
| OECD Digital Government Outlook 2026 | Shared digital infrastructure, data governance, interoperability, adoption and delivery performance | Benchmark actual cross-institution use, not just existence of APIs or strategies |
| World Bank GovTech Maturity Index 2025 | 48 indicators across core government systems, online services, citizen engagement and enabling conditions | Use as a public-sector comparison layer, not as a substitute for organization-level tests |
| NIST AI RMF 1.0 | GOVERN, MAP, MEASURE, MANAGE functions for AI risk | Apply to Symthaea and AI-assisted institutional workflows; keep human institutional accountability explicit |
| W3C Verifiable Credentials Data Model 2.0 | Issuer, holder and verifier model for tamper-evident claims with privacy and accessibility considerations | Reuse interoperable credential patterns for scoped institutional claims; do not equate a valid credential with legal authority or truth |

## 3. Important current observations

### ISO governance standards

ISO 37000:2021 applies to organizations regardless of type, size, location, structure or purpose. ISO's catalogue records the standard as reviewed and confirmed in August 2026. ISO 37004:2023 separately provides governance maturity measurement guidance.

**Implication:** LISE's differentiation should not be "we invented measurable governance." That territory already has a formal standards foundation. The stronger contribution is a reproducible test and assurance layer that connects maturity claims to observable actions, evidence and adversarial behavior.

### Participation

OECD participation guidance emphasizes that participation should only be used where people can meaningfully influence the decision. It also stresses clear expected outcomes, inclusive recruitment, accessible methods, feedback to participants, evaluation, privacy and adequate resourcing.

**Implication:** A benchmark must detect performative participation. Attendance, comment volume or token voting alone cannot establish meaningful influence.

### Digital government and institutional interoperability

OECD's 2026 Digital Government Outlook describes a gap between infrastructure availability and operational adoption. It also emphasizes that interoperability depends on governance arrangements, clear data-sharing rules, semantic standards and accountability, not merely technical connections.

**Implication:** LISE should test whether independent institutions can complete real workflows under explicit agreements, with clear liability, privacy, authority and dispute handling.

### AI risk governance

NIST AI RMF 1.0 organizes AI risk work around GOVERN, MAP, MEASURE and MANAGE. As of 13 November 2026, NIST says AI RMF 1.0 is being revised and has released a 7 April 2026 concept note for a Trustworthy AI in Critical Infrastructure profile. LISE should track the revision and any relevant profile rather than implying that version 1.0 is the final future state.

**Implication:** Symthaea-assisted governance should inherit lifecycle risk-management, measurement, documentation and independent-review disciplines. The institutional layer must additionally identify who is accountable for accepting, rejecting or acting on AI-generated recommendations.

### Verifiable credentials

W3C Verifiable Credentials Data Model 2.0 is a Recommendation. Its verification step establishes the authenticity and currentness of an issuer's statement under the relevant proof and status mechanisms; it explicitly does not establish the truth of the claims. A verifier separately validates whether the issuer and claims are acceptable for its purpose. A W3C VC Data Model 2.1 Working Draft was published in September 2026; it is work in progress and must not be represented as a Recommendation.

**Implication:** Mycelix should distinguish proof verification from claim validation, issuer authority, evidence quality, claim freshness, revocation, purpose and relying-party policy. It should record why a relying party accepted or rejected a claim, with a route to challenge that decision.

### Organizational and public-sector digital twins

Organizational digital twins are already an active research topic. Edrisi et al. describe a Digital Twin of the Organization in IEEE Access (2024). A 2025 Data & Policy study finds that digital-twin adoption in public administration remains limited and identifies institutional readiness, privacy and fragmented governance as barriers. A working paper posted in October 2026, Learning-by-Twinning, studies organizational digital twins combining firm-specific traces, AI-based behavioural simulation and governance rules.

**Implication:** LISE must not claim to invent organizational digital twins. Its proposed contribution should be narrower and testable: rights-aware, constitution-linked twins for plural and federated institutions, with explicit authority, rule-in-use observations, contestability, exit and independent validation. The twin must not become a single authoritative representation of the institution.

### Institutional analysis and rules-in-use

The Ostrom Institutional Analysis and Development (IAD) framework is an established foundation for examining actors, action situations, rules, incentives, interactions and outcomes. Later analysis of IAD highlights a crucial distinction between formal legal rules and working rules: the two can align, partly overlap or diverge in practice.

**Implication:** LISE must model both the declared constitution and observed rules-in-use. A policy engine can verify that software follows a formal rule; it cannot establish that people actually follow that rule, that informal norms are legitimate, or that the formal rule is appropriate.

## 4. LISE's proposed engineering contribution

The candidate gap is not another abstract principle catalogue. It is a reproducible institutional workflow evaluation stack:

1. **Institutional model:** define roles, rights, obligations, authorities, policies and relationships.
2. **Workflow trace:** record proposal, authorization, execution, observation, review, appeal and remedy.
3. **Evidence graph:** bind claims to provenance, time, uncertainty, conflicts and verification.
4. **Policy test harness:** execute positive, negative and boundary tests against selected rules.
5. **Adversarial scenario runner:** simulate capture, collusion, coercion, insider abuse, outages and succession failure.
6. **Cross-organization conformance:** test negotiated interfaces and mutual recognition without forced organizational merger.
7. **Outcome evaluation:** measure cost, time, inclusion, burden, distribution, rights impacts and ecological effects.
8. **Independent assurance:** allow evaluators to reproduce results without trusting the system developer.

These are proposed capabilities. They must be implemented and evaluated before being claimed as delivered.

## 5. Comparison strategy

Use standards as reference frameworks and build matched task comparisons around them.

| Comparison level | Question |
|---|---|
| Principle alignment | Does the institution address the relevant governance principles? |
| Process conformance | Does the workflow follow its declared rules? |
| Behavioral robustness | Does it resist seeded violations and adversarial actions? |
| Evidence quality | Can an independent reviewer reconstruct what happened and why? |
| Interoperability | Can different institutions complete a shared task under their own rules? |
| Outcomes | Does the workflow improve real outcomes without unacceptable rights or distributional harms? |
| Lifecycle resilience | Can the institution survive turnover, provider failure, exit and dissolution? |

A framework can be strong at principle alignment while weak at operational conformance. A system can be technically robust while institutionally illegitimate. Report these dimensions separately.

## 6. Research priorities

1. Map IPB v0.1 metric families to ISO 37000 and ISO 37004.
2. Map participation scenarios to OECD's participation principles and methods.
3. Map Symthaea-related AI risk controls to NIST AI RMF.
4. Map Mycelix credential and authority claims to W3C VC semantics and lifecycle requirements.
5. Map cross-institution interoperability scenarios to OECD DPI and World Bank GovTech indicators.
6. Identify gaps not covered by existing frameworks: adversarial institutional capture tests, evidence-linked workflow reconstruction, cross-constitutional conformance and succession under attack.
7. Ask independent governance, legal, standards and cooperative researchers to challenge the mappings.

## 7. Primary sources

- ISO 37000:2021, Governance of organizations — Guidance: https://www.iso.org/standard/65036.html
- ISO 37004:2023, Governance of organizations — Governance maturity model — Guidance: https://www.iso.org/standard/65037.html
- OECD Guidelines for Citizen Participation Processes (2022): https://www.oecd.org/en/publications/oecd-guidelines-for-citizen-participation-processes_f765caf6-en.html
- OECD Digital Government Outlook 2026: https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en.html
- World Bank GovTech Maturity Index 2025 Update: https://www.worldbank.org/en/programs/govtech/gtmi-2025-update
- NIST AI Risk Management Framework 1.0 and current revision status: https://www.nist.gov/itl/ai-risk-management-framework
- W3C Verifiable Credentials Data Model v2.0 Recommendation: https://www.w3.org/TR/vc-data-model/
- W3C Verifiable Credentials Data Model v2.1 Working Draft (30 September 2026): https://www.w3.org/TR/2026/WD-vc-data-model-2.1-20260930/
- Edrisi et al., Developing and Evolving a Digital Twin of the Organization (IEEE Access, 2024): https://doi.org/10.1109/ACCESS.2024.3381778
- Lost in translation: why digital twins thrive in research but falter in politics and public administration (Data & Policy, 2025): https://doi.org/10.1017/dap.2025.10027
- Choudhury, Du and Lawrence, Learning-by-Twinning: Organizational Digital Twins and the Future of Search (working paper, posted 1 October 2026): https://papers.ssrn.com/sol3/papers.cfm?abstract_id=7547723
- Cole, Laws, norms, and the Institutional Analysis and Development framework (2017): https://doi.org/10.1017/S1744137416000486
- Ostrom Workshop, IAD Framework: https://ostrom.indiana.edu/courses-teaching/teaching-tools/iad-framework/index.html

## 8. Research integrity

This landscape is a starting point. Standards may have paywalled normative text, jurisdictional interpretations and sector-specific requirements. The summaries above rely on public abstracts and official framework material. Before claiming conformance, obtain and review the applicable full standard, determine its legal relevance, and commission competent independent assessment.
