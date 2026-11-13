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

NIST AI RMF organizes AI risk work around GOVERN, MAP, MEASURE and MANAGE, with governance cross-cutting the lifecycle. Its measurement guidance includes uncertainty, benchmarks, documentation and independent review.

**Implication:** Symthaea-assisted governance should inherit these risk-management disciplines. The institutional layer must also address who is accountable for accepting, rejecting or acting on AI-generated recommendations.

### Verifiable credentials

W3C VC Data Model 2.0 defines a three-party ecosystem of issuers, holders and verifiers. A credential proves that an issuer made a claim under a particular security model; it does not by itself establish that the claim is true, current, legally sufficient or relevant to a particular decision.

**Implication:** Mycelix should distinguish credential validity, issuer authority, claim freshness, revocation status, context and relying-party policy.

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
- NIST AI Risk Management Framework 1.0: https://nvlpubs.nist.gov/nistpubs/ai/NIST.AI.100-1.pdf
- W3C Verifiable Credentials Data Model v2.0: https://www.w3.org/TR/vc-data-model/

## 8. Research integrity

This landscape is a starting point. Standards may have paywalled normative text, jurisdictional interpretations and sector-specific requirements. The summaries above rely on public abstracts and official framework material. Before claiming conformance, obtain and review the applicable full standard, determine its legal relevance, and commission competent independent assessment.
