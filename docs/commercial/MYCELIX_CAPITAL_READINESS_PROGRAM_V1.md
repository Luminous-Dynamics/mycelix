# Mycelix Capital Readiness Program v1

Status: Draft program overview  
Tracking: #880, #881, #882  
Related technical roadmap: #805

## 1. Objective

The goal is not to optimize Mycelix for fundraising in the abstract.

The goal is to make productive capital easy to deploy into Mycelix while preserving the authority boundaries that make the network trustworthy.

```text
qualified trust fabric
-> controlled pilots
-> independently verifiable outcomes
-> repeatable operations
-> appropriate capital
-> wider adoption
```

Capital accelerates a validated system; it does not become evidence that the system is correct.

## 2. Three coupled program tranches

### MYC-CAP-001 — Capital Constitution

Defines:

- what capital can own;
- what it can influence;
- what it cannot unilaterally control;
- how public/common infrastructure remains neutral;
- how financing provenance and control rights remain auditable.

Normative draft: `docs/capital/MYCELIX_CAPITAL_CONSTITUTION_V1.md`.

### MYC-COM-001 — Trust Fabric commercial projection

Defines:

- Mycelix Authority;
- Mycelix Evidence;
- Mycelix Federation;
- AGENT-to-product mapping;
- qualification vs non-authoritative conformance lanes;
- controlled design-partner workflows;
- commercial maturity gates.

Normative draft: `docs/commercial/MYCELIX_TRUST_FABRIC_V1.md`.

### MYC-EVID-001 — Evidence Manifest

Defines:

- technical evidence;
- security evidence;
- commercial evidence;
- capital evidence;
- claim authority classes;
- staleness/currentness;
- deterministic investor/customer status projections.

Normative draft: `docs/evidence/MYCELIX_EVIDENCE_MANIFEST_V1.md`.

## 3. Program dependency shape

```text
existing qualified generic authority
        |
        v
AGENT roadmap (#805)
        |
        +---------------------------+
        |                           |
        v                           v
Trust Fabric                  Evidence Manifest
commercial projection         machine-current truth
        |                           |
        +-------------+-------------+
                      v
             controlled paid pilots
                      |
                      v
               measured outcomes
                      |
                      v
               capital readiness
                      |
                      v
                larger-scale growth

Capital Constitution wraps the whole program and prevents financing rights
from silently becoming protocol authority.
```

## 4. Do not create a parallel commercial authority stack

The AGENT roadmap already owns the core semantics required for a credible commercial authority product.

Commercial work should therefore focus on:

- packaging;
- managed operations;
- external standards adapters;
- deployment tooling;
- assurance;
- design-partner integrations;
- measurable customer outcomes.

It should not invent alternate `PrincipalId`, grant, delegation, currentness, action, broker or effect semantics.

## 5. Two-speed execution

### Technical qualification

Continue the exact AGENT dependency order. A failed or pending theorem remains unavailable to production authority.

### Commercial learning

In parallel, allow non-authoritative conformance experiments and controlled workflow discovery.

This lane may exercise external protocols, enterprise identity systems and agent frameworks, but must be incapable of minting production authority before the required theorem stack qualifies.

## 6. Capital-readiness phases

| Phase | Engineering evidence | Commercial evidence | Appropriate capital focus |
|---|---|---|---|
| C0 Foundation | IP/licensing/status boundaries coherent | narrow product narrative | founder/grants |
| C1 Qualified core | AGENT authority kernel qualifying | simulation/design partners | grants/design-partner funding |
| C2 Controlled pilot | broker/effects/receipt/verifier path | first paid bounded pilot | seed/deep-tech |
| C3 Bounded production | external security assurance current | 3-5 paying organizations, measurable value | institutional/strategic venture |
| C4 Federation | TCK + multi-org evidence | repeatable deployment and expansion | Series A/growth/strategic |
| C5 Sovereign | sovereign/public-infrastructure profiles | municipal/national pilots | DFI/public/strategic/project capital |
| C6 Ecosystem | independent implementations/conformance | vendor and federation ecosystem | infrastructure-scale capital |

These are readiness phases, not valuation targets.

## 7. Immediate work order

The near-term order should be:

1. keep AGENT exact qualification moving without weakening its evidence discipline;
2. reconcile repository licensing/IP inconsistencies before investor diligence;
3. freeze Trust Fabric product language around Authority, Evidence and Federation;
4. build non-authoritative enterprise/standards translation fixtures in parallel;
5. define the canonical Evidence Manifest schema and generated status table;
6. recruit bounded design partners only when the pilot can be explicit about what is and is not qualified;
7. instrument every pilot with preregistered baseline and outcome metrics;
8. obtain external security review for the narrow production boundary before R4 promotion;
9. route public-good/sovereign work to non-VC capital where appropriate;
10. raise larger institutional capital only after deployment repeatability and customer value are visible.

## 8. Investor-facing thesis

The desired concise narrative is:

> Mycelix is an open trust fabric for humans, organizations and autonomous agents. It separates identity, delegated authority, exact action permission, durable effect evidence and independent verification. Enterprises pay for managed operation, integration and assurance; the underlying protocol remains designed for federation and independently verifiable interoperability rather than centralized lock-in.

This narrative is a commercial projection. Each technical clause must remain bounded by the Evidence Manifest.

## 9. Success criteria

The program is succeeding when:

- product language becomes simpler while technical semantics remain precise;
- investor/customer claims are generated from current evidence rather than stale prose;
- external standards increase interoperability without becoming hidden trust roots;
- each design-partner deployment produces reusable product artifacts;
- customer value is measured against explicit baselines;
- capital can receive meaningful economic upside without silently gaining protocol sovereignty;
- public/neutral infrastructure can be financed without forcing all costs onto venture equity;
- federation value grows without requiring centralized ownership of participant institutions or data.

## 10. Nonclaims

This program does not establish:

- that Mycelix has product-market fit today;
- that any financing round will close;
- any valuation;
- any legal entity structure before counsel review;
- regulatory compliance;
- technical PASS beyond exact underlying evidence;
- wisdom or safety of an AI decision merely because its authority was valid.

The program's purpose is convergence: **qualified research -> narrow product -> evidence-bearing deployment -> repeatable economics -> compatible capital.**
