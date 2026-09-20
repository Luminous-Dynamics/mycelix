# CIV-RES-000A — Civic Resilience composition boundary v1

Status: architecture/qualification contract only  
Program: CIV-RES-000 / issue #2006  
Frozen source base: `main@a85369699099d4c7524e502e531735eed4ab36f4`

## Purpose

Civic Resilience is a composition layer over existing Mycelix institutional, authority, evidence, privacy/accountability, anti-capture, governance, municipal, and scientific boundaries.

This tranche exists to prevent a second civic authority system from emerging inside the resilience program.

It changes no runtime semantics, creates no Holochain zome, grants no institutional capability, processes no Johannesburg resident data, and gives Symthaea no civic authority.

## Core theorem

```text
composition != authority mint
reference to upstream work != qualification inheritance
software capability != institutional legitimacy
model evidence != adopted civic state
```

A Civic Resilience component may consume already-defined semantics, but it may not silently widen their scope, validity period, consequence class, disclosure authority, delegation rights, or effect authority.

Where an upstream dependency is draft, unmerged, unqualified, or not in executable ancestry, CIV-RES records that fact rather than pretending the dependency is active.

## Semantic ownership map

The v1 ownership map is deliberately about semantic responsibility, not current merge status.

| Concern | Semantic owner / line | CIV-RES rule |
|---|---|---|
| Institutional legitimacy, jurisdiction, rulebook, public-institution planes | GOVSYS-002 / PR #784 | Reuse; do not create a Civic sovereign or generic `official` bit. |
| Administrative case/procedure/review/finality | ADMIN-001/002/003 | Compose when an operational flow becomes an administrative decision; do not duplicate procedure/finality. |
| Structural anti-capture and civic standing | AC-* beginning at AC-001 / PR #870 | Reuse refusals; Civic signals never become guilt, sanction, or rights loss automatically. |
| Person-linked access accountability | reciprocal accountability / PR #28 | Protected lookups must compose the access-receipt/accountability boundary; a receipt is not permission. |
| Governance authority provenance and conservation | GOV-AUTH / GOV-CONS architecture / PR #329 | Civic coordination cannot amplify authority through routing, retries, delegation, representation changes, or fallback. |
| Municipal/public-institution integration | municipal/public-institution lines | Civic Resilience may carry municipal references and workflows but does not become the municipal source of legitimacy. |
| Mycelix ↔ Symthaea state/model ownership | REGEN Phase-E ownership theorem / PR #1523 | Mycelix owns adopted state; Symthaea consumes frozen evidence cuts and returns model evidence/proposals. |
| Evidence currentness/qualification | shared evidence/qualification lines | A referenced artifact, run, receipt, or historical PASS must satisfy its own currentness and qualification theorem. |

These references are architectural dependencies only. CIV-RES-000A does not claim that every named line is merged, qualified, mutually converged, or present in this branch's executable ancestry.

## Five composition planes

CIV-RES preserves the public-institution separation and adds no shortcut between planes:

1. `Legitimacy` — who/what institution or community process is entitled to act under which jurisdiction/rulebook.
2. `Authority` — exact scoped capability/delegation/currentness required for a consequential decision.
3. `Evidence` — observations, reports, attestations, verification, uncertainty, provenance, conflicts, and model evidence.
4. `Procedure` — filing, notice, response, review, appeal, finality, and other process requirements when applicable.
5. `Effect` — external operations and their outcome/reconciliation evidence.

A Civic Resilience workflow may connect these planes only through explicit typed evidence/authority edges owned by the appropriate subsystem.

## Required non-equivalences

The following boundaries are normative and machine-checked by this tranche:

```text
CivicObservation != AdministrativeDecision
CivicSignal != AdjudicatedFinding
ServiceRequest != Entitlement
Commitment != CompletionEvidence
CompletionEvidence != IndependentVerification
IndependentVerification != OutcomeEffect
ModelEstimate != Observation
ModelRecommendation != CivicAuthority
ProcurementOpportunity != AwardAuthority
Award != Contract
Contract != Payment
ReportedCrime != TrueCrimeIncidence
LowReportedCrime != ImprovedSafety
AggregateTrend != IndividualRisk
AccessReceipt != PermissionGrant
SoftwareCapability != InstitutionalLegitimacy
QualifiedUpstream != QualifiedComposition
CrossRepoSchema != SharedMutableTruth
HistoricalPass != CurrentQualifiedEvidence
EmergencyContext != PermanentRetentionAuthority
```

## Authority-preserving composition

CIV-RES components must fail closed when composition would strengthen an upstream claim.

Examples:

```text
observation + model score -> review input
```

not:

```text
observation + model score -> sanction
```

```text
service request + verification evidence -> operational/review evidence
```

not:

```text
service request -> entitlement by software fiat
```

```text
procurement opportunity + capability evidence -> referral/proposal
```

not:

```text
opportunity match -> award / contract / payment authority
```

```text
Symthaea analysis receipt -> evidence/proposal
```

not:

```text
Symthaea analysis receipt -> mutate adopted Mycelix state
```

## No implicit qualification inheritance

CIV-RES-000A freezes an explicit rule:

```text
qualified(A) AND qualified(B) != qualified(compose(A, B))
```

A later composition theorem must bind the exact upstream subjects, versions, profiles, bridge schemas, evidence cuts, and qualification receipts actually used.

Likewise:

```text
draft dependency reference != executable dependency
unmerged architecture != active runtime semantics
queued workflow != PASS
source review != executable qualification
```

## Cross-repository boundary

For Civic Resilience, the intended ownership direction is:

```text
Mycelix authoritative/adopted civic state
        |
        | immutable, versioned evidence cut
        v
Symthaea analysis / simulation / counterfactual evaluation
        |
        | evidence-bearing analysis receipt
        v
Mycelix evidence / review / adoption process
```

Symthaea may propose, estimate, simulate, identify uncertainty, expose counterexamples, and compare interventions. It cannot silently mutate the authoritative civic graph or manufacture institutional legitimacy, legal authority, police authority, procurement authority, benefit entitlement, sanction authority, or external-effect authority.

## Privacy/accountability ownership boundary

CIV-RES may define civic data-product metadata and projection rules, but it must not create a parallel person-access authority system.

Protected person-linked access must compose the cross-domain accountability contract. In particular:

```text
AccessReceipt != PermissionGrant
query logged != query lawful
query permitted != downstream disclosure permitted
aggregate release != permission for person-level reconstruction
```

The Johannesburg-specific privacy, retention, cross-border, special-information, safety-reporting, and low-bandwidth deployment profile belongs in CIV-RES-000B, not this universal ownership tranche.

## Runtime and source boundary

This exact tranche is documentation + machine validation only.

It deliberately does not depend on unmerged Rust APIs from the referenced draft lines. Current `main` may have a narrower Civic runtime surface; CIV-RES-000A freezes the future composition contract without pretending those draft APIs already exist in executable ancestry.

No `.rs`, `Cargo.toml`, `Cargo.lock`, DNA, zome, frontend, database, network adapter, model, or policy-engine source belongs in this tranche.

## Next dependency order

```text
CIV-RES-000A  composition/ownership boundary        <-- this tranche
CIV-RES-000B  Johannesburg deployment/threat/privacy profile
CIV-RES-001A  domain-neutral observation/need/intervention/outcome vocabulary
CIV-RES-001B  privacy projection + accountability composition
CIV-RES-001C  commitment -> evidence -> verification -> outcome observation
```

Only after these foundations should service-operation, opportunity, sensitive-safety, Symthaea analysis, and pilot-evidence tranches proceed.

## Deliberate non-features

CIV-RES-000A does not introduce:

- a Civic sovereign, superuser, or master resident database;
- a universal reputation/social-credit score;
- individual criminality or risk scoring;
- autonomous rights-affecting AI decisions;
- police targeting or coercive-act authority;
- procurement award/contract/payment authority;
- entitlement authority;
- person-level cross-domain join authority;
- generic emergency bypass authority;
- permanent emergency-data retention authority;
- Symthaea mutation authority over civic state;
- a replacement administrative-procedure system;
- qualification inheritance from referenced PRs.

## Qualification claim ceiling

A PASS for this exact tranche may establish only that the checked documentation/manifest preserve the frozen ownership map, non-equivalences, composition rules, continuation order, and nonclaims.

It does **not** establish runtime enforcement, upstream dependency qualification, legal validity, POPIA compliance, Johannesburg adoption, municipal legitimacy, public-sector readiness, privacy/security adequacy, causal effectiveness, crime reduction, procurement authority, law-enforcement authority, or deployment readiness.
