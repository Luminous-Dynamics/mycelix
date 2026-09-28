# Integral Economic OS Interoperability Contract v1

Status: engineering reference / bounded interoperability target.

## Purpose

The Economic Fabric should make Integral interoperable **without flattening Integral into a generic currency or generic transaction model**.

Current public Integral materials describe five coupled systems:

- CDS — governance/decision intelligence
- OAD — design intelligence
- COS — production/operational intelligence
- ITC — contribution/access accounting
- FRS — feedback/review intelligence

The Development Guide presents these as a minimal, faithful build direction rather than a completed implementation. citeturn0search14turn0search12

This contract therefore treats Integral as a source-owned economic architecture and defines the Mycelix boundary around it.

## Canonical economic OS flow

```
Need / Decision
    ↓
Certified Design
    ↓
Production Plan
    ↓
Observed Work + Material Flow
    ↓
Evidence / Verification
    ↓
Economic Event
    ↓
Policy / Valuation
    ↓
ITC or other Entitlement
    ↓
Authorization
    ↓
Settlement / Access Effect
    ↓
Outcome Observation
    ↓
FRS Feedback
    ↓
Correction / Revaluation / Redesign
```

The important distinction is that **ITC is not itself the universal economic event layer**.

Mycelix's Economic Fabric sits underneath the domain-specific policy:

```
economic event
    ├── Integral ITC policy
    ├── mutual-credit policy
    ├── Valueflows policy
    ├── conventional accounting policy
    ├── resource-credit policy
    └── other local instruments
```

## Integral semantic boundaries

### CDS

A CDS decision is a governance artifact.

```
recommendation != decision
decision != authorization
authorization != effect receipt
```

A decision cannot be inferred merely because an economic event exists.

### OAD

OAD design/certification is design authority and provenance.

```
design != production
certification != production authorization
production authorization != production observation
```

A certified design can constrain COS production, but the design is not evidence that production occurred.

### COS

COS is the operational observation boundary.

```
plan != execution
declared work != observed work
planned material != consumed material
useful output != safety qualification
one successful run != general capability
```

This directly connects Integral COS with Mycelix ProductiveLoopV1 and its evidence/temporal validity rules.

### ITC

ITC is a domain-specific contribution/access policy.

```
labor observation != ITC entitlement
ITC entitlement != generic currency
ITC != legal tender
ITC != market price
ITC != universal store of value
```

The adapter may represent an ITC-derived entitlement, but must preserve the exact Integral policy and source provenance.

### FRS

FRS is a feedback/diagnostic boundary.

```
FRS finding != source observation
FRS recommendation != decision
FRS recommendation != authorization
prediction != observation
confidence != calibrated probability
```

Public Integral material describes FRS as aggregating ecological, production, contribution/access, node, dependency, and member-feedback signals and generating recommendations back into the other systems. citeturn0search13

## Economic event algebra

The Economic Fabric uses these source-neutral primitives:

- Observation
- Evidence
- EconomicEvent
- Obligation
- Entitlement
- Offer
- Commitment
- Allocation
- Authorization
- Settlement
- Dispute
- Correction
- EconomicOutcome

These are **semantic primitives**, not a mandatory implementation schema.

Valueflows provides a useful external interoperability target because it explicitly distinguishes Intent, Commitment, Claim, and EconomicEvent, and models corrections without mutating the original event. citeturn0search0turn0search2turn0search3

## Economic transition contract

Allowed high-level transitions:

```
Observed
  → Evidenced
  → Valuated
  → Entitled
  → Authorized
  → Submitted
  → Settled
  → OutcomeObserved
  → Corrected
```

Important forbidden shortcuts:

```
Observed → Entitled
Observed → Authorized
Observed → Settled
LaborObserved → ITC
ProductionObserved → Compensation
Recommendation → Authorization
Intent → EconomicEvent
Commitment → Settlement
Recognition → LocalIssuance
Correction → HistoricalMutation
```

The exact allowed transitions remain policy/profile-specific; the generic fabric only enforces semantic boundaries.

## Integral ↔ Economic Fabric adapter

Required source trace:

```
Integral source ID
→ source type
→ source version
→ source evidence refs
→ adapter profile
→ Mycelix EconomicEvent ID
→ target policy/instrument
→ authorization
→ settlement/effect receipt
```

The adapter must preserve:

- source identity
- source ownership/origin
- evidence references
- quantities and units
- temporal validity
- correction lineage
- policy version
- authority reference
- target instrument identity

## Reversibility requirement

Every economic projection must answer:

> “Which source observation or decision produced this economic meaning?”

and:

> “What economic meaning was added by the target policy that did not exist in the source?”

This is the economic equivalent of the semantic-loss ledger.

## New economic OS invariant

Define:

```
EconomicMeaning
=
SourceEvidence
+
ExplicitPolicy
+
ExplicitAuthority
+
ExplicitSettlement
```

No single term may be silently substituted for another.

In particular:

```
Evidence ≠ Policy
Policy ≠ Authority
Authority ≠ Settlement
Settlement ≠ Outcome
```

## Conformance target

A future executable conformance harness should run the same source event through:

```
Integral
→ Mycelix Economic Fabric
→ Valueflows
→ TEND
→ accounting
```

and verify:

1. source identity preserved;
2. origin preserved;
3. quantity/unit preserved;
4. validity preserved;
5. correction lineage preserved;
6. claim ceiling does not widen;
7. no authority is created by observation alone;
8. no settlement is created by transport alone;
9. duplicate settlement remains idempotent;
10. corrections append lineage rather than rewriting history.

## Claim ceiling

This contract establishes only a **portable semantic interoperability target**.

It does not establish:

- Integral implementation completeness;
- ITC policy correctness;
- financial performance;
- economic fairness;
- legal/regulatory status;
- accounting compliance;
- physical production;
- settlement finality;
- governance legitimacy.
