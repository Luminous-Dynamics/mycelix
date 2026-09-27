# MYC-INT-017D — Integral implementation delta and dependency-ordered PR plan

Status: design / source-audited planning subject
Observed external sources: 2026-09-27
Tracks: #3202
Parent: MYC-INT-017C / PR #3201

## Purpose

This document converts the current Integral-to-Mycelix coverage audit into an implementation program.

The goal is not to create five new Integral-flavoured Mycelix subsystems. The goal is to compose existing Mycelix owners behind explicitly versioned Integral external contracts while preserving source ownership, derived-state boundaries, governance authority, and replaceability.

```text
Integral external semantics
        +
existing Mycelix domain owners
        +
thin policy/projection adapters
        +
shared delivery/evidence primitives
        =
Integral Reference Solution candidate
```

The following remain non-equivalent:

```text
conceptual overlap
!= implemented adapter
!= locally tested adapter
!= repository-qualified adapter
!= Integral-ratified conformance
```

## 1. External source state

The currently observed Integral public source families are not yet one stable executable contract.

### 1.1 Technical Specifications website

Observed 2026-09-27:

- six `DRAFT` core data structures:
  - `SPEC-DS-01 Certified Design`
  - `SPEC-DS-02 Labor Event`
  - `SPEC-DS-03 Material Consumption Event`
  - `SPEC-DS-04 ITC Ledger Entry`
  - `SPEC-DS-05 FRS Signal Packet`
  - `SPEC-DS-06 Decision Packet`
- three `PENDING` interfaces:
  - `SPEC-IF-01 OAD -> COS`
  - `SPEC-IF-02 COS -> ITC`
  - `SPEC-IF-03 FRS -> CDS`
- `SPEC-STACK-01` remains `PENDING`.

The website says these specifications are the builder-facing source of truth, but their current statuses explicitly remain DRAFT/PENDING.

### 1.2 `integral-specifications`

Observed `README.md` blob:

`e27b9a67a1de841d4c0f69c359935bf77a5d2de2`

The repository currently contains no ratified per-schema/interface artifacts. Its README says authoritative schema output will live there after the schema-design exercise.

Therefore:

```text
website field list
!= ratified schema artifact in integral-specifications
```

### 1.3 Development Guide

Observed `DEVGUIDE.md` blob:

`a99f095eae0790014163fd932488baf566ce4e6b`

The Development Guide defines:

- richer versions of several data structures;
- support objects such as `ITCAccount`, `DiagnosticFinding`, and `Recommendation`;
- twelve primary cross-system data contracts;
- proposed interface signatures.

It explicitly says those interface signatures are Development-Guide architectural proposals subject to community ratification.

### 1.4 Five system repositories

Observed README blobs:

| System | Blob | Current public statement |
|---|---|---|
| CDS | `97ff81fb838dcabd6e78fa3a2f35dec230a0b7e8` | No implementation has begun |
| OAD | `13d38bbdbc86b558f37201a11f23f56a6387002b` | No implementation has begun |
| ITC | `9b8071cb0fbb914bb6a55485cd38a59302a1f449` | No implementation has begun |
| COS | `4f50d464ae49da98c52f2dd3815df01de61143bd` | No implementation has begun |
| FRS | `73b601bd5727f41d85c00578ed4592c7f56e20fd` | No implementation has begun |

### 1.5 Decision repository

Observed `integral-decisions/README.md` blob:

`f90217377f80d55568a0c97c78b1ecb1187fb631`

No Decision Records have yet been ratified. The repository defines ratified records as immutable and superseded rather than edited.

### 1.6 Holochain status

The Development Guide describes Holochain as a reference technology / potential technical component, not a committed architectural choice.

The current Integral website repository additionally calls Holochain the leading candidate for the future node-network web-presence layer. This is useful alignment evidence but does not supersede the pending whole-stack decision.

Therefore:

```text
Holochain architectural fit
!= Holochain selected as Integral core runtime
```

## 2. Source identity and drift rule

Every Integral compatibility object must bind a source generation.

At minimum:

```text
ExternalSchemaSource {
    source_family,
    source_status,
    source_revision,
    observed_at,
    schema_name,
    schema_generation
}
```

The exact type may reuse the existing semantic/source-reference line rather than creating a parallel identity system.

Required theorem:

```text
external source changed
!= adapter automatically updated
!= adapter automatically requalified
```

A future DRAFT -> RATIFIED transition is a new source fact and may be a new schema generation even when the visible object name stays the same.

## 3. Existing Mycelix reuse baseline

### 3.1 CDS substrate

Current Mycelix Governance code already includes concrete proposal, amendment, discussion, execution, timelock, voting, constitutional, council, jurisdiction and related surfaces.

The inspected proposal integrity source includes:

- `Proposal`
- `ProposalAmendment`
- `DiscussionContribution`
- `DiscussionReflection`
- explicit status transitions and version checks.

The execution integrity source includes:

- `Timelock`
- `Execution`
- explicit result/error state
- veto/override machinery.

These are reusable implementation mechanics.

They are not Integral CDS policy.

Integral's current CDS repository describes consensus without majority voting, whereas current Mycelix Governance includes MATL/stake/quadratic-voting policy defaults.

Therefore:

```text
Mycelix governance storage/lifecycle
= reusable

Mycelix voting policy
!= Integral CDS policy
```

### 3.2 OAD and COS substrate

Current Fabrication contains design/version/material/verification/print/Symthaea integration surfaces.

Current Manufacturing canonical types include:

- `WorkOrder`
- `BillOfMaterials`
- `BomItem`
- `Operation`
- `RoutingSequence`
- `RoutingStep`
- `Machine`
- `MrpResult`
- `PlannedOrder`
- `ScheduledOperation`
- `CapacityWarning`
- `MaterialShortage`.

Commons contributes real resource-domain owners for water, food, property, housing, transport, care and mutual aid.

Craft contributes skills, credentials/work history and labour-market/work-profile primitives.

This makes OAD/COS primarily a composition-and-policy problem, not a greenfield production platform problem.

### 3.3 ITC substrate

Mycelix Finance already supplies mature ledger/account/economic persistence patterns, but its policy is explicitly the Mycelix three-currency system.

```text
Integral ITC
!= MYCEL
!= SAP
!= TEND
```

Reuse durable mechanics only where neutral.

### 3.4 FRS substrate

Mycelix already has:

- source-domain observations;
- evidence/provenance identities;
- Knowledge claims/attestations/derived confidence;
- optional Symthaea analysis;
- source-owned vs derived-state work.

What is missing is the exact Integral FRS assembly and its routing/currentness contracts.

## 4. Field-ownership corrections

### 4.1 LaborEvent

The Development Guide explicitly states that the white paper separates raw `LaborEvent` from `WeightedLaborRecord`, while the Development Guide consolidates them for Phase-2 convenience.

The consolidated record includes ITC outputs such as weighting fields and `itc_credits_issued`.

Internal model:

```text
COSLaborObservation
    participant/task/time/verification/ecological source facts

ITCWeightingAssessment
    policy generation + weighting factors/result

ITCCreditIssuance
    exact ledger/account effect
```

Compatibility projection:

```text
Integral LaborEvent view
= COSLaborObservation
+ optional ITCWeightingAssessment
+ optional ITCCreditIssuance
+ field-origin/provenance map
```

Never infer that COS authored ITC credit results merely because the convenience external record contains them.

### 4.2 CertifiedDesign

The current external design package includes `itc_access_cost`, while ITC is responsible for access-cost logic.

Treat the field as a bound derived snapshot:

```text
CertifiedDesign.itc_access_cost
    -> ITC-derived value
    -> policy/source generation
    -> OAD package inclusion
```

not:

```text
OAD owns value policy
```

### 4.3 FRSSignalPacket

The Development Guide states that the human-readable packet expands the white paper's abstract signal-envelope model.

Internal ladder:

```text
SourceObservation
-> NormalizedSignalEnvelope
-> DerivedSummary
-> FRSSignalPacket
-> DiagnosticFinding
-> Recommendation
```

Every step retains derivation/provenance/currentness.

### 4.4 Decisions

Integral distinguishes:

- issue/proposal/deliberation;
- immutable ratified Decision Record;
- DecisionPacket used for dispatch;
- implementation/effect.

Mycelix must additionally preserve its existing explicit authorization boundary.

```text
Proposal
!= RatifiedDecisionRecord
!= DecisionPacket
!= Authorization
!= ExecutionAttempt
!= ImplementationReceipt
!= OutcomeObservation
!= ReviewCandidate
```

## 5. Twelve-seam map

The Development Guide defines twelve proposed primary data contracts.

| # | Integral seam | Mycelix owner direction | Main adapter work |
|---:|---|---|---|
| 1 | OAD -> COS Certified Design Package | Fabrication/Manufacturing -> production | certification/version/projection |
| 2 | OAD -> ITC Design Intelligence | Fabrication/Manufacturing -> Integral ITC | labour/material/ecological design inputs |
| 3 | OAD -> FRS Design Event | OAD source facts -> FRS | design assumption/currentness/supersession signals |
| 4 | FRS -> OAD Operational Recalibration | FRS derived evidence -> OAD review | advisory recalibration/review trigger, no certification authority |
| 5 | COS -> ITC Labor/Materials | Manufacturing/Commons/Craft -> ITC | raw-vs-derived ownership; idempotency |
| 6 | COS -> FRS Operational Signal | source domains -> FRS | normalized source envelopes/currentness |
| 7 | ITC -> FRS Credit/Access | Integral ITC ledger -> FRS | frontier-bound account/ledger summaries |
| 8 | FRS -> CDS Sensemaking | FRS derived artifacts -> CDS | recommendation/issue-candidate routing, no authority |
| 9 | CDS -> FRS Governance Signal | decision/history -> FRS | intended outcome/review basis + governance history |
| 10 | CDS -> OAD Design Mandate | CDS decision + separate authority -> OAD | mandate envelope/policy generation |
| 11 | CDS -> COS Production Mandate | CDS decision + separate authority -> COS | authorization-bound operational envelope |
| 12 | CDS -> ITC Policy Signal | CDS policy decision -> ITC | policy snapshot/version/currentness |

Only three of these are currently listed as PENDING `SPEC-IF-*` objects on the public Technical Specifications surface. The remaining nine remain Development-Guide proposals in Mycelix source metadata until Integral says otherwise.

## 6. Shared delivery model

Do not build a second Integral message bus.

Reuse the provider-neutral semantic seam and durable-effect work, especially where applicable:

- #3142 — versioned semantic seam/delivery profile;
- #1535 — crash-consistent transition/effect outbox;
- #803 — durable attempt/effect receipt composition;
- #746 — prepared transport journal boundary;
- #1150 — replaceable intermediary non-authority model.

Universal distinction:

```text
semantic object
!= delivery attempt
!= transport/provider acceptance
!= endpoint receipt
!= semantic admission
!= policy acceptance
!= authorization
!= external effect
```

This distinction applies equally to PostgreSQL/HTTP, Holochain signals/remote calls, Xenia transport, or a later alternate runtime.

## 7. Dependency-ordered PR program

### 017E — provisional external DTO/profile layer (#3203)

First executable schema tranche.

Purpose:
- encode the six current Technical-Specification DRAFT shapes;
- encode required Development-Guide support objects under a distinct namespace/status;
- bind exact source/status/generation;
- strict serde and unknown-field behavior;
- no authority behavior.

This is the prerequisite for every later adapter.

### 017F — CertifiedDesign composition (#3204)

Compose OAD projection from existing Fabrication + Manufacturing owners.

Do not create a new CAD/design repository.

### 017G — COS labor/material source events (#3205)

Create the source-owned event boundary before ITC so later accounting cannot redefine operational history.

### 017H — Integral ITC adapter (#3206)

Implement Integral accounting semantics using neutral durable mechanics only.

### 017I — FRS assembly + optional Symthaea (#3207)

Create packet/finding/recommendation assembly with strict source/derived/non-authority semantics.

### 017J — CDS DecisionRecord/DecisionPacket (#3208)

Project Integral decision semantics over reusable governance infrastructure without importing Mycelix voting policy.

### 017K — twelve-seam delivery profiles (#3209)

Bind the above semantic objects through shared versioning/retry/acknowledgment/receipt infrastructure.

### 017L — five-system I0 loop (#3210)

Run the complete water/resource loop through the same oracle-hidden evaluation methodology used by the interoperability work.

### 017M — Leptos product binding (#3211)

Bind qualified/read-model outputs into the participant/operator experience.

### 017N — external source-drift monitor (#3213)

Detect external Integral source/status changes and invalidate compatibility assumptions without automatic rewriting.

### 017O — federation/privacy/foreign recognition (#3215)

Define what crosses node boundaries and what foreign objects can mean locally.

### 017P — expertise/certification standing (#3214)

Address OAD technical-expertise authority explicitly rather than collapsing reputation/credential/expertise/standing.

### 017Q — exact qualification umbrella (#3216)

Every executable tranche gets exact-head qualification and retained source/environment evidence.

## 8. R1 critical path

```text
017E external schema/profile layer
        |
        +--> 017F OAD CertifiedDesign
        |
        +--> 017G COS source events
                 |
                 v
             017H ITC
                 |
                 +----------+
                 |          |
                 v          v
             017I FRS    017J CDS
                 \          /
                  \        /
                   v      v
                 017K interfaces
                       |
                       v
                 017L I0 five-system loop
                       |
                       v
                 017M Leptos product
```

Cross-cutting:

```text
017N source drift
017O federation/privacy
017P expertise standing
017Q qualification
```

## 9. Runtime plan

Do not couple semantic completion to one runtime.

Recommended order:

1. dependency-light Rust semantic/reference types;
2. PostgreSQL conventional service implementation;
3. independent protocol conformer;
4. Holochain/Mycelix implementation after required semantic/runtime gates pass;
5. hybrid implementation where PostgreSQL is a rebuildable/local query projection rather than a shadow semantic owner.

```text
PostgreSQL row ID
!= SemanticRef

Holochain ActionHash
!= Integral object identity

projection database
!= source of truth by convenience
```

## 10. Symthaea plan

Symthaea is optional and typed.

```text
AnalysisRequest
-> Symthaea
-> AnalysisArtifact
-> explicit provenance/derivation
-> optional FRS finding/recommendation
```

Never:

```text
Symthaea output
-> Decision
-> Authorization
```

without the explicit governance/authority path.

Candidate analytical roles:

- FRS diagnostics;
- prediction/counterfactuals;
- uncertainty/sensitivity analysis;
- OAD design/material/process exploration;
- production optimization candidates;
- anomaly detection;
- outcome review.

## 11. Future deployment-critical boundary: Interface Cooperative

Tracked separately as #3212.

The Development Guide treats the Interface Cooperative as the primary bridge for early real-world nodes interacting with existing legal/market infrastructure.

It should compose existing Mycelix external commerce/supply-chain/finance/legal/identity tools without becoming a sixth internal Integral semantic owner.

## 12. Qualification doctrine

For each tranche:

```text
Designed
!= SourceImplemented
!= LocallyTested
!= RepositoryQualified
!= PilotObserved
```

Each exact qualifier must pin:

- subject SHA;
- parent SHA;
- external source/profile generations;
- allowed changed files;
- tests/mutations;
- environment/toolchain capsule;
- deterministic evidence manifest.

Source drift creates a successor qualification subject. It does not erase historical PASS evidence for the pinned predecessor.

## 13. Immediate next implementation

The next code PR should be **017E**, not a large all-five-systems implementation.

017E can safely establish:

- source-status enum/profile;
- external schema namespace;
- the six current DRAFT DTOs;
- devguide support DTO namespace;
- strict parsing/serialization controls;
- field-origin metadata for mixed-owner convenience objects;
- negative authority conversions.

Then 017F and 017G can proceed mostly independently.

This maximizes parallelism while preserving the most important semantic boundaries before policy logic is added.

## Nonclaims

This plan does not claim:

- Integral has ratified the current public DRAFT structures;
- the Development Guide's nine additional seams are Technical Specifications;
- Integral has selected Mycelix, Holochain, PostgreSQL, Symthaea, Xenia, or Nix;
- Mycelix's governance/economic defaults are Integral defaults;
- synthetic technical qualification establishes real-world economic/social/political efficacy.
