# MYC-INT-007C — Integral → Mycelix source-aware crosswalk

Status: mapping / architecture artifact. This document does **not** make Integral normative for Mycelix and does not select Mycelix as Integral's preferred implementation.

Tracks #3194. Parent contribution package: #3191 / PR #3193.

Observed source snapshot: 2026-09-27.

## 1. Purpose

Map Integral's current public document families, five systems, current Phase-0 data structures, and cross-system seams onto existing Mycelix semantic/domain owners without creating duplicate subsystems or importing Integral-specific policy into neutral Mycelix core.

The target theorem is:

```text
Integral concept
+ exact source/status
+ explicit mapping class
+ existing Mycelix owner(s)
+ preserved non-equivalences
= adapter composition candidate
```

not:

```text
similar name/purpose
= same type
```

The expected implementation shape is therefore:

```text
Integral documents/specifications
          │
          ▼
Integral adapter-owned schemas/policy
          │
          ▼
neutral Mycelix interoperability waist
SchemaRef / SemanticRef / provenance / authority / receipts
          │
          ▼
existing Mycelix domain owners
          │
          ▼
optional Holochain / PostgreSQL / hybrid runtime
```

## 2. Source authority ladder

Current Integral public source families are not interchangeable.

| Source family | Current role | Mapping treatment |
|---|---|---|
| White Paper v0.1 | Conceptual / normative architecture reference | `ExternalNormativeSemantics` + source provenance |
| Development Guide v0.1 | Builder bridge / implementation proposal | adapter hypotheses, seam inventory, fixture candidates |
| Technical Specifications | Builder-facing source of truth by individual spec status | adapter schema/interface authority at each DRAFT/PENDING/RATIFIED status |
| Decision Record | Project architecture/governance decision provenance | source lineage; not local Integral-node authority by default |
| Q&A / system pages | Explanatory context | `NoMapping` unless promoted into a technical contract |
| Node Preparation Guide | Operational/pilot context | pilot/readiness mapping, not semantic type authority |

Preserve:

```text
white-paper concept
!= development-guide proposal
!= technical specification
!= implementation
!= conformance evidence
```

Source URLs currently used by the Integral program:

- `https://integralcollective.io/documents/`
- `https://integralcollective.io/documents/devguide.html`
- `https://integralcollective.io/documents/specifications.html`
- `https://integralcollective.io/documents/faq.html`
- `https://integralcollective.io/community/node-guide.html`

Exact external bytes/passages should ultimately be captured through the qualified EPI/WEB-CAPTURE path rather than ad-hoc URL hashes.

## 3. Mapping classes

Every row in this crosswalk uses one or more explicit classes.

| Class | Meaning |
|---|---|
| `ReuseNeutralPrimitive` | Existing neutral Mycelix primitive is directly composable without importing Integral policy |
| `DomainAdapter` | Integral object remains Integral-owned; adapter composes one or more existing Mycelix domain capabilities |
| `DerivedProjection` | Read model / summary derived from a source-owned domain; never becomes new source authority |
| `ExternalNormativeSemantics` | Integral-specific economic/governance rules remain outside neutral Mycelix core |
| `GapCandidate` | Possible generic primitive only after an ownership census proves no existing owner fits |
| `NoMapping` | Explanatory/contextual source content, not a technical implementation contract |

Default rule:

```text
when in doubt:
External/Adapter first
Generic core primitive only after proof of reuse need
```

## 4. Mycelix owner inventory used by this mapping

This mapping is grounded in current repository surfaces, not only names.

### Shared / ecosystem infrastructure

- `mycelix-workspace` — orchestration, unified hApp, SDKs, bridge infrastructure, test harnesses.
- shared `crates/` + existing interoperability work — semantic identity, evidence/provenance, authority/effect boundaries, transport/delivery receipts.
- `SchemaRef` / `SemanticRef` line — namespace-safe external semantic identity.

### Governance / civic

- `mycelix-governance` — proposals, voting/delegation, execution/timelock, constitution.
- `mycelix-civic` — justice evidence/enforcement, emergency incidents/triage/resources/coordination, media fact-checking/attribution.

### Identity / credentials

- `mycelix-identity` — DIDs, credential schemas/versioning, credential status/revocation, recovery.
- `mycelix-craft` — opt-in professional/skill profiles, skill/work-history attestations and credential pointers.
- `mycelix-praxis` — credential/privacy/provenance patterns that may be reusable but remain education-owned.

### Production / resources

- `mycelix-manufacturing` — BOM, planning, machines, operations, work orders, manufacturing bridges.
- `mycelix-commons` — property, housing, care, mutual aid, water, food, transport source domains and cross-cluster bridge.
- marketplace / supply-chain surfaces where deployment scope requires them.

### Economics / contribution

- `mycelix-finance` — MYCEL/SAP/TEND domain economics, accounting/payment/treasury/bridge infrastructure.
- `mycelix-attribution` — immutable usage receipts, attestations, reciprocity pledges/metrics.
- commons timebank domains — care/mutual-aid time exchange where locally relevant.

### Knowledge / evidence / environmental observations

- neutral EPI/provenance lines — source/evidence identity, derivation and capture boundaries.
- `mycelix-knowledge` — claims, attestations, confidence-derived views (kept namespace-separated from core epistemic semantics).
- `mycelix-climate` — emissions, verification, ecological/environmental records.
- `mycelix-energy` — project/production/consumption/operational source records.
- `mycelix-commons` water/food/transport observations.
- manufacturing operational observations.
- optional Symthaea bridge — analysis/simulation/recommendation only.

## 5. Five-system crosswalk

### 5.1 CDS — Collaborative Decision System

Integral role: governance/deliberation/decision dispatch.

Primary Mycelix composition:

| Integral capability | Mycelix owner | Class | Boundary |
|---|---|---|---|
| issue/proposal lifecycle | `mycelix-governance` + neutral governance refs | `DomainAdapter` | Integral issue taxonomy/process remains Integral-owned |
| deliberation/objections | governance + evidence/provenance | `DomainAdapter` | current Mycelix vote defaults are not Integral consensus semantics |
| decision rationale/evidence | governance + EPI/knowledge refs | `ReuseNeutralPrimitive` + `DomainAdapter` | evidence supports decision; evidence is not authority |
| delegation | governance + shared authority/delegation semantics | `DomainAdapter` | Integral delegation rules remain Integral-owned |
| decision record | governance decision lineage | `DomainAdapter` | `Decision != Authorization` |
| dispatch to other systems | seam/delivery contract + domain adapters | `ReuseNeutralPrimitive` | delivery does not strengthen authority |
| emergency path | civic emergency + bounded authorization profile | `DomainAdapter` + `ExternalNormativeSemantics` | emergency condition alone mints no unlimited authority |
| implementation/effect execution | authority/effect infrastructure + receiving domain | `ReuseNeutralPrimitive` | CDS decision does not itself prove effect completion |

Required anti-collapse:

```text
Integral CDS governance model
!= current Mycelix MATL/stake weighting
!= quadratic voting default
!= Mycelix token/economic policy
```

Reuse substrate; do not import policy defaults.

### 5.2 OAD — Open Access Design

Integral role: open design commons, design evaluation/certification, production handoff.

Primary Mycelix composition:

| Integral capability | Mycelix owner | Class | Boundary |
|---|---|---|---|
| design identity/version | SemanticRef/SchemaRef + provenance | `ReuseNeutralPrimitive` | design ID is not ActionHash/database ID |
| bill of materials | manufacturing `bom` | `DomainAdapter` | OAD BOM schema remains Integral-owned |
| production steps | manufacturing planning/operations/workorders | `DomainAdapter` | plan acceptance remains COS/OAD policy |
| machines/process refs | manufacturing machines/operations | `DomainAdapter` | runtime machine record != certification |
| designer/certifier identity | identity + credentials | `DomainAdapter` | credential valid != OAD certification accepted |
| design evidence/rationale | EPI + knowledge | `DomainAdapter` | claim/confidence != certification decision |
| ecological evidence | climate/energy/commons source domains | `DerivedProjection` | derived score != source observation |
| skill/qualification refs | craft + identity | `DomainAdapter` | skill attestation != design authority |
| certification policy | Integral adapter | `ExternalNormativeSemantics` | never generalized into Mycelix core without independent need |

Do not create a new Mycelix OAD repo. Compose manufacturing + evidence + identity + domain observations.

### 5.3 ITC — Integral Time Credits

Integral role: contribution/access accounting under Integral-specific economic rules.

Primary Mycelix composition:

| Integral capability | Mycelix owner | Class | Boundary |
|---|---|---|---|
| participant identity | identity | `ReuseNeutralPrimitive` | identity != balance/standing/authority |
| contribution source refs | manufacturing/craft/commons/attribution | `DomainAdapter` | work/resource fact != economic valuation |
| append-only accounting record | finance/ledger/receipt infrastructure | `DomainAdapter` | Integral ledger semantics remain Integral-owned |
| access accounting | Integral ITC adapter over finance primitives | `ExternalNormativeSemantics` | do not map to existing currencies |
| decay rules | Integral ITC adapter | `ExternalNormativeSemantics` | SAP/TEND/MYCEL rules are unrelated defaults |
| cross-node recognition | federation + adapter recognition rules | `DomainAdapter` | foreign balance/credit != local authority |
| contribution/privacy attestations | attribution/identity/ZK patterns when justified | `DomainAdapter` | proof of contribution != universal value |

Critical theorem:

```text
Integral ITC
!= MYCEL
!= SAP
!= TEND
!= generic Mycelix currency
!= reputation
```

Existing Mycelix finance gives implementation techniques and reusable infrastructure, not semantic identity.

### 5.4 COS — Cooperative Organization System

Integral role: production, labor/material coordination, execution of production mandates.

Primary Mycelix composition:

| Integral capability | Mycelix owner | Class | Boundary |
|---|---|---|---|
| production plan | manufacturing planning | `DomainAdapter` | CDS/OAD mandate != manufacturing completion |
| work order | manufacturing workorders | `DomainAdapter` | work order != labor evidence automatically |
| operation execution | manufacturing operations | `DomainAdapter` | command/registration != real-world outcome |
| machines/capabilities | manufacturing machines | `DomainAdapter` | capability record != availability theorem |
| materials/BOM | manufacturing BOM + commons/supply-chain source refs | `DomainAdapter` | COS consumes design/material refs; does not rewrite source lineage |
| shared physical resources | commons domains | `DomainAdapter` | commons state remains source-owned |
| worker skills/history | craft + identity | `DomainAdapter` | credential != assignment authorization |
| labor/material event emission | COS adapter with source provenance | `DomainAdapter` | events feed ITC/FRS but remain COS-owned facts |
| cooperative organizational policy | Integral COS adapter | `ExternalNormativeSemantics` | no automatic import of Mycelix domain governance defaults |

### 5.5 FRS — Feedback & Review System

Integral role: cross-system sensing, diagnosis, recommendation, review loop.

Primary Mycelix composition:

| Integral capability | Mycelix owner | Class | Boundary |
|---|---|---|---|
| source observations | source domain + EPI refs | `ReuseNeutralPrimitive` | source remains owner |
| cross-domain snapshot | explicit source-frontier projection | `DerivedProjection` | snapshot != current truth without bounded currentness |
| diagnostic finding | knowledge/EPI derived assessment adapter | `DerivedProjection` | finding != observation |
| confidence label | Integral FRS adapter | `ExternalNormativeSemantics` | Integral low/medium/high != Mycelix verification/reliability/probability |
| recommendation | recommendation/evidence semantic line | `DomainAdapter` | recommendation has no execution authority |
| signal packet | evidence/assessment bundle + source refs | `DerivedProjection` | bundle does not own underlying facts |
| outcome review | decision review basis + outcome observations | `DomainAdapter` | later reviewer cannot rewrite original success criteria |
| analysis/simulation | optional Symthaea bridge | `DerivedProjection` | model output != observation/authority |

Critical chain:

```text
source observation
!= derived finding
!= prediction
!= recommendation
!= CDS decision
!= authorization
!= implementation receipt
!= outcome observation
```

## 6. Current Technical Specification data objects

Current public Technical Specifications expose six DRAFT structures. Each remains an Integral schema, referenced by exact source family/version/status.

### SPEC-DS-01 — CertifiedDesign

**Mapping:** `DomainAdapter`.

Compose:

- Integral `SchemaRef` / object `SemanticRef`;
- manufacturing BOM/planning/operation refs;
- source/design provenance;
- ecological evidence refs;
- designer/certifier identity/credential refs;
- translation/admission receipt into COS.

Preserve:

```text
CertifiedDesign
!= generic Mycelix Artifact

credential validity
!= OAD certification acceptance

external certification
!= local-node acceptance
```

### SPEC-DS-02 — LaborEvent

**Mapping:** `DomainAdapter`.

Source owner: COS / operational domain.

Compose work/task/participant/source evidence from manufacturing, craft, identity and provenance. Translate to ITC separately.

Preserve:

```text
work performed
!= work verified
!= ITC valuation
!= ITC credits posted
```

If the Integral source schema carries `itc_credits_issued`, preserve it as an Integral source field, but do not make that field the source-of-truth for the labor fact inside Mycelix.

### SPEC-DS-03 — MaterialConsumptionEvent

**Mapping:** `DomainAdapter`.

Compose manufacturing/resource-flow source record, source/provenance refs, production ref and ecological evidence refs.

ITC and FRS consume adapter projections.

```text
material fact owner = COS/source domain
FRS summary owner != material fact owner
ITC accounting owner != material fact owner
```

### SPEC-DS-04 — ITCLedgerEntry

**Mapping:** `DomainAdapter` + `ExternalNormativeSemantics`.

Reuse identity, append-only/event, receipt, idempotency and accounting infrastructure where appropriate.

Never promote this object into a generic Mycelix value/currency semantic.

### SPEC-DS-05 — FRSSignalPacket

**Mapping:** `DerivedProjection`.

Represent as a source-bound assessment/recommendation bundle with explicit input refs/frontier/currentness and derivation provenance.

```text
SignalPacket accepted by transport
!= CDS semantically admitted
!= CDS agrees
!= decision created
```

### SPEC-DS-06 — DecisionPacket

**Mapping:** `DomainAdapter`.

Represent governance decision lineage including issue, rationale/evidence refs, dissent, constraints, dispatch targets and review basis.

Preserve:

```text
DecisionPacket
!= Authorization
!= ImplementationAttempt
!= ImplementationReceipt
!= desired Outcome
```

A receiving domain may require a separate local authorization/admission step.

## 7. Twelve Development Guide cross-system contracts

The Development Guide currently names twelve primary cross-system contracts. Only three currently have public Phase-0 `SPEC-IF-*` entries on the Technical Specifications page. Absence of a public spec ID is recorded here as source status, not as a claim of defect.

| # | Integral seam | Current public SPEC-IF | Mycelix owner/composition | Class | Critical boundary |
|---:|---|---|---|---|---|
| 1 | OAD → COS — Certified Design Package | SPEC-IF-01 PENDING | semantic refs + manufacturing admission + translation receipt | `DomainAdapter` | certification evidence != COS/local admission |
| 2 | OAD → ITC — Design Intelligence Signal | no public ID observed | OAD adapter → ITC adapter; design/source refs + accounting policy | `DerivedProjection` + `ExternalNormativeSemantics` | design efficiency/ecology evidence != ITC value automatically |
| 3 | OAD → FRS — Design Event Signal | no public ID observed | manufacturing/design provenance → FRS projection | `DerivedProjection` | design event != diagnostic conclusion |
| 4 | FRS → OAD — Operational Recalibration | no public ID observed | evidence/assessment bundle → OAD review/admission | `DerivedProjection` | recommendation != certification mutation |
| 5 | COS → ITC — Labor and Materials Record | SPEC-IF-02 PENDING | source-owned operations → Integral accounting adapter | `DomainAdapter` | source fact != valuation; retry != duplicate credit |
| 6 | COS → FRS — Operational Signal | no public ID observed | manufacturing/commons observations → FRS projection | `DerivedProjection` | summary != source-owned current state |
| 7 | ITC → FRS — Credit and Access Signal | no public ID observed | ITC source-owned state → bounded FRS projection | `DerivedProjection` | FRS must not reconstruct authoritative current account state from partial history |
| 8 | FRS → CDS — Sensemaking Artifact | SPEC-IF-03 PENDING | evidence/recommendation seam → governance | `DomainAdapter` | recommendation != decision/authority |
| 9 | CDS → FRS — Governance Signal | no public ID observed | governance policy/decision refs → FRS monitoring profile | `DomainAdapter` | decision may set review criteria; FRS remains analytical |
| 10 | CDS → OAD — Design Mandate | no public ID observed | governance decision + bounded authorization → OAD adapter | `DomainAdapter` | governance decision != design/certification completion |
| 11 | CDS → COS — Production Mandate | no public ID observed | governance decision + bounded authorization → manufacturing/workorder admission | `DomainAdapter` | mandate != work/effect receipt |
| 12 | CDS → ITC — Policy Signal | no public ID observed | governance policy envelope → Integral ITC adapter | `DomainAdapter` + `ExternalNormativeSemantics` | Mycelix finance defaults do not define Integral policy |

### Cross-seam contract fields Mycelix should require from the adapter profile

Where relevant, every seam should state:

- producer semantic owner;
- consumer profile;
- source schema version;
- interface version;
- exact payload/semantic refs;
- source provenance/currentness;
- authority carried, if any;
- authority explicitly **not** carried;
- idempotency/dedup identity;
- retry semantics;
- ordering assumptions;
- delivery/acknowledgement states;
- translation losses;
- recipient admission/rejection state.

Do not implement a new Integral-specific message bus. Reuse the generic seam/delivery contract line.

## 8. Cross-cutting mapping

| Integral concern | Primary Mycelix owner | Mapping class | Notes |
|---|---|---|---|
| identity | identity | `ReuseNeutralPrimitive` | Integral participant/node identity profile may be adapter-specific |
| credentials/certification evidence | identity + domain evidence | `DomainAdapter` | evidence != acceptance |
| schema/version identity | SchemaRef/SemanticRef | `ReuseNeutralPrimitive` | source schema != interface version |
| exact external evidence | EPI/WEB-CAPTURE/provenance | `ReuseNeutralPrimitive` | URL != captured bytes != interpretation |
| federation | workspace/bridge + neutral federation profiles | `DomainAdapter` | foreign evidence may cross; authority does not automatically |
| transport | Holochain/Xenia/HTTP/etc behind seam profile | `DomainAdapter` | transport route != semantic identity |
| database/query projection | PostgreSQL/SQLite/hybrid implementation profile | `DerivedProjection` | database row ID != semantic owner |
| emergency operations | civic + bounded authority | `DomainAdapter` | emergency trigger != unlimited authority |
| environmental telemetry | climate/energy/commons/source domains | `DomainAdapter` | FRS consumes; source domain owns |
| analysis/simulation | knowledge + optional Symthaea | `DerivedProjection` | prediction != observation |
| participant-facing explanation | domain records + provenance/decision lineage | `DomainAdapter` | explanation should expose source/rule/authority/review path |

## 9. Explicit non-equivalence registry

The adapter must make these relations testable, not merely document them:

```text
Integral object ID
!= Holochain ActionHash
!= PostgreSQL/SQLite row ID
!= transport message ID

Integral source schema
!= Mycelix schema because fields look similar

Integral CDS rules
!= current Mycelix governance voting defaults

Integral ITC
!= MYCEL
!= SAP
!= TEND
!= reputation

Integral FRS confidence
!= cryptographic verification
!= source reliability
!= evidence independence
!= empirical maturity
!= calibrated probability
!= authority

CertifiedDesign
!= generic credential
!= local certification acceptance

LaborEvent
!= ITC credit posting

MaterialConsumptionEvent
!= FRS summary

FRSSignalPacket
!= source truth
!= CDS decision

DecisionPacket
!= Authorization
!= EffectReceipt

Recommendation
!= Decision
!= Authorization
!= EffectReceipt

Prediction
!= Observation

DeliveryReceipt
!= ImplementationReceipt

ImplementationReceipt
!= desired real-world Outcome

foreign identity recognized
!= foreign credential accepted
!= foreign decision authoritative locally

DerivedProjection
!= source-owned current state
```

## 10. What should **not** be added to Mycelix core from Integral

Do not add generic core types merely because Integral uses them:

- `ITC` or time-credit policy;
- Integral scarcity/access rules;
- Integral CDS consensus policy;
- Integral OAD certification criteria;
- Integral FRS confidence scale;
- Integral ecological valuation formula;
- Integral production mandate semantics;
- specific Integral recommendation/diagnostic enums when neutral evidence/assessment composition suffices.

These belong in an adapter/profile unless independent Mycelix requirements prove them generic.

## 11. Candidate neutral gaps to evaluate later

No new core primitive is approved by this document. Possible gap candidates are recorded only for census:

1. generic decision review basis / preregistered intended-outcome refs, if existing governance/evidence types cannot compose it cleanly;
2. generic source-frontier/current-state witness for derived projections, if provenance/currentness lines do not already own it;
3. generic bounded emergency authorization profile, if existing authority/delegation semantics are insufficient;
4. generic translation/admission receipt composition across domain bridges, if current receipt types cannot express it.

Each must first pass:

```text
existing-owner search
→ composition attempt
→ two independent non-Integral use cases
→ only then consider core primitive
```

## 12. Implementation order

Recommended sequence:

```text
source registry / capture
        ↓
SchemaRef + SemanticRef exact qualification
        ↓
evidence/provenance composition
        ↓
007C crosswalk freeze
        ↓
field-level Integral adapter fixtures
        ↓
twelve-seam contract profiles
        ↓
neutral conformance corpus
        ↓
independent conventional conformers
        ↓
PostgreSQL service baseline
        ↓
Mycelix/Holochain conformer
        ↓
hybrid conformer
        ↓
runtime replacement/export proof
```

Do not make the mapping executable by importing Integral policy into shared crates before the semantic-reference/evidence qualification gates pass.

## 13. Suggested future mapping generations

### 007D — White Paper module census

Map all white-paper modules to existing Mycelix domains as:

- owner already exists;
- adapter composition;
- external policy;
- intentional non-goal;
- potential neutral gap.

Avoid 45 new Mycelix module names simply because the white paper names 45 modules.

### 007E — field-level Technical Specification crosswalk

For each exact source field in SPEC-DS-01..06 record:

- source field meaning;
- source owner;
- Mycelix refs composed;
- translation class;
- loss/unknown behavior;
- authority effect;
- source-version identity.

### 007F — pilot/node preparation crosswalk

Map Integral's operational node-preparation guidance onto Mycelix deployment/pilot requirements without treating community practice guidance as software semantics.

## 14. Review checklist

Before accepting a mapping row, ask:

- Is the exact Integral source family/status recorded?
- Is the source owner explicit?
- Is the chosen Mycelix owner real and current?
- Are we reusing a capability or accidentally importing policy?
- Does the mapping preserve source identity/version?
- Does it preserve source-owned vs derived state?
- Does it avoid authority strengthening?
- Can the mapping be exported without Holochain/database IDs becoming canonical?
- Is translation loss explicit?
- Would another governance/economic model still be able to use the neutral primitive?

If the final answer to the last question is no, keep it in the Integral adapter.

## 15. Nonclaims

This crosswalk:

- does not endorse or oppose Integral's political/economic model;
- does not declare Mycelix the preferred Integral technology stack;
- does not claim current Mycelix domain policies match Integral policy;
- does not claim source-implemented Mycelix components are production-qualified;
- does not establish any currently queued qualification as PASS;
- does not create new authority through mapping, translation, federation or transport.

The desired result is **semantic interoperability without semantic capture**.