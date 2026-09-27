# MYC-INT-017A — Integral Reference Solution R1

Status: architecture candidate / not Integral-ratified / not production-qualified

Observed source date: 2026-09-27

## 1. Purpose

Define a complete, replaceable reference implementation for Integral's minimum viable five-system loop without making Integral-specific governance or economic policy canonical Mycelix semantics.

```text
complete reference solution
!= Integral-selected stack
!= Mycelix policy made normative
!= mandatory Holochain
!= mandatory Symthaea
!= production readiness
```

The target is strong enough to support the progression described in Integral's current Development Guide:

```text
minimum viable CDS/OAD/ITC/COS/FRS loop
-> simulation / virtual-node testing
-> real-world pilot deployment
```

The public Technical Specifications currently expose six DRAFT core data structures, three PENDING interface contracts, and a PENDING technology-stack decision. R1 therefore treats Integral public specs as external source-owned contracts, preserving every source status rather than silently ratifying them.

## 2. Architecture rule

Integral concepts stay Integral concepts.

Mycelix contributes reusable identity, evidence, provenance, semantic identity, authority lineage, domain operations, federation, receipts and conformance infrastructure underneath an adapter boundary.

```text
Integral source semantics
        ↓
Integral adapter contracts
        ↓
Mycelix neutral/domain substrate
        ↓
replaceable persistence/federation runtime
        ↓
participant/operator applications
```

Optional Symthaea analysis is beside the authority path, not inside it.

```text
                 Integral R1
                     │
       ┌─────────────┴─────────────┐
       │                           │
       ▼                           ▼
   Mycelix                    Symthaea
semantic state                analysis only
provenance                    simulation
identity                      prediction
governance lineage            optimization
authority                     diagnostics
receipts                      recommendations
       │                           │
       └─────────────┬─────────────┘
                     ▼
              Integral adapters
                     ▼
               Integral UI
```

## 3. Repository ownership

Do not create parallel `mycelix-integral-cds`, `mycelix-integral-frs`, etc. domain systems.

Use one thin integration/application package that composes existing owners.

Tentative layout:

```text
integrations/integral/
├── README.md
├── contracts/
│   ├── source-registry/
│   ├── schemas/
│   ├── interfaces/
│   └── translation/
├── adapters/
│   ├── cds/
│   ├── oad/
│   ├── itc/
│   ├── cos/
│   └── frs/
├── runtime-postgres/
├── runtime-holochain/
├── runtime-hybrid/
├── symthaea-bridge/
├── fixtures/
├── conformance/
├── apps/
│   └── leptos/
└── deploy/
```

This path is provisional. Before code is created, reconcile it against existing workspace conventions and avoid duplicating bridge, SDK, evidence, authority or domain functionality.

## 4. Product surfaces

### 4.1 Participant application

Canonical Mycelix product UI profile: Rust + Leptos.

A complete participant app should expose:

- identity and participation eligibility;
- current node/system status;
- CDS issue framing, evidence, alternatives, objections, deliberation, decisions and reviews;
- OAD design submission, lineage, evidence, review and certification state;
- ITC contribution/access/account history and source references;
- COS tasks, work orders, resource/material state, progress, QA and completion evidence;
- FRS findings, predictions, recommendations, uncertainty and review triggers;
- provenance/source inspection;
- explanation/contest/review paths;
- degraded/offline/unknown state explicitly.

The UI must never visually collapse advisory and authoritative states.

Examples:

```text
Recommendation
Decision
Authorization
EffectReceipt
```

must render as distinct lifecycle states.

### 4.2 Operator / node administration

R1 operator surface should include:

- node bootstrap;
- environment/runtime profile selection;
- participant/role/credential configuration under Integral policy;
- schema/interface compatibility state;
- inbox/outbox and delivery health;
- local/federated recognition profiles;
- PostgreSQL health/projection/rebuild state when enabled;
- Holochain conductor/network state when enabled;
- backup/restore/export/import;
- evidence/audit export;
- emergency authority expiry/review;
- privacy/federation disclosure profiles;
- upgrade/migration readiness;
- visible `Unknown`/indeterminate conditions.

## 5. CDS adapter

Composition owners:

- Mycelix Governance for proposal/decision workflow capabilities where compatible;
- shared authority/delegation/effect semantics;
- evidence/provenance infrastructure;
- Civic only when a deployment explicitly requires emergency/enforcement workflows.

Integral deliberation/consensus policy remains adapter-owned.

Canonical lifecycle:

```text
Issue
-> Evidence
-> Alternatives
-> Objections / dissent
-> Deliberation
-> Decision
-> Authorization
-> Dispatch
-> EffectReceipt
-> OutcomeObservation
-> ReviewCandidate
-> supersession / amendment / closure
```

Required invariants:

```text
DecisionPacket != Authorization
Authorization != Dispatch
Dispatch != external effect
external effect != desired outcome
Recommendation != Decision
foreign decision != local authority
```

A decision should bind its review basis at decision time where available:

- intended outcomes;
- assumptions;
- constraints;
- known dissent/trade-offs;
- review trigger/window;
- relevant evidence criteria;
- amendment/supersession route.

## 6. OAD adapter

Composition owners:

- Manufacturing: BOM/planning/machines/operations/work orders;
- Identity/Praxis/Craft: identity, credentials, skills, work-history references;
- Knowledge/evidence: design claims and supporting evidence;
- Climate/Energy/Commons: source-owned ecological/resource observations where applicable;
- Symthaea: optional analysis/design-space search, never certification authority.

Canonical lifecycle:

```text
DesignSubmission
-> DesignVersion lineage
-> evidence / analysis / review
-> CertificationCandidate
-> explicit certification acceptance
-> CertifiedDesign
-> COS admission
-> production/outcome feedback
-> recalibration / supersession
```

Required invariants:

```text
SymthaeaDesignCandidate != CertifiedDesign
verification evidence != certification acceptance
CertifiedDesign != production authorization
source design version != translated local projection
```

## 7. ITC adapter

Composition owners:

- Finance/accounting infrastructure;
- Attribution/contribution receipts where useful;
- Identity participant references;
- Commons timebank only as a reusable local contribution mechanism where semantically compatible.

Integral ITC remains external normative semantics.

```text
ITC != MYCEL
ITC != SAP
ITC != TEND
ITC != generic Mycelix value
```

R1 capabilities:

- append-only ledger lineage;
- labor/material/source references;
- contribution/decay/access/adjustment events;
- current account projection separate from historical ledger;
- correction/supersession model;
- idempotent admission;
- explicit cross-node recognition/clearing only when Integral defines it;
- privacy/minimization profile;
- source-owned timestamps/currentness rather than reconstructed guesses when authoritative state exists.

## 8. COS adapter

Composition owners:

- Manufacturing: planning, BOM, work orders, operations, machines;
- Commons: shared resources/property/water/food/transport where needed;
- Craft/Praxis/Identity: skills and credentials;
- supply-chain/service domains where deployment scope requires them.

Canonical capabilities:

- tasks/work orders;
- design-version binding;
- participant assignment;
- labor events;
- material consumption events;
- resource constraints;
- production progress;
- QA evidence;
- completion evidence;
- source-owned operational current state;
- explicit output to ITC/FRS interfaces.

Required invariants:

```text
COS operational fact != ITC valuation
COS operational fact != FRS interpretation
FRS finding != mutation of COS source state
```

## 9. FRS adapter

FRS is a derived feedback/analysis layer over source-owned state.

Composition owners:

- EPI/provenance/semantic identity;
- Knowledge claims/assessments with namespace-safe semantics;
- source domains for observations;
- optional Symthaea analysis.

Outputs may include:

- `DiagnosticFinding`;
- `Prediction`;
- `CounterfactualResult`;
- `Recommendation`;
- `ReviewCandidate`;
- explicit `Unknown` or abstention state.

Required invariants:

```text
Observation != DerivedFinding
DerivedFinding != Prediction
Prediction != Recommendation
Recommendation != Decision
FRS output != Authorization
FRS confidence != verification status
FRS confidence != source reliability
FRS confidence != calibrated probability unless explicitly calibrated/provenanced
```

## 10. Current public Integral data structures

R1 must represent the six currently public DRAFT structures as adapter-owned schemas:

1. `SPEC-DS-01 CertifiedDesign`
2. `SPEC-DS-02 LaborEvent`
3. `SPEC-DS-03 MaterialConsumptionEvent`
4. `SPEC-DS-04 ITCLedgerEntry`
5. `SPEC-DS-05 FRSSignalPacket`
6. `SPEC-DS-06 DecisionPacket`

Their source status remains DRAFT until Integral changes it.

R1 uses `SchemaRef`/`SemanticRef`-class identity and provenance rather than replacing source IDs with database/Holochain IDs.

```text
Integral object ID
!= database row ID
!= Holochain ActionHash
!= transport message ID
```

## 11. Interface layer

The currently public Phase-0 interface contracts remain PENDING source contracts:

- `SPEC-IF-01 OAD -> COS`
- `SPEC-IF-02 COS -> ITC`
- `SPEC-IF-03 FRS -> CDS`

R1 may implement candidate adapters/tests for them but must not label them ratified.

The Development Guide's remaining seams should also be supported through adapter contracts without inventing Integral spec IDs.

Every interface profile should declare:

- source schema version;
- interface version;
- producer/consumer;
- authentication profile;
- payload identity/commitment;
- delivery mode;
- retry/idempotency/deduplication;
- acknowledgement semantics;
- currentness/expiry;
- error/rejection/indeterminate states;
- authority direction;
- privacy/minimization properties;
- translation/provenance receipt.

Key theorem:

```text
schema compatibility
!= semantic equivalence
!= policy compatibility
!= authority recognition
!= effect authorization
```

## 12. Runtime profiles

### 12.1 Profile A — PostgreSQL conventional reference node

Purpose: realistic conventional service baseline.

PostgreSQL owns service persistence, transactional inbox/outbox, projections, search/reporting and operational state for the conventional profile.

Do not use database-native identity as semantic identity.

```text
row_id != SemanticRef
transaction commit != external effect
RLS != governance authority
LISTEN/NOTIFY != durable semantic receipt
```

### 12.2 Profile B — Mycelix/Holochain federated node

Purpose: agent-centric distributed/federated conformer.

Holochain implements the same adapter semantics but does not redefine them.

```text
ActionHash != Integral object ID
DHT validation != policy acceptance
remote signal != durable delivery theorem
```

### 12.3 Profile C — hybrid

Use Holochain/Mycelix for distributed semantic/history/federation concerns and PostgreSQL for rebuildable relational projections, search, analytics and local operations.

```text
PostgreSQL projection != second semantic owner
```

Every projection should bind to a source frontier/version and be rebuildable or explicitly declared source-owned when it is not merely a projection.

## 13. Symthaea analytical plane

Symthaea integration is optional.

The bridge is read-only with respect to authority/state semantics:

```text
AnalysisRequest
  source refs
  model/profile ref
  constraints
  question/task
        ↓
    Symthaea
        ↓
AnalysisArtifact
  findings
  predictions
  counterfactuals
  recommendations
  uncertainty
  provenance
  optional proof receipt
```

Default:

```text
authority = none
```

A valid proof of computation proves only the committed computation/procedure, not source truth, policy desirability or authorization.

## 14. Reference scenarios

R1 qualification should grow from the existing I0 water scenario into a small cross-system suite.

### I0 — water/resource issue

Exercises observation -> evidence -> recommendation -> decision -> authorization -> effect -> outcome -> review.

### I1 — design certification / production

Exercises OAD version lineage, certification evidence, explicit acceptance, COS production binding and later supersession.

### I2 — labor/material -> ITC

Exercises source-owned labor/material events, idempotent accounting admission, corrections and current-state projection.

### I3 — FRS -> CDS review loop

Exercises derived findings, prediction/recommendation boundaries, preregistered review basis and supersession.

### I4 — bounded emergency authority

Exercises explicit scope, expiry, effect receipts, renewal and mandatory review.

### I5 — federation disagreement

Exercises foreign identity/credential/decision recognition without local authority escalation.

### I6 — partition/reconnect/version skew

Exercises duplicate/reordered delivery, indeterminate state, stale schemas and reconciliation.

### I7 — privacy/federation disclosure

Exercises aggregation/linkability/inference leakage and declared disclosure budgets.

## 15. Qualification architecture

Candidates never receive the oracle.

```text
frozen corpus/oracle
       │
       X
       │
neutral stimulus
       ↓
sanitized candidate input
       ↓
implementation under test
       ↓
observed semantic facts/results
       ↓
separate evaluator + oracle
       ↓
qualification manifest
```

Track separately:

- designed;
- source-implemented;
- locally tested;
- exact repository-qualified;
- directly compared;
- pilot-observed.

Never infer later states from earlier ones.

## 16. Deployment profiles

### Dev profile

- one local node;
- synthetic fixtures;
- deterministic reset/replay;
- optional local PostgreSQL;
- optional local conductor;
- Symthaea disabled by default.

### Virtual-node profile

- multiple isolated nodes;
- explicit network partitions/reconnects;
- version skew;
- independent operator identities;
- synthetic participants;
- failure injection;
- observability/evidence export.

### Pilot profile

- reproducible Nix deployment;
- backup/restore rehearsal;
- privacy/security review;
- least-privilege operator capability;
- export/migration path;
- component replacement documentation;
- explicit incident/recovery playbooks;
- no hidden dependency on Luminous infrastructure.

Xenia can later provide secure remote/operator channels, but R1 semantics must not require it.

## 17. Portability and exit

R1 is incomplete unless Integral can leave it.

Required proof direction:

```text
R1 runtime A
-> runtime-neutral export
-> R1 runtime B / independent consumer
```

Preserve where representable:

- semantic identities;
- source schema identities;
- provenance;
- historical decision/accounting lineage;
- supersession;
- unresolved conflicts/unknowns;
- explicit loss declarations.

The ability to export bytes is not enough; semantic loss must be visible.

## 18. Security/privacy minimum

Before pilot use, R1 needs evidence for:

- authentication and credential lifecycle;
- authorization/capability boundaries;
- secret/key storage;
- dependency/supply-chain integrity;
- backup confidentiality/integrity;
- audit/evidence integrity;
- federation disclosure minimization;
- rate/abuse controls;
- emergency authority bounds;
- remote administration boundaries;
- recovery from lost/corrupt local state.

No security claim should be inherited merely because a lower-layer runtime provides cryptography.

## 19. Implementation order

### Gate 0 — source fidelity

- source registry;
- document-status identity;
- 007C crosswalk;
- twelve-seam coverage matrix.

### Gate 1 — neutral interoperability waist

- SchemaRef/SemanticRef exact qualification;
- EPI/provenance composition;
- Unicode/display hardening;
- semantic seam/delivery profile;
- neutral candidate input/results protocol;
- reusable validator.

### Gate 2 — Integral contracts

- six SPEC-DS schemas/fixtures;
- three current SPEC-IF candidate profiles;
- remaining Development Guide seams as source-labeled adapter contracts;
- translation receipts.

### Gate 3 — complete conventional node

- PostgreSQL schema/migrations;
- all five adapters;
- participant Leptos app;
- operator surface;
- I0-I4 qualification;
- export/import.

### Gate 4 — portability controls

- independent non-Mycelix conformer;
- same neutral inputs/results;
- cross-conformer semantic evaluation.

### Gate 5 — federated/hybrid nodes

- Holochain conformer;
- hybrid projection profile;
- I5-I7 federation/partition/privacy scenarios;
- migration replacement proof.

### Gate 6 — Symthaea

- read-only AnalysisRequest/AnalysisArtifact protocol;
- hidden-outcome I0 analytical pilot;
- OAD engineering/design pilot;
- calibration/abstention/provenance evidence.

### Gate 7 — pilot readiness

- Nix deployment;
- backup/restore;
- security/privacy review;
- observability;
- operator playbooks;
- participant legibility/contestability tests;
- failure/recovery drills.

## 20. What not to build

Do not build:

- duplicate Integral-specific identity when Mycelix Identity fits;
- duplicate evidence/provenance ontology;
- generic `IntegralValue` in Mycelix core;
- FRS as a source-of-truth database;
- Symthaea as governance authority;
- PostgreSQL projections as hidden canonical history;
- Holochain ActionHash as external business identity;
- a custom transport bus where existing durable delivery/inbox/outbox owners fit;
- a one-way migration path that makes the stack effectively mandatory.

## 21. Definition of R1 complete

R1 can be called a **complete reference solution candidate** when:

1. all five Integral systems are usable through one participant/operator product surface;
2. all six current public data structures are represented with source status/version provenance;
3. all three current public interface seams are implemented as clearly candidate/PENDING-compatible profiles;
4. the twelve Development Guide seams have explicit owners/contracts/status;
5. the five-system loop passes synthetic oracle-hidden conformance;
6. PostgreSQL reference node is operationally reproducible;
7. independent conformer proves the protocol is not Python/Mycelix-specific;
8. optional Holochain/hybrid profiles preserve the same semantics;
9. Symthaea remains optional/non-authoritative and produces provenance/calibration evidence;
10. export/import replacement proof succeeds;
11. backup/restore and failure recovery have evidence;
12. privacy/security/operator/participant-legibility gates are documented and tested.

Even then, call it a reference solution candidate until Integral contributors/governance independently evaluate it.

## 22. External handoff package

When evidence is sufficient, the most useful package for Integral is:

- source/status crosswalk;
- twelve-interface coverage matrix;
- versioned schemas/interface candidates;
- conformance corpus and validator;
- complete reference node;
- PostgreSQL/Holochain/hybrid comparison evidence;
- migration/exit demonstration;
- security/privacy/operations evidence;
- optional Symthaea analytical extension;
- explicit known limitations/open questions.

The handoff should invite review, substitution and rejection of components rather than asking Integral to accept the stack wholesale.
