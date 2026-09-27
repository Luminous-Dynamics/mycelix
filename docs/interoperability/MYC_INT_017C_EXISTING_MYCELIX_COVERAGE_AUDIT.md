# MYC-INT-017C — Existing Mycelix coverage audit for Integral R1

Status: **source audit / documentation only**  
Tracks: #3200  
Parent: MYC-INT-017B / draft PR #3199  
Observed: 2026-09-27

## Purpose

Determine how much of Integral's current five-system architecture and public Phase-0 contract surface is already materially owned by existing Mycelix code, so the Integral Reference Solution R1 reuses existing capability rather than rebuilding it.

This document is a **reuse audit**, not a platform score and not a conformance claim.

```text
conceptual similarity
!= existing code ownership
!= Integral schema compatibility
!= Integral conformance
!= repository qualification
```

## External source ceiling

The current public Integral Technical Specifications expose:

- six DRAFT core data structures (`SPEC-DS-01` through `SPEC-DS-06`);
- three PENDING interface contracts (`SPEC-IF-01` through `SPEC-IF-03`);
- one PENDING technology-stack decision (`SPEC-STACK-01`).

R1 may implement adapter candidates against those sources, but does not ratify them or treat DRAFT/PENDING material as stable.

## Coverage vocabulary

| Class | Meaning |
|---|---|
| `DirectOwner` | Existing Mycelix code already owns the neutral capability materially required by the Integral concept. |
| `ComposableOwners` | Capability exists across several Mycelix owners and needs an Integral composition/adapter layer. |
| `PolicyAdapterRequired` | Infrastructure exists but Integral-specific normative semantics must remain adapter-owned. |
| `DerivedAssemblyRequired` | Source data/analysis ingredients exist but the Integral derived object/workflow is not assembled yet. |
| `Gap` | No adequate existing owner was found in the inspected evidence. |
| `Unknown` | Evidence is insufficient; do not infer coverage. |

## Executive finding

The current evidence does **not** support building five new Integral domain systems.

The strongest observed architecture is:

```text
existing Mycelix domain owners
        +
Integral-owned external schemas / policy adapters
        +
versioned interface + delivery contracts
        +
FRS derived assembly
        +
Leptos participant/operator experience
```

The audit finds substantial existing machinery for:

- identity and credentials;
- deliberation/proposals/discussion/voting/execution;
- design/versioning/materials/verification/fabrication;
- BOM/routing/work orders/machines/MRP;
- resource coordination across water, food, transport, property, housing, care and mutual aid;
- work-history/skill attestations;
- finance/time-exchange/accounting infrastructure;
- claims/attestations/derived confidence;
- cross-domain bridges and Holochain runtime composition.

The largest remaining R1 work is **composition and exact semantics**, especially:

1. exact Integral adapter DTOs and source identities;
2. Integral-specific policy for CDS and ITC;
3. exact `LaborEvent` / `MaterialConsumptionEvent` emission;
4. FRS derived-state assembly and currentness/staleness rules;
5. the three exact Integral interface contracts;
6. Decision -> Authorization -> EffectReceipt separation at the Integral adapter boundary;
7. source-owned state vs derived summaries;
8. participant/operator UI and operational runtime profiles.

---

# 1. Five-system audit

## 1.1 CDS — Collaborative Decision System

**Classification:** `ComposableOwners` + `PolicyAdapterRequired`

### Concrete existing owners

`mycelix-governance` contains concrete zomes for:

- proposals;
- voting;
- execution;
- constitution;
- councils;
- jurisdiction;
- budgeting;
- threshold signing;
- bridge integration.

The proposals integrity code already defines a `Proposal` with:

- stable proposal ID;
- title/description;
- author DID;
- status lifecycle;
- actions;
- discussion URL;
- voting interval;
- timestamps;
- explicit version.

It also defines:

- `ProposalAmendment`;
- `DiscussionContribution` with threaded replies and explicit stance;
- `DiscussionReflection` with participation metrics, stance distribution, unaddressed concerns, readiness and summary;
- immutable proposal content after leaving Draft except through controlled lifecycle transitions.

The execution integrity zome already defines:

- `Timelock`;
- `Execution`;
- execution status/results/errors;
- guardian veto and override structures.

### What can be reused directly

- proposal identity/lifecycle machinery;
- DID-bound authorship;
- discussion records and threaded deliberation;
- support/oppose/neutral/amend stance representation;
- amendment lineage;
- bounded status transitions;
- timelock/execution machinery;
- bridge and identity integration;
- evidence/knowledge references through adjacent owners.

### What must remain Integral-specific

Current Mycelix governance includes its own normative policy defaults such as MATL/stake weighting, proposal types, quorum/approval rules and optional quadratic voting. Those are **not** Integral CDS semantics.

R1 should therefore adapt the substrate while keeping Integral rules external:

```text
Mycelix Proposal machinery
!= Integral CDS policy
```

### Important remaining CDS work

- exact Integral `DecisionPacket` composition;
- explicit `issue_ref` semantics;
- exact dissent-position projection;
- dispatch targets;
- implementation constraints;
- preregistered review trigger/basis;
- separate Integral Authorization after Decision;
- effect and outcome receipts;
- Integral standing/eligibility adapter.

### Reuse conclusion

CDS is **not a greenfield subsystem**. The durable proposal/deliberation/execution substrate already exists, but R1 must prevent Mycelix governance policy from leaking into Integral's own governance model.

---

## 1.2 OAD — Open Access Design

**Classification:** `ComposableOwners`

OAD currently has the deepest overlap with existing Mycelix engineering/fabrication work.

### Fabrication owner

`mycelix-workspace/happs/fabrication` already exposes concrete surfaces for:

- design CRUD/search/versioning;
- material specifications;
- verification/safety claims;
- printer registry;
- print jobs;
- HDC-enhanced design representation/search;
- a Symthaea integration zome;
- cross-hApp bridge integration.

Its documented design/fabrication architecture also links to material passports, knowledge safety claims, energy grounding and other Mycelix domains.

### Manufacturing owner

`mycelix-manufacturing` has six concrete zome families:

- `bom`;
- `planning`;
- `operations`;
- `workorders`;
- `machines`;
- `bridge`.

The canonical `manufacturing_common` code already defines:

- `BillOfMaterials { design_id, revision, items, ... }`;
- `RoutingSequence { design_id, revision, steps, ... }`;
- `Operation`;
- `WorkOrder` and guarded lifecycle transitions;
- `Machine` and machine state transitions;
- `MrpResult`;
- `PlannedOrder`;
- `ScheduledOperation`;
- `CapacityWarning`;
- `MaterialShortage`.

### Adjacent owners

- Identity: designer/certifier DID and credential lifecycle;
- Knowledge/evidence: claims and supporting evidence;
- Craft: skills/work-history credentials;
- Commons/energy/climate: ecological/resource source observations;
- Symthaea: optional design search/simulation/analysis, never certification authority.

### What is already close to Integral `CertifiedDesign`

| Integral field | Existing Mycelix material | Classification |
|---|---|---|
| `design_id` | Fabrication design identity + manufacturing `design_id` | `DirectOwner` capability |
| `version` | Fabrication design versioning + manufacturing `revision` | `ComposableOwners` |
| `bill_of_materials` | `BillOfMaterials` | `DirectOwner` capability |
| `production_steps` | `RoutingSequence.steps` / `Operation` | `DirectOwner` capability |
| `ecological_flag` | fabrication/resource/material/environment evidence | `ComposableOwners` |
| `itc_access_cost` | finance/accounting adapter required | `PolicyAdapterRequired` |
| `design_lineage` | fabrication versioning + provenance refs | `ComposableOwners` |

### Remaining OAD work

The missing piece is not basic fabrication capability. It is the exact Integral certification envelope:

```text
existing design + BOM + routing + evidence
        ↓
Integral OAD review/policy
        ↓
Integral CertifiedDesign
```

and critically:

```text
Mycelix verification evidence
!= Integral certification acceptance
```

### Reuse conclusion

OAD should be implemented primarily as a **thin composition/certification adapter over Fabrication + Manufacturing + Evidence/Identity**, not as a new design/fabrication platform.

---

## 1.3 ITC — Integral Time Credits

**Classification:** `PolicyAdapterRequired`

### Concrete existing owners

`mycelix-finance` already has a substantial shared Rust type layer plus Holochain zomes for:

- MYCEL recognition;
- TEND mutual-credit time exchange;
- SAP/TEND payments;
- treasury;
- bridge/collateral flows;
- staking.

The shared finance type crate explicitly models three existing Mycelix currencies:

```text
MYCEL
SAP
TEND
```

including transferability, demurrage/limit policy and contribution categories.

`TEND` already provides a time-based exchange substrate and Mycelix Commons also contains care/mutual-aid timebank domains.

### What can be reused

- identity/account references;
- amount and accounting primitives where neutral;
- time-contribution infrastructure;
- transaction/receipt patterns;
- correction/lifecycle patterns where semantically compatible;
- finance bridge/routing infrastructure;
- simulation infrastructure for policy testing;
- Craft/Attribution/Identity references for contribution provenance.

### What must not be reused as semantics

```text
Integral ITC
!= TEND
!= SAP
!= MYCEL
```

Integral's own rules for credit issuance, decay, access redemption, need adjustment, transferability and clearing remain **Integral-owned policy**.

The fact that TEND is time-based does not make it the Integral ledger.

### Reuse conclusion

ITC needs a **new Integral adapter schema and policy engine**, but it does **not** require a new generic finance platform.

---

## 1.4 COS — Cooperative Organization System

**Classification:** `DirectOwner` capabilities + `ComposableOwners`

### Production and operations

`mycelix-manufacturing` already implements the core operational concepts COS needs:

- work orders;
- BOMs;
- process/routing steps;
- machines/workstations;
- production planning;
- capacity warnings;
- material shortages;
- bridge integration.

### Resource coordination

`mycelix-commons` already spans concrete Holochain domains for:

- property;
- housing;
- care;
- mutual aid;
- water;
- food;
- transport;
- cross-cluster bridging.

The water domain alone already includes flow, purity, capture, stewardship and knowledge zomes, making the existing I0 water scenario naturally source-owned by Mycelix Commons rather than an Integral-specific database.

### Workforce/skills

`mycelix-craft` already provides:

- skill credentials/pointers;
- peer attestations;
- work history with peer verification;
- apprenticeship/job lifecycle concepts.

### Remaining COS work

The main missing work is the **Integral event boundary**, especially:

- exact `LaborEvent` emission;
- exact `MaterialConsumptionEvent` emission;
- binding those events to source work order/design/material state;
- explicit producer identity and provenance;
- idempotent delivery to ITC and FRS;
- exact currentness/correction semantics.

### Reuse conclusion

COS should be a **coordination adapter over Manufacturing + Commons + Craft + source-specific domains**. Rebuilding operations/resource infrastructure would duplicate substantial existing code.

---

## 1.5 FRS — Feedback & Review System

**Classification:** `DerivedAssemblyRequired`

FRS has strong ingredients but is the least already assembled as one Integral-shaped subsystem.

### Existing source owners

Operational facts already have plausible source homes:

- Manufacturing: work orders, operations, machines, material/planning state;
- Commons: water/food/transport/property/housing/care/mutual-aid observations;
- Finance: economic/account state;
- Identity/Craft: participant/credential/work-history refs;
- other domain clusters for energy/climate/etc.

### Existing epistemic/analysis owners

`mycelix-knowledge` already implements:

- claims;
- attestations;
- challenge/endorse/acknowledge behavior;
- derived confidence;
- contradiction-sensitive scoring;
- subject discovery.

However its existing confidence semantics are **not automatically Integral FRS confidence semantics**, and earlier interoperability work already established that Knowledge epistemic axes must remain namespaced from core evidence semantics.

Optional Symthaea can provide:

- diagnostics;
- prediction;
- counterfactuals;
- sensitivity analysis;
- recommendations;
- abstention/unknown;

through the separate read-only bridge.

### What still needs assembly

No direct Integral `FRSSignalPacket` owner was found in the inspected current surfaces.

R1 still needs:

- exact source-frontier collection;
- labor/material/ITC/QA/ecological summaries;
- `DiagnosticFinding` adapter objects;
- `Recommendation` adapter objects;
- explicit staleness/currentness/completeness;
- provenance for every derived summary;
- contradiction/unknown preservation;
- source-owned fact vs derived finding separation;
- FRS -> CDS versioned delivery and acknowledgement.

### Reuse conclusion

FRS should be a **derived composition layer**, not another source-of-truth database.

```text
source domain state
        ↓
FRS projection / analysis
        ↓
FRSSignalPacket
        ↓
CDS deliberation
```

and never:

```text
FRS derived summary
= source-owned current state
```

---

# 2. Six current SPEC-DS objects

## SPEC-DS-01 — CertifiedDesign

**Coverage:** `ComposableOwners`

Strong existing building blocks:

- Fabrication design/version/material/verification surfaces;
- Manufacturing `BillOfMaterials`;
- Manufacturing `RoutingSequence` / `Operation`;
- identity/credentials;
- provenance/evidence;
- environmental/resource evidence;
- optional Symthaea analysis.

Missing adapter work:

- exact external DTO;
- exact design-lineage translation;
- Integral certification decision/acceptance;
- exact ecological flag interpretation;
- exact ITC access-cost policy.

No new generic design subsystem appears necessary.

## SPEC-DS-02 — LaborEvent

**Coverage:** `ComposableOwners` + `PolicyAdapterRequired`

Candidate source composition:

```text
participant_id  <- Identity DID/ref
task_ref        <- COS/Manufacturing work-order/task ref
hours_verified  <- source work/time attestation
skill_tier      <- Craft/credential evidence
itc_credits_issued <- Integral ITC policy result
ecological_flag <- deployment/domain evidence
```

Important separation:

```text
labor performed
!= credits issued
```

The current Integral draft places both in one external record. R1 must preserve which fields are source observations versus derived accounting results.

Exact canonical LaborEvent emission remains adapter work.

## SPEC-DS-03 — MaterialConsumptionEvent

**Coverage:** `ComposableOwners`, with a likely event-record gap

Existing owners cover:

- material identity/specification;
- BOM requirements;
- production/work-order refs;
- material shortages/planning;
- material provenance/ecological evidence in adjacent fabrication/supply/resource domains.

The inspected manufacturing common types do **not** by themselves establish the exact Integral consumption-event record. R1 should add the Integral event adapter rather than reinterpret BOM/planning records as proof of actual consumption.

```text
planned material requirement
!= observed material consumption
```

## SPEC-DS-04 — ITCLedgerEntry

**Coverage:** `PolicyAdapterRequired`

Finance provides substantial accounting/economic infrastructure, but the exact Integral entry kinds and policy remain external:

```text
LABOR_EVENT
DECAY_APPLIED
ACCESS_REDEEMED
NEED_ADJUSTMENT
```

R1 needs an Integral-specific append/history object with explicit source refs and current-balance projection.

Do not alias to any existing MYCEL/SAP/TEND record.

## SPEC-DS-05 — FRSSignalPacket

**Coverage:** `DerivedAssemblyRequired`

Source ingredients exist, but the exact packet does not currently have a direct owner in the inspected code.

R1 must compose:

- labor summary;
- material summary;
- ITC summary;
- QA summary;
- ecological summary;
- findings;
- recommendations;

with exact provenance/currentness and no authority promotion.

This is one of the clearest genuinely new **Integral adapter objects**.

## SPEC-DS-06 — DecisionPacket

**Coverage:** `ComposableOwners` + `PolicyAdapterRequired`

Current governance code already owns much of the required information:

- proposal/issue identity;
- description/content;
- discussion contributions;
- explicit stances;
- discussion reflection and unaddressed concerns;
- proposal outcome/status;
- actions;
- versioned lifecycle;
- execution/timelock lineage.

Likely adapter mappings:

```text
decision_id              <- Integral adapter semantic ID
issue_ref                <- proposal/issue lineage ref
outcome                  <- CDS policy result
rationale                <- decision rationale + evidence refs
dissenting_positions     <- preserved opposing/amend/concern records
dispatch_targets         <- explicit adapter targets, not opaque actions alone
implementation_constraints <- adapter-owned constraints
review_trigger           <- explicit new Integral review-basis field
```

The current governance `Proposal` itself must **not** be serialized as `DecisionPacket` merely because fields overlap.

And:

```text
DecisionPacket
!= Authorization
!= Execution
```

---

# 3. Three current SPEC-IF interfaces

## SPEC-IF-01 — OAD -> COS

**Coverage:** `ComposableOwners`

Internal Mycelix already has design/fabrication/manufacturing bridge concepts and compatible design/BOM/routing machinery.

Still required:

- exact external interface version;
- CertifiedDesign admission rules;
- producer/consumer schema negotiation;
- authentication/admission profile;
- error taxonomy;
- retry/idempotency semantics;
- certification-recognition policy;
- translation receipt.

The hard part is now the **contract**, not basic design or production machinery.

## SPEC-IF-02 — COS -> ITC

**Coverage:** `ComposableOwners` + `PolicyAdapterRequired`

Existing production, work, identity and finance owners provide the endpoints.

Still required:

- exact LaborEvent/MaterialConsumptionEvent producers;
- durable/idempotent admission;
- duplicate handling;
- correction/supersession;
- ITC policy evaluation;
- source-event identity distinct from ledger-entry identity.

```text
COS event accepted
!= ITC credit necessarily issued
```

## SPEC-IF-03 — FRS -> CDS

**Coverage:** `DerivedAssemblyRequired`

This is currently the weakest of the three external seams because FRS itself still needs assembly.

Existing Knowledge/Governance bridges and generic Mycelix transport/federation work are reusable, but R1 still needs:

- exact `FRSSignalPacket` generation;
- recommendation routing;
- signal versioning;
- delivery/acknowledgement semantics;
- CDS semantic admission;
- explicit no-authority promotion.

```text
FRS packet delivered
!= recommendation accepted
!= decision made
!= authorization granted
```

---

# 4. Cross-cutting coverage already present

## Identity and credentials — strong existing owner

`mycelix-identity` already provides:

- decentralized identifiers;
- credential schemas;
- verifiable credentials;
- revocation/suspension;
- recovery;
- selective disclosure / ZK-oriented privacy surfaces.

R1 should not create Integral-specific identity infrastructure.

## Resource source domains — strong existing owners

`mycelix-commons` already contains source-oriented domains for water, food, transport, property, housing, care and mutual aid.

This is especially important for FRS architecture: these domains should remain source owners while FRS consumes projections.

## Skills/work history — existing owner

`mycelix-craft` already models credential pointers, peer attestations and work history with peer verification.

Use these as evidence/refs where useful; do not let Craft skill metadata automatically determine Integral governance standing or ITC valuation.

## Claims/derived epistemics — existing owner, semantic caution required

`mycelix-knowledge` already models claims, attestations, challenges and derived confidence.

Its existing confidence/truth-engine semantics are not a universal evidence ontology and must remain namespaced when consumed by FRS.

## Fabrication verification — existing owner, acceptance still external

Fabrication already contains verification/safety concepts and design lifecycle machinery.

That evidence can support OAD, but:

```text
verification evidence
!= Integral OAD certification acceptance
```

---

# 5. What R1 should *not* rebuild

Do not build new generic implementations of:

- identity/DID/credential lifecycle;
- generic proposal/discussion/voting storage;
- generic work orders;
- BOM/routing;
- machine registry;
- production planning/MRP;
- generic material/design registries;
- water/food/transport/property operational stores;
- generic timebank infrastructure;
- generic work-history/skill attestations;
- generic claim/attestation graph;
- a second FRS source-of-truth database;
- five new `mycelix-integral-*` domain hApps.

Prefer adapter composition.

---

# 6. What R1 genuinely needs to add

## Adapter contract layer

- six exact external `SPEC-DS-*` adapter schemas;
- three exact current `SPEC-IF-*` identities/contracts as Integral defines them;
- source/version/status registry;
- translation receipts;
- compatibility negotiation;
- explicit source-owner mapping.

## Integral policy layer

- CDS policy adapter;
- ITC accounting/credit policy adapter;
- OAD certification-acceptance policy;
- federation recognition policy;
- standing/eligibility interpretation.

## Derived FRS layer

- summary frontier/currentness;
- diagnostics/findings;
- recommendation objects;
- explicit unknown/abstention;
- optional Symthaea analysis artifacts;
- FRS signal packet assembly;
- FRS -> CDS delivery/acknowledgement.

## Exact event producers

- LaborEvent;
- MaterialConsumptionEvent;
- source correction/supersession semantics.

## Product layer

- Leptos participant/operator experience;
- explanation/provenance inspection;
- maturity/conformance state;
- degraded/offline/unknown-state visibility.

## Runtime/conformance layer

- PostgreSQL reference profile;
- Holochain conformer after runtime gates;
- hybrid profile;
- oracle-hidden I0-I7 fixtures;
- export/import replacement proof;
- pilot operations/security/privacy qualification.

---

# 7. Architectural consequence

The audit changes the implementation picture from:

```text
build Integral CDS
build Integral OAD
build Integral ITC
build Integral COS
build Integral FRS
```

into:

```text
                 Integral contracts/policy
                          │
         ┌────────────────┼────────────────┐
         │                │                │
         ▼                ▼                ▼
 existing Mycelix    thin Integral      derived FRS
 domain owners       adapters           assembly
         │                │                │
         └────────────────┼────────────────┘
                          ▼
                  Reference Solution R1
```

The practical implication is that a large fraction of the hard domain work is already present in Mycelix. The highest-value new work is **semantic integration and evidence-backed conformance**, not duplicating existing domains.

---

# 8. Evidence state / limits

Inspected concrete current code or module surfaces include:

- `mycelix-governance/zomes/proposals/integrity/src/lib.rs`;
- `mycelix-governance/zomes/execution/integrity/src/lib.rs`;
- `mycelix-manufacturing/crates/manufacturing_common/src/lib.rs`;
- `mycelix-workspace/happs/fabrication/`;
- `mycelix-finance/types/src/lib.rs`;
- `mycelix-commons/`;
- `mycelix-craft/`;
- `mycelix-identity/`;
- `mycelix-knowledge/`.

Some owner claims above remain `DocumentedCapability` where this pass inspected architecture/README surfaces rather than executable tests. Follow-up implementation work should promote individual mappings only after source/type/test-level verification.

No statement in this document upgrades any queued qualifier to PASS.

## Next action

Use this audit to make MYC-INT-017B/017D implementation **reuse-first**:

1. add exact adapter schema stubs;
2. reference existing owner IDs/types rather than copying them;
3. implement field-level translators with explicit loss metadata;
4. build FRS as a derived projection/analysis layer;
5. add contract tests proving Integral adapters cannot mutate source-owned semantics or inherit unrelated Mycelix policy.
