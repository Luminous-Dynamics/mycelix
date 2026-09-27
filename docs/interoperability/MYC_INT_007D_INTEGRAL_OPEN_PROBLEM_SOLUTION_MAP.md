# MYC-INT-007D — Integral Open Engineering Problems → Luminous Candidate Contributions

Status: adoption-neutral architecture/research. Tracks #3247. Child of MYC-INT-007C / PR #3195.

Observed Integral source date: 2026-09-27.

## 1. Purpose

Map Integral's current unresolved engineering work to **specific reusable Luminous candidates and explicit gaps**.

The point is not to argue that Integral should adopt Mycelix. It is to make a falsifiable contribution inventory:

```text
Integral requirement/open question
-> candidate Luminous owner
-> current maturity
-> proof still required
-> Integral decision still required
-> external expertise still required
```

No scalar readiness score and no overall technology-stack winner are defined here.

## 2. Current source basis

Integral's current public Technical Specifications present six DRAFT data structures and three PENDING cross-system interface contracts:

- `SPEC-IF-01` — OAD → COS;
- `SPEC-IF-02` — COS → ITC;
- `SPEC-IF-03` — FRS → CDS.

`SPEC-IF-03` is described by Integral as the most architecturally critical seam. `SPEC-STACK-01` — languages, database, API protocol and deployment model — is also PENDING. The same page identifies interface contracts and the technology-stack decision as highest-priority current work.

Integral's current Q&A states that no software has been built yet, that the first software target is an architecturally complete five-system MVS, that nodes are intended to be autonomous/federated through shared protocols, and that computational/institutional adequacy at scale remains an empirical question for the federation phase.

These are external requirements/open questions, not evidence that Integral accepts any Luminous interpretation.

## 3. Status vocabulary

Rows use:

- `ExistingQualifiedCapability`;
- `ExistingUnqualifiedCapability`;
- `DesignedCandidate`;
- `PrototypeCandidate`;
- `ExternalExpertiseRequired`;
- `IntegralPolicyDecisionRequired`;
- `NoLuminousSolutionClaim`.

When one row needs multiple statuses, retain all of them.

## 4. Current engineering fit

| Integral requirement / open problem | Luminous candidate contribution | Status | Main remaining proof / decision |
| --- | --- | --- | --- |
| OAD → COS versioning, authentication, errors, retries | MYC-INT semantic seam/delivery contracts, SchemaRef/SemanticRef, translation receipts, provenance; optional Xenia transport | ExistingUnqualifiedCapability / DesignedCandidate | Exact qualification; Integral acceptance/certification semantics |
| COS → ITC batch/streaming + idempotent ledger admission | durable inbox/outbox/effect infrastructure; idempotent semantic admission; conventional/Holochain/hybrid conformers | ExistingUnqualifiedCapability | Executable cross-runtime conformance; Integral ITC policy remains external |
| FRS → CDS delivery/version/recommendation/ack | recommendation→decision→authorization separation; typed delivery/ack states; source-owned vs derived state; optional Symthaea analysis | ExistingUnqualifiedCapability / DesignedCandidate | Exact qualification and Integral routing/ack semantics |
| Technology stack decision | protocol-only, Holochain reference-node and full-optional profiles; fair SQLite/PostgreSQL/Holochain/hybrid evaluation | DesignedCandidate / IntegralPolicyDecisionRequired | Execute comparable I0 benchmark + migration/exit evidence |
| Interface/schema evolution across heterogeneous nodes | versioned semantic refs, exact source schema identity, translation receipts, historical-version retention | ExistingUnqualifiedCapability | Execute version-skew/adversarial fixtures |
| Verification / oracle integrity | EPI/provenance, independent-source semantics, conflicting-source preservation, Edge physical acquisition, H1 physical bench | PrototypeCandidate | Physical qualification, calibration regimes and domain reference standards |
| Real-world proto-node telemetry | Luminous Edge sensor/provider fabric + H1 wet bench + Nix host + optional Mycelix federation | DesignedCandidate / PrototypeCandidate | Build/qualify physical H1; then repeat at another node/profile |
| Autonomous operation during federation outage | local Edge acquisition; runtime-neutral source ownership; partition/reconnect fixtures | DesignedCandidate | Execute exact partition campaigns on selected runtime(s) |
| Federation/version skew/reconciliation | heterogeneous federation contracts, local-vs-foreign authority, Holochain candidate runtime, conventional conformer | DesignedCandidate | 007E heterogeneous-node campaign |
| Governance cognitive load | attention-budget metrics, evidence-linked summarization, optional non-executive Symthaea, bounded standing envelopes where ratified | DesignedCandidate / IntegralPolicyDecisionRequired | Human studies + Integral policy; no software-only legitimacy claim |
| Governance velocity vs emergency response | bounded capability/authority, expiry, receipts, mandatory review, hard local safety paths | DesignedCandidate / IntegralPolicyDecisionRequired | Integral emergency policy + executable assurance |
| Foreign artifact recognition without authority leakage | explicit identity/credential/certification/contribution/decision recognition distinctions | ExistingUnqualifiedCapability | Cross-node conformance campaign |
| Federation privacy | privacy/export profiles, source minimization, local-first data paths, explicit inference limitations | DesignedCandidate | Concrete field-level disclosure policy and privacy tests |
| Participant legibility / contestability | source→rule→derivation→authority→uncertainty→review explanation chain; replaceable Leptos presentation | DesignedCandidate | Participant comprehension/usability studies |
| Manipulated evidence / technical sabotage | provenance, integrity, Xenia sessions, Nix reproducibility, supply-chain evidence, contradiction preservation, scoped authority | ExistingUnqualifiedCapability / DesignedCandidate | Threat-model-specific qualification; social capture remains outside claim |
| Ecological feedback semantics | measurement/provenance/unit identity, derived assessment, uncertainty, optional simulation/analysis | ExternalExpertiseRequired + DesignedCandidate | Authoritative ecological coefficients, methods, calibration, field evidence |
| Legal/tax/labor/property interfaces | jurisdiction/profile identity, evidence/audit substrate | ExternalExpertiseRequired | Jurisdiction-specific legal expertise and Integral policy |

## 5. Three implementation families remain worth evaluating

The Luminous contribution should make the comparison *more rigorous*, not prejudge it.

### 5.1 Conventional relational/service stack

Candidate profile:

```text
PostgreSQL
+ typed application services
+ HTTP/RPC/events
+ durable inbox/outbox
+ explicit identity/provenance/authority application semantics
```

Engineering advantages to test:

- mature operational ecosystem;
- familiar developer model;
- strong query/reporting ergonomics;
- transactional constraints;
- established backup/recovery/monitoring paths;
- broad contributor familiarity.

Engineering costs/questions to test:

- node autonomy/federation semantics must be built explicitly;
- runtime/database IDs must not become protocol identity;
- cross-node provenance, foreign-authority rules and offline reconciliation need deliberate application contracts;
- central-service assumptions can creep in unless the topology forbids them.

### 5.2 Mycelix + Holochain reference node

Candidate profile:

```text
Integral-owned domain semantics
+ Mycelix interoperability/evidence contracts
+ Holochain local/distributed persistence profile
```

Engineering properties worth testing:

- agent/node-local data and federation are natural architectural concerns;
- local/foreign records and provenance can be first-class;
- the architecture does not require one central application database;
- node independence aligns with Integral's stated federation direction.

Engineering costs/questions to test:

- smaller contributor/runtime ecosystem than PostgreSQL/HTTP;
- distributed debugging and operations are more specialized;
- exact offline/reconciliation behavior is profile-dependent, not automatic;
- current Mycelix subjects have mixed qualification maturity;
- installation, migration/export and long-lived operations require measured evidence;
- license/IP clarity under #884 remains an external-offer gate.

### 5.3 Hybrid

Candidate profile:

```text
PostgreSQL / conventional local source systems
+ runtime-neutral Mycelix semantic/evidence waist
+ optional Holochain federation where it earns its complexity
```

Properties worth testing:

- incremental adoption;
- conventional local queries/operations;
- explicit federation rather than global DB replication;
- source-domain ownership can remain with existing services;
- Mycelix becomes interoperability/evidence layer rather than universal storage.

Costs/questions:

- two runtime families increase operational complexity;
- canonical source ownership must be exact;
- projections/synchronization can create accidental dual truth if not strongly typed;
- the hybrid is only justified if federation/interoperability benefits exceed this complexity.

## 6. Where Luminous currently has unusually direct overlap

This is an engineering observation, not an adoption verdict.

Integral's three currently public pending interfaces ask directly for several properties already central to the MYC-INT work:

```text
versioning
idempotency
retry / unknown delivery
acknowledgement
recommendation routing
schema identity
node autonomy
federation
```

The Luminous stack also brings adjacent capabilities not required to be part of the Integral core:

- physical Edge acquisition/provider fabric;
- Nix/NixOS reproducibility;
- Xenia secure/delegated transport;
- Symthaea read-only analysis/simulation;
- formal-verification/qualification discipline;
- heterogeneous external adapter/conformance fixtures.

These should be offered modularly. A rejection of any optional layer must not invalidate the protocol-only contribution.

## 7. Problems Luminous should not claim to solve

### Economic/governance policy correctness

Mycelix can encode, version, simulate and audit rules. It cannot establish that Integral's ITC rules, governance procedures or allocation criteria are socially desirable, legitimate or effective.

### Ecological truth

Symthaea can analyze evidence. It cannot manufacture authoritative ecological coefficients, valid sensor calibration or trustworthy lifecycle inventories.

### Human participation and legitimacy

Sensemaking tooling may reduce information burden. It cannot prove meaningful participation, prevent all institutional capture or decide legitimate authority for a community.

### Legal acceptance

Cryptographic evidence and reproducible policy versions do not answer jurisdiction-specific labor, tax, property, regulatory or liability questions.

### Real-world scale

A synthetic 512-node test would still not prove institutional/economic scale. It would establish only the measured software/network properties under the exact workload and environment.

## 8. Offerable contribution packages

Rather than one monolithic pitch, prepare independently useful packages.

### Package A — interface specifications

- twelve-contract traceability;
- version/evolution policy;
- retry/idempotency/ack taxonomy;
- conformance fixtures.

No Mycelix runtime dependency.

### Package B — evidence/oracle profile

- reports/observations/measurements/findings/recommendations distinctions;
- provenance/independence/currentness;
- physical acquisition and calibration evidence profile.

No Holochain requirement.

### Package C — federation conformance pack

- runtime-neutral semantic identities;
- local/foreign authority separation;
- version skew;
- partition/reconnect;
- translation/migration receipts.

Implementable by multiple runtimes.

### Package D — reference implementations

- PostgreSQL conventional conformer;
- Mycelix/Holochain conformer;
- hybrid conformer.

Measured, not ranked by architectural preference.

### Package E — physical proto-node kit

- Luminous Edge acquisition/provider profile;
- H1 wet bench;
- exact run/evidence manifests;
- optional Mycelix/Integral adapter.

### Package F — optional analysis/security/ops

- Symthaea analysis;
- Xenia transport/admin capability;
- Nix deployment/recovery;
- formal-assurance candidates.

Never required for basic Integral protocol conformance.

## 9. Next proof

The next useful demonstration is MYC-INT-007E: an eight-node heterogeneous federation containing:

- one real H1 physical node;
- multiple role-specialized logical Integral-style nodes;
- a deliberately degraded/version-skewed node;
- one conventional non-Holochain conformer.

Its job is to measure autonomy, partition behavior, version skew, authority boundaries and runtime replaceability — not to create a visually impressive but semantically empty mesh.

## 10. Nonclaims

This document does not endorse or oppose Integral's political/economic design, recommend an adoption decision, identify an overall technology-stack winner, predict scalability, or promote any queued/unqualified Luminous subject to PASS.
