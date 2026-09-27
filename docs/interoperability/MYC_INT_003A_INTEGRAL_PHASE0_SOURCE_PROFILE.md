# MYC-INT-003A0 — Integral Phase-0 source hierarchy and registry fixture

Status: Documentation/fixture preregistration. No executable Integral compatibility is claimed.

## Purpose

Freeze the public Integral source hierarchy that an eventual Mycelix adapter must respect before field-level translation code is written.

This tranche exists because Integral currently publishes several overlapping but non-identical specification surfaces. They are useful together, but they are not interchangeable and must not be flattened into one anonymous `Integral` schema.

## Source hierarchy observed 2026-09-27

### 1. Technical Specifications

Public source: `https://integralcollective.io/documents/specifications.html`

The page describes itself as the builder-facing source of truth for schemas, interfaces, and contracts. It distinguishes `RATIFIED`, `DRAFT`, and `PENDING` work.

At the observation date it lists six **DRAFT** core data structures:

- `SPEC-DS-01` — Certified Design
- `SPEC-DS-02` — Labor Event
- `SPEC-DS-03` — Material Consumption Event
- `SPEC-DS-04` — ITC Ledger Entry
- `SPEC-DS-05` — FRS Signal Packet
- `SPEC-DS-06` — Decision Packet

It also lists three **PENDING** interface contracts:

- `SPEC-IF-01` — OAD → COS
- `SPEC-IF-02` — COS → ITC
- `SPEC-IF-03` — FRS → CDS

and one **PENDING** technology-stack decision:

- `SPEC-STACK-01` — technology stack.

Therefore:

```text
DRAFT schema
!= RATIFIED schema

PENDING interface
!= implemented interface

published architecture
!= chosen technology stack
```

### 2. Development Guide v0.1

Public source: `https://integralcollective.io/documents/integral_devguide_v01.pdf`

The guide is a builder-oriented bridge from the white-paper architecture to a first implementation. It contains more detailed Phase-2/minimal record shapes, additional objects such as `ITCAccount`, `DiagnosticFinding`, and `Recommendation`, and proposed API function signatures.

The guide explicitly states that some minimal schemas consolidate higher-level white-paper objects and that the interface function signatures are development-guide architectural proposals rather than direct white-paper transcriptions.

Therefore:

```text
DevelopmentGuide object
!= automatically the current TechnicalSpecification object

minimal/consolidated schema
!= lossless white-paper formal model

proposed interface signature
!= ratified interface contract
```

### 3. White Paper v0.1

Public source: `https://integralcollective.io/documents/whitepaper.html`

The white paper owns the higher-level architecture and formal specification intent. It is explicitly a versioned reference open to revision.

The adapter must preserve its source identity when a field or object is taken from the white paper rather than silently attributing it to a newer builder specification.

### 4. System/module pages

Pages such as `/system/cds.html`, `/system/oad.html`, and `/system/frs.html` provide useful explanatory descriptions and simulations. They are evidence about documented behavior and terminology, but they do not silently override the Technical Specifications or Development Guide.

## Governing source theorem

```text
same project
!= same specification authority

same object name
!= same source revision

same field spelling
!= same semantic contract
```

Every external fixture must retain enough metadata to answer:

1. Which source family defined this object?
2. What project/document version or status was observed?
3. Was it DRAFT, RATIFIED, PENDING, explanatory, or a higher-level formal reference?
4. When was it observed by the fixture author?
5. Which mapping/translation profile interpreted it?
6. Did a sibling Integral document define a materially different or broader shape?

## Registry-fixture scope

The companion JSON fixture freezes only the currently observed **registry surface** and mapping risk. It does not claim to reproduce complete Integral schemas.

Field-level fixtures should be added incrementally and should cite/source the exact Integral definition they model.

## Initial Mycelix mapping hypotheses

These are test hypotheses, not canonical equivalence claims.

| Integral object | Candidate Mycelix view | Required boundary |
| --- | --- | --- |
| Certified Design | versioned artifact/design + certification evidence | external certification != local certification acceptance |
| Labor Event | contribution/work attestation profile | contribution evidence != universal value |
| Material Consumption Event | resource/production observation profile | observation != ecological policy conclusion |
| ITC Ledger Entry | external accounting/contribution event | ITC amount != universal currency/value |
| FRS Signal Packet | bounded derived summary/evidence bundle | summary != source-owned current fact |
| Decision Packet | governance decision lineage + constraints/review refs | decision != local authorization |
| Diagnostic Finding | external analytical finding | finding/severity/confidence != truth or authority |
| Recommendation | external non-executive recommendation | recommendation != decision/authorization/effect |

## Source-status preservation

A fixture imported while `SPEC-DS-06` is DRAFT must remain historically identified as a DRAFT observation even if Integral later ratifies a successor.

```text
DraftV1 observed at T1
-> later RatifiedV2
```

must not become:

```text
rewrite DraftV1 history as RatifiedV2
```

A new source observation/schema identity is required.

## Unknown and future fields

Unknown external fields are not silently discarded when they could affect identity or meaning. A profile must explicitly choose one of:

- reject unsupported revision;
- preserve opaque extension data under bounded rules;
- project known fields while emitting explicit loss metadata.

A successful parser is not evidence of semantic equivalence.

## Source drift

If the Technical Specifications, Development Guide, and white paper differ, the adapter records the disagreement. It does not select whichever definition is most convenient.

A later mapping decision may define an explicit precedence/profile for a particular integration, but that decision is itself versioned adapter policy and cannot rewrite the source documents.

## Authority boundary

Nothing in this source registry establishes:

- that Integral has selected Mycelix, Holochain, Xenia, or Symthaea;
- that a DRAFT specification is ratified;
- that a Development Guide proposal is production API behavior;
- that an imported Decision Packet creates local authority;
- that an FRS finding/recommendation is true or executable;
- that an OAD certification is locally accepted;
- that ITC values have generic Mycelix economic meaning.

## Qualification direction

A later exact fixture qualifier should independently verify at least:

1. fixture JSON parses under a closed fixture schema;
2. exactly six Technical-Specification core data structures are registered for this observation;
3. each is marked `draft`;
4. exactly three interface specifications are registered as `pending`;
5. the technology-stack decision is `pending`;
6. Development-Guide-only objects remain distinguishable from Technical-Specification objects;
7. mutation from `draft` to `ratified` changes the source observation/fixture identity;
8. source-family substitution fails;
9. mapping hypotheses grant no authority;
10. no external source status is strengthened by Mycelix.

## Relationship

- #3140 owns the broader MYC-INT-003A external-schema fixture program.
- #3139 owns composition with existing Mycelix EPI/provenance roots.
- #3142 owns the later generic semantic seam/delivery profile.
- #3119 remains the eventual water-system end-to-end qualification.

## Nonclaims

This document is a source-registry preregistration only. It does not establish complete schema fidelity, executable adapter behavior, network compatibility, source authenticity, currentness beyond the recorded observation, political legitimacy, factual truth, or production readiness.
