# AeroCommons Holochain Validation Boundary V1

## Status

Design contract. This document defines the validation ceiling for a future AeroCommons integrity layer. It does not implement a Holochain graph and does not establish engineering truth, certification, or airworthiness.

## 1. Core rule

A Holochain validation callback may establish protocol validity for an authored operation and deterministic dependency relationships.

It must never silently transform protocol validity into:

- physical correctness;
- safety;
- certification;
- airworthiness;
- manufacturing conformity;
- truth of a simulation;
- validity of a measurement beyond the evidence semantics already declared by its producer and method.

Holochain validation is therefore the **protocol integrity boundary**, not the engineering authority boundary.

## 2. Determinism ceiling

Validation must produce the same result for the same operation and addressable dependencies.

Integrity validation must not depend on:

- current link collections;
- current DHT metadata;
- wall-clock time;
- mutable external services;
- live reputation or MATL state;
- aggregate counters;
- model inference;
- current certification/status projections;
- non-addressable mutable state.

If a required dependency cannot be retrieved, validation must remain indeterminate rather than inventing a value or treating absence as invalidity.

## 3. Dependency classes

### D0 — operation-local

Data contained directly in the operation, action, or entry.

Examples:

- schema/profile identifier;
- engineering identity reference;
- evidence lifecycle value;
- canonical field shape.

Validation may inspect D0 directly.

### D1 — addressable structural dependency

A dependency identified by a deterministic hash/address.

Examples:

- predecessor action;
- referenced entry;
- configuration record;
- explicitly referenced evidence record.

Use deterministic `must_get_*` retrieval. Retrieval alone does not establish dependency validity.

### D2 — inductively validated dependency

A dependency whose protocol semantics require the referenced CreateRecord to have passed validation.

Use `must_get_valid_record` where appropriate.

Important limitation: this establishes the referenced CreateRecord's validity contract. It does not automatically prove that later Update/DeleteLink operations associated with that record are valid.

### D3 — external engineering payload

Large CAD, STEP/AP242, QIF, CAE, manufacturing, metrology, or test payloads stored outside the DHT.

The integrity layer may validate:

- explicit external identity;
- declared content commitment;
- profile/version;
- provenance binding.

It does not validate the physical contents of the payload.

### D4 — epistemic/engineering interpretation

Examples:

- whether a simulation predicts reality;
- whether an inspection demonstrates a requirement;
- whether a test result is sufficient for certification;
- whether a configuration is safe to operate.

These are outside Holochain validation.

## 4. Identity separation

The following MUST remain distinct:

| Identity | Meaning |
|---|---|
| EntryHash | Holochain entry-content identity |
| ActionHash | Holochain authored action identity |
| Artifact identity | Engineering artifact identity |
| Content identity | Payload/content commitment |
| Configuration identity | Frozen engineering configuration |
| Execution identity | One execution/observation instance |
| SourceEvidence identity | Source/evidence event identity |
| EpistemicRelation identity | Consequential claim/relation identity |

No validator may infer one identity from another unless an explicit protocol mapping says so.

In particular:

- EntryHash != engineering identity;
- ActionHash != execution identity;
- foreign STEP/QIF identifier != native AeroCommons identity;
- reputation identity != evidence identity.

## 5. Links are navigation, not truth

Ordinary Holochain links may index structural relationships.

A validator MUST NOT use mutable link collections as evidence of current engineering state.

Consequential epistemic relationships require their own provenance-bearing records or references to such records. A link pointing at a relation does not itself become the relation.

## 6. Lifecycle and invalidation

A configuration change may cause evidence to become:

- unaffected;
- conditionally valid;
- requiring review;
- invalidated;
- unknown.

That classification belongs to the engineering evidence/impact layer.

The Holochain validator only verifies that the submitted ChangeSet/Evidence/Relation record conforms to the deterministic protocol and references addressable dependencies correctly.

It does not decide whether the underlying engineering conclusion is correct.

## 7. Adversarial qualification corpus

Future integrity tests MUST include:

1. valid evidence followed by an invalidating update;
2. valid evidence followed by a deleted/index link;
3. temporarily unavailable dependency;
4. peers observing different link state;
5. valid CreateRecord with invalid later operation;
6. EntryHash substituted for ActionHash;
7. ActionHash substituted for Execution identity;
8. reputation changes without evidence changes;
9. current certification projection changes without source-record changes;
10. external STEP/QIF identifier collision;
11. private payload with public commitment;
12. superseded relation referenced as current without an explicit lifecycle record.

## 8. Digital-thread boundary

External standards remain the engineering artifact layer.

NIST describes STEP, QIF, and MTConnect as complementary standards for lifecycle integration and has demonstrated spatial/temporal association across design, execution, and quality data.

AeroCommons therefore binds these external artifacts into an evidence/provenance envelope rather than attempting to replace their engineering semantics.

## 9. Authority ceiling

The strongest statement an AeroCommons integrity validator may make is:

> This authored operation conforms to the protocol's deterministic structural and dependency rules under the referenced identity/profile environment.

It may not say:

> This aircraft/component/design is safe, compliant, certified, or airworthy.

Those claims require the appropriate engineering, inspection, organizational, and regulatory authorities.

## 10. Exit condition

No AeroCommons Holochain engineering graph should be considered implementation-ready until:

- every dependency class has an explicit validation rule;
- every non-deterministic dependency is excluded;
- EntryHash/ActionHash/engineering identity substitution tests exist;
- CreateRecord versus later-operation semantics are tested;
- unresolved dependencies are explicitly modeled;
- negative evidence remains queryable;
- validation receipts cannot be interpreted as certification evidence.

## References

- Holochain validation and deterministic dependency semantics.
- Holochain `must_get_*` and `must_get_valid_record` semantics.
- NIST Digital Thread for Smart Manufacturing.
- NIST spatial/temporal alignment of STEP AP242, MTConnect, and QIF data.
