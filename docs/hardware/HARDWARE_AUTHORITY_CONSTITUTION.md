# Mycelix Hardware Authority Constitution

**Status:** MHW-000
**Authority:** architectural boundary specification
**Scope:** open hardware, manufacturing interoperability, engineering evidence, physical observations, and hardware federation

## Purpose

Mycelix Hardware defines the semantic identity and relationship layer for physical technology. It does not replace CAD, EDA, manufacturing execution, simulation, safety engineering, cryptographic identity, or reproducible-build infrastructure. Instead, it provides stable meanings and typed references that those systems can share without silently acquiring authority they do not possess.

This document freezes the first authority boundaries for the hardware workstream before new Holochain entries, automation, or design-generation features are introduced.

## Semantic root

Mycelix is the semantic root for:

- hardware project identity;
- design identity and revision lineage;
- component identity and design composition;
- declared hardware requirements;
- relationships between designs, modules, components, and derivatives;
- documentation and licensing metadata;
- references to engineering, manufacturing, provenance, and field evidence;
- federation of those semantic records.

A Mycelix record may state that an external artifact, receipt, report, or attestation exists and is associated with a subject. That reference does not inherit the authority of the referenced system unless an explicit verification rule says so.

## System responsibilities

### Mycelix Hardware

Owns semantic identity, lineage, typed relationships, documentation profiles, component graphs, and evidence references.

It MUST NOT:

- claim that a design is safe merely because evidence exists;
- claim that a physical specimen conforms merely because a manufacturing receipt exists;
- treat a signed assertion as physical truth;
- redefine solver, CAD, regulatory, or manufacturing semantics that belong to their source systems;
- turn missing information into a positive result.

### Symthaea Engineering

Owns engineering reasoning, requirement analysis, tool orchestration, interpretation, uncertainty tracking, and proposal generation.

It MAY:

- propose requirements or alternatives;
- request simulations and checks;
- compare evidence against scoped claims;
- identify contradictions and unknowns;
- propose design changes.

It MUST NOT:

- silently mutate authoritative design state;
- mark a claim satisfied solely because a tool invocation converged or exited successfully;
- convert inferred compatibility into qualified equivalence;
- convert documentation completeness into safety or regulatory approval.

### Symthaea Fabrication Kernel

Owns exact artifact inventories, provenance, attestations, audit evidence, fabrication-oriented geometry/toolpath mechanics, and other execution-evidence primitives already defined by that subsystem.

Hardware work SHOULD reuse those primitives instead of introducing parallel digest, signature, provenance, or audit schemes.

### Mycelix Manufacturing

Owns operational production semantics including work orders, manufacturing BOMs, routing, machines, capacity, scheduling, material requirements, and execution state.

It does not own canonical design truth.

In particular:

```text
design composition
!= manufacturing BOM

engineering-approved component
!= procurement SKU

design quantity
!= production quantity

work order complete
!= conforming specimen
```

### Xenia

Owns cryptographic identity, authorization binding, signature verification, and trust relationships where Xenia is explicitly used.

A valid signature proves only the signed statement under the verified identity/key policy. It does not prove the physical world matches the signed statement.

### Nix / Nixward

Own reproducible and policy-constrained engineering environments where adopted.

A reproducible environment can strengthen a claim about repeatable execution. It does not by itself prove that a design is correct, manufacturable, safe, or physically reproduced.

### External engineering tools

KiCad, ngspice, OpenROAD/OpenLane, FreeCAD, OpenFOAM, OpenSees, and other external tools remain authoritative for their own native execution semantics.

Adapters MUST preserve the distinction between:

- tool execution;
- parsed result;
- interpretation;
- engineering claim;
- release/deployment authorization.

## Core non-equivalences

The following boundaries are normative:

```text
design source available
!= open-hardware compliant

open-hardware compliant
!= documentation complete

documentation complete
!= reproducible

reproducible
!= manufacturable

manufacturable
!= manufactured

manufactured
!= conforming specimen

conforming specimen
!= safe

simulation converged
!= simulation supports the intended claim

ERC clean
!= electrically correct

DRC clean
!= manufacturable by a specific facility

candidate substitute
!= verified replacement

pin-compatible
!= electrically equivalent

electrically compatible
!= firmware-compatible

bench validated
!= production qualified

test passed
!= regulatory certification

cryptographic provenance
!= physical truth

valid signature
!= truthful assertion

measurement exists
!= requirement satisfied
```

Code and schemas in the hardware workstream SHOULD encode these distinctions structurally where practical rather than relying only on documentation.

## Evidence model

Evidence is typed and scoped.

Every evidence-bearing relationship SHOULD identify at least:

- the subject it applies to;
- the claim or observation class;
- the exact referenced artifact or receipt identity;
- the producing authority or tool;
- the relevant environment/configuration identity where material;
- any profile or conditions under which the evidence is interpreted;
- whether the relationship is direct observation, simulation, derivation, assertion, inspection, proof, or other evidence class.

Evidence presence MUST NOT automatically discharge an engineering obligation.

The hardware layer SHOULD depend on the hardened Symthaea formal-safety receipt model rather than duplicating it.

## Physical-world boundary

Physical observations require stronger scoping than digital artifacts.

A physical observation SHOULD bind, when applicable:

- specimen identity;
- design revision identity;
- manufacturing batch or process identity;
- measured quantity and units;
- uncertainty or tolerance;
- instrument identity;
- calibration reference;
- procedure identity;
- environmental conditions;
- raw-data digest;
- observation timestamp;
- observer/operator identity or agent reference.

A physical observation is evidence about the identified specimen under the recorded conditions. It MUST NOT be generalized automatically to every specimen, every batch, or every future revision.

## Unknown is a first-class state

Hardware interoperability and matchmaking MUST preserve unknown information.

For any constraint evaluation, adapters SHOULD prefer a result model such as:

```text
Satisfied
Unsatisfied
Unknown
NotApplicable
```

rather than coercing missing information into `false`, zero, an empty string, or a successful default.

## Interoperability policy

Mycelix Hardware uses an internal canonical model and external standards through loss-aware adapters.

Initial interoperability targets include:

- Open Know-How;
- Open Know-Where;
- SPDX Hardware profiles;
- CycloneDX hardware/manufacturing BOM representations;
- OSHWA-compatible metadata and documentation expectations;
- DIN SPEC 3105 documentation/assessment concepts where applicable.

Adapters MUST report semantic loss explicitly. A recommended report shape is:

```text
InteropReport
├── preserved
├── approximated
├── omitted
├── unsupported
└── warnings
```

An adapter MUST NOT invent facts solely to satisfy a target schema.

## Design versus manufacturing model

The semantic hardware core should introduce a design-oriented composition model rather than reusing `mycelix-manufacturing::BillOfMaterials` as canonical design truth.

Recommended separation:

```text
HardwareProject
  -> DesignRevision
      -> DesignComposition
          -> ComponentIdentity / sub-design references

DesignRevision
  -> ManufacturingProjection
      -> BillOfMaterials
      -> RoutingSequence
      -> WorkOrder
```

The projection is directional and evidence-bearing. Manufacturing may derive operational records from a released design, but manufacturing state does not rewrite the design revision.

## Release model

A hardware release is a semantic profile over exact artifacts and provenance, not a new artifact-security subsystem.

A release SHOULD compose:

- design revision identity;
- exact artifact-set identity;
- artifact provenance identity;
- component/design-composition identity;
- documentation profile identity;
- license references;
- environment/toolchain identity where relevant;
- engineering evidence references.

Existing Symthaea fabrication-kernel artifact-set, provenance, signature, trust, and audit primitives SHOULD be reused where technically appropriate.

## Mutation policy for engineering integrations

Initial CAD/EDA integrations SHOULD follow:

```text
observe
-> analyze
-> propose
-> verify
-> human/app authority review
-> apply
```

Read-only adapters are preferred before write-capable adapters.

A later write-capable integration MUST bind the proposed mutation to:

- exact base design identity;
- exact requested operation;
- actor/authority;
- generated diff or replacement artifact identity;
- verification results;
- explicit apply authorization.

## Federation boundary

The pure semantic core MUST remain usable without Holochain.

Holochain projection/federation SHOULD be layered after:

1. semantic types stabilize;
2. serialization and validation rules are deterministic;
3. external-standard adapters prove the model can round-trip useful information;
4. the target Mycelix Holochain dependency baseline is normalized.

This avoids binding canonical hardware meaning to a transient runtime version.

## Initial implementation order

The first hardware tranche is:

1. **MHW-000** — this authority constitution;
2. **MHW-001** — pure Rust hardware semantic core;
3. **MHW-002** — standards/interoperability airlock;
4. **MHW-003** — hardware release profile over existing artifact/provenance primitives;
5. **SHW-001** — hardware evidence adapter after formal-safety hardening;
6. **SHW-002** — read-only KiCad bridge;
7. **SHW-003** — component-substitution assessment;
8. manufacturing projection/capability work;
9. physical observation receipts;
10. federation and UI after the semantic/evidence model is stable.

## Non-goals for the first tranche

The first tranche explicitly does not attempt to:

- replace KiCad, FreeCAD, OpenROAD, OpenLane, SPICE, FEA, or CFD tools;
- automatically approve generated hardware;
- autonomously release safety-critical physical products;
- claim regulatory certification;
- create a second cryptographic provenance stack;
- create a second generic formal-safety system;
- add a Holochain DNA before the pure semantic ABI is validated;
- build an AI PCB generator before read-only analysis and evidence boundaries are trustworthy.

## Acceptance criteria for this constitution

MHW-000 is complete when subsequent hardware PRs can answer all of the following without ambiguity:

1. Which subsystem owns the semantic fact being introduced?
2. Which subsystem produced the evidence?
3. What exact claim does that evidence support?
4. What does the evidence explicitly *not* prove?
5. Is missing information represented as unknown rather than inferred success?
6. Does the change reuse existing provenance/evidence infrastructure where possible?
7. Does operational manufacturing state remain separate from design truth?
8. Can the semantic type exist independently of Holochain/runtime transport?

If any answer is unclear, the dependent PR should remain blocked until the authority boundary is explicit.
