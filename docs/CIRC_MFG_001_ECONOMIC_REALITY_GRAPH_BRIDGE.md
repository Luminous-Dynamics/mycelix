# CIRC-MFG-001 — Manufacturing-to-Circularity Economic Reality Graph bridge

Status: architecture freeze for #3095. This document defines event/ref ownership and nonclaims. It does not establish recycled content, environmental superiority, circularity certification, legal compliance, supplier sustainability, physical recovery yield, or production qualification.

## Purpose

Connect typed manufacturing provenance to the shared Mycelix Economic Reality Graph so products, components and materials can move through use, service, repair, reuse, refurbishment, remanufacturing, parts harvesting, recycling, recovery and disposal without creating a second lifecycle database.

The graph is circular by construction:

```text
resource / feedstock
 -> processing
 -> manufacturing
 -> distribution
 -> use
 -> service / repair
 -> reuse / refurbishment / remanufacture
 -> recovery / recycling
 -> qualified secondary resource
 -> manufacturing
```

## Core non-equivalences

```text
manufacturing record
!= complete lifecycle
```

```text
circularity event
!= sustainability claim
!= verified impact assessment
!= regulatory compliance
```

```text
product-passport projection
!= canonical lifecycle database
```

```text
recycled-content claim
!= lineage/accounting evidence
```

## Ownership

### Manufacturing owns

- work-order / batch / lot / serial production subjects;
- exact Symthaea process-plan / recipe or recipe-commitment refs;
- resource/provider/site assignments;
- manufacturing execution/evidence refs;
- DesignBOM / PurchasedBOM / AsBuiltBOM refs;
- output material/component/article subjects;
- inspection/qualification refs.

### Economic Reality Graph owns

The canonical event history over physical/economic subjects, including transformation, custody, ownership, location and evidence links.

Manufacturing and Circularity are semantic profiles over this shared graph rather than independent provenance systems.

### Circularity owns lifecycle event profiles

At minimum:

```text
Return
Reuse
Resell
Service
Repair
Refurbish
Remanufacture
HarvestParts
Recycle
RecoverMaterial
Dispose
```

A recovered component or material may later become an admitted manufacturing input under a new qualified state. It does not remain permanently typed as waste.

### FIELD / metrology own physical observations

Physical composition, condition, inspection, recovery yield and article state remain evidence-bearing observations. A database transition cannot invent physical state.

### Symthaea owns engineering intent and lifecycle co-design

Symthaea may propose repair/disassembly/remanufacture/recovery routes and evaluate tradeoffs. Mycelix records and coordinates actual distributed lifecycle events.

## Manufacturing output bridge

A manufacturing output should be able to bind exact refs to:

```text
canonical process plan
plan instance
work order / production scope
process step / assignment
recipe or private recipe commitment
input material/component subjects
DesignBOM / PurchasedBOM / AsBuiltBOM
execution receipt
output lot / serial / article
inspection / FIELD / QIF evidence
```

The output article then becomes a normal Economic Reality Graph subject that can participate in downstream lifecycle events.

## Lifecycle event subject

A future event profile should retain concepts equivalent to:

```text
LifecycleEventV1 {
  event_id,
  event_kind,
  subject_refs,
  input_subject_refs,
  output_subject_refs,
  facility / party / provider refs,
  process_plan_or_recipe_refs,
  evidence_refs,
  time / location refs,
  chain_of_custody_profile_ref,
  correction / supersession refs,
}
```

This is descriptive event history. It is not a compliance or impact conclusion.

## Append-only corrections

Lifecycle history must preserve the distinction between event time and knowledge/correction history.

```text
Event A
 -> Correction B supersedes A
 -> Retraction C invalidates a claim derived from A
```

Do not mutate the original event as though incorrect or newly discovered history never existed.

## Chain-of-custody strategy

Physical identity is not preserved equally across all transformations. Keep accounting strategy explicit, for example:

```text
IdentityPreserved
Segregated
ControlledBlending
MassBalance
BookAndClaim
```

These are not interchangeable.

A recycled polymer blend, smelter batch or chemical pool must not claim fictitious serial-level atom identity merely because its database ancestors are individually named.

## Material-flow closure

Material accounting should support split, combine, loss, waste and recovery flows.

Required invariant under the declared accounting profile:

```text
attributable recovered output
<= attributable input + separately evidenced added material
```

Any process loss, scrap, residue or disposal required to close material balance remains explicit.

Do not double-count the same physical quantity across multiple circular outcomes.

Example:

```text
component harvested intact
```

and

```text
same component mass recycled as constituent material
```

cannot both receive full retained-mass allocation under one accounting lineage.

## Value-retention disposition

Keep lifecycle dispositions explicit rather than collapsing them to `recycled=true`:

```text
ReusedAsIs
RepairedAndReturnedToUse
Refurbished
Remanufactured
ComponentHarvested
MaterialRecycled
MaterialRecovered
Downcycled
Disposed
UnknownDisposition
```

A value-retention classification is descriptive under a profile. It does not automatically establish a sustainability ranking.

## Secondary feedstock bridge

Recovered material becomes useful to manufacturing only after its new state is established.

Target lineage:

```text
end-of-life article
 -> recovery event
 -> recovered-material subject
 -> characterization / purification / qualification evidence
 -> qualified secondary-feedstock material state
 -> new manufacturing input
```

Required theorem:

```text
recovered material
!= specification-grade secondary feedstock
```

Material state/property authority remains with the appropriate engineering/material evidence layer.

## Environmental and social claims are projections over events

Physical events may support many claims without becoming those claims themselves:

```text
physical transformation event
  ├── provenance claim
  ├── quality claim
  ├── environmental-impact claim
  ├── circularity claim
  ├── labor/compliance attestation
  ├── legal/compliance decision
  └── commercial/financial records
```

Each claim binds its own method, evidence and authority.

```text
factory event exists
!= carbon footprint measured
```

```text
LCA result exists
!= factory emissions observed
```

## Product-passport projections

Passports are interoperable views/projections over canonical product and lifecycle subjects.

```text
Economic Reality Graph
   ├── EU Digital Product Passport projection
   ├── GS1 EPCIS / Digital Link projection
   ├── ISO 59040 Product Circularity Data Sheet projection
   ├── consumer provenance view
   ├── repair passport
   └── recycler/material passport
```

Adapters are versioned and mapping fidelity must be explicit.

A standards adapter may not silently become the canonical internal ontology.

## Purchase Passport vs Product Passport

Keep consumer-private purchase context separate from public/regulatory product lifecycle information.

```text
Digital Product Passport
= product/value-chain information view

Purchase Passport
= customer-controlled receipt/warranty/return/repair/recycling projection
```

A Purchase Passport may point to a DPP. It must not write customer identity, payment details or private purchase history into public/regulatory product-passport data.

## Privacy and selective disclosure

Manufacturing and lifecycle actors may need to prove bounded facts without revealing proprietary process details.

Compose existing typed refs and recipe/aggregate commitment semantics for:

- exact public evidence;
- private encrypted evidence refs;
- commitment-only evidence;
- aggregate capability/lifecycle evidence;
- selective-disclosure profile refs.

Required theorem:

```text
commitment verifies
!= hidden payload disclosed
!= hidden claim established beyond the verifier/profile semantics
```

Privacy mechanisms may not weaken material-flow closure or qualification/currentness requirements.

## Interoperability adapters

Initial outward mappings may include:

- GS1 EPCIS / Digital Link;
- ISO 59040 Product Circularity Data Sheets;
- EU ESPR Digital Product Passport;
- chain-of-custody profiles aligned with applicable ISO 22095-family semantics;
- W3C Verifiable Credentials for portable attestations where useful;
- sector-specific environmental/product passport formats.

All mappings are versioned adapters.

```text
mapping implemented
!= conformance certified
```

## Adversarial corpus

1. article has a DPP reference but no repair event -> cannot claim repaired;
2. recycled-content claim lacks material-flow/custody evidence -> unresolved;
3. mixed recycled feedstock cannot retain fictitious serial-level atom identity;
4. the same recovered mass cannot satisfy both parts-harvest and material-recycle allocation;
5. private recipe commitment cannot reveal or imply secret recipe contents;
6. corrected lifecycle event appends/supersedes instead of rewriting history;
7. disposal cannot be omitted merely to improve a circularity view;
8. adapter-version change does not mutate canonical event history;
9. consumer purchase data remains outside public/regulatory passport projections;
10. circularity evidence cannot mint legal compliance;
11. output article with missing inspection evidence remains an unqualified output subject;
12. recovered constituent without characterization cannot become strict manufacturing feedstock;
13. mass-balance accounting cannot be presented as identity-preserved custody;
14. external impact assessment cannot rewrite the underlying physical event;
15. later remanufacturing creates a new article/configuration lineage rather than overwriting the original.

## Initial implementation sequence

1. `CIRC-MFG-001A` — this ownership/event architecture.
2. `CIRC-MFG-001B` — typed manufacturing-output -> lifecycle-event bridge.
3. `CIRC-MFG-001C` — material-flow/mass-balance subject refs and hostile fixtures.
4. `CIRC-MFG-001D` — return/repair/refurbish/remanufacture/recycle event profiles.
5. `CIRC-MFG-001E` — secondary-feedstock bridge back into manufacturing.
6. `CIRC-MFG-001F` — ISO 59040 / DPP / EPCIS projection adapters.
7. `CIRC-MFG-001G` — privacy/selective-disclosure qualification.
8. `CIRC-MFG-PILOT-001` — synthetic closed loop from primary feedstock through product/use/repair/recovery into secondary feedstock and a second manufacturing subject.

## Exit criterion

Mycelix can preserve one traversable event history from manufacturing through service, return, repair, reuse/remanufacture, recovery and subsequent manufacturing; close material/accounting flows without fictitious traceability; expose versioned passport/circularity views without duplicating canonical state; and keep physical events, impact/circularity claims, private customer context and compliance decisions as distinct authority planes.