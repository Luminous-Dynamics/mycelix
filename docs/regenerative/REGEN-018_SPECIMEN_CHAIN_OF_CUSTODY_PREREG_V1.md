# REGEN-018 — Specimen Chain-of-Custody Preregistration v1

Status: preregistration only. This document defines evidence semantics for physical specimens and their handling. It establishes no sampling protocol, laboratory competence, scientific truth, legal authority, agronomic recommendation, or physical-action authority.

## 1. Purpose

REGEN already distinguishes regenerative subjects, environmental observations, material batches, contamination evidence, field trials, and contextual suitability. A remaining evidentiary gap sits between those layers:

> how do we know which physical specimen an analytical result refers to, and what happened to that specimen between collection and analysis?

REGEN-018 freezes that boundary before executable code exists.

Its core theorem is:

```text
subject identity
+ specimen reference
+ declared collection
+ declared custody chain
+ declared preparation / subsampling lineage
+ analytical result binding
!= representative sampling
!= laboratory competence
!= analytical correctness
!= legal authority
```

Chain of custody strengthens traceability. It does not upgrade the scientific or legal strength of the underlying evidence by itself.

## 2. Identity discipline

The current qualified REGEN-002 identity grammar contains site/plot, biomass lot, biochar/compost/amendment batch, trial, treatment arm, facility, project, quality profile, and evidence-bundle identities.

It does **not** currently define a typed `SampleId` or `SpecimenId`.

REGEN-018 therefore MUST NOT silently widen the already-qualified REGEN-002 identity grammar.

The first implementation SHOULD use a bounded exact opaque `specimen_ref` / `subspecimen_ref` supplied by the owning collection or laboratory system.

A future typed regenerative specimen identity may be introduced only through a separately reviewed identity-protocol revision with its own compatibility vectors and qualification lineage.

```text
opaque specimen_ref in REGEN-018
!= new REGEN-002 subject kind
```

## 3. Supported parent subjects

A specimen may be declared as sampled from an exact existing subject such as:

- `SoilPlotId`;
- `BiomassLotId`;
- `BiocharBatchId`;
- `CompostBatchId`;
- `CoCompostedAmendmentBatchId`;
- a field-trial arm/context where the owning trial contract supplies the exact relation.

The parent binding asserts only the declared sampling relationship.

```text
specimen says it came from subject X
!= independently proven origin
```

Origin evidence remains separately reviewable.

## 4. Collection event

A specimen collection record SHOULD bind at least:

```text
specimen_ref
parent_subject
collection_event_ref
collector_ref?
collection_time
collection_spatial_support?
collection_method_ref?
sample_support
container_or_seal_ref?
collection_evidence_refs
```

Unknown values remain unknown. Implementations MUST NOT manufacture collection time from ingestion time, collector identity from uploader identity, or spatial support from the parent subject merely to complete a record.

The collection method reference identifies a declared method/protocol revision. Its presence does not prove the method was correctly followed.

## 5. Collection event != representativeness

A valid collection record does not establish that the specimen represents its parent plot, lot, batch, trial arm, or population.

```text
valid specimen origin binding
!= representative specimen
```

Representativeness depends on the relevant sampling design, spatial/temporal structure, heterogeneity, exclusions, sample count, compositing logic, and consuming protocol.

No generic REGEN-018 `representative=true` flag should exist in v1.

## 6. Point, composite, and pooled specimens

REGEN-010 already distinguishes point/composite/unspecified support for soil evidence. REGEN-018 generalizes the physical lineage concept without replacing domain-specific sampling semantics.

For composite or pooled specimens, the lineage SHOULD identify constituent specimen references when known.

```text
composite specimen
!= homogeneous parent material
!= equal constituent contribution
!= representative composite
```

Unknown constituent count or contribution MUST remain explicit rather than being inferred from the final container.

## 7. Custody events

A custody chain is an ordered sequence of declared custody/transfer events, conceptually including:

```text
specimen_ref
from_custodian_ref?
to_custodian_ref
transfer_event_ref
transferred_at?
received_at?
condition_or_storage_ref?
container_or_seal_state_ref?
evidence_refs
```

A custodian reference identifies the actor/system/facility declared to possess the specimen at that stage. It does not itself establish identity assurance, legal ownership, professional qualification, or permission to possess/process the material.

```text
custody
!= ownership
!= collection authority
!= transfer authority
!= analytical competence
```

## 8. Gaps must remain visible

REGEN-018 MUST be able to distinguish at least:

- declared continuous custody evidence;
- a known custody gap;
- unresolved custody state;
- conflicting custody records.

A missing record MUST NOT be silently interpreted as an uneventful transfer.

```text
no recorded custody problem
!= continuous custody proven
```

A downstream profile may reject a specimen with a known/unresolved gap, but the generic chain records the state rather than inventing one universal acceptance rule.

## 9. Seal / container semantics

A seal/container identifier or declared intact state may strengthen tamper evidence when its owning system is trustworthy.

It does not prove:

- the correct material was placed in the container;
- the specimen was uncontaminated before sealing;
- the seal technology is secure;
- environmental storage conditions were acceptable;
- the specimen is representative.

```text
seal intact
!= specimen scientifically valid
```

## 10. Physical transformation and subsampling lineage

A laboratory or field workflow may split, combine, grind, dry, sieve, homogenize, digest, extract, dilute, preserve, or otherwise transform a specimen before analysis.

REGEN-018 SHOULD represent those operations as explicit specimen-lineage events rather than overwriting the original specimen identity.

Conceptually:

```text
SpecimenTransformation {
    operation_ref,
    input_specimen_refs,
    output_specimen_refs,
    performed_at?,
    performer_ref?,
    method_ref?,
    evidence_refs
}
```

A child aliquot/subsample MUST retain its parent relation.

```text
parent specimen
-> subsample A
-> subsample B
```

is not equivalent to mutating the parent specimen into A and losing B.

## 11. Split, merge, pool, consume

Physical lineage MUST preserve split/merge/pool/consumption semantics where material accounting matters.

If quantities are recorded, incompatible quantity bases cannot be silently reconciled, and unmeasured loss or residual material cannot be forced to zero.

REGEN-018 does not define a universal physical-quantity ontology; it should reuse an owning quantity/evidence contract when available.

## 12. Destructive analysis

Some analytical operations consume all or part of a specimen.

A destructive operation SHOULD be able to mark the relevant child/aliquot as consumed or no longer available without erasing historical identity/evidence.

```text
specimen consumed
!= specimen record deleted
```

Historical result provenance must remain reviewable after physical material is gone.

## 13. Analytical result binding

A laboratory result should bind an exact specimen/subspecimen reference to the exact PEF observation/product being used downstream.

Conceptually:

```text
AnalyticalResultBinding {
    specimen_ref,
    observation_id,
    expected_phenomenon,
    expected_class?,
    analytical_method_ref?,
    laboratory_ref?,
    report_ref?,
    analysis_event_ref?
}
```

Resolved validation inherits the REGEN-010 / PEF provenance firewall:

```text
Reported | Observed
-> may be raw validated evidence

Derived | Inferred | Forecast | Scenario
-> validated lineage required
```

The binding MUST NOT duplicate PEF measurement, unit, uncertainty, spatial, temporal, or generic provenance fields.

## 14. Specimen substitution firewall

A resolver or adapter MUST NOT silently substitute one specimen for another because both refer to the same parent plot/batch or because their labels are similar.

```text
requested specimen_ref == resolved specimen_ref
```

should be exact unless a separately defined alias/migration theorem applies.

Likewise:

```text
same parent batch
!= same specimen
```

## 15. Result identity != immutable result bytes

A specimen/result binding by observation ID is not automatically a cryptographic commitment to immutable analytical-result bytes.

REGEN-018 inherits the same limitation identified in REGEN-010:

```text
same logical identifier
!= content-addressed immutable artifact
```

Where a laboratory report, raw instrument file, or canonical evidence object exposes an algorithm-qualified content digest, that digest SHOULD be retained through the existing evidence/provenance layer rather than replaced with a REGEN-specific hash field.

A future canonical evidence-commitment profile may strengthen this separately.

## 16. Laboratory reference != laboratory competence

A laboratory reference identifies the declared analytical actor/system.

It does not prove:

- accreditation;
- current scope of accreditation;
- operator competence;
- calibration status;
- method validation;
- proficiency-test performance;
- absence of contamination or transcription errors.

Those propositions require their own evidence.

## 17. Method reference != method execution

Similarly:

```text
method_ref present
!= method followed correctly
```

Protocol deviations, substitutions, failed controls, reruns, dilutions, and other material departures SHOULD remain explicit evidence rather than being normalized away.

## 18. Quality-control specimens

Blanks, duplicates, spikes, controls, reference materials, and other QA/QC specimens may be represented through the same lineage concepts when appropriate.

A downstream analytical profile may interpret those results.

REGEN-018 itself does not turn a passing control into universal analytical validity or a failing control into a hidden deletion rule.

Adverse/failing control evidence remains visible.

## 19. Contamination evidence composition

REGEN-016 contamination semantics depend strongly on specimen traceability.

Recommended layering:

```text
material / plot subject
-> REGEN-018 specimen lineage + custody
-> PEF analytical observation/product
-> REGEN-016 contamination interpretation against exact adopted profile
```

This preserves:

```text
analyte result
!= specimen identity proof
!= representative batch proof
!= universal material safety
```

## 20. Field-trial composition

REGEN-015 trial endpoints may also rely on specimens.

The specimen contract SHOULD allow blinded or pseudonymous specimen labels where the trial protocol requires them, while retaining a separately controlled mapping to treatment arm/subject when authorized.

REGEN-018 MUST NOT force personally identifying information into specimen records.

## 21. Privacy and sensitive context

Collector/custodian/participant references should use the minimum identity needed for the evidence proposition.

The dependency-light core should not require names, addresses, health data, or other sensitive personal information.

Access control, selective disclosure, institutional records, and Holochain persistence remain adapter/application concerns.

## 22. Clock semantics

Timestamps order declared events only to the extent that their clock/source can be trusted.

```text
timestamp order
!= trusted physical causality
```

Unknown collection/transfer/analysis time remains explicit. No generic trusted-time theorem is introduced in v1.

## 23. Corrections and invalidation

Later evidence that a specimen was mislabeled, compromised, substituted, contaminated, or processed incorrectly MUST NOT require deleting the original record.

The architecture SHOULD support append-only correction/invalidation relationships such as:

```text
record A was originally asserted
record B later challenges/invalidates A for purpose P
```

The original observation remains historical evidence; downstream acceptance may change.

## 24. No automatic inheritance across specimen lineage

A child specimen does not automatically inherit every qualification of its parent, and a parent does not automatically inherit every result from one child.

Examples:

```text
qualified batch
!= every subsample analytically valid

clean subsample
!= entire heterogeneous batch clean

failed aliquot
!= every other aliquot identical
```

The consuming profile decides what inference is justified.

## 25. Proposed dependency-light implementation

A later initial crate may be named approximately:

```text
crates/mycelix-regenerative-specimen
```

Likely direct dependencies:

- `mycelix-regenerative-core` for parent subject identities;
- `mycelix-core-types` for PEF evidence/provenance;
- qualified `mycelix-regenerative-evidence` conventions where useful;
- optional serde.

It SHOULD NOT directly depend on:

- Holochain/HDK;
- Symthaea runtime;
- Marketplace/Finance;
- Manufacturing runtime;
- Climate authority;
- databases/network clients;
- device/process-control systems.

Those integrations remain adapters.

## 26. Qualification target

Executable REGEN-018 should wait for the shared REGEN-010 evidence waist to qualify and should follow REGEN-008 ProductFrozen semantics where practical.

A future test campaign SHOULD include at least:

1. exact specimen-ref preservation;
2. parent-subject kind preservation;
3. known custody gap remains a gap;
4. unresolved custody does not become continuous custody;
5. child subsample retains parent relation;
6. sibling subsamples cannot substitute for each other;
7. merge/pool lineage retains all declared inputs;
8. destructive consumption does not delete provenance;
9. observation ID/phenomenon mismatch rejection;
10. computed analytical products require PEF lineage;
11. lineaged raw-class laundering remains rejected;
12. unknown timestamps remain unknown;
13. missing laboratory/method evidence is not manufactured;
14. serialization revalidates bounded refs and lineage structure;
15. no authority-bearing or recommendation field exists in the core contract.

## 27. Relationship to REGEN-010C

The planned REGEN-010C adversarial-completeness successor should remain focused on the soil evidence waist itself.

REGEN-018 owns physical-specimen traceability and MUST NOT be used to retroactively overstate what REGEN-010B qualified.

## 28. Deliberate non-claims

REGEN-018 establishes no:

- sampling representativeness;
- specimen authenticity beyond the declared evidence chain;
- legal collection/transfer authority;
- custody ownership/title;
- laboratory competence/accreditation;
- analytical correctness;
- absence of contamination;
- soil/material safety;
- agronomic suitability;
- treatment efficacy;
- climate/carbon claim;
- market value;
- governance authority;
- process execution authority;
- physical actuation.

Its proposition is intentionally narrower:

> make the physical specimen and its collection/custody/transformation/result lineage explicit enough that later analytical, contamination, trial, and suitability claims cannot silently substitute or erase the material evidence path.
