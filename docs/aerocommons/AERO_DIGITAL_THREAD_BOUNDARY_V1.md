# AeroCommons Digital Thread Boundary v1

## Purpose

AeroCommons should complement, not replace, established engineering and manufacturing data standards.

The protocol boundary is:

- engineering standards carry domain artifacts and semantics;
- external systems remain authoritative for their own engineering calculations and manufacturing/inspection workflows;
- Mycelix carries agent identity, provenance, lifecycle state, attestations, disputes, configuration lineage, and evidence relationships;
- Holochain validates the integrity rules of the commons;
- Symthaea reasons over the resulting evidence graph without becoming the authority that certifies the physical system.

## 1. Three-layer model

### Layer A — Engineering artifacts

Examples:

- STEP/AP242 product definition and PMI
- QIF inspection/metrology data
- CAD assemblies
- CAE models
- simulation outputs
- manufacturing instructions
- machine/process records
- maintenance records

AeroCommons should reference these artifacts by stable identifiers and cryptographic content references rather than reimplementing their native schemas.

NIST's digital-thread work explicitly identifies STEP, QIF, and MTConnect as existing standards that can support lifecycle information integration. QIF is designed to carry quality information from design through inspection planning, execution, analysis, and reporting.

### Layer B — AeroCommons evidence envelope

The envelope answers:

- Who produced this artifact or observation?
- Which configuration did it belong to?
- What claim is being made?
- What evidence supports it?
- Which toolchain/method was used?
- Which inputs were consumed?
- What was independently reproduced?
- Has the evidence been superseded, disputed, revoked, or withdrawn?
- Which ChangeSet can invalidate it?
- What downstream obligations result?

The envelope does not replace the engineering artifact.

### Layer C — Holochain commons

Holochain provides agent-centric source-chain history and DHT validation for the shared protocol records.

Integrity zomes should enforce deterministic structural and relationship rules. Coordinator zomes should orchestrate retrieval, indexing, queries, and integrations.

Large engineering payloads should not be forced into DHT entries merely because they are important. The DHT record should contain the canonical identity, content hash, media/type metadata, access policy, and provenance links needed to verify or retrieve the payload.

## 2. Artifact identity

AeroCommons should distinguish four identities:

1. Artifact identity — what object/data item is being referenced.
2. Content identity — the exact bytes or canonical representation.
3. Configuration identity — the complete state in which the artifact participates.
4. Evidence identity — the claim/observation record about the artifact.

A filename, URL, human-readable version, or database row ID is not sufficient as the sole identity.

Where standards already provide persistent identifiers, preserve them rather than replacing them. Where they do not, AeroCommons may supply an external identifier that binds the artifact to a content digest and provenance record.

## 3. Storage classes

### Class P — Public protocol records

Suitable for Holochain DHT:

- artifact references;
- configuration records;
- ChangeSets;
- evidence envelopes;
- attestations;
- disputes;
- lifecycle events;
- dependency edges;
- revalidation obligations.

### Class C — Controlled engineering payloads

Potentially external to the DHT:

- CAD files;
- STEP files;
- QIF documents;
- simulation datasets;
- manufacturing programs;
- inspection datasets;
- large images/scans;
- test data.

The Holochain record should retain enough information to establish integrity and provenance without requiring the entire payload to be replicated to every validating peer.

### Class S — Sensitive/private records

Examples:

- export-controlled information;
- proprietary manufacturing information;
- personal data;
- security-sensitive facility information.

These require explicit access-control and disclosure policy. A public provenance record MUST NOT imply public access to the underlying payload.

## 4. Holochain validation boundary

Integrity validation should answer questions that are deterministic and protocol-level:

- Is the record structurally valid?
- Does the referenced predecessor exist and have the correct type?
- Is the lifecycle transition legal?
- Is the author allowed to make this class of assertion under the declared protocol rules?
- Are required dependencies explicitly referenced?
- Does a ChangeSet correctly identify its predecessor configuration?
- Does an attestation reference an existing evidence record?

Validation should not answer:

- Is this aircraft safe to fly?
- Is this engineering analysis technically correct?
- Is a manufacturing process adequate?
- Is a test sufficient for certification?
- Is a deviation acceptable?

Those require domain-specific engineering and regulatory authority.

Holochain's validation model is a strong fit for this distinction because validation is deterministic and can retrieve explicitly referenced DHT dependencies; unresolved dependencies are not silently treated as valid.

## 5. Source-chain semantics

Agent actions should remain attributable to the agent's source chain.

Examples:

- manufacturer publishes manufacturing record;
- inspector publishes inspection evidence;
- test laboratory publishes test result;
- independent reproducer publishes reproduction;
- engineer publishes review;
- maintainer publishes maintenance event.

A later attestation does not erase the earlier record.

This preserves temporal and authorship lineage without requiring a central ledger.

## 6. Evidence graph

The shared graph should use typed edges, not arbitrary social links.

Initial relationship vocabulary:

- defines
- contains
- implements
- derived_from
- depends_on
- manufactured_from
- inspected_as
- tested_as
- observes
- predicts
- reproduces
- supersedes
- disputes
- invalidates
- requires_review
- requires_revalidation

Every edge that changes engineering interpretation should itself have provenance.

## 7. Configuration lineage

Configuration identity is the backbone of the physical digital thread.

A ChangeSet should be able to answer:

What changed, from exactly which configuration, and which claims might that change affect?

The impact engine must operate conservatively:

- known unaffected
- conditionally valid
- invalidated
- requires review
- unknown

No status should be inferred merely from social agreement.

## 8. Symthaea boundary

Symthaea can consume the graph to:

- find contradictory claims;
- compare predictions to observations;
- detect unexplained prediction error;
- identify missing dependencies;
- summarize configuration lineage;
- propose candidate revalidation paths;
- explore engineering alternatives;
- surface evidence whose applicability is uncertain.

Symthaea should emit proposals and explanations with links back to evidence.

It should never silently promote a model prediction to a physical observation.

## 9. External-standard mapping

The initial mapping should be deliberately small:

| Domain | External artifact | AeroCommons envelope |
| --- | --- | --- |
| Product definition | STEP/AP242 | artifact identity + configuration + provenance |
| Inspection/metrology | QIF | inspection evidence + producer + lifecycle |
| Manufacturing data | process/MES/MTConnect records | manufacturing event + toolchain + artifact lineage |
| Analysis | CAE/OpenMDAO/Aviary/etc. | analysis/simulation evidence + inputs + validity domain |
| Test | laboratory/test-system record | test evidence + observation + method + provenance |
| Maintenance | maintenance record | maintenance event + affected configuration |

This keeps AeroCommons interoperable instead of becoming a proprietary replacement stack.

## 10. Reproducibility

A reproduction record must identify:

- the artifact/configuration reproduced;
- source content identity;
- environment;
- method/toolchain;
- operator/agent;
- physical artifact where applicable;
- differences from the original;
- observed result;
- relationship to the original evidence.

Two attestations are not automatically independent because they have different authors or record IDs.

Independence must be an explicit, reviewable property.

## 11. First implementation target

The first implementation should be a non-flight-critical physical artifact.

The reproducibility challenge should exercise:

1. open product definition;
2. content identity;
3. configuration identity;
4. manufacturing record;
5. inspection evidence;
6. physical test;
7. independent reproduction;
8. configuration fork;
9. automated impact analysis;
10. human review obligations.

Only after this works should the protocol be connected to increasingly consequential aerospace workflows.

## 12. Design invariant

Engineering standards describe the thing. AeroCommons describes why the system believes its claims about the thing.

That distinction should remain stable as the protocol expands from aircraft into other physical commons.
