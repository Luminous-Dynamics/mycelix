# STEW-000I — Standards Interoperability Boundary v0.1

Status: preregistration / architecture boundary

Parent: STEW-000

## Purpose

Freeze how the Mycelix stewardship program should interoperate with mature external standards and community-governed protocols without turning those standards into new sources of authority or cloning their semantics into incompatible Mycelix-specific substitutes.

The intended direction is:

```text
small canonical Mycelix theorem
+ explicit versioned mapping/profile
+ external identifier / artifact
= interoperable projection
```

not:

```text
imported vocabulary
= imported authority
```

and not:

```text
useful external standard exists
-> reimplement it under new names
```

## Constitutional rule

Interoperability must preserve the STEW authority separations:

```text
syntax compatibility
!= semantic equivalence
!= verified provenance
!= permission
!= ownership
!= cultural legitimacy
!= preservation success
```

A conforming projection can prove that one Mycelix artifact was encoded or mapped according to a profile. It cannot silently strengthen the underlying evidence or authority.

## Version and profile binding

Every external-standard projection that matters to authority or evidence should bind at least:

```text
standard family
standard/profile version
profile identifier
canonical subject identity
projection digest
producer identity/build when relevant
explicit scope
explicit non-claims
```

Where an external standard permits extensibility, Mycelix extensions should use a documented namespace and remain optional to consumers that only understand the base standard.

## ODRL — policy interchange

ODRL is an appropriate interoperability target for expressing permissions, prohibitions, duties, actions, constraints, parties, and assets.

Mycelix should keep its own small, typed, fail-closed knowledge-use theorem as the authority-bearing core and provide a versioned ODRL profile/projection around it.

Core separation:

```text
Mycelix admitted policy theorem
-> ODRL projection

ODRL document received
!= automatically admitted Mycelix authority
```

An inbound ODRL policy must be parsed, canonicalized, profile-checked, and independently admitted before it can affect Mycelix access decisions.

Unknown actions or constraints in a protected-content policy must not be silently ignored.

## PROV-O — provenance interchange

PROV-O is an appropriate interchange vocabulary for entities, activities, agents, derivations, attributions, generations, usages, and related provenance relations.

Mycelix should retain typed provenance relations internally so security- or domain-critical semantics are not reduced to arbitrary RDF strings.

The preferred relationship is:

```text
STEW typed provenance graph
<-> versioned PROV-O projection
```

not:

```text
arbitrary PROV-O graph
-> trusted Mycelix lineage
```

A PROV-O projection should preserve exact STEW subject/revision/representation identifiers and evidence references where possible.

## PREMIS — preservation metadata

PREMIS is an appropriate semantic reference for preservation objects, preservation events, agents, and rights relevant to digital preservation.

STEW preservation work should align terminology and export capability with PREMIS while keeping qualification claims narrow.

For example:

```text
PREMIS event recorded
!= event successfully performed
!= artifact remains recoverable
```

Preservation success requires separate fixity, replica, recovery, and process evidence.

## OCFL — durable object layout and version history

OCFL is an appropriate implementation/interchange target for durable versioned object storage layouts.

STEW should treat OCFL as one possible qualified storage profile, not as the universal Mycelix object store.

Core theorem:

```text
valid OCFL object structure
!= sufficient replica diversity
!= sufficient media durability
!= successful disaster recovery
```

A later OCFL adapter should bind the exact OCFL version/profile and object root into the preservation manifest rather than importing an unversioned storage assumption.

## RO-Crate — portable research/artifact package

RO-Crate is an appropriate portability format for packaging data, software, workflows, contextual metadata, and related research artifacts using JSON-LD.

Mycelix can use RO-Crate as a portable envelope for selected public or authorized exports of a stewarded subject and its evidence graph.

The crate should be a projection of admitted Mycelix state:

```text
admitted subject + selected evidence + disclosure policy
-> RO-Crate export
```

not a way to bypass disclosure policy by serializing protected fields into a convenient archive.

Import likewise remains evidence ingestion, not automatic truth or authority.

## C2PA — signed content provenance

C2PA is an appropriate interoperability target for tamper-evident media provenance and signed content credentials.

Mycelix must preserve the distinction:

```text
valid C2PA signature / manifest
= evidence about provenance assertions
!= truth of depicted event
!= legal ownership
!= consent
!= cultural permission
```

C2PA assertions may become evidence inputs to STEW provenance or authenticity workflows, but their issuer trust and claim semantics remain explicit.

A Mycelix-produced C2PA assertion should bind the exact STEW representation/content digest and a narrowly scoped assertion set.

## Local Contexts — community-governed cultural protocols

Local Contexts Traditional Knowledge and Biocultural Labels are specifically designed to let Indigenous and local communities express provenance, protocol, permission, responsibility, and community expectations around knowledge and collections.

Mycelix must not create, assign, customize, or impersonate a community's Local Contexts Labels merely because corresponding concepts exist in STEW.

The safe interoperability theorem is:

```text
community-controlled external Label / identifier
+ authorized reference/import
-> Mycelix may preserve and display that reference according to its terms
```

not:

```text
Mycelix policy engine
-> self-issues community cultural Label
```

Where a Local Contexts Label is referenced, the record should preserve the external identifier, issuing community/context, fetched or supplied metadata digest where appropriate, and provenance of who supplied the reference.

Mycelix-specific policy may additionally enforce stricter local handling, but must not claim that this changes the meaning of the external Label.

## CARE Principles — governance lens, not machine authority

The CARE Principles for Indigenous Data Governance provide an important governance lens around Collective Benefit, Authority to Control, Responsibility, and Ethics.

CARE should inform STEW architecture and evaluation, especially where FAIR/open-data assumptions could ignore power, collective rights, and community self-determination.

CARE is not reduced to a numeric score or a machine-generated authorization token.

```text
CARE-aligned design review
!= community consent
!= authority to control a specific artifact
```

Specific authority must still come from the relevant community/governance evidence.

## FAIR and openness

STEW should distinguish findability/interoperability from universal openness.

```text
findable metadata
!= publicly retrievable payload

interoperable metadata
!= reusable without permission
```

This allows public discovery descriptors for protected material while keeping payload access governed separately.

## Crosswalk architecture

A later implementation should prefer small adapter/profile crates or modules rather than putting every external standard into `mycelix-stewardship-core`.

Suggested shape:

```text
mycelix-stewardship-core
    canonical typed theorems
          |
          +-- stewardship-odrl-profile
          +-- stewardship-prov-profile
          +-- stewardship-premis-profile
          +-- stewardship-ocfl-profile
          +-- stewardship-ro-crate-profile
          +-- stewardship-c2pa-profile
          +-- stewardship-local-contexts-reference
```

The core should remain usable without RDF, JSON-LD, XML, storage engines, signature stacks, or network clients.

## Projection receipts

For standards that can affect auditability, Mycelix should consider a generic projection receipt:

```text
InteropProjectionReceiptV1 {
    subject,
    source_profile,
    target_standard,
    target_version,
    mapping_profile,
    projection_digest,
    producer_build,
    generated_at,
    scope,
}
```

The receipt proves which transformation was claimed over which exact source subject. It does not make the external artifact authoritative by itself.

## Round-trip discipline

Not every standard supports lossless round trip.

Every mapping profile should classify fields/semantics as:

```text
Exact
RepresentableWithExtension
Lossy
Unsupported
OutOfScope
```

Loss must be explicit.

```text
successful serialization
!= semantic round-trip
```

Golden vectors should test canonical examples and adversarial ambiguity, especially for policy actions, identities, provenance direction, time ranges, and rights/duties.

## Import firewall

Inbound interoperable artifacts must remain untrusted inputs until profile validation completes.

General sequence:

```text
external bytes
-> parse with resource bounds
-> standard/version identification
-> canonical/profile validation
-> issuer/provenance verification where applicable
-> semantic crosswalk
-> explicit authority admission
-> Mycelix typed state
```

No parser or standards library should directly mint access, rights, stewardship, governance, or effect authority.

## Export firewall

Exports must be policy-aware.

```text
caller can access protected content
!= caller may export/redistribute it
```

A serializer must receive a separately admitted disclosure/export capability when protected information is included.

Public-safe metadata exports should be possible without requiring protected payload disclosure.

## AI interoperability

Machine-readable standards must not become an implicit AI-training permission surface.

For example:

```text
RO-Crate discoverable
!= train_ai permitted

C2PA credential valid
!= model ingestion permitted

PROV-O graph public
!= every linked payload public
```

STEW-003 and later Symthaea KNOW-AUTH profiles remain authoritative for Mycelix AI-use decisions.

## Initial standards test corpus

Later adapters should include synthetic golden fixtures such as:

1. ODRL policy allowing `view` while prohibiting `train_ai` and requiring attribution.
2. PROV-O derivation mapping that preserves direction and exact STEW subject IDs.
3. PREMIS migration event that does not itself claim recovery success.
4. OCFL version history whose object is structurally valid but deliberately has only one replica, proving storage format != redundancy.
5. RO-Crate public-safe export that omits protected payloads.
6. C2PA assertion with a valid signature but an intentionally unsupported factual assertion, proving signature != truth.
7. Local Contexts reference fixture using a synthetic/non-community test identifier so tests never counterfeit a real community Label.

## Proposed follow-on tranche

```text
STEW-003      knowledge-use policy theorem
STEW-003O     ODRL mapping profile
STEW-006      typed provenance theorem
STEW-006P     PROV-O profile
STEW-007      preservation manifest
STEW-007P     PREMIS projection
STEW-007O     OCFL storage profile
STEW-008R     RO-Crate export/import profile
STEW-009C     C2PA provenance adapter
STEW-009L     Local Contexts external-reference profile
```

Names may be adjusted as the parent tranches land; authority boundaries should not.

## Deliberate non-claims

This preregistration establishes no conformance certification for ODRL, PROV-O, PREMIS, OCFL, RO-Crate, C2PA, Local Contexts, CARE, or FAIR.

It grants no copyright, cultural authority, Indigenous/community consent, access right, disclosure right, AI-training permission, ownership, authenticity, factual truth, preservation success, or legal compliance.

It creates no runtime mapping code.

It freezes one principle only:

```text
interoperate with mature standards through narrow versioned profiles;
do not duplicate them and do not let interoperability manufacture authority.
```
