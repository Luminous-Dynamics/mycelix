# STEW-020A — Protected Metadata and Existence Boundary v0.1

## Purpose

Freeze a confidentiality boundary that payload encryption alone cannot satisfy.

```text
protected payload
!= protected metadata
!= protected existence
```

A valid encrypted/container representation can still leak sensitive information if a public envelope reveals a title, summary, community association, location, cultural-protocol identifier, relationship graph, or merely the fact that the protected subject exists.

## Current STEW-002 limitation

`StewardshipEnvelopeV1` is explicitly a **public** envelope and requires a non-empty `PublicDescriptorV1.title`. A `ProtectedRepresentation` therefore means only that the representation payload is not to be inferred public from the envelope.

It does **not** mean:

- the title is safe to publish;
- the summary is safe to publish;
- all envelope references are safe to publish;
- the existence of the subject is safe to reveal;
- relationships among protected subjects are safe to reveal;
- index/search membership is safe to reveal.

This is a documented limitation of the v1 envelope, not a property to hide behind encryption.

## Three disclosure planes

### 1. Payload disclosure

The representation bytes themselves: plaintext, media, transcript, dataset, score, source material, or other payload.

Owned by STEW-020 and later qualified encrypted-storage/key/capability profiles.

### 2. Metadata disclosure

Human-facing or machine-readable information about the subject, including but not limited to:

- title and summary;
- names of people, communities, institutions, or traditions;
- geographic or site information;
- dates or seasonal context;
- cultural-protocol identifiers;
- provenance/stewardship/reciprocity references;
- subject/revision/representation identifiers when those identifiers reveal semantics;
- preservation locations/custodians;
- link/index membership.

Metadata requires its own authority decision. Payload protection never implies metadata-publication permission.

### 3. Existence disclosure

Whether an unauthorized observer may learn that the subject/object/event exists at all.

If existence is protected, a public STEW-002 envelope is structurally the wrong carrier because publication of the envelope already discloses existence.

```text
existence protected
-> do not publish a public STEW-002 envelope for that subject
```

No generic placeholder title fixes an existence leak.

## v1 public-envelope admission classes

A later executable theorem should distinguish at least:

```text
PublicExistencePublicDescriptor
PublicExistenceRestrictedMetadata
ProtectedExistence
```

### PublicExistencePublicDescriptor

A public envelope may be possible, but every descriptor field and reference still requires an explicit public-safe basis.

### PublicExistenceRestrictedMetadata

The fact that the subject exists may be public, while some or all descriptive metadata is restricted. STEW-002 v1 cannot fully express this because it requires a public title. A future envelope/profile must provide an intentionally minimal public projection without inventing protected metadata.

### ProtectedExistence

No public envelope/index/search result should be emitted. Discovery itself is protected.

## Search and AI boundary

STEW-003 already separates:

```text
Discover != InspectMetadata != Retrieve != View != Disclose != TrainAi
```

Protected-metadata work must preserve those separations.

Examples:

```text
permit(Retrieve) != permit(Discover publicly)
permit(View) != permit(InspectMetadata by unrelated principals)
model can access protected object != model may mention that object exists
```

A model/RAG index, embedding store, cache, search autocomplete surface, analytics log, or recommendation feature can leak protected existence/metadata even when payload bytes remain encrypted.

## Reference-graph leakage

Opaque identifiers are not automatically harmless. Their presence, frequency, co-occurrence, timing, link topology, or externally resolvable meaning can leak sensitive relationships.

Therefore:

```text
envelope reference present
!= reference safe to publish
```

Cultural-protocol, stewardship, provenance, preservation, and reciprocity references may themselves require protected treatment.

## Location and cultural-safety boundary

Precise locations of sacred sites, endangered resources, graves, archaeological material, vulnerable ecosystems, or private communities must not be forced into public descriptors or indexes merely because the underlying payload is protected.

Similarly, a public community/tradition label can itself reveal a relationship that the affected community does not want publicly asserted.

## Proposed executable child

`STEW-020B` should define a small metadata/existence-disclosure admission theorem that consumes a proposed public projection and returns only a structural publication candidate. It should not decide cultural authority itself.

A later authorized projector should require independent evidence for:

- existence-disclosure authority;
- descriptor-field disclosure authority;
- reference disclosure authority;
- indexing/discovery authority.

Unknown or absent authorization must fail closed.

## Synthetic qualification corpus

1. **Public work** — existence, title, summary and selected references explicitly public-safe.
2. **Public existence / protected details** — discoverable opaque record, restricted descriptive fields not projected.
3. **Protected existence** — no public envelope/index result emitted.
4. **Protected site location** — content may be governed, but location reference is absent from public projection.
5. **AI index** — retrieval capability does not permit public mention, autocomplete, embedding export, or training.
6. **Reference leak** — protected cultural/stewardship reference is rejected from public projection.

Use synthetic fixtures only for restricted/sacred scenarios.

## Non-claims

This document does not establish metadata confidentiality, anonymous discovery, private information retrieval, searchable encryption, traffic-analysis resistance, secure deletion, location privacy, community consent, cultural authority, access authorization, AI-use authorization, or historical erasure.

It establishes only the disclosure boundary that later implementations must preserve.
