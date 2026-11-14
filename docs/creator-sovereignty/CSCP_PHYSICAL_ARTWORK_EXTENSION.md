# CSCP Physical Artwork and Cultural Object Extension
**Status:** Research/architecture proposal v0.1  
**Parent:** [CSCP Creative Asset Passport and Rights Graph](CSCP_CREATIVE_ASSET_PASSPORT.md) and [CSCP-ASSET-PASSPORT-001 / #5181](https://github.com/Luminous-Dynamics/mycelix/issues/5181)  
**Scope:** physical artworks, sculptures, installations, editions, and cultural objects  
**Claim ceiling:** portable evidence and coordination; not a legal registry, authenticity oracle, or ownership adjudicator

## 1. Why an extension is needed

The existing Creative Asset Passport correctly separates assets, versions, contributors, rights pools, scoped usage grants, approvals, and revenue evidence. Physical artworks add a lifecycle that cannot be represented adequately as only a file or media asset:

- one physical object may have many photographs, scans, catalog records, and digital surrogates;
- the object may be exhibited, loaned, transported, conserved, damaged, sampled, restored, or held by different custodians;
- legal ownership, physical custody, artistic attribution, copyright, reproduction permissions, and community authority are different claims;
- condition and spatial geometry change over time;
- provenance may be incomplete or contested, and some records or locations must remain restricted.

The extension should preserve the existing CSCP asset/rights graph and add a physical-object layer. Do not create a separate identity ledger or new trust system.

## 2. Core ontology: distinguish the thing from its representations

Use separate typed identities for:

1. **Physical artwork / object** — the unique physical work or object.
2. **Edition instance** — a particular numbered or unnumbered instance where an edition exists.
3. **Component** — a constituent element of a multi-part work or installation.
4. **Digital surrogate** — a photograph, scan, photogrammetry result, mesh, point cloud, radiance field, or other representation.
5. **Documentation asset** — certificate, invoice, catalogue entry, exhibition record, conservation report, correspondence, or scientific report.
6. **Rights object / grant** — a permission or restriction scoped to an asset, representation, use, territory, time and purpose.

A photograph or scan is not the artwork itself. A sculpture's digital mesh is not proof of legal ownership. A gallery's possession is not proof of title. A creator credit is not proof of every rights share.

## 3. Required record groups

### 3.1 Object identity

At minimum, map ICOM Object ID categories:
- object type;
- materials and techniques;
- measurements;
- inscriptions and markings;
- distinguishing features;
- title;
- subject;
- date or period;
- maker.

Add local stable ID, object/edition/component relationships, language-tagged titles, identifier namespace/source, creation claims, and record version. Never synthesize official external identifiers or treat one as evidence of ownership.

### 3.2 Assertions and evidence

Every provenance/attribution/rights/condition statement is an assertion with:
- claim ID and subject/predicate/object;
- asserting agent, role and credential reference where applicable;
- asserted time and valid time, if known;
- source or method;
- evidence references and content digests;
- status: asserted, corroborated, disputed, withdrawn, superseded, or rejected;
- explicit supersession links and reviewer decisions.

Do not collapse contradictory assertions into one “truth.” Keep issuer credibility, cryptographic integrity, evidentiary sufficiency and legal effect separate. Verifiable Credentials 2.0 proves who issued a credential and whether it was altered; it explicitly does not itself prove that the enclosed claim is true.

### 3.3 Lifecycle events

Represent creation, acquisition, sale, transfer, consignment, custody change, loan, exhibition, transport, inspection, conservation, restoration, damage, digitization, and deaccession as distinct events.

Ownership and custody must be distinct event/claim types. Each event carries:
- event ID, type and time interval;
- participating agents and roles;
- related assertions;
- evidence and source;
- location reference (which may be restricted);
- previous/event lineage digest;
- correction/supersession relationship.

Do not expose the exact current location of a valuable, vulnerable, or culturally restricted object in a public claim graph.

### 3.4 Rights and permissions

Extend the existing CSCP scoped-use-grant model. Represent at least:
- physical ownership/title assertion;
- copyright and moral-rights assertions;
- photography and reproduction permissions;
- public display and exhibition permissions;
- loan and transport permissions;
- commercial licensing and derivative use;
- AI training and model-evaluation permissions;
- publication of location, cultural context, or restricted knowledge.

Each grant binds exact asset/representation version, grantor capacity, grantee, purpose, territory/jurisdiction, channel, start/end, restrictions, revocation, and approval evidence. Permission to photograph or display a work does not grant permission to train on it. Ownership of a physical object does not automatically transfer copyright.

For Indigenous, traditional, sacred, community-held, or otherwise sensitive cultural material, access and use must depend on documented community authority and consent. “Publicly visible” does not mean “freely reusable.”

### 3.5 Condition and conservation observations

A condition record is a dated observation, not a permanent truth or diagnosis. Capture:
- examiner and professional role;
- method, instrument, calibration and condition scale;
- time, environment and measurement units;
- observed feature/region and severity where the adopted rubric supports it;
- limitations, uncertainty and unobserved surfaces;
- referenced images/scans/reports and their digests;
- intervention history and the person/institution authorizing it.

Only conservators or explicitly designated experts can author professional condition opinions. Automated systems can point to evidence or suggest follow-up inspection; they cannot silently upgrade those suggestions into an expert diagnosis.

### 3.6 Spatial and digital-surrogate references

Reference Sol-Atlas assets using:
- geometry/appearance type;
- media type, URI or content-addressed locator, and digest;
- coordinate frame, units, calibration and transforms;
- capture method/device/time and operator;
- geometric uncertainty and coverage gaps;
- separate appearance and metric-geometry quality;
- rights/access classification.

A NeRF or Gaussian splat may be valuable for view synthesis but is not automatically a metrically faithful mesh. Spatial accuracy claims must link to independent reference geometry and calibration.

### 3.7 Access, privacy and disclosure

Profiles:
- public;
- controlled (authenticated/approved users);
- restricted (named roles or custodians);
- sealed (existence or content may itself be sensitive).

Keep private owner identities, contract terms, valuations, exact storage locations, security arrangements, and culturally restricted information outside public DKG projections by default. Selective disclosure must preserve enough evidence for the verifier to understand what was checked and what was withheld.

## 4. Standards and interoperability map

Use existing standards as semantic adapters, not as competing database schemas:

| Concern | Interoperability anchor | CSCP responsibility |
|---|---|---|
| Minimum object identification | ICOM Object ID | Validate and preserve the nine information categories |
| Event/relationship semantics | CIDOC CRM / ISO 21127:2023 | Map claims and lifecycle events without losing provenance |
| Collection metadata exchange | LIDO | Import/export collection descriptions with source lineage |
| Signed assertions | W3C VC Data Model 2.0 + Data Integrity | Preserve issuer, proof, schema, status and verifier-policy results |
| Provenance lineage | W3C PROV-O concepts | Represent derivation, sources, agents and activities |
| Image/manifest delivery | IIIF Presentation where applicable | Reference media/annotations and preserve rights |
| Long-term preservation | OAIS/PREMIS and persistent identifiers | Preserve representation, fixity, migration and event history |
| Spatial representation | glTF, PLY, OBJ and domain-specific formats | Record format, coordinate frame, calibration and digest |

Do not claim conformance merely because a similarly named field exists. Each adapter needs version pinning, a mapping table, round-trip fixtures, and explicit unsupported-field reporting.

## 5. Cross-system responsibility boundaries

- **Mycelix / CSCP:** canonical IDs, signed claims, rights grants, approvals, disclosure policy, event and import lineage.
- **Sol-Atlas:** geometry, component/scene graph, coordinate frames, registration transforms, spatial uncertainty, and capture completeness.
- **Symtropy:** sensor traces, exposure histories, bounded mechanical scenarios, and reproducible simulation receipts.
- **Symthaea:** multimodal discovery, relationship analysis, discrepancy surfacing, evidence explanations and suggested next capture/research steps.

No system independently certifies legal ownership or authenticity. The system that can verify a signature must not decide that the issuer is a trusted expert for every claim type.

## 6. Adoption sequence

1. Add the physical-object extension to CSCP without changing existing music/creator asset semantics.
2. Build a local-first validator and synthetic examples.
3. Add Object ID fields and a small CIDOC CRM/LIDO mapping matrix.
4. Add signed creator, institution, exhibition and custody assertions.
5. Add privacy/access profiles and negative disclosure tests.
6. Add spatial asset references from Sol-Atlas.
7. Add Symtropy condition/exposure records only after units, sensor provenance and expert review are specified.
8. Pilot with a small, rights-cleared collection and participating artists/institutions before any public DKG publication.
9. Only after user validation, add institutional adapters and cross-platform discovery.

## 7. Acceptance gates

- Zero conflation of physical work, edition instance, component, digital surrogate and documentation asset.
- Zero conflation of attribution, credit, ownership, custody, copyright, condition and permission.
- Every material assertion has an issuer, role, time, evidence references or an explicit unsupported/unverified status.
- Conflicting claims remain visible and cannot be auto-resolved by confidence score or majority vote.
- Any modified evidence bytes or event content fail digest verification.
- Event corrections preserve supersession lineage; source imports are replay-safe across retries and restarts.
- Scoped rights cannot exceed asset/version, purpose, territory, channel or term.
- AI training and derivative permissions are independent from publication/display permissions.
- Public projections leak no private owner data, sensitive location, contract contents or sealed cultural knowledge.
- Import/export tests preserve supported semantics and report all loss/unsupported fields.
- A curator, conservator, practicing artist, and relevant community/rights representative review the pilot schema.
- Pilot reports measure time-to-register, provenance completeness, dispute/unknown rates, diligence time and creator outcomes without presenting them as market-wide impact.

## 8. Explicit non-goals

- No global authenticity score.
- No automatic legal-title or copyright adjudication.
- No investment-grade valuation or buy/sell recommendation.
- No compulsory tokenization or public-chain ownership claim.
- No automatic restoration or intervention.
- No presumption that cultural materials should be public, licensable, or AI-trainable.

## 9. Sources

- [ICOM Object ID](https://icom.museum/en/resources/standards-guidelines/objectid/)
- [CIDOC CRM](https://cidoc-crm.org/)
- [ICOM documentation standards](https://icom-documentation.mini.icom.museum/standards/cidoc-standards-guidelines/)
- [LIDO Primer (2024)](https://lido-schema.org/documents/primer/latest/lido-primer.html)
- [W3C Verifiable Credentials Data Model 2.0](https://www.w3.org/TR/vc-data-model-2.0/)
- [W3C: VC 2.0 family became Recommendations](https://www.w3.org/news/2025/the-verifiable-credentials-2-0-family-of-specifications-is-now-a-w3c-recommendation/)
- [Europeana persistent identifier policy](https://pro.europeana.eu/post/policy-for-persistent-identifiers-in-the-data-space)
- [EU report on long-term preservation of 3D cultural heritage data](https://digital-strategy.ec.europa.eu/en/library/report-long-term-preservation-3d-data-cultural-heritage)
