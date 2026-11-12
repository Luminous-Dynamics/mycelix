# NIST IR 8536 ↔ Mycelix traceability crosswalk v0.1

**Snapshot:** 2026-11-12  
**Status:** research crosswalk / runner-independent specification  
**Tracking:** [Mycelix #6098](https://github.com/Luminous-Dynamics/mycelix/issues/6098)  
**Related owner:** [LEX-NET-015 #1110](https://github.com/Luminous-Dynamics/mycelix/issues/1110)  
**Related Mycelix issues:** #5149, #4901, #5106, #4900  
**Related Symtropy owners:** #3331, #3025, #3164, #3121  
**Claim class:** conceptual mapping only; not NIST conformance, certification, or tested interoperability.

## 1. Why this exists

NIST IR 8536, *Supply Chain Traceability: Manufacturing Meta-Framework*, was finalized in September 2026. It describes common structural patterns for encapsulating, linking, exchanging and querying traceability information across manufacturing supply chains. Its approach is intended to preserve and interoperate with existing industry practices and standards rather than force every industry into one replacement vocabulary or centralized data store.

The report highlights source/traceability-event encapsulation, interoperable interfaces, temporal ordering, cryptographically verifiable linkage, privacy/selective disclosure, and common patterns that can be tailored by sector. NIST’s September 9, 2026 announcement said its Python reference implementation was undergoing final review. The final IR’s Appendix F.4 now points to a public repository: https://github.com/usnistgov/ir8536-reference-implementation. Appendix F describes the implementation as an early-stage laboratory/research MVP, not production software; the described example uses three illustrative ecosystems and text-file repositories (with NoSQL as a configuration option). I have not directly inspected the repository’s current code, licence, tests or release revision in this pass, so no parity or execution claim is made.

Official sources:
- NIST IR 8536 final: https://csrc.nist.gov/pubs/ir/8536/final
- NIST finalization announcement (2026-09-09): https://www.nist.gov/news-events/news/2026/09/finalized-manufacturing-supply-chain-traceability-meta-framework
- NIST project page: https://www.nccoe.nist.gov/projects/supply-chain-traceability-principles-manufacturing-meta-framework

## 2. Architecture rule

**Do not build another universal supply-chain schema.** Extend the existing Mycelix cross-standard bridge in #1110 with a small versioned profile/crosswalk. Preserve original source records and their native standards. Add only the minimum mapping needed for identity, relationships, time, provenance, evidence references, disclosure policy and explicit uncertainty.

The core separation is:

`event observation ≠ issuer authentication ≠ source truth ≠ custody conclusion ≠ product conformity ≠ legal ownership ≠ regulatory compliance ≠ action authority`

A valid signature establishes that a key signed a particular representation under a declared verification policy. It does not establish that the event physically occurred or that a product is safe, lawful, sustainable, compliant, accepted or delivered.

## 3. Existing ownership map

| Concern | Existing owner | Crosswalk responsibility |
|---|---|---|
| Cross-standard supply-chain semantics | Mycelix #1110 LEX-NET-015 | NIST concept mapping, source-profile/version preservation, conflict and unknown semantics |
| Construction provenance and accountable approvals | Mycelix #5149 | Approval identity/scope, signer authority, supersession, correction, dispute and revocation |
| Logistics event and custody semantics | Mycelix #4901 | Shipment/custody handoffs, receiving and event order |
| Forestry/wood lot provenance | Mycelix #5106 | Batch/lot, source, measurements and transformation references |
| Federated logistics architecture | Mycelix #4900 | Participant-owned data and permissioned exchange |
| Manufacturing component / as-built trace | Symtropy #3331 | Component identity, design revision, material/process lot, inspection, transport, installation and as-built records |
| Manufacturing standards interoperation | Symtropy #3025 | MTConnect + QIF + ISO 23247 semantic preservation |
| Manufacturing twin change impact | Symtropy #3164 | Time-, scope- and claim-aware requalification propagation |
| Composed manufacturing twin VVUQ | Symtropy #3121 | Component/interface/end-to-end validation boundaries |

This crosswalk does not own a new event store, credential format, identity authority, approval ledger, product passport, digital twin, measurement authority, quality decision, or release process.

## 4. Crosswalk: concepts, current surfaces and gaps

| Traceability concern | Existing Mycelix surface | Current status from source inspection | Gap / required qualification |
|---|---|---|---|
| Subject identity | Mycelix supply-chain SDK has `productId` and optional `batchId`; Mycelix #5149 and #4901 own construction/logistics IDs | Structurally present in SDK model; active supply-chain workspace is marked `unverified` | Separate SKU/family, serial/article, lot/batch, material heat, shipment unit, assembly and facility IDs; preserve source identifier scheme and issuer |
| Event type and action | SDK checkpoint action: received/processed/shipped/delivered/inspected; archived reference crate defines PRODUCED/TRANSFORMED/SHIPPED/RECEIVED/CERTIFIED | Different models exist; no proven cross-profile semantic equivalence | Map event vocabulary per source standard/version; do not treat similar words as equivalent without rule and test |
| Event time | SDK checkpoint has a timestamp; archived reference VC had issuanceDate and subject timestamp | At least one timestamp is present in the SDK checkpoint path | Keep occurrence, source-system, ingestion, issuance, effective, observation and verification times separate; support clock uncertainty and partial ordering |
| Actor / issuer | SDK checkpoint stores handler and constructs a `did:mycelix:` string; VC issuer exists in archived reference | Identity fields exist; method semantics and signature verification require qualification | Resolve actor/issuer under an explicit method and trust policy; do not treat a string prefix or reputation as authenticated identity |
| Provenance links | SDK in-memory service stores previousCheckpointId; archived Rust reference has previous claim IDs and a lineage hash | Structural links exist in different paths | Bind exact source/target subject, revision, transformation, predecessor and evidence; independent offline reconstruction and tamper controls required |
| Evidence references | SDK stores typed evidence with arbitrary data, timestamp and caller-supplied `verified` boolean; #5149 owns project evidence | Evidence is representable; local boolean is not an independent verification receipt | Carry source digest/locator, media type, source version, method, instrument/calibration where relevant, signer, status and disclosure policy; unknown stays unknown |
| Custody / shipment | Mycelix #4901 and #5149; SDK has shipment/checkpoint concepts | Ownership exists; active end-to-end Holochain cluster is not qualified by this crosswalk | Test duplicate/competing handoffs, exact quantity, partial shipment, returns, corrections, disputes and stale inventory |
| Inspection / conformity | Symtropy #3331, #3025; Mycelix #5149 approval receipts | Relevant interfaces and owner issues exist | Measurement uncertainty, calibration, exact characteristic, method, applicability and competent approval must remain separate |
| Standards interop | Mycelix #1110; Symtropy #3025 | Cross-standard work is specified | Pin exact standard, profile, schema, document kind and immutable source; preserve unknown extension data and do not silently coerce |
| Selective disclosure | Mycelix identity/privacy work and NIST design principle | Capability depends on exact credential/proof implementation and verification support | Do not claim SD-JWT/BBS+ support for the archived standalone implementation; active stack needs independent external-vector proof and privacy-leakage tests |
| Corrections / revocation | Mycelix #5149, #5106, #1110 | Explicitly owned at design level | Preserve immutable history and link correction/revocation/supersession; verify currentness at decision time |
| Query / lineage view | SDK exposes in-memory chain API and a Holochain client wrapper | The two paths are different; workspace inventory says supplychain `unverified` | Compare local demo with Holochain path and deployed state; no production-scale or federation claim yet |

The “current status” column is a bounded source inspection, not an exhaustive audit of all active Mycelix submodules or a claim that the owners have no further implementation.

## 5. Standards profiles: preserve source semantics

The first profiles should be adapters into the existing bridge, not a replacement schema:

1. **GS1 EPCIS / CBV 2.0** for interoperable visibility events and event/capture/query semantics.
2. **QIF 3.0** for product definition and inspection planning/results where quality measurement is in scope.
3. **MTConnect** for machine/process observations, preserving Samples, Events, Conditions, Assets, device/component/data-item identity and versioned schema semantics.
4. **IFC 4.3 / ISO 16739-1:2024** as an infrastructure asset exchange representation where relevant; an IFC model is not itself manufacturing inspection or NDT evidence.
5. **ISO 23247** manufacturing digital-thread/twin composition context, without treating a linked model as proof of correct physical state.

Relevant maintained cross-standard owner: Symtropy #3025. Pin exact document version, namespace, schema path, source commit/blob and downloaded-byte digest per admitted artefact. MTConnect model-browser version, XML XSD release, JSON schema and formal standard publication are different release surfaces; one must not be inferred from another.

## 6. Profile-neutral exchange envelope: minimum fields to map, not a new canonical schema

The adapter mapping should retain the following independently:

- **Source identity:** source standard, profile, version, document kind, source artifact digest, source locator and original native IDs.
- **Subject identity:** identifier scheme/value, subject type, parent/child and assembly relation, exact revision/serial/lot where supplied.
- **Event:** native event type, event ID, source-system sequence, occurrence time, time basis/clock, capture/ingestion time, effective interval, and correction/supersession relationship.
- **Actor:** source actor/issuer, organization/facility, role and authority reference; authentication result and trust-policy version.
- **Relationship:** input/output lots, predecessor/successor events, transformation, shipment/custody handoff, assembly or installed-asset relation.
- **Evidence:** exact source locator/digest, media type, measurement method, instrument/calibration, uncertainty, observation status, issuer and independent validation reference.
- **Disclosure:** recipient/purpose policy, fields released, proof/profile version and what remains intentionally undisclosed.
- **Disposition:** supported, unsupported, incomplete, disputed, revoked, stale, indeterminate or verified under a named verification policy. Avoid one boolean.

The original source payload or an immutable, retrievable source reference must be retained beside any normalized projection. A digest proves integrity relative to those bytes, not that the source statement is true.

## 7. Benchmark and conformance plan

### Phase A — source-preserving fixture suite (runner-independent)
Author public, non-sensitive, synthetic fixtures for:
- valid event and transformation chain across two standards profiles;
- missing predecessor and missing required source event;
- duplicate event ID and replayed event;
- wrong product/lot/article identity scheme;
- timestamp disagreement, clock offset and overlapping event-time intervals;
- unknown schema/profile version and unknown extension fields;
- conflicting custody histories;
- revoked/expired/superseded credential;
- source artifact changed while normalized projection is unchanged;
- selective-disclosure request revealing an unauthorized field;
- inspection result without calibration/method/characteristic linkage;
- measurement interval overlapping a tolerance boundary.

Do not put secret answer keys or personal/cultural-sensitive location data in the public development corpus.

### Phase B — adapters and independent verifier
For each standard/version profile:
- parse using a maintained standards-aware library and official schema where practical;
- preserve native source IDs, source payload, schema/version and unknown fields;
- record every mapping/transformation and its version;
- export a portable bundle;
- independently reconstruct links and validate digest/signature/status without the originating process;
- return Unsupported/Indeterminate when a source feature is not understood.

### Phase C — benchmark metrics
Report each independently:
- source-byte and source-locator preservation;
- schema/version admission precision and unknown-version abstention;
- exact identity/link resolution;
- temporal ordering accuracy and ambiguity detection;
- correction/revocation/currentness accuracy;
- signature/key/issuer verification;
- selective-disclosure correctness and leakage;
- round-trip semantic fidelity by profile/version;
- offline reconstruction and tamper detection;
- cost, latency, payload size and operator integration effort.

Hard gates: a broken lineage, wrong physical subject, stale/revoked authority, invalid signature, unauthorized disclosure, inferred missing event or silent semantic loss cannot be offset by throughput or an aggregate score.

### Prospective targets (hypotheses, not achieved)
- 100% rejection of registered invalid/tampered/mismatched fixtures.
- 100% preservation of native IDs and source provenance for supported profiles.
- 100% offline reconstruction for the explicitly supported portable-bundle profile.
- 100% unsupported/unknown required semantics remain Unsupported/Indeterminate rather than silently coerced.
- External standards vectors and independent implementation comparison before any conformance claim.
- Performance and scale targets only after source/semantic correctness gates pass.

## 8. Claim ceiling

A crosswalk establishes intended mapping and gap visibility. A fixture pass establishes only that a named adapter/verifier handles the frozen synthetic cases. Neither proves actual event truth, legal title, lawful origin, sustainability, product conformity, regulatory compliance, cargo delivery, physical safety, or release authority. Those remain with authorized source owners, laboratories, inspectors, engineers, regulators and competent authorities.

## 9. Next decisions

1. Establish the active, maintained Mycelix supply-chain implementation owner and exact source revision. Do not resume development in the archived standalone repository.
2. Keep the SDK in-memory demo and Holochain zome client as separate evaluation arms; qualify persistence, authority, replay and deployed hApp separately.
3. Align the LEX-NET-015 crosswalk with Symtropy #3025/#3331, then prepare public fixtures.
4. Only after the maintained code path and test custody qualify, inspect and run the pinned NIST MVP in its documented lab scope, then test adapters against public GS1/QIF/MTConnect artefacts and independent vectors.
