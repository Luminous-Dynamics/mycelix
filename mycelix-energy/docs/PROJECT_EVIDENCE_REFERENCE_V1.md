# Mycelix Energy Project Evidence Reference v1

**Status:** Pure reference profile and conformance design, 4 November 2026.  
**Parent:** [ENERGY-PROJECT-EVIDENCE-REF-001 #5796](https://github.com/Luminous-Dynamics/mycelix/issues/5796).  
**Consumers:** Symthaea [#10439](https://github.com/Luminous-Dynamics/symthaea/issues/10439), [#10444](https://github.com/Luminous-Dynamics/symthaea/pull/10444), Symtropy [#3603](https://github.com/Luminous-Dynamics/symtropy/issues/3603), Sol-Atlas [#519](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/519).

## 1. Purpose and non-duplication

Energy needs to reference evidence for a specific project claim without treating a URL as proof or creating a second artifact/event ledger.

The canonical DeSci kernel already defines the following:
- EvidenceArtifact with ID, BLAKE3 content hash, media type, locator, license and availability.
- AtomicClaim with claim ID, research-object ID, statement and optional textual scope.
- ScientificEventEnvelope with event ID, claim stream, sequence, previous hash, actor, optional acting organization and occurred-at time.
- SignedScientificEvent with Ed25519 signature verification.
- AppendReceipt with stream, sequence, event ID, event hash and server received-at time.
- Correction, withdrawal, supersession and retraction event families.

Energy should reference those identities and receipts. It must not copy artifacts or replicate the DeSci append-only event stream.

**Important semantic boundary:** DeSci's AtomicClaim scope is human-readable scientific scope. It is not a typed Energy site ID, meter boundary, point-of-connection identity, contract boundary or equipment identity. Energy's project binding supplies those domain-specific bindings and the verifier checks them independently.

## 2. Reference model

ProjectEvidenceReferenceV1 has six independently meaningful layers:

1. **Project binding:** Energy project revision, site, subject, asset, point of connection and exact boundary.
2. **Claim binding:** the proposition this artifact is offered to support; materiality and whether it is a required gate.
3. **Canonical DeSci identity:** research object, atomic claim, artifact ID and digest, evidence-attached event, claim-stream position and append receipt.
4. **Issuer profile:** actor and organization identity, claimed domain role, authority profile and exact credential-registry revision/receipt.
5. **Time and lifecycle:** valid period, event time, server receipt-time, correction/supersession/dispute/retraction/expiry state.
6. **Independent verification receipt:** a policy-versioned, content-addressed result produced by an authorized verifier.

The JSON Schema is project-evidence-reference-v1.schema.json.

## 3. What the reference proves—and does not prove

| Item | What it can establish | What it does not establish |
|---|---|---|
| Artifact content hash | Retrieved bytes match the referenced BLAKE3 digest | The bytes are factually true |
| Signed event signature | The envelope was signed by the corresponding private key | The signer had the right role or told the truth |
| Append receipt | The canonical event log accepted this event at a server receipt time and stream sequence | The event's asserted occurred-at or physical valid-time is true |
| Credential/authority receipt | The actor/key/organization/role was authorized under a specific registry revision and policy | The issuer's substantive factual claim is true |
| Energy subject/boundary binding | The project verifier matched evidence to the target project, site, asset, point of connection and boundary | The physical site or equipment is safe or feasible |
| Lifecycle projection | The event stream currently projects the artifact/claim as current, superseded, disputed or retracted under the selected policy | A previously published record can be erased |

No single Boolean such as signature_verified, evidence_verified or project_ready is sufficient. The verification receipt binds exact verifier, policy/version, checked-at time, artifact/event identities and separate signature, authority, scope and temporal dispositions.

## 4. Time semantics

Keep three times separate:
- occurred_at: timestamp asserted in the signed DeSci event envelope;
- received_at: server receipt time in the append receipt; this anchors the first-known-at frontier for the canonical event;
- valid_from / valid_until: when the underlying proposition or measurement applies, established by source evidence and reviewed under the selected policy.

A past-period measurement may be valid evidence for a later decision. A contract or grid allocation claimed to be in force at the decision date must cover that date. A future-known source revision cannot enter an earlier historical replay even when its valid period overlaps.

Do not replace the authoritative append receipt received_at with client-submitted known_at. If the canonical receipt is unavailable, the exact known-at frontier is unavailable; do not guess.

## 5. Issuer identity and authority

DeSci verifies the signed envelope and resolves actor/key/organization/role against a governed credential registry at a specific revision. Energy then maps that exact governed role into a domain profile:
- system_operator for a grid-capacity/allocation statement;
- offtaker for an executed offtake/settlement term;
- epc_contractor for an EPC offer;
- independent_verifier for a reconciled measurement;
- model_registry for a versioned model-qualification record;
- central_bank or an official statistical issuer for a published FX observation.

A signature does not confer a role. A role does not make a statement true. A developer's self-assertion must not satisfy a system-operator gate.

Official-source publications may be unsigned data releases. Verify publisher identity, exact release/series, retrieval, content digest and policy; do not fabricate a cryptographic contract signature.

## 6. Source families and copied evidence

Every reference names an upstream source_family_id and source release. Ten consultant reports that reproduce one operator map are not ten independent sources. Preserve upstream lineage when artifacts are rehosted, transformed, summarized or incorporated into a model. A transformation adds lineage; it does not create independent corroboration.

## 7. Availability and privacy

DeSci distinguishes public, controlled, embargoed and unavailable artifacts. Energy's project reference records the access decision and license/redaction profile. A public reference must not leak controlled EPC, land/title, household meter, security or commercially sensitive site data. A denied or unavailable artifact may remain visible as historical reference but cannot satisfy a required evidence gate.

The schema intentionally uses an artifact locator reference, not a free-form public download URL. The authorized resolver decides how to retrieve content under its access policy. The schema supports an explicit legacy branch: raw locators retain their original string but canonical DeSci, issuer, provenance, temporal and verification fields must be null; they can never satisfy a required gate.

## 8. Compatibility and migration

The current Energy project hApp is pinned to Holochain HDK 0.6.1 / HDI 0.7.1. Energy Fabric #4882 already tracks the migration boundary to Holochain 0.7. Do not introduce this schema into the legacy integrity zome until that surface has an explicit compatibility/migration plan.

The current ProjectMilestone verification_evidence: Option<String> is legacy raw locator data. During migration:
1. Preserve it as unassessed_legacy_locator.
2. Do not hash the URL string and call that the document's content hash.
3. Do not infer issuer, signature, site boundary, valid-time, known-at or artifact availability from the locator.
4. Do not satisfy an issuer-signed, authenticated-meter or independent-measurement gate with a legacy URL.
5. Add a typed reference or append-only evidence-link event as a versioned successor; never reinterpret existing Holochain entry bytes silently.
6. Keep the DeSci verifier in an adapter/service boundary. Holochain integrity validation must not perform ambient network calls.

### Current DeSci service boundary

As inspected on 4 November 2026, the DeSci README describes its canonical path as an architectural refoundation requiring durable PostgreSQL events, governed credentials, write leases and receipt-time authority evidence. mycelix-desci/src/api/src/state.rs explicitly rejects DESCI_STORAGE_BACKEND=knowledge_dht because the Holochain DHT adapter was moved to the standalone mycelix-desci-holochain-bridge crate and is not linked into mycelix-api.

Therefore this schema does not assume an in-process DeSci API or direct Energy-zome-to-DeSci-DHT lookup. A live integration needs an explicitly deployed and authorized service/bridge, exact capability profile and receipt verification. Service unavailability yields indeterminate/unavailable, never verified.

## 9. Conformance requirements

- Resolve artifact ID and verify retrieved bytes against exact BLAKE3 content digest.
- Resolve the EvidenceAttached event, verify Ed25519 signature and compare artifact ID/digest to the reference.
- Verify event hash and append receipt event ID, stream ID, sequence and event hash.
- Resolve authority receipt and exact credential-registry revision; verify actor, key, organization, role and effective interval.
- Verify role maps to the domain issuer profile for the claim type.
- Match Energy project revision, site, subject, asset, point of connection and boundary exactly.
- Verify valid-time and server known-at/receipt-time; reject future-known, expired or out-of-frontier evidence.
- Replay correction, withdrawal, supersession, retraction and dispute events; never hide prior history.
- Preserve source-family identity across copies and transformations.
- Enforce availability, license, redaction and authorization policy.
- Return indeterminate when any required service, receipt, revision or lifecycle head is unavailable.

## 10. Hostile conformance corpus

project-evidence-reference-v1.fixtures.json provides synthetic fixtures, including one schema-shaped valid example, one legacy-locator example and mutation descriptions:
1. Valid reference to a signed artifact, receipt, authorized issuer and exact project boundary.
2. Legacy raw URL with no content digest or authority receipt.
3. Same artifact but wrong site/point-of-connection boundary.
4. A future-known append receipt.
5. Valid Ed25519 signature from an actor not authorized as a system operator.
6. Superseded artifact whose earlier evidence must remain historical but not current.
7. Two artifacts copied from one upstream source family.
8. Embargoed artifact requested through an unauthorized public view.

The public fixtures define expected dispositions and are not a secret certification set.

## 11. Nonclaims

A conformant reference proves only that the selected source artifact/event and authority receipt match the reference and policy, and that the exact project/boundary/time/lifecycle/access constraints were applied. It does not prove the source's factual truth, land title, grid allocation, contractual enforceability, meter calibration, physical delivery, engineering safety, regulatory compliance, settlement finality, bankability or investment suitability.

No DeSci/energy reference grants lending, brokerage, custody, capital-allocation, payment or equipment-actuation authority.
