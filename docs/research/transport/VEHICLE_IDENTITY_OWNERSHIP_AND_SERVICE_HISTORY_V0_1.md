# Vehicle Identity, Ownership, Custody and Service History — Contract v0.1

**Status:** Research contract; not implemented, deployed, or legally authoritative.  
**Date:** 2026-10-17  
**Intended home:** Mycelix Commons, adjacent to (not silently conflated with) the existing transport-sharing and property-transfer domains.

## Why this work exists

Affordable transport depends on more than vehicle design. Owners need trustworthy access to vehicle configuration, maintenance history, parts provenance, repair instructions, recall notices, cost history, and a way to transfer a usable record when a vehicle changes hands.

Mycelix is a strong candidate for the **user-consented provenance and coordination layer**. It must not become a substitute for a government title register, a manufacturer's engineering authority, or an independent safety inspector.

The Mycelix root README describes Commons as pre-alpha, and `mycelix-commons/zomes/DORMANT_TRANSPORT.md` documents the existing route/sharing/impact zomes as dormant and flags missing deployment and integration evidence. Its audit date is 2026-07-30 and explicitly calls for re-checking moving source. Therefore, do not count those zomes as a functioning vehicle registry or booking service. Before implementation, re-audit the exact current head and preserve that honest maturity boundary.

## Paradigm shift: a private-by-default vehicle dossier with independently attested events

Model the vehicle as a persistent engineering/service identity and its lifecycle as a sequence of typed claims with evidence, not one mutable “car profile” whose latest value erases disagreement.

```text
design/configuration manifest (Symthaea / manufacturers)
                    |
                    v
simulation and test evidence (Symtropy / labs / engineers)
                    |
                    v
vehicle dossier + consent / attestations (Mycelix)
                    |
                    v
maintenance, parts, operating-cost and ownership decisions
                    |
                    +----> new field observations feed research (consent-based)
```

Every edge is a reference with a claim ceiling. Recording a simulation, service visit or ownership assertion never promotes it to physical qualification, roadworthiness, legal title or verified performance.

## Core record model (conceptual, not yet a Holochain schema)

Each record envelope should be versioned and validate strictly. Proposed common metadata:

| Field | Meaning |
|---|---|
| `schema_version` | Exact record-contract version; never infer from a display label. |
| `event_id` | Unique event identity, stable across export/import. |
| `vehicle_ref` | Opaque, non-guessable dossier reference; not a public VIN by default. |
| `event_kind` | One typed event kind from the catalogue below. |
| `actor_ref` | Pseudonymous participant reference; public attribution only where justified and consented. |
| `occurred_at` / `recorded_at` | Distinct occurrence and record times, with clock/provenance caveats. |
| `payload_digest` | Digest over canonicalized payload bytes, with algorithm and canonicalization version. |
| `evidence_refs` | References to source documents, signatures, measurements, or engineering bundles. |
| `visibility_policy` | Explicit audience and release purpose; never assume DHT publication is private. |
| `claim_status` | `asserted`, `counterparty_attested`, `independently_verified`, `disputed`, or `superseded`. |
| `supersedes` / `dispute_refs` | Append-only correction and dispute links; do not erase prior signed history silently. |

Canonicalization, signatures, identities, timestamps, evidence schemas and revocation semantics need a concrete implementation profile before any event is accepted as qualified.

### Event catalogue

1. **Vehicle dossier creation:** opaque asset reference, coarse vehicle class and a private identity binding where appropriate.
2. **Configuration/build manifest:** model family, model-year/variant scope, exact part/configuration identifiers where known, and manifest digest. The engineering source remains the manufacturer or engineering author.
3. **Ownership or custodial claim:** claimant, role, effective interval, source evidence and counterparties. Distinguish owner, legal title holder, keeper, lender/lienholder, lessee, fleet manager, shared user and temporary custodian.
4. **Transfer proposal and acknowledgement:** seller/transferor and recipient/beneficiary attest to the same transfer digest. A unilateral record is a claim, not completed transfer. Conflicting or disputed assertions remain visible as conflicts.
5. **Maintenance/repair event:** work performed, parts installed/removed, provider identity, odometer/meter reading and units, measurement source, date, procedure/version, evidence, and post-repair checks. Do not require public disclosure of the vehicle's exact location or owner's identity.
6. **Part provenance and compatibility:** part identity/revision, maker or source, batch/serial where warranted, installation context and explicit compatible configuration scope. “Fits” is not “safety-qualified.”
7. **Inspection / test evidence:** issuer, exact scope, method and protocol version, instrument/calibration status, result, limitations, expiry/refresh rules and evidence reference.
8. **Recall/service bulletin:** issuer and authentic source reference, affected variant/serial scope, publication date, remedy and completion evidence.
9. **Cost event:** repair/part/labour/energy/insurance cost with currency and date. Private by default; aggregated research exports require consent and re-identification review.
10. **Dispute, correction, loss, theft or retirement:** typed event with appropriate evidence. Do not expose sensitive location/security information to public peers.

## Authority and evidence boundaries

The record layer must preserve these distinctions:

```text
signed claim != true claim
recorded keeper != legal title holder
ownership evidence != proof of safe vehicle
repair invoice != verified correct repair
part identity != part authenticity
part compatibility != engineering qualification
inspection receipt != universal roadworthiness
simulation result != physical test result
vehicle observed != trip history approved for disclosure
```

- Legal ownership/title is jurisdiction-dependent. Store signed copies or references to competent external evidence and state verification scope; do not market a Mycelix event as creating or replacing legal title.
- Service providers can attest to their work, and owners can attest to their own observations. These are different claim classes.
- A shop credential proves who signed a claim only to the extent the credential and signing path were verified; it does not make the claim infallible.
- Keep source evidence, not only a summary score. Conflicts and negative findings must remain inspectable by authorized parties.
- Ownership and maintenance records cannot authorize vehicle motion, change safety limits, certify self-driving, or override recall/inspection requirements.

## Privacy and sovereignty

Holochain supports private entry content as well as shared, peer-validated data, but public action metadata may still leak relationships or activity. This design must make that a first-order threat model, not assume private payloads make all metadata private.

Default policy:
- Keep names, contact information, proof of legal identity/title, finance agreements, detailed odometer history and precise trip/location data off shared DHT records unless disclosure has a well-defined legal basis and is expressly authorized.
- Use opaque asset refs and purpose-specific, expiring disclosure grants.
- Share only the minimum evidence required for a transaction or repair; use selective disclosure or a verifiable claim where feasible.
- Let owners export the dossier in a documented, machine-readable format and transfer permitted records between providers without a vendor account.
- Separate authorization to view the dossier from authorization to edit, attest, dispute or transfer it.
- Support loss/recovery and succession without an invisible universal administrator. Do not pretend cryptography can resolve contested legal ownership by itself.
- Threat-model stalking, theft targeting, fraudulent repair claims, cloned identities, compromised workshops, colluding peers, stolen devices, coerced consent and denial of access.

## Integration contracts

The first integration should be reference-based, versioned and offline-capable, not a deep synchronous runtime dependency.

- **Symthaea → Mycelix:** immutable references to design/configuration artifacts and evidence bundles, with exact producer commit, inputs, provenance and claim ceiling.
- **Symtropy → Mycelix:** simulation run reference, model/solver revision, parameter and scenario digests, output digest, uncertainty, and explicit `simulation_only` evidence type.
- **Mycelix → Symthaea/Symtropy:** consented, minimized, pseudonymized service/failure observations with units, collection method, quality flags and selection-bias notes. No raw owner identity or trip traces by default.
- **Manufacturer / repairer / lab → Mycelix:** signed assertions tied to verified organizational credentials and exact scope; verification and dispute state remain visible.
- **Recall issuer → dossier:** source reference and affected configuration scope; completion is a separate event.

A repair procedure is represented separately by Symtropy's [Repairability Task Graph v0.1](https://github.com/Luminous-Dynamics/symtropy/blob/research/family-vehicle-simulation-v0_1/schemas/transport/repair_task_graph_v0_1.schema.json) and its [synthetic graph fixtures](https://github.com/Luminous-Dynamics/symtropy/blob/research/family-vehicle-simulation-v0_1/fixtures/personal_vehicle/repair_task_graph_fixtures_v0_1.json). A procedure graph says what should happen and which dependencies/preconditions apply; a Mycelix maintenance event is an actor's claim about what actually happened. Never copy estimated task durations into the dossier as measured shop performance, or treat an observed-shaped synthetic fixture as field evidence.\n\nThe canonical common engineering artifact contract is the [Mobility Engineering Evidence Envelope v0.1](https://github.com/Luminous-Dynamics/symtropy/blob/research/family-vehicle-simulation-v0_1/schemas/transport/engineering_evidence_envelope_v0_1.schema.json), with synthetic acceptance/negative fixtures at [engineering evidence envelope fixtures](https://github.com/Luminous-Dynamics/symtropy/blob/research/family-vehicle-simulation-v0_1/fixtures/personal_vehicle/engineering_evidence_envelope_fixtures_v0_1.json). It binds versioned artifact/subject/configuration identity, producer revision/tool, input/output digests, evidence class, scope/conditions, uncertainty, source/rights references, privacy class, limitations and claim ceiling.

Keep this engineering-artifact envelope **separate** from Mycelix's owner consent, signature and lifecycle-event envelope. Mycelix events may reference a design/simulation/measurement/review artifact by ID and digest; they must not mutate its evidence class. A signature authenticates a signer only after cryptographic and credential-scope verification—it does not make the signed statement true, convert a simulation into a physical test, or establish legal title. The shared schema and Rust semantic fixtures are in a draft PR branch and await exact-head validation; do not describe them as deployed or qualified. Do not put owner PII into the public engineering envelope.


## Current schema and independent-validator status (2026-10-17)

The draft includes a Draft 2020-12 envelope schema, 19 synthetic decision fixtures, 9 concrete synthetic record-instance fixtures, a standalone Rust validator under `tools/vehicle-record-audit/`, and a focused workflow under `.github/workflows/vehicle-record-schema-audit.yml`.

The vehicle record requires an opaque `veh_` reference, a separate top-level `envelope_digest`, and a signature envelope containing `signed_digest`, canonicalization-profile name and domain-separation context. The current Rust audit checks that the signed digest matches the declared envelope digest, that the profile names match, and that the context matches the event schema. **It does not recompute canonical envelope bytes/digests or cryptographically verify the signature/credential.** Production must define canonical serialization, recompute the digest, perform cryptographic signature verification and check signer credential scope/authorization.

The `maintenance_performed` event now has a strict event-specific payload schema. It must reference the repairability task graph and digest, procedure revision, per-task results bound to component revision, and a post-repair verification result. Any reported *observed* duration must bind to an observation reference, timestamp and source refs; estimates require an explicit note. A passed/failed/inconclusive post-check needs its own observation/evidence. The schema cannot prove that the procedure graph is accurate, the task really happened, a source is authentic, or the check was correctly performed. A service event is an actor's claim with evidence, not proof of roadworthiness.

A recursive shared/public payload rule rejects canonical sensitive field names such as `owner_name`, `raw_vin`, `precise_location`, `trip_points`, and common finance/registration identifiers. This is a field-name guard only—not a comprehensive PII detector. Value-level leakage, alternate key encodings, external attachments/metadata, consent validity and credential scope remain separate review and verification obligations.

The Rust audit now reads both the decision-fixture census and concrete record-instance corpus, validates Draft 2020-12 instance shape, checks digest-binding semantics, and emits SHA-256 digests for schema/fixtures/record instances. Negative instance fixtures cover missing signature, digest mismatch, missing procedure graph, observed duration without source, post-check without evidence, public raw VIN, missing independent-verification attestation and missing dispute reference.

The schema, fixtures and Rust code are committed on the draft branch, but **the validator has not been compiled or executed in this environment**. Hosted exact-head CI is required before reporting PASS. The workflow path filters include both fixture files.

A schema-valid envelope establishes only format conformance plus the declared-reference consistency checks. It cannot establish legal ownership, the truth of signed claims, authentic parts, correct repairs, roadworthiness, or private treatment of arbitrary external attachments/metadata.
## Initial implementation sequence

**M0 — Exact-head audit.** Re-check the transport zomes, manifests, SDK, client and tests at the implementation commit. Confirm current workspace membership and whether WASM compilation/HApp packaging or real multi-agent tests exist. Do not infer current state from an older coverage audit.

**M1 — Offline schema and independent validator.** Define strict record types and a stdlib/independent validation oracle first. Freeze canonical fixtures and negative controls. Validate duplicate event IDs, bad/missing digests, malformed signatures, conflicting ownership claims, inconsistent meter chronology, wrong part compatibility scope, superseded and disputed records, expired credentials, and privacy-class violations.

**M2 — One repair-history vertical slice.** On a test DNA, create a pseudonymous vehicle dossier; accept a provider-signed repair event; link the fitted part and post-repair measurement; export/import and independently verify it; demonstrate that private owner data is absent from shared fixtures.

**M3 — Transfer and consent.** Add two-party transfer acknowledgment, authorized disclosure, revocation/expiry of disclosure grants, dispute handling, and concurrent conflicting claims. No “last writer wins” for ownership.

**M4 — Integration tests and deployment evidence.** Compile intended WASM targets; build/package the actual DNA/HApp; execute non-ignored multi-agent tests over at least two agents and conductor restart; test frontend-to-zome calls only after the backend contracts hold. Keep the cluster marked pre-alpha until the evidence actually supports a maturity upgrade.

## Benchmarks and acceptance criteria

These are engineering gates, not claims of present capability.

| Benchmark | Initial acceptance target |
|---|---|
| Schema determinism | Same canonical fixture → same digest across two independent implementations; malformed/noncanonical variants rejected. |
| Tamper detection | 100% rejection of the frozen mutation corpus; include payload, actor, subject, timestamps, evidence and schema/version mutations. |
| Ownership conflicts | 100% of frozen unilateral, double-transfer and concurrent-conflict cases become unresolved/disputed rather than silently accepted. |
| Privacy | Zero owner names, raw VINs, precise trip points or finance details in the public fixture corpus; automated leak scan over export examples. |
| Repair lineage | Every accepted repair fixture binds vehicle generation + installed part revision + work procedure + issuer + post-work result + evidence refs, or explicitly records missing fields. |
| Portability | Round-trip export/import preserves event IDs, signatures, evidence refs, conflicts and claim boundaries. |
| Availability | Once packaged for the declared test profile, dossier read/verify works offline from exported records; remote peers are not required to reconstruct the owner’s private data. |
| Multi-agent correctness | Real two-agent transfer and service-attestation flows under conductor restart; no single-agent serde test may stand in for this. |
| Deployment | Exact current source builds for intended WASM target and the actual DNA/HApp packs; receipt binds source SHA and artifact digest. |

### Negative controls that must remain

- unsigned invoice presented as provider-attested service;
- service claim references a different vehicle generation;
- duplicate event with altered payload;
- identical display VIN attached to conflicting pseudonymous dossier identities;
- legal-title claim lacking the external evidence required by that market profile;
- unauthorized third party attempts to read private ownership/trip data;
- one party unilaterally claims completed transfer;
- concurrent transfers accepted through last-write-wins;
- repair marked successful despite failed post-repair measurement;
- expired or revoked workshop credential;
- recall record omitted from a sale view due to stale cached status;
- deletion/supersession that erases prior signed history.

## Claim ceiling

A passing software benchmark establishes conformance to the named data contract on the tested fixtures. It does not establish legal title, fraud-free ownership, authentic parts, correct workmanship, safe design, roadworthiness, real-world deployment, or production readiness.

## References

- Mycelix Commons transport zome audit: `mycelix-commons/zomes/DORMANT_TRANSPORT.md` (re-check claims against exact current HEAD before implementation).
- Holochain, [Working with Data](https://developer.holochain.org/build/working-with-data/).
- Holochain, [DHT and validation model](https://developer.holochain.org/concepts/4_dht/).
- Cross-project engineering consumers: [Symthaea transport architecture #6244](https://github.com/Luminous-Dynamics/symthaea/issues/6244), [road vehicle profile #6247](https://github.com/Luminous-Dynamics/symthaea/issues/6247), and [Symtropy whole-life transport optimization #1601](https://github.com/Luminous-Dynamics/symtropy/issues/1601).
