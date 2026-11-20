# Space Operations Federation Profile v0.1

**Programme:** Federated Space Resilience  
**Owning work item:** [Mycelix #6362](https://github.com/Luminous-Dynamics/mycelix/issues/6362)  
**Status:** Research contract proposal; synthetic/read-only scope  
**Version:** 0.1.0-draft  
**Date:** 2026-11-20

## 1. Purpose

Define an interoperable, sovereignty-preserving evidence exchange profile for civil space-safety coordination. Independent satellite operators, ground-segment providers, public agencies and research institutions should be able to exchange safety-relevant evidence without centralizing mission authority or exposing all proprietary telemetry.

The first implementation is a **read-only evidence federation**. It does not issue spacecraft commands, execute maneuvers, target objects, interfere with other spacecraft or replace an operator's safety-of-flight process.

The profile is designed to compose existing Luminous work:

| Owner | Existing responsibility | Space-federation contribution |
|---|---|---|
| Mycelix #6141 and `mycelix-space` | Scientific event/evidence kernel and space-domain observations | Signed evidence events, source lineage, scoped sharing and multi-party replay |
| Mycelix #6209 / #6147 | Source-native time/frame preservation and read-only VOEvent adapter patterns | Loss-aware source adapters and exact temporal/spatial semantics |
| Mycelix #6018 / #5762 | Partition-tolerant coordination and dependency/correlation accounting | Degraded-mode exchange and common-cause evidence lineage |
| Symthaea #12075 | Anomaly/hypothesis/investigation loop | Explainable, uncertainty-aware decision support with abstention |
| Symtropy #6167 / #6329 / #6351 / #4217 | Space capability crosswalk, interface conformance, robust compatibility and astrodynamics interop | Digital-twin context and independent differential qualification |
| Sol-Atlas #359 | Federated digital-twin evidence interoperability | Rights, jurisdiction, stewardship and auditable governance |

No layer may self-certify another layer's claims.

## 2. Design invariants

1. **Local authority remains local.** A federation can distribute evidence and coordination messages; it cannot grant another operator authority over an asset.
2. **Source integrity is not physical truth.** A valid signature or digest proves a limited provenance/integrity property, not that the physical assertion is correct.
3. **Source semantics are preserved.** Time scales, coordinate frames, reference positions, units, covariance representation, event identity and source-native fields must not be silently normalized away.
4. **Unknown is a first-class result.** Missing, stale, contradictory, out-of-domain or unsupported evidence yields `indeterminate`, `stale` or `unsupported`, never a silent pass.
5. **No false independence.** Rewrapped copies, mirrors, shared sensors, common calibration, shared clocks, shared catalogues and common processing pipelines do not become independent corroboration by receiving distinct IDs.
6. **Append, do not overwrite.** Corrections, supersessions, revocations and disputes are linked events. Historical state remains reconstructable.
7. **Partitioned views are visibly incomplete.** Local operation continues under local policy; a partial federation view must not masquerade as globally complete.
8. **Evidence is purpose-bound.** Only the minimum data authorized by the source owner and sharing agreement is disclosed.
9. **Humans and competent authorities retain consequential decisions.** The first profile is read-only and non-actuating.
10. **Every claim has a ceiling.** Synthetic conformance, public-data replay, physical test and operational qualification are distinct evidence classes.

## 3. Threat and failure model

The profile addresses accidental and malicious failure at the information, identity, coordination and service-continuity layers:

- corrupted, incomplete, malformed, stale or replayed messages;
- invalid, expired, revoked or out-of-scope identities/credentials;
- a correctly authenticated issuer making an incorrect or unsupported claim;
- source, sensor, calibration, clock, catalogue or processing common-cause dependencies;
- conflicting operator reports and contradictory federation views;
- clock skew, time-scale confusion, coordinate-frame mismatch and unit conversion errors;
- delayed, duplicated, reordered or partially replicated events;
- network partition, node restart, queue backlog and incomplete recovery;
- unauthorized disclosure or use beyond an agreed purpose;
- unavailable ground stations, provider services or information sources.

This is not an offensive cyber playbook. The v0.1 work does not specify counterspace capabilities, intrusion techniques, target selection, jamming, interception, weapons, autonomous engagement or spacecraft command injection. Security testing is limited to defensive control validation, synthetic fault injection and independent assessment.

## 4. Canonical envelope

The canonical envelope is a Mycelix event/evidence record plus a space-safety profile. Domain payloads remain source-native. The envelope must not replace CCSDS schemas.

### 4.1 Required logical sections

| Section | Required semantics |
|---|---|
| Identity | Stable event ID, schema/profile version, event kind, source-native ID |
| Issuer | Issuer DID, credential reference, verification method, authority scope and role |
| Artifact | Original media type, byte length, digest algorithm/value, immutable locator or controlled retrieval reference |
| Time | Phenomenon interval, source publication time, federation receipt time, processing time, clock/time-scale identity and validity interval |
| Space | Original frame/reference-position/coordinate declarations, units, coordinate epoch and transform receipts, if any |
| Uncertainty | Original representation, dimensions, units, frame, epoch, coverage meaning, producer and dependence receipt references |
| Lineage | Source artifact IDs, parent events, processing profile, tool/version, configuration and output digest |
| Rights | Data owner, permitted purpose, permitted recipients, retention/disclosure constraints and expiry |
| Authority | Asset/service scope, jurisdiction or policy reference, permitted federation actions |
| Disposition | `accepted_for_analysis`, `indeterminate`, `stale`, `rejected`, `disputed`, `superseded` or `retracted`, with reason codes |
| Claim ceiling | Explicitly limits what the envelope or downstream projection is allowed to claim |

The complete envelope binds the exact original bytes. Any canonical projection is a separate, reproducible derived artifact.

### 4.2 Time semantics

Keep these clocks distinct:

- **phenomenon time:** when the physical observation/event is asserted to have occurred;
- **source publication time:** when the source emitted the record;
- **federation receipt time:** when a node received it;
- **processing time:** when a particular transformation/evaluation ran;
- **knowledge/effectivity time:** when an assertion became known or valid within a given policy/state.

Each timestamp binds its time scale and clock identity. UTC, TT, GPS, TDB and monotonic clocks are not interchangeable. A transformation requires an explicit, versioned transform receipt. Missing time is absent/unknown, not zero or an inferred default.

### 4.3 Spatial and orbital semantics

Preserve source-native frame, origin/reference position, epoch, units, coordinate axes, object identity and any covariance/uncertainty metadata. A transformed state must identify:

- input and output frames/origins;
- input and output epochs/time scales;
- transform implementation, version, constants/kernel IDs and configuration;
- any interpolation/propagation method and model scope;
- uncertainty transformation and assumptions;
- exact input/output artifact digests.

A friendly name, object catalogue ID or standard label is not sufficient proof of physical identity or profile compatibility.

### 4.4 Uncertainty semantics

Do not collapse these into a generic `sigma`:

- deterministic hard bound;
- standard uncertainty;
- expanded uncertainty;
- probabilistic coverage interval;
- prediction interval;
- covariance matrix with declared frame/order/units;
- ensemble or sample distribution;
- unknown or unsupported representation.

The adapter preserves and scopes these records. It does not silently compute orbit determination, covariance realism, probability of collision or physical truth. Those remain with qualified domain owners and reference algorithms.

## 5. Federation protocol

The initial topology is a set of independently administered peers. No single peer is presumed to be the universal source of truth.

### 5.1 Exchange lifecycle

`publish -> authenticate issuer -> verify bytes -> preserve source semantics -> check sharing policy -> record receipt -> distribute authorized projection -> acknowledge / dispute / correct / revoke -> replay`

Each step produces a receipt linking input and output digests, software/profile revision, time, policy scope and disposition.

### 5.2 Idempotency and corrections

- Event identity is source-scoped and stable.
- A duplicate delivery is recorded as a delivery/replay observation, not as a new physical observation.
- Reordering must not alter source event time or historical order.
- Corrections append a new event with an explicit `supersedes` or `corrects` relationship.
- Revocation prevents future authorization under the revoked scope; it does not erase historical evidence that the credential was previously used.
- Conflicts remain visible. No silent last-write-wins overwrite of physical observations or authority assertions.

### 5.3 Partition and recovery

Each node must be able to distinguish:

- locally verified records;
- remote records received and verified;
- remote records received but not yet verified;
- records known to be stale;
- peers currently unreachable;
- unresolved conflicts;
- incomplete time ranges or catalogue coverage.

A disconnected peer may continue local operations under its own authorized policy. The federation must not promise global consistency while partitioned. On recovery, nodes reconcile by stable event identity and explicit supersession relationships, preserve conflicts, and produce a replayable reconciliation receipt.

### 5.4 Authorization and rights

Authorization is an explicit relation between issuer, action, asset/service scope, recipient, purpose, policy version and validity interval. Use the existing Mycelix DID/VC and access-policy owners. Do not invent a parallel trust score or credential registry.

The following are separate permissions: read, publish, acknowledge, dispute, correct, revoke, and disclose a derived projection. A participant allowed to publish an observation is not thereby allowed to access another participant's raw telemetry or authorize a maneuver.

## 6. System boundary and responsibility split

### Mycelix

Own the federation event envelope, identity/credential references, policy evaluation receipts, source provenance, revocation, disputes, append-only history and multi-peer replay. Holochain/DHT participation is not a physics oracle and must not be placed in a real-time spacecraft control loop.

### Symthaea

Consume qualified evidence projections. Generate ranked hypotheses, anomaly summaries and read-only follow-up suggestions. Report source dependencies, uncertainty, counter-hypotheses, evidence gaps and abstentions. It may not convert an anomaly score into a physical fact, command or authorization.

### Symtropy

Provide simulation/digital-twin context and benchmark the physical/engineering domains within their declared scope. Keep orbital propagation, frame transformation, covariance estimation and conjunction algorithms tied to explicit reference implementations and external comparison. Synthetic physics is not flight evidence.

### Sol-Atlas

Preserve the rights and governance envelope: data ownership, consent, jurisdiction, permitted purpose, environmental/public-interest impacts, dispute process and audit record. A governance decision is not a physical measurement; a technical result does not override rights.

## 7. Benchmark design

### 7.1 Evidence tiers

| Tier | Inputs | Claim ceiling |
|---|---|---|
| T0 — synthetic contract | Frozen known-answer fixtures and synthetic peer events | Schema/policy/replay conformance only |
| T1 — public reference | Applicable NASA CARA public test cases and TraCSS verification dataset | Differential result for exact tested conventions and corpus |
| T2 — independent testbed | Separate implementations, security review, external evaluator and consented operator data | Bounded testbed qualification |
| T3 — operator shadow pilot | Read-only deployment with explicit data-sharing agreement | Pilot evidence only; not operational certification |
| T4 — operational/flight | Applicable external certification, safety authority and mission evidence | Only the exact authority-approved scope |

The TraCSS dataset requires access request and includes a published answer key. Its publisher says it is not intended for live operations or formal certification. This limitation must be reflected in every report.

### 7.2 Synthetic corpus

Freeze at least 30 known-answer cases before implementation scoring. Include:

1. valid, correctly signed and in-scope source record;
2. valid issuer but malformed payload;
3. valid signature with unsupported physical assertion;
4. byte mutation after signing;
5. digest mismatch;
6. duplicate event ID and duplicate mirrored packet;
7. delayed delivery with old phenomenon time;
8. reordered event arrival;
9. same event relayed through two brokers;
10. unknown issuer;
11. expired credential;
12. revoked credential;
13. credential valid for wrong asset;
14. publisher role without correction permission;
15. invalid sharing purpose;
16. revoked data-sharing agreement;
17. source frame mismatch;
18. time-scale mismatch;
19. unresolved coordinate transform;
20. unit mismatch;
21. missing covariance provenance;
22. coverage interval mislabeled as a hard bound;
23. shared sensor or calibration incorrectly marked independent;
24. stale ephemeris;
25. conflicting operator assertions;
26. partitioned peer with incomplete remote view;
27. node restart with queued deliveries;
28. recovery with duplicate/reordered events;
29. correction/supersession chain;
30. retraction and historical replay;
31. AI-generated summary presented as an observation;
32. consensus/identity validity incorrectly used as physical truth;
33. out-of-domain model result;
34. incomplete dependency graph;
35. rights-restricted artifact requested by an unauthorized recipient.

Expected statuses are independently hand-derived and frozen. The evaluator must be separately implemented from the producer.

### 7.3 Metrics

| Metric | Acceptance target |
|---|---|
| Integrity | 100% detection of mutated bytes on the frozen corpus |
| Semantic loss | Zero silent loss of required source-native time/frame/unit/covariance semantics |
| Authority | Zero unauthorized protected reads/writes accepted in negative controls |
| Revocation | Zero new disclosures/writes authorized by expired, revoked or wrong-scope credentials |
| Freshness | All stale or incomparable-clock fixtures marked stale/indeterminate |
| Replay | 100% identical derived projection for identical frozen inputs and versions |
| Idempotency | Zero duplicate authoritative events or side effects after duplicate/reordered delivery |
| Common cause | Unknown dependencies never treated as independent |
| Partition | Incomplete global view never labelled complete; local authority is not overwritten |
| Recovery | Deterministic reconciliation with explicit conflicts and no silent overwrite |
| Audit | 100% of derived claims linked to source artifact, transform, policy and software/configuration revisions |
| Interoperability | 100% expected status agreement with an independent oracle on the frozen corpus |

For Symthaea's analysis stage, report precision, recall, false alerts per asset-day, missed-event classes, detection lead time, Brier/log score, empirical calibration and abstention coverage. Scores are per event family and lead-time band; no composite score may hide a safety-critical miss.

For performance, measure ingestion throughput, p50/p95/p99 processing latency, backlog recovery time, storage growth and peer-reconciliation cost. Do not set an aspirational percentage faster than a provider until a matched hardware, input, coverage and correctness comparison exists.

## 8. External references and baseline requirements

- [NASA CARA public software](https://www.nasa.gov/cara/publicly-available-cara-software/) — reuse the published 2D/3D probability-of-collision, Monte Carlo, collision-consequence and covariance-realism reference algorithms as applicable; do not rebuild them without an evidenced gap.
- [NASA 2026 Conjunction Assessment Handbook](https://www.nasa.gov/cara/) — use its ephemeris, covariance, screening, risk-assessment, mitigation and operator-coordination practices as a baseline.
- [TraCSS Conjunction Assessment Verification dataset](https://space.commerce.gov/dataset-for-conjunction-assessment-verification/) — public-domain CC0 dataset; access request required; not a live-ops certification corpus.
- [TraCSS updated specifications](https://space.commerce.gov/osc-publishes-updated-tracss-specifications/) — CDM v2.1, OMM v1.2 and OCM v2.1.
- [CCSDS Space Data Link Security Protocol 355.0-B-2](https://ccsds.org/view/bluebooks/entry/3258/) — link-layer security profile.
- [NIST IR 8401](https://csrc.nist.gov/pubs/ir/8401/final) — ground-segment command-and-control cybersecurity framing.
- [CISA space-system operator recommendations](https://www.cisa.gov/resources-tools/resources/recommendations-space-system-operators-improving-cybersecurity) — defensive control baseline.
- [ISO 24113:2023](https://www.iso.org/standard/83494.html) — debris-mitigation baseline; monitor the fifth-edition revision.

Commercial comparisons should include the published capabilities of LeoLabs, Slingshot Aerospace, Kayhan Space and COMSPOC. Vendor claims are not independent test results. The competitive question is whether Luminous reduces evidence semantic loss, stale-data acceptance, false corroboration, policy ambiguity, coordination friction and recovery failure—not whether it owns better sensors or currently predicts orbits more accurately.

## 9. Delivery gates

- **P0 — contract freeze:** envelope, authority matrix, rights model, time/frame semantics, reason codes and claim ceilings.
- **P1 — corpus:** 30+ synthetic cases, independent oracle, positive/negative controls.
- **P2 — read-only adapters:** selected CDM/OEM/OCM versions, exact source-byte preservation and loss reports.
- **P3 — three-peer test:** delayed/reordered/duplicate delivery, partition, restart, revocation and reconciliation.
- **P4 — public differential:** applicable NASA CARA / TraCSS reference cases with convention matching and exact build/data hashes.
- **P5 — independent review:** security, privacy, orbital-safety, governance and standards review.
- **P6 — consented shadow pilot:** read-only exchange with operators; no command or maneuver execution path.

## 10. Non-claims

A passing synthetic corpus does not establish sensor accuracy, physical truth, orbit determination quality, calibrated probability of collision, maneuver safety, flight qualification, cybersecurity certification, legal authority, spacecraft control or readiness for operational deployment.

The profile's intended contribution is a narrow but important one: independently verifiable evidence federation that helps space operators coordinate while retaining local sovereignty and accountable human authority.
