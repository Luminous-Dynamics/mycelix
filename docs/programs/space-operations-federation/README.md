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

The TraCSS Dataset for Conjunction Assessment Verification is a public, CC0-1.0 reference set with access available through an OSC request form (Google-account access is also offered). The published package includes a 20.73 GB ephemeris archive, separate spherical-screening and SFSH rectangular-screening answer-key results (198.8 MB and 62.4 MB), an SFSH object-to-volume mapping file (145 KB), and a user guide (363 KB). The two answer-key profiles must be benchmarked separately; do not merge their event counts or screening-volume assumptions. OSC describes the set as including stressing edge cases alongside an operational catalog, but explicitly says it is not comprehensive and is not intended for live operations or formal system certification/validation. It is an external self-assessment corpus, not independent propagation truth or Pc-calibration ground truth. Preserve its CC0 notice, original archive digest, file manifest, request/access provenance and user-guide version in the benchmark receipt.

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
- [TraCSS Conjunction Assessment Verification dataset](https://space.commerce.gov/dataset-for-conjunction-assessment-verification/) — CC0-1.0, access request via OSC; 20.73 GB ephemerides plus distinct spherical and SFSH answer keys, mappings and user guide; not a live-ops or formal-certification corpus.
- [TraCSS updated specifications](https://space.commerce.gov/osc-publishes-updated-tracss-specifications/) — CDM v2.1, OMM v1.2 and OCM v2.1.
- [CCSDS Space Data Link Security Protocol 355.0-B-2](https://ccsds.org/view/bluebooks/entry/3258/) — link-layer security profile.
- [NIST IR 8401](https://csrc.nist.gov/pubs/ir/8401/final) — ground-segment command-and-control cybersecurity framing.
- [CISA space-system operator recommendations](https://www.cisa.gov/resources-tools/resources/recommendations-space-system-operators-improving-cybersecurity) — defensive control baseline.
- [ISO 24113:2023](https://www.iso.org/standard/83494.html) — debris-mitigation baseline; monitor the fifth-edition revision.

Commercial comparisons should include the published capabilities of LeoLabs, Slingshot Aerospace, Kayhan Space and COMSPOC. Vendor claims are not independent test results. The competitive question is whether Luminous reduces evidence semantic loss, stale-data acceptance, false corroboration, policy ambiguity, coordination friction and recovery failure—not whether it owns better sensors or currently predicts orbits more accurately.

## 9. Industry baseline snapshot and fair comparison (2026-11-20)

This portfolio is not entering an empty market. The following are **provider-published capability descriptions**, not independently reproduced performance results:

| Organisation / service | Publicly described capability | Implication for the benchmark |
|---|---|---|
| NOAA Office of Space Commerce — TraCSS | Public civil SSA and operator coordination. Its November 2026 overview reports 90 pilot user organisations representing more than 15,580 satellites and 13 national government accounts. | Treat TraCSS as a public interoperability and operational adoption reference, not as an adversary to replace. |
| LeoLabs | Radar-based space-domain awareness. Its site advertises a 29K+ object catalog, state-vector accuracy below 15 metres and latency below two minutes for specified products. | Vendor-published metrics only; require orbit-regime, object class, epoch, sensor coverage and independent truth before comparison. |
| Slingshot Aerospace | Global Sensor Network and orbital-data products; Agatha AI anomaly analytics; Digital Space Twin; TALOS AI maneuver decision support; Beacon conjunction alerts and operator-to-operator coordination; developer APIs and tracking tasking. | This already covers many product concepts in our initial proposal. Our differentiation cannot simply be “AI + digital twin + coordination”; it must be measured semantic fidelity, evidence lineage, replay, rights and multi-provider resilience. |
| COMSPOC | On-premise Space Situational Awareness Suite, multi-sensor ingestion/fusion, Extended Kalman Filter-Smoother orbit estimation and covariance, mission-awareness analysis, training/event simulation and data portal. | Treat orbit/covariance estimation and on-premise processing as mature comparison categories. We must compare against reference data and never assume our current orbit estimates are better. |
| Kayhan Space | Autonomous safety-of-flight alerts and notifications, decision plans, satellite catalog and Dynamics astrodynamics estimation/simulation. | Operator alert latency and decision-support usefulness are existing competitive capabilities, not a novel claim for Luminous. |
| NASA CARA | Published 2-D and 3-D probability-of-collision methods, Monte Carlo, single-covariance maximum Pc, covariance-realism and collision-consequence tools, plus operational process guidance. | Use as a primary technical reference and method-applicability baseline, rather than treating generic algorithm labels as equivalent implementations. |

Primary public references:
- TraCSS overview / November 2026 pilot figures: https://space.commerce.gov/traffic-coordination-system-for-space-tracss/
- LeoLabs Space Domain Awareness product claims: https://leolabs.space/space-domain-awareness/
- Slingshot product overview: https://www.slingshot.space/product-overview
- COMSPOC SSASuite: https://www.comspoc.com/ssasuite
- Kayhan Space: https://kayhan.space/
- NASA CARA public software: https://www.nasa.gov/cara/publicly-available-cara-software/

Do not convert vendor-published figures into a common league table: their sensor coverage, orbit populations, object classes, latency definitions, coordinate conventions and data-access products are not matched.

### Where Luminous should compete

The defensible initial position is **not better sensors, larger catalogs or more accurate orbit determination**. It is a complementary assurance/federation layer:

1. Preserve source-native CDM/OEM/OCM bytes, schema revision, units, frame, covariance and event-time semantics.
2. Bind every projection, transform, hypothesis and decision to exact inputs, code/configuration revision, rights scope and digest.
3. Expose unknown common-source dependencies rather than counting mirrors, brokers or model summaries as independent observations.
4. Let operators share only authorized projections while retaining their local records, policy and operational authority.
5. Reproduce a decision from immutable inputs after partitions, corrections, retractions and software upgrades.
6. Separate a proposed decision, operator acknowledgement, operator authorization, execution report and independently observed outcome.
7. Abstain when evidence is stale, clock/frame-incomparable, out of domain or insufficient for the requested decision.

These are programme hypotheses. Public product pages do not establish that competitors lack these properties; the comparison must be performed by testing actual interfaces, data products and independently available evidence.

### 9.1 Operational cross-provider parity baseline (2026-10-07)

The most useful new industry benchmark is the joint **TraCSS–EU SST operational information exchange study**, published by the Office of Space Commerce on October 7, 2026:
https://space.commerce.gov/eusst-tracss-publish-joint-study-on-operational-information-exchange/

For its specific analysis period, the study reports that TraCSS published about **45× as many conjunction events** as EU SST, while publishing CDMs for **65% of EU SST's 1,432 events**. Approximately 70% of TraCSS-only LEO events involved SPCAT ephemerides for Starlink secondaries that EU SST generally does not publish when preferable orbit sources are available; 95% of TraCSS-only GEO events were below general EU SST publication-risk thresholds. The report describes EU SST as prioritizing actionable products from high-priority sources and TraCSS as screening/distributing more comprehensively. It also notes that output converges when both services use comparable operator-provided data. OSC explicitly cautions that the methodological differences do not establish superiority of either system.

The operational implications are important:

1. **Do not use event count as an accuracy metric.** A higher CDM volume may result from wider input populations, more permissive publication thresholds, or more comprehensive screening—not better orbit estimates.
2. **Separate detection from publication.** Measure the internal candidate-event set and the events distributed to operators as distinct stages.
3. **Stratify by input lineage.** Compare operator ephemeris/covariance, government catalog, SPCAT-like elements, commercial tracks, sensor-network solutions and TLE-only cases separately.
4. **Normalize the experiment.** Re-run common inputs using matched time windows, screening volumes, HBR assumptions, covariance availability, frame/time conventions and publication policy.
5. **Explain every unmatched event.** Distinguish missing inputs, source-quality differences, no covariance, screening-volume differences, event-search differences, publication thresholds and unknown causes.
6. **Measure convergence on shared operator data.** This is the strongest publicly described signal in the joint study for separating source-data effects from analysis-policy effects.

The published study's scope and numbers are not universal. They must not be turned into a vendor ranking or generalized false-negative rate. The dedicated comparison protocol is tracked in #6388.

### 9.2 Regime-specific screening-volume baseline (NASA CARA, March 2026)

NASA CARA's public *Unique Conjunction Events* page (last updated November 2, 2026) states that screening volumes designed before megaconstellations no longer fully and consistently satisfied the capture-rate requirement. The page describes revised volumes implemented since March 2026 as designed to capture **99% of conjunctions with Pc > 1e-4 in the current orbital environment**.

| Orbit regime | Perigee / regime | Radial (km) | In-track (km) | Cross-track (km) |
|---|---|---:|---:|---:|
| LEO-1 | < 500 km | ±1 | ±50 | ±55 |
| LEO-2 | 500–750 km | ±1 | ±30 | ±30 |
| LEO-3 | 750–1200 km | ±1 | ±30 | ±25 |
| LEO-4 | 1200–2000 km | ±0.5 | ±5 | ±5 |
| MEO | > 2000 km | ±15 spherical | — | — |
| GEO / DS | geosynchronous / disposal | ±5 | ±25 | ±15 |
| HEO | eccentricity > 0.25 | ±30 | ±85 | ±85 |

These current dimensions are materially different from NASA's pre-March-2026 table (for example, LEO-2 changed from ±0.5 / ±17 / ±20 km to ±1 / ±30 / ±30 km). Preserve historic and current profiles as separate versioned fixtures; do not average their dimensions or claim the 99% capture design target as a universal theorem.

**Benchmark consequence:** compare candidate-generation capture and compute burden by orbit regime, source class and profile revision. A hit in a screening volume is only a broad-phase candidate; it is not a collision warning or a Pc result. The matcher and event search must run afterward, with their own miss/abstention metrics. NASA's process explicitly separates event prediction from risk assessment and mitigation.

References:
- NASA CARA unique conjunction events and March-2026 volume table: https://www.nasa.gov/cara/unique-conjunction-events/
- NASA CARA screening description: https://www.nasa.gov/cara/step-1-conjunction-event-prediction/
- NASA CARA risk assessment: https://www.nasa.gov/cara/step-2-close-approach-risk-assessment/

Tracked in #6395; time-domain search and continuous-time candidate follow-through remains in #6381.

### Fair comparative protocol

For any performance comparison, freeze the same input artifacts and evaluation budget across systems. Bind:

- product/version, configuration and API/schema profile;
- source observation set and access permissions;
- orbital regime, object class and sensor coverage;
- epoch, time scale, reference frame, origin and units;
- covariance provenance, correlation assumptions and hard-body radius;
- screening volumes, propagation model, force model and reference truth;
- hardware/compute budget, tasking budget and observation cut-off;
- exact output bytes, latency definition, failures and missing-data behavior.

If the competitor's required input, model configuration or result is unavailable, report **not comparable**; do not fill the gap with a marketing claim or synthetic substitute.

### Metrics to publish separately

| Layer | Required measurements |
|---|---|
| Tracking / orbit estimation | Position and velocity residuals by orbit regime and lead time; empirical covariance coverage; maneuver/uncorrelated-track detection; latency and custody gaps |
| Conjunction assessment | Event recall and false-alert burden reported separately for TraCSS spherical and SFSH screening volumes; TCA/miss-distance error against independent trajectory truth; Pc calibration and interval coverage against method-compatible references; applicability/abstention rate by method; correlated-error sensitivity; false-negative severity |
| Cross-provider parity | Common-input event overlap, source-class-specific miss rates, unmatched-event reason distribution, detection-vs-publication split, operator-data convergence |
| Interoperability | Byte/digest preservation; mandatory/optional field correctness; unit/frame/time/covariance semantic loss; schema-version support; duplicate/correction handling |
| Evidence and governance | Replay equality; provenance coverage; dependency completeness; unauthorized disclosure/action rejection; revocation and purpose enforcement |
| Federation | Partition visibility; stale-view detection; reconciliation correctness; recovery time; silent-overwrite count; local autonomy during disconnection |
| Decision support | Human review time; decision lead time; recommendation calibration; abstention quality; operator acceptance/rejection and later observed outcome |

No single composite score should hide a poor result in one safety-critical category.

### Stretch targets — prospective, not achieved

- 100% correct source-field disposition on the frozen supported-message corpus; zero silent unit, frame, time or covariance reinterpretation.
- Zero unauthorized reads/writes and zero false-independent corroboration on the frozen negative-control corpus.
- 100% deterministic replay of the frozen evidence projection for identical inputs, parser, model and policy revisions.
- Three independent federation nodes survive delayed/duplicate/reordered delivery and partition/recovery with no silent overwrite.
- At matched detection recall on a held-out, independently adjudicated corpus, target 20% fewer false alerts than the strongest matched baseline.
- At equivalent correctness and provenance completeness, target 2× faster cross-operator evidence reconciliation than the measured baseline.

The last two are research targets. They become claims only after preregistration, matched hardware/data/coverage, a blind holdout, exact-head CI, independent oracle and external review.

## 10. Interoperability profile matrix and data-fidelity routing

TraCSS publishes multiple related but non-interchangeable space-data profiles. Mycelix must select an adapter by explicit profile/version and declare what downstream calculations the parsed source can support.

| Profile | Content | Encoding / semantics | Intended use and limit |
|---|---|---|---|
| CCSDS CDM 508.0-B-1 v1.0 | Conjunction result at TCA | Standards-native KVN object blocks; CCSDS message version 1.0 | Interchange of conjunction assessment results; baseline parser tracked in #6369 |
| TraCSS CDM Spec 001 Release 2.1 | Conjunction result | KVN, XML, JSON-ST, CSV, JSON-TraCSS; CCSDS message version 2.0 for the KVN/XML profile; ITRF in four profiles and EME2000 in JSON-TraCSS | Provider-specific CDM profile family, tracked in #6383 |
| TraCSS OMM Spec 004 Release 1.2 | Mean Keplerian elements for one object at one epoch | KVN and JSON; UTC; Earth origin; declared mean-element theory; Alpha-5 support in TLE-related fields | Appropriate for the profile's mean-element / antenna pointing / contact-planning uses; not automatically suitable for high-fidelity conjunction assessment |
| TraCSS OCM Spec 002 Release 2.1 | Comprehensive object/orbit message | Ordered sections; ephemeris time histories, optional covariance, physical properties, maneuvers, perturbations and orbit-determination data | Candidate high-fidelity orbit input only after section, time-span, frame, covariance and maneuver-suitability checks; tracked in #6384 |

Normative references:
- TraCSS CDM v2.1: https://space.commerce.gov/wp-content/uploads/2026/01/TraCSS-Spec-001-v2.1_CDM.pdf
- TraCSS OMM v1.2: https://space.commerce.gov/wp-content/uploads/2026/01/TraCSS-Spec-004-v1.2_OMM.pdf
- TraCSS OCM v2.1: https://space.commerce.gov/wp-content/uploads/2026/01/TraCSS_SE_SPEC_002_v2.1.pdf
- January 22, 2026 specification update: https://space.commerce.gov/osc-publishes-updated-tracss-specifications/

### Non-interchangeability rules

- The TraCSS **document release** (for example, CDM specification 2.1) is distinct from the message-internal version (for example, `CCSDS_CDM_VERS=2.0`). Record both.
- The five TraCSS CDM formats are different source profiles. KVN/XML use the CCSDS representation; JSON-ST/CSV are not perfectly identical to CCSDS; JSON-TraCSS has different keys and an EME2000 state-vector convention. Never infer the profile from one or two field names.
- The OMM specification describes mean Keplerian elements at a single epoch and expressly says OMM is not recommended for assessing mutual physical interference, collaborative maneuver development or precise orbit propagation. Do not silently promote an OMM into a high-fidelity ephemeris for collision-risk decisions.
- OCM section order, mandatory/optional/conditional fields, relative-versus-absolute time tags, `EPOCH_TZERO`, usable time span, covariance/Orbit Data section matching and maneuver discontinuities must remain explicit. A flattened state vector is not a lossless OCM representation.
- OCM with ephemeris and covariance is only a candidate input to a qualified screening path. Parsing success does not validate orbit-determination quality, covariance realism, force-model applicability or Pc.
- A format, profile, unit, frame, time scale or covariance convention that is unsupported must produce a typed unsupported/indeterminate outcome, never be coerced into the closest available type.

### Paradigm shift: fidelity-aware ingestion

Instead of one generic `orbit_data` type, route by source fidelity and intended calculation:

`raw artifact -> explicit profile -> typed source semantics -> fidelity/suitability assessment -> permitted calculation -> evidence receipt`

The router should distinguish at least:
- mean elements at an epoch (OMM);
- TLE/SGP4 elements and their age/quality limitations;
- state vector at an epoch;
- ephemeris time history with declared interpolation/usable span;
- ephemeris plus compatible covariance;
- pre-/post-maneuver or otherwise discontinuous trajectory segments.

This prevents a format that is adequate for one purpose from being treated as evidence for a more demanding calculation.

## 11. Delivery gates

- **P0 — contract freeze:** envelope, authority matrix, rights model, time/frame semantics, reason codes and claim ceilings.
- **P1 — corpus:** 30+ synthetic cases, independent oracle, positive/negative controls.
- **P2 — read-only adapters:** selected CDM/OEM/OCM versions, exact source-byte preservation and loss reports.
- **P3 — three-peer test:** delayed/reordered/duplicate delivery, partition, restart, revocation and reconciliation.
- **P4 — public differential:** applicable NASA CARA algorithm references plus separate TraCSS spherical/SFSH event-screening tracks, with convention matching and exact build/data hashes.
- **P5 — independent review:** security, privacy, orbital-safety, governance and standards review.
- **P6 — consented shadow pilot:** read-only exchange with operators; no command or maneuver execution path.

## 12. Non-claims

A passing synthetic corpus does not establish sensor accuracy, physical truth, orbit determination quality, calibrated probability of collision, maneuver safety, flight qualification, cybersecurity certification, legal authority, spacecraft control or readiness for operational deployment.

The profile's intended contribution is a narrow but important one: independently verifiable evidence federation that helps space operators coordinate while retaining local sovereignty and accountable human authority.
