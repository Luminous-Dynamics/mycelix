# Logistics Standards and Interoperability Registry — Draft v0.1

**Status:** design draft; not an implementation or conformance claim.  
**Reviewed on:** 2026-10-13.  
**Parent work:** [Mycelix #437 — loss-aware transport interoperability](https://github.com/Luminous-Dynamics/mycelix/issues/437), [#1110 — supply-chain traceability bridge](https://github.com/Luminous-Dynamics/mycelix/issues/1110), and [#1112 — semantic translation fidelity](https://github.com/Luminous-Dynamics/mycelix/issues/1112).  
**Related physical/logistics work:** [Mycelix #4900](https://github.com/Luminous-Dynamics/mycelix/issues/4900), [#4901](https://github.com/Luminous-Dynamics/mycelix/issues/4901), and [Symtropy PR #1602](https://github.com/Luminous-Dynamics/symtropy/pull/1602).

## 1. Purpose and boundary

Mycelix should catalogue, version, map, and qualify external standards that participants use to exchange logistics, cargo, organization, and space-network information. It should **not** republish proprietary normative text, replace standards bodies, assert whole-standard conformance from a narrow adapter, or make an external schema the canonical definition of internal authority.

The registry is a catalogue of **versioned semantic profiles and adapters**. Mycelix keeps its own domain semantics and uses adapters to interpret source-specific claims while preserving provenance, uncertainty, loss, and nonclaims. Symtropy owns deterministic route/module/equipment feasibility and optimization. Sol Atlas consumes permitted, versioned projections for exploration. A registry entry is never itself a certificate that a cargo unit, organization, shipment, route, or mission is safe or legally compliant.

Governing boundaries:

- standard publication != artifact access permission != adapter implementation != tested conformance;
- schema validity != semantic equivalence != legal equivalence != authority equivalence;
- carrier/provider-signed event != independent physical observation != verified custody != proof of delivery;
- container geometry/interface mapping != safe lift/restraint approval;
- communication contact/message delivery != physical movement/delivery;
- visualization != qualification.

## 2. Mandatory registry-entry contract

Each entry should have stable identity and an append-only change history. At minimum record:

1. **Source identity:** standards body/owner, canonical identifier, official source URL, domain, and named profile.
2. **Exact version:** edition/release, schema/API/ontology version as applicable, publication date if known, and the precise artifact(s) used. Avoid vague labels such as “latest” or “2.x” in a qualification record.
3. **Status:** published, under publication, draft/beta, withdrawn, superseded, or not reviewed. Do not infer status from a secondary blog when the standards owner has an official status page.
4. **Access/rights:** public, member/partner-restricted, licensed/paid, redistributable under stated terms, or unknown. Record terms/review date. Store pointers and permitted hashes, not copied normative material that the project has no redistribution right to publish.
5. **Adapter:** implementation name/version/commit; exact supported subset; import/export directions; supported field mapping; units/time/location/identity rules; required policy context.
6. **Mapping result:** preserved, normalized, narrowed, widened, derived, unknown, conflicting, omitted/unmapped, or unsupported fields; machine-readable loss class; source and target commitments.
7. **Qualification:** fixture/conformance-suite identity and digest; test result and exact tested source SHA; implementation coverage; known gaps; reviewer and review date. “Catalogued” and “fixture-tested” must never be represented as “fully conformant.”
8. **Lifecycle:** supersedes/superseded-by, deprecation date or review trigger, migration guidance, and compatibility notes.

Registry changes must not silently rewrite previous evidence. An event is interpreted using the versioned profile recorded with that event; a new adapter may reprocess it, but creates a new derivation and mapping receipt rather than overwriting the original claim.

## 3. Initial standards inventory

The rows below are a **research starting point**, not an assertion that adapters exist or that every listed artifact is freely redistributable. Re-check the authority's current publication/access page when implementing each adapter.

| Domain | Standard/profile | Status and use at review | Initial treatment |
|---|---|---|---|
| Cross-company visibility | [GS1 EPCIS Standard 2.0.1](https://ref.gs1.org/standards/epcis/2.0.1/) and [CBV 2.0.0](https://ref.gs1.org/standards/cbv/2.0.0/) | GS1's EPCIS archive listed 2.0.1 as the latest published EPCIS issue and CBV archive listed 2.0.0 as latest at review on 2026-10-14; pin these artifacts separately | **P0:** map only a declared EPCIS 2.0.1 / CBV 2.0.0 subset; preserve source payload/unknown fields, and keep `eventTime` distinct from repository `recordTime`; never claim adapter conformance from the version pin alone |
| Ocean container visibility | [DCSA Track & Trace 2.2.0](https://dcsa.org/standards/track-and-trace) | DCSA describes 2.2.0 as the established foundation for shipment, transport and equipment visibility | **P0 baseline candidate:** pin the exact currently accessible spec/API artifacts and their terms before claiming adapter conformance |
| Ocean IoT/reefer visibility | [DCSA Track & Trace 3.0.0](https://dcsa.org/newsroom/track-and-trace-3-0-0-iot-reefer-visibility) | DCSA announced 3.0.0 on 2026-09-30 with IoT and reefer events; the announcement says initial access is for members/DCSA+ partners and public release is planned for H2 2027 | **Catalogued/restricted:** record the announced version and access state; do not claim reproducible public implementation or conformance until artifacts/fixtures are lawfully accessible |
| Air-cargo data graph/API | [IATA ONE Record cargo ontology](https://onerecord.iata.org/ns/cargo/index-en.html) and [API ontology](https://onerecord.iata.org/ns/api/index-en.html) | Versioned pages show cargo ontology 3.3.0 and API ontology 2.3.0 in 2026; pages identify the ontology specification as draft | **P0 mapping candidate:** pin ontology and API versions separately; confirm the applicable endorsed programme release/status before making conformance claims; model object links, not only events |
| Cross-modal trade semantics | [UN/CEFACT Multimodal Transport Reference Data Model](https://unece.org/trade/uncefact) | Cross-domain reference model; precise release and supported subset must be pinned by the adapter | **P0 crosswalk:** select explicit version/release and map only the reviewed semantic subset |
| Trade locations | [UN/LOCODE Recommendation 16](https://unlocode.unece.org/recommendation16/) | Location identifiers and evolving code-list data | **P0:** version each downloaded list snapshot and preserve code-list revision/lookup date; a port code is not an exact coordinate or proof of presence |
| Container dimensions/ratings | [ISO 668](https://www.iso.org/standard/76912.html) | Series 1 intermodal container dimensions/classifications/ratings; the applicable edition must be explicitly recorded | **P0 physical profile metadata:** geometry/rating mapping only; not operational compatibility or lifting approval |
| Container fittings | [ISO 1161](https://www.iso.org/standard/86404.html) | ISO page shows ISO 1161 Edition 6 (2026-10) as **under publication**, replacing published ISO 1161:2016 | **P0 status-sensitive:** keep published 2016 and Edition 6 under-publication identities distinct; do not silently mark Edition 6 published before the authority updates its status |
| Container handling/securing | [ISO 3874:2017](https://www.iso.org/standard/63826.html) | Published; specifies handling and securing methods for Series 1 containers built/tested to ISO 1496 | **P0 reference profile:** link requirements to equipment-specific operation evidence; a profile match does not certify a lift or securement |
| Aircraft unit-load devices | [IATA ULD programme](https://www.iata.org/uld) and applicable ULD Regulations | Aviation equipment has aircraft/type/position, restraint, gross-mass, centre-of-gravity and serviceability constraints; exact applicable edition and access rights must be checked through the authorized source | **P0 isolated profile family:** no inference from ISO container geometry; do not copy restricted normative content into the repository |
| Organization supply-chain security management | [ISO 28000:2022](https://www.iso.org/standard/79612.html) and [Amd 1:2024](https://www.iso.org/standard/88413.html) | Published organizational security-management system requirements; Amendment 1 concerns climate-action changes | **P1 governance reference:** maps management-system claims only; not a per-shipment safety certificate or proof a supplier meets every legal requirement |
| Delay-Tolerant Networking | [IETF Bundle Protocol v7 — RFC 9171](https://www.rfc-editor.org/rfc/rfc9171.html); [BPSec — RFC 9172](https://www.rfc-editor.org/rfc/rfc9172.html) | Public RFC standards for bundle transfer and security mechanisms | **P1 space/network adapter:** map protocol/service evidence; BPSec does not establish Mycelix application truth or permission |
| Scheduled space communications | [CCSDS Schedule-Aware Bundle Routing](https://ccsds.org/publications/allpubs/entry/3167/) | External scheduled-contact routing specification family; pin the exact Blue Book/release used by an adapter | **P1 separate contact graph:** no physical cargo movement semantics in this adapter |
| Lunar communications/PNT | [NASA/ESA/JAXA LunaNet Interoperability Specification](https://www.nasa.gov/directorates/somd/space-communications-navigation-program/lunanet-interoperability-specification/) | NASA page identifies Version 5, published 2025-02-07, as the current listed version at review | **P1 reference profile:** track the exact document/version and partner-compatible services; this is not a physical cargo interface standard |
| Space logistics architecture | [NASA Moon to Mars Architecture — Components](https://www.nasa.gov/moontomarsarchitecture-components/) | Architecture reference separates logistics, transportation, mobility, communications/PNT, data systems and ISRU | **Reference only:** use for capability decomposition and scenario vocabulary, not as a claim of standardized end-to-end lunar/Mars cargo-unit interfaces |

### Profiles that should remain research-only until reviewed

UN Transparency Protocol (UNTP), national/customs profiles, verifiable trade documents, organizational identity credentials (including LEI/vLEI), dangerous-goods rules, and jurisdiction-specific product-passport profiles may be valuable additions. Catalogue them only after review of exact publication status, data model, rights, scope, and relationship to existing Mycelix identity/evidence types. Do not add a long list simply to imply coverage.

## 4. Adapter invariants

Every adapter must preserve the core constraints already proposed by #437 and #1112:

- **No semantic promotion:** authentication of a carrier event does not make its physical content independently true; traceability does not establish ownership, origin, conformity, regulatory compliance, delivery or payment.
- **No invented facts:** never synthesize a missing arrival, custody transfer, coordinate, certification, time, quantity, or issuer to make a target schema look complete.
- **Preserve unknowns and conflicts:** unknown source values must not become false/zero/satisfied. Contradictory source histories remain visible until an explicit reconciliation policy derives a result.
- **Qualified time:** keep business-event time, sensor-observation period, repository record time, ingestion time, authoring time and clock scale distinct. In EPCIS, `eventTime` is when the capturing application asserts the business step occurred; sensor `startTime` / `endTime` describe an observation period; `recordTime` is repository bookkeeping and says nothing about when the real-world event occurred. Never use `eventTime` or `recordTime` as a substitute for the actual capacity/condition observation time without an explicit source-semantic mapping ([EPCIS 2.0.1, §7.3](https://ref.gs1.org/standards/epcis/2.0.1/)).
- **Qualified location:** a port/facility identifier or named place must not silently become an exact coordinate or proof that an asset was there.
- **Typed identity and aggregation:** distinguish shipment/consignment, package/piece, pallet/handling unit, container/equipment, vehicle/vessel/flight, booking/waybill/bill-of-lading and aggregation/association. Similar identifier strings in different schemes are not equal by default.
- **Unit and direction integrity:** conversions must preserve original units, conversion method/version, rounding/loss and directionality. Do not normalize kg/lb, currency, temperature, mass and volume without an explicit profile.
- **Version stability:** mapping receipts bind the exact source profile, target profile and adapter implementation. Reinterpreting old evidence with a new adapter creates a derivation.
- **Rights-respecting artifacts:** only check in standards material, example payloads or conformance vectors that repository policy and applicable terms allow us to redistribute. When tests use restricted fixture content, document the access/reproduction procedure without publishing prohibited files.
- **Separate graphs:** physical routes, communication contact opportunities and evidence reconciliation remain different typed graphs. A successful network transfer cannot satisfy a physical delivery predicate.
- **Advisory intelligence:** AI-assisted mapping can suggest candidate correspondences, but only frozen deterministic rules and test fixtures qualify a mapping.

Every mapping receipt should state a loss class such as `Lossless`, `RepresentationOnly`, `Narrowing`, `Widening`, `Partial`, `Ambiguous`, `ConflictPreserving` or `Unsupported`. Round-trip representation equality is not by itself legal or authority equivalence.

## 5. Implementation order

1. **Registry contract and reviewed inventory.** Add the typed registry metadata, status/access enums, source/artifact commitments, and lifecycle/supersession relations. Avoid embedding proprietary normative texts.
2. **EPCIS 2.0/CBV narrow adapter.** Freeze permitted fixtures for identity, event/record time, location, sensor value/unit, business step and disposition. Preserve unmapped fields and original source commitment.
3. **DCSA public baseline.** Pin and review an accessible release (2.2.0 is the documented predecessor baseline); represent 3.0.0 as restricted/unqualified until its exact artifacts and test fixtures are available to the intended implementers.
4. **ONE Record profile.** Pin cargo ontology, API ontology, API version and programme endorsement separately; preserve shipment/piece/transport-object links rather than flattening them into an event stream.
5. **UN/CEFACT + UN/LOCODE crosswalk.** Version the exact MMT release and location-code snapshot; report ambiguous code/name/country mappings.
6. **Physical interface reference links.** Store exact ISO/IATA profile metadata and qualified evidence references. Symtropy still owns the actual dimension, mass, equipment, operational-envelope and route-feasibility checks.
7. **Space contact adapters.** Add BPv7/BPSec, the exact CCSDS contact-routing release and LunaNet profiles at Mycelix’s CommunicationContactGraph boundary, not inside the physical transport graph.
8. **Independent conformance/fidelity suite.** Keep fixtures independent of the mapping implementation where practical. Include adversarial cases: record-time promotion, code-list version drift, unknown-to-zero, signed event treated as delivery, revoked/expired credentials, lossy round-trips, wrong identifier schemes, restricted/draft profiles masquerading as stable, and contact success treated as cargo movement.

## 6. Acceptance and claims

The first adapter tranche is complete only when:
- each registry entry identifies its source version, current status, access/rights status and exact adapter subset;
- fixture corpus, expected outputs and loss reports are pinned and reproducible offline;
- unknown fields, source identity, source time semantics and conflicts survive import/export as declared;
- adversarial tests prove that schema compatibility cannot upgrade a claim's epistemic or legal authority;
- version changes require an explicit adapter/profile change and invalidate no prior evidence silently;
- tests run at the exact PR head and the result records the tested SHA and artifact/fixture digests.

Passing the adapter suite establishes only that the reviewed mapping preserves/discloses the specified semantics for the tested subset. It does not establish factual truth, legal equivalence, universal vendor interoperability, standard-owner certification, physical cargo safety, custody, delivery, or operational readiness.

## References reviewed

- GS1, [EPCIS/CBV](https://www.gs1.org/standards/epcis)
- DCSA, [Track & Trace](https://dcsa.org/standards/track-and-trace) and [3.0.0 access/publication announcement (2026-09-30)](https://dcsa.org/newsroom/track-and-trace-3-0-0-iot-reefer-visibility)
- IATA, [ONE Record cargo ontology](https://onerecord.iata.org/ns/cargo/index-en.html), [ONE Record API ontology](https://onerecord.iata.org/ns/api/index-en.html), and [ULD programme](https://www.iata.org/uld)
- UN/CEFACT, [Trade standards and data models](https://unece.org/trade/uncefact), [UN/LOCODE Recommendation 16](https://unlocode.unece.org/recommendation16/)
- ISO, [668](https://www.iso.org/standard/76912.html), [1161](https://www.iso.org/standard/86404.html), [3874](https://www.iso.org/standard/63826.html), [28000:2022](https://www.iso.org/standard/79612.html), [28000 Amd 1:2024](https://www.iso.org/standard/88413.html)
- IETF, [RFC 9171](https://www.rfc-editor.org/rfc/rfc9171.html) and [RFC 9172](https://www.rfc-editor.org/rfc/rfc9172.html)
- CCSDS, [Schedule-Aware Bundle Routing](https://ccsds.org/publications/allpubs/entry/3167/)
- NASA/ESA/JAXA, [LunaNet Interoperability Specification](https://www.nasa.gov/directorates/somd/space-communications-navigation-program/lunanet-interoperability-specification/)
- NASA, [Moon to Mars Architecture components](https://www.nasa.gov/moontomarsarchitecture-components/)
