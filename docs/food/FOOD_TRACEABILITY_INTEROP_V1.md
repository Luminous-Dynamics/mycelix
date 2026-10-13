# Mycelix Food Traceability Interoperability Profile V1

**Status:** Draft contract for review; not an implementation or food-safety certification.  
**Related work:** [MYC-FOOD-001 #5001](https://github.com/Luminous-Dynamics/mycelix/issues/5001), [hydroponic showcase #3222](https://github.com/Luminous-Dynamics/mycelix/issues/3222), [ProductiveLoopV1 #3303](https://github.com/Luminous-Dynamics/mycelix/issues/3303).

## 1. Purpose

Give the existing Mycelix Commons food-production and food-distribution zomes a small, testable interoperability profile for community gardens, small farms, ranches, dairy producers, cheesemakers, food hubs and neighborhood pickup networks. This profile complements existing zomes; it does not replace them or introduce a second identity, trust, currency, or provenance system.

The profile separates four concerns:

1. **Planning and forecasting** — intended or estimated future production.
2. **Traceable events** — what a producer/operator/device reports happened, with evidence and source timestamps.
3. **Commerce state** — offers, reservations, quantities allocated, payments and order fulfillment.
4. **Safety / regulatory determinations** — decisions made by the legally accountable operator and competent authorities under applicable requirements.

These are related but not interchangeable records.

## 2. Existing canonical owners

Reuse the existing Mycelix boundaries:

- food-production owns plot, crop, season-plan, resource-input and yield records.
- food-distribution owns markets, listings, orders and their lifecycle.
- food-preservation is the prospective owner for storage, processing, shelf-life and loss concepts.
- Mycelix supply-chain/DKG/credential code carries evidence references and provenance where configured.
- Integral/FRS observation contracts remain the source for typed measurements, source identity, freshness, calibration and derived findings when the integration is available.
- Symthaea supplies non-authoritative forecasts, anomaly findings and candidate recommendations.
- Sol Atlas consumes permitted spatial projections; it is not the inventory or evidence authority.
- Symtropy may support simulation where its model is suitable; simulation output is not physical observation.

No new shared truth model should be created just for food. The schema in this directory is an **interchange envelope** for the boundary between existing owners.

## 3. Standards alignment

Use standards as interoperability seams, not as a claim that Mycelix is certified or fully conforms.

- **GS1 Global Traceability Standard (GTS):** use Critical Tracking Events (CTEs) and Key Data Elements (KDEs) as a design checklist for events such as production/harvest, transformation, packing, dispatch, receipt, storage and handoff.
- **GS1 EPCIS 2.0 / CBV:** map events to EPCIS event semantics where applicable, including event time, business step, disposition, read point/business location, traceable objects/quantities, transformation inputs/outputs, and sensor elements. The Mycelix envelope is not itself an EPCIS payload; implement and test an explicit adapter. Keep custom profile/extension terms named and versioned.
- **UN/CEFACT Recommendation 20:** use a pinned release of standardized trade-unit codes where suitable. Store the code, code system and code-list version. Do not rely on display strings like kg, litre, dozen, box as machine identifiers. A package/count unit is not automatically convertible to mass or volume.
- **W3C Verifiable Credentials / existing Mycelix credentials:** credentials can attest to claims and their issuer. A signature verifies integrity/provenance under a key; it does not by itself prove the claim is factually correct, current, legally sufficient, or a food-safety determination.

References:
- GS1 Global Traceability Standard: https://www.gs1.org/standards/gs1-global-traceability-standard/current-standard
- GS1 EPCIS / CBV 2.0: https://www.gs1.org/standards/epcis
- GS1 EPCIS 2.0 example payloads: https://ref.gs1.org/docs/epcis/examples
- UN/CEFACT Recommendation 20: https://unece.org/trade/documents/2021/06/uncefact-rec20-0
- South African Department of Health Food Control Regulations index: https://www.health.gov.za/food-control-regulations/

## 4. Envelope contract

The machine-readable JSON Schema is at docs/food/schemas/food-traceability-event.v1.schema.json; a valid example is at docs/food/examples/eggs-produced.v1.json. V1 pins unit-code versions to REC20-REV17-2021 so a vague value such as latest cannot silently change interpretation.

Every event carries:

- eventId: stable source event identity. Reuse of this identifier with different semantic content is a conflict, not a harmless duplicate.
- eventType: a declared event kind. PLANNED and FORECASTED can never be promoted to observed production merely by ingestion.
- eventTime: source event time; preserve its timezone/offset.
- recordedAt: time the current system accepted the record. Do not overwrite event time with receive time.
- actorRef, siteRef: stable identity references. Public payloads do not have to reveal a farm's exact address or household identity.
- subjects: product-profile and lot references with a typed quantity state.
- inputs and outputs: required for transformation events, preserving the many-to-many relationship from input lots to output lots.
- handlingObservations: sensor observations, explicitly marking missing and invalid measurements.
- evidenceRefs: references to source evidence with a claim/verification status.
- supersedesEventId and correctionReason: corrections/retractions append a new record and point to the record they supersede; history is not silently overwritten.

### Quantity representation

The interchange profile represents decimal amounts as base-10 strings to avoid loss of precision from binary floating-point serialization. Values must be non-negative and use no exponent notation. Runtime implementations must also enforce domain-specific maximum precision and magnitude.

A quantity includes:
- state: planned, forecast, observed, or unknown;
- decimal-string value;
- unit code;
- unit code system (UNECE_REC20);
- exact code-list version.

Unknown must not become zero or a nominal value. Planned/forecast amounts must not feed available inventory. The product profile is responsible for permitted units and package semantics: eggs can be listed by count or dozen; milk by volume; cheese by weight or declared unit; fresh produce commonly by mass or count. Conversions are allowed only with a declared, product-valid conversion. **Never assume one litre equals one kilogram.**

## 5. Event semantics and invariants

| Event | Semantics | Required invariant |
|---|---|---|
| PLANNED | Intention or planting/production plan | Does not create current supply |
| FORECASTED | Model estimate or projected output | Forecast provenance, horizon and model version stay identifiable; not inventory |
| PRODUCED / HARVESTED | Reported production or crop harvest | Keep source evidence and measurement method; event does not prove edible or marketable status |
| TRANSFORMED | One or more lots transformed into one or more lots | Record input/output lots and quantities; preserve lineage and any loss/by-product |
| PACKED | Product packed or repacked | Preserve source lots, packaging/aggregation relations and quantities |
| DISPATCHED / RECEIVED | Shipment dispatch/receipt | Both events bind the same shipmentRef and explicit origin/destination; dispatch is not proof of arrival and receipt is distinct |
| STORED | Placement/status event for a lot at a site | Location/capability claims are separate from measured handling observations |
| HANDOFF | Physical handover to an authorized recipient/channel | Order intention is not fulfillment; keep receipt separate |
| HANDLING_OBSERVED | Sensor observation or explicit sensor gap/invalid reading | Preserve units, sensor identity, event/receive times and calibration status |
| RECALLED / SPOILED | Lot restriction or loss event | Affected lot lineage propagates to dependent output lots; do not silently continue offering affected stock |
| CORRECTION / RETRACTION | Correct a previous assertion | Append-only link to superseded event, reason and verifier/issuer provenance |

Additional invariants:

- Valid signature != true-world claim.
- Certificate exists != certificate current; check scope, issuer, validity period, revocation/supersession and relevant product/site.
- Sensor reading != safety determination; calibration, sampling gaps, limits and operator procedures matter.
- Missing != safe; stale != current; conflict != average-as-truth.
- Ordered != reserved != dispatched != received != handed off.
- Same event replay is idempotent only when stable event identity and content digest match. Same ID with different content must be quarantined as equivocation/conflict. A correction must not supersede itself or introduce a cycle in the supersession graph.
- Physical quantities in an event remain attributable to source evidence. Derived and estimated quantities must be labeled as such.
- Do not publish household purchase history or exact private producer locations in public views by default.

## 6. Cold-chain / handling evidence

Use EPCIS 2.0 sensor-element concepts through an adapter where appropriate. A handling observation should bind the specific lot/shipment context, sensor identity, measured parameter and unit, observedAt, receivedAt, calibration/currentness status, calibration evidence reference when applicable, and explicit missing/invalid intervals. Thresholds belong to a versioned product/handling rule profile for the relevant jurisdiction and product category—not to a universal Mycelix temperature constant.

The event schema intentionally does not attempt to encode all legal rules or infer safety. An out-of-range observation can trigger a finding or quarantine recommendation, but:
- the finding is not itself a confirmed pathogen or safety verdict;
- a recommendation is not authority to release/dispose product;
- a missing interval is not an acceptable temperature;
- a later in-range measurement does not erase an earlier excursion;
- clearing quarantine requires the designated responsible process and evidence.

For a South African operation, scope at least R638 (general food-premises/transport hygiene), R961 (milking sheds and milk transport), relevant milk/dairy rules, and any applicable perishable-food/product rules with a qualified local operator/regulator. The official Department of Health index is the discovery starting point; this profile is not legal advice and must not guess an applicable temperature or licensing condition.

## 7. Deterministic H0 cases

Before physical integration, test these event sequences using a frozen fixture manifest:

1. Nominal egg lot recorded by dozen and allocated to two pickup hubs.
2. Milk lot stored and shipped with in-range observed segments.
3. Temperature measurement marked missing during a sensor outage.
4. Stale sensor calibration followed by a reading; status is not silently upgraded.
5. Cheese production with a transformation event linking consumed milk to output lot and recorded loss/by-product.
6. Planned harvest and model forecast are ingested; neither increases available inventory.
7. Duplicate delivery of an event with the same ID and same canonical content digest.
8. Same event ID delivered with different content; verifier quarantines conflict.
9. Dispatch with no receiving event; it remains in transit/unknown, not fulfilled.
10. Recall of an input lot; transformation descendants and affected offers become discoverable.
11. Correction supersedes a previously wrong quantity without deleting the original assertion.
12. Invalid unit-code syntax, unpinned code-list version, disallowed product-unit pair, negative amount, NaN, exponent, excess decimal precision, and unit-list membership mismatch.
13. Private producer/household references remain absent from public map projection.
14. Model estimate conflicts with current inventory observation; preserve both and route for resolution.

Each fixture should carry an immutable ID, schema/version, seed where generated, source class (synthetic vs observed), and a content digest. Synthetic fixture success is evidence of protocol behavior only, not agronomic validation, food-safety validation or operational readiness.

## 8. Minimal acceptance gates

- [ ] JSON Schema parses under a draft-2020-12 validator; valid and invalid examples are tested.
- [ ] Cross-field validator checks event-type/quantity-state rules, unit code membership and product-profile allowed-unit policy.
- [ ] EPCIS adapter mapping has round-trip tests for supported fields and explicit loss reports for non-representable fields.
- [ ] Idempotency key/content-digest conflict behavior is tested.
- [ ] Transformation, dispatch/receipt, handoff and recall lineage tests pass.
- [ ] Missing/stale/calibration-invalid handling observations fail closed for any configured control that requires valid current evidence.
- [ ] No forecast or plan can create stock or trigger payment.
- [ ] Mycelix multi-agent/Holochain integrity validation and offline reconnect behavior are tested before field use.
- [ ] Maturity is labelled separately: schema-valid != unit-tested != multi-agent-tested != field-validated != compliant operation.

## 9. Out of scope for V1

No new marketplace, payment rail, food-safety certifier, global product registry, autonomous farm controller, or second DKG. The immediate deliverable is a stable interchange and test contract that can be implemented incrementally by existing product/food zomes and connected to the other projects through bounded read-only projections.
