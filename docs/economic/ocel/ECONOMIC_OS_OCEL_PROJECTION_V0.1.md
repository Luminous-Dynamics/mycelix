# Economic OS object-centric process projection v0.1

**Status:** synthetic OCEL 2.0 JSON fixture, semantic crosswalk, owner-scoped causal sidecar and public adversarial corpus. This is not a production exporter, canonical ledger, standards certification or live domain adapter.

## 1. Architectural decision

The Economic OS keeps FIN-EFFECT owner effects, domain-owned journals and source evidence canonical. OCEL is a derived analytical view for object-centric process mining.

The projection is not another transaction lifecycle. One event may relate to several objects, and objects may be related without appearing in the same event. This is essential for a procurement process where an obligation, purchase order, supplier, delivery lot, invoice, payable, payment instruction, account statement and beneficiary all evolve independently.

The first fixture is a synthetic procurement process with 11 object instances and 12 events. It deliberately separates:

- budget reservation from purchase-order commitment;
- supplier commitment from delivery dispatch;
- delivery visibility from goods receipt and beneficiary acceptance;
- invoice issuance from payable recognition;
- payment instruction from provider status;
- statement entry observation from a recipient-authored receipt claim.

All monetary amounts are integer minor units with explicit `SAP-NOTE-V1` instrument profile. Physical quantities carry an explicit unit profile. No floating-point money or inferred FX/unit conversion is permitted.

## 2. Standards baseline as of 25 November 2026

### Stable first target: OCEL 2.0 JSON

The first export target is the officially documented OCEL 2.0 JSON representation with the four top-level collections:

- `eventTypes`
- `events`
- `objectTypes`
- `objects`

Events carry `id`, `type`, an ISO-formatted `time`, attributes, and qualified event-to-object `relationships`. Objects carry `id`, `type`, time-valued attributes and qualified object-to-object relationships. The supported attribute types are string, time, integer, float and boolean. The normative JSON representation does not have a dedicated event-to-event causal-edge collection.

Official references:
- https://www.ocel-standard.org/specification/formats/json/
- https://ocel-standard.org/2.0/ocel20_specification.pdf (Section 8)

The synthetic fixture contains only standard OCEL root collections. Its descriptive metadata, loss manifest and causal graph are separate sidecar files; nonstandard keys are not injected into the OCEL JSON root.

### Experimental compatibility target: OCEL 2.1.0pre5

The official site currently documents an OCEL 2.1.0pre5 specification dated 1 October 2026. It describes single-file CSV and bundled CSV/Parquet representations. The bundled format uses typed event/object/object-change tables and E2O/O2O relation tables.

References:
- https://www.ocel-standard.org/specification/formats/csv/
- https://www.ocel-standard.org/specification/formats/bundled/
- https://ocel-standard.org/2.1/ocel20_specification.pdf

Because the cited specification is explicitly pre-release, v0.1 does not make it the canonical target. It is a secondary compatibility experiment. Pin the precise pre-release version in any test report and re-evaluate when a stable 2.1 release is published.

The official tool-support catalogue lists PM4Py, FLOWVAULT, Ocelot and other OCEL-compatible tools. The official event-log catalogue includes Procure-to-Pay and synthetic relational/reference OCEL datasets in multiple formats. These are more meaningful interoperability baselines than self-authored JSON examples alone:
- https://www.ocel-standard.org/tool-support/overview/
- https://www.ocel-standard.org/event-logs/overview/
- https://www.ocel-standard.org/event-logs/simulations/synthetic-relational-databases/

## 3. Loss-aware event time

OCEL 2.0 events require `time`; object attribute assignments also carry time. The exporter therefore requires a qualified source event-time assertion and a provenance reference.

Each synthetic event includes:
- `time`: the event time represented in the OCEL standard field;
- `time_basis`: `source_record_event_time` in this fixture;
- `time_evidence_ref`: an explicit reference to the synthetic source-time assertion;
- `source_event_ref`: a source-native event identifier.

Object attribute changes carry their own OCEL `time`; production adapters must also preserve the source time basis and provenance in declared object attributes or a separately versioned provenance sidecar.

Never substitute:
- record time or database commit time for event time;
- ingestion time or import order for event time;
- owner-local sequence or topological layer for wall-clock time;
- the exporter’s current clock for an absent source timestamp.

Missing or contradictory event time is held outside the OCEL event set and recorded in the loss manifest. It is not assigned a guessed timestamp. This means the manifest is part of the projection evidence: an OCEL JSON file alone does not prove that the source event set was complete.

## 4. Object model and relationship direction

The machine-readable crosswalk is `economic-os-ocel-crosswalk.v0.1.json`. It maps FIN-EFFECT roles and source-domain assertions to OCEL event types, object types and qualified relationship names.

O2O relationships must be directionally and semantically justified by a source-domain record. For example:
- an invoice may reference a purchase order;
- a payable may reference an invoice;
- a payment instruction may reference a payable;
- a statement entry may correlate with a payment instruction.

These relations do not imply that:
- the invoice proves delivery;
- the payable proves payment;
- the instruction proves settlement;
- an object is legally owned by another object;
- an agent named in provenance has authority.

Object relationship qualifiers describe analytical relationships only. Authority, legal ownership and settlement finality remain separate qualified claims.

## 5. Causal sidecar

Event-to-object and object-to-object links do not represent event-event happens-before. The Mycelix causal graph therefore remains a separate sidecar.

The sample sidecar uses owner-scoped nodes with:
- `owner_id`;
- `event_id` unique within that owner;
- `ocel_event_id` identifying the derived OCEL event;
- zero-based `owner_sequence`;
- exact `owner_predecessor`;
- explicit cross-owner `caused_by` references.

No global sequence or timestamp is synthesized. A topological layer means incomparable in the supplied graph, not simultaneous. The sidecar must be validated by the independent causal verifier from SAP causal frontier PR #6653 (or a separately pinned compatible verifier version).

An OCEL export that omits the sidecar is an incomplete analytical projection for claims requiring causal reconstruction. It remains usable for process-mining queries that do not assert those causal claims, provided the omission is explicit.

## 6. Corrections, reversals and partial fulfilment

A correction is a new event that references the original assertion/effect. It does not overwrite or delete the original source event. A reversal is a separately identified effect with an explicit relation to the original; it is not the erasure of history.

A partially fulfilled order or obligation remains partial. Split invoices and deliveries are separate objects/events related to the original order/obligation. Conservation, asset identity, quantities, units, fees and rounding remain verified by their domain-owned economic kernels, not by OCEL relationships.

## 7. Loss manifest and adversarial corpus

- `fixtures/economic-os-procurement-loss-manifest.v0.1.json` records source records held out because time evidence is missing or contradictory.
- `benchmarks/economic-os-ocel-adversarial-corpus.v0.1.json` contains 36 synthetic cases covering event-time promotion, identity collisions, dangling relationships, semantic promotion, unit/currency confusion, partial/split fulfilment, corrections, missing provenance and causal graph attacks.

The cases are public development fixtures, not hidden evaluation. Passing a structural workflow is not full JSON Schema conformance or process-mining tool interoperability.

## 8. Benchmark programme

### Safety and semantic preservation (non-compensatory)

Targets, not achieved results:

| Metric | Target |
|---|---:|
| Silent loss of required identity, owner, role, unit, quantity or provenance | 0 |
| False event-time promotion | 0 |
| Plan-to-observed-event promotion | 0 |
| Visibility-to-acceptance/title promotion | 0 |
| Status-to-settlement/legal-discharge promotion | 0 |
| False authority/ownership inference | 0 |
| Accepted dangling E2O/O2O/cause references | 0 |
| Non-deterministic export on frozen inputs | 0 |
| Rust/independent-verifier disagreement | 0 |

### Matched comparison arms

A. **Flattened case-based log:** one case per obligation or purchase order, with explicit measurement of event duplication and lost cross-object context.

B. **Native standards validation only:** source-format/OCEL structure validation without FIN-EFFECT owner/claim profiles or causal sidecar.

C. **OCEL plus Mycelix loss manifest:** object-centric projection and explicit semantic-loss/exclusion reporting.

D. **OCEL plus owner-scoped causal sidecar:** adds causality/frontier reconstruction without pretending OCEL itself contains the owner-event happens-before relation.

Use the same frozen source records, mapping profile, hardware and resource budget. Keep hidden expected claims and the expert evaluation subset outside candidate reach. LLM evaluation is not the primary semantic oracle.

### Performance dimensions

At 100, 1,000, 10,000 and 100,000 events/objects/relationships, measure p50/p95/p99 export and validation time, throughput, peak memory, output size, bytes per object/event/relation, read amplification, and operator reconciliation effort. Separate JSON parsing, mapping, causal projection, loss-manifest generation and storage/transport costs.

## 9. Implementation stages

1. Freeze the crosswalk, sample fixture, causal sidecar, loss manifest and adversarial corpus (this increment).
2. Implement a deterministic Rust OCEL 2.0 JSON exporter using typed inputs and exact values; validate the output with independent OCEL tooling.
3. Add independent Rust/Go or Rust/third-party verifier parity for export identity and relationship/loss classifications.
4. Test public OCEL Procure-to-Pay and synthetic relational reference datasets through round-trip import/export, comparing semantic graph preservation rather than byte formatting.
5. Experiment with OCEL 2.1 CSV and bundled CSV/Parquet as a pinned compatibility layer, without replacing v2.0 JSON until the spec status and tool ecosystem warrant it.
6. Integrate actual domain adapters only after owner authorship, authority and source-time provenance are qualified.

## 10. Claim ceiling

The present artefacts are synthetic structural fixtures. They do not establish an exporter implementation, standards certification, field performance, legal compliance, physical delivery, title, supplier legitimacy, accounting correctness, settlement finality, source-record authenticity or data-source completeness.
